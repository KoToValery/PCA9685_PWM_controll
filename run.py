#!/usr/bin/env python3
"""
PCA9685 PWM Controller for Home Assistant
"""
import json
import logging
import os
import signal
import sys
import threading
import time
import subprocess

try:
    import requests
    HAS_REQUESTS = True
except ImportError:
    HAS_REQUESTS = False

from smbus2 import SMBus
import paho.mqtt.client as mqtt

logger = logging.getLogger("pca9685_pwm")
_handler = logging.StreamHandler(stream=sys.stdout)
_handler.setFormatter(logging.Formatter("%(asctime)s %(levelname)s %(message)s"))
logger.addHandler(_handler)
logger.setLevel(logging.INFO)

# --- Kernel Module Check ---
try:
    logger.info("Checking i2c-dev module...")
    subprocess.run(["modprobe", "i2c-dev"], check=True)
except subprocess.CalledProcessError as e:
    logger.error("Failed to load i2c-dev module: %s", e)
    sys.exit(1)
except Exception as e:
    logger.warning("Failed to run modprobe i2c-dev: %s", e)

# Global I2C synchronization
I2C_BUS = 1
i2c_lock = threading.Lock()
try:
    shared_bus = SMBus(I2C_BUS)
except Exception as e:
    logger.error("Failed to open I2C bus %d: %s", I2C_BUS, e)
    sys.exit(1)

MODE1 = 0x00
MODE2 = 0x01
PRESCALE = 0xFE
LED0_ON_L = 0x06

MODE1_RESTART = 0x80
MODE1_SLEEP = 0x10
MODE1_AI = 0x20
MODE2_OUTDRV = 0x04

OSC_HZ = 25_000_000


class PCA9685:
    def __init__(self, bus: SMBus, address: int):
        self.address = address
        self.bus = bus
        with i2c_lock:
            self._write8(MODE1, MODE1_AI)
            self._write8(MODE2, MODE2_OUTDRV)
        time.sleep(0.01)

    def close(self):
        pass

    def _write8(self, reg, val):
        self.bus.write_byte_data(self.address, reg, val & 0xFF)

    def _read8(self, reg):
        return self.bus.read_byte_data(self.address, reg)

    def set_pwm_freq(self, freq_hz: float):
        prescale = int(round(OSC_HZ / (4096.0 * float(freq_hz)) - 1.0))
        prescale = max(3, min(255, prescale))
        with i2c_lock:
            oldmode = self._read8(MODE1)
            sleepmode = (oldmode & 0x7F) | MODE1_SLEEP
            self._write8(MODE1, sleepmode)
            time.sleep(0.005)
            self._write8(PRESCALE, prescale)
            time.sleep(0.005)
            self._write8(MODE1, oldmode)
            time.sleep(0.005)
            self._write8(MODE1, oldmode | MODE1_RESTART | MODE1_AI)
            time.sleep(0.005)

    def set_pwm(self, channel: int, on: int, off: int):
        channel = int(channel)
        if not (0 <= channel <= 15):
            raise ValueError("channel must be 0..15")
        on = int(max(0, min(4095, on)))
        off = int(max(0, min(4095, off)))

        reg = LED0_ON_L + 4 * channel
        data = [on & 0xFF, (on >> 8) & 0xFF, off & 0xFF, (off >> 8) & 0xFF]
        with i2c_lock:
            self.bus.write_i2c_block_data(self.address, reg, data)

    def set_duty_12bit(self, channel: int, duty: int):
        duty = int(max(0, min(4095, duty)))
        self.set_pwm(channel, 0, duty)


class PCA9539:
    """PCA9539 16-bit I2C GPIO expander."""
    INPUT0 = 0x00
    INPUT1 = 0x01
    OUTPUT0 = 0x02
    OUTPUT1 = 0x03
    CONFIG0 = 0x06
    CONFIG1 = 0x07

    def __init__(self, bus: SMBus, address: int):
        self.address = address
        self.bus = bus
        # Configure all pins as inputs by default (1 = Input, 0 = Output)
        with i2c_lock:
            self.bus.write_byte_data(self.address, self.CONFIG0, 0xFF)
            self.bus.write_byte_data(self.address, self.CONFIG1, 0xFF)

    def read_inputs(self):
        """Read all 16 pins. Returns a 16-bit integer."""
        with i2c_lock:
            low = self.bus.read_byte_data(self.address, self.INPUT0)
            high = self.bus.read_byte_data(self.address, self.INPUT1)
        return (high << 8) | low

    def close(self):
        pass


class PCA9540B:
    """PCA9540B 1-of-2 I2C multiplexer."""
    CH_NONE = 0x00
    CH0 = 0x04
    CH1 = 0x05

    def __init__(self, bus: SMBus, address: int):
        self.address = address
        self.bus = bus

    def select_channel(self, channel_code: int):
        """Select channel: 0x04 for CH0, 0x05 for CH1, 0x00 for none."""
        with i2c_lock:
            self.bus.write_byte(self.address, channel_code)

    def deselect_channels(self):
        """Deselect all channels."""
        self.select_channel(self.CH_NONE)

    def close(self):
        pass


class BME280:
    def __init__(self, bus: SMBus, address: int = 0x76):
        self.address = address
        self.bus = bus
        self.cal = {}
        # Check Chip ID (BME280 = 0x60, BMP280 = 0x58)
        with i2c_lock:
            chip_id = self.bus.read_byte_data(self.address, 0xD0)
            if chip_id not in [0x60, 0x58]:
                raise RuntimeError(f"Unexpected Chip ID: {hex(chip_id)}. Expected 0x60 or 0x58.")
            self.is_bme = (chip_id == 0x60)
            self._load_calibration()
            # Initialize sensor
            if self.is_bme:
                self.bus.write_byte_data(self.address, 0xF2, 0x01)  # Humidity oversampling x1
            self.bus.write_byte_data(self.address, 0xF4, 0x27)  # Temp/Press oversampling x1, normal mode
            self.bus.write_byte_data(self.address, 0xF5, 0xA0)  # Standby 1000ms, filter off

    def _load_calibration(self):
        def read_u16(reg):
            data = self.bus.read_i2c_block_data(self.address, reg, 2)
            return data[0] | (data[1] << 8)

        def read_s16(reg):
            val = read_u16(reg)
            return val if val < 32768 else val - 65536

        self.cal['T1'] = read_u16(0x88)
        self.cal['T2'] = read_s16(0x8A)
        self.cal['T3'] = read_s16(0x8C)
        self.cal['P1'] = read_u16(0x8E)
        self.cal['P2'] = read_s16(0x90)
        self.cal['P3'] = read_s16(0x92)
        self.cal['P4'] = read_s16(0x94)
        self.cal['P5'] = read_s16(0x96)
        self.cal['P6'] = read_s16(0x98)
        self.cal['P7'] = read_s16(0x9A)
        self.cal['P8'] = read_s16(0x9C)
        self.cal['P9'] = read_s16(0x9E)
        if self.is_bme:
            self.cal['H1'] = self.bus.read_byte_data(self.address, 0xA1)
            self.cal['H2'] = read_s16(0xE1)
            self.cal['H3'] = self.bus.read_byte_data(self.address, 0xE3)
            h4_msb = self.bus.read_byte_data(self.address, 0xE4)
            h4_lsb = self.bus.read_byte_data(self.address, 0xE5)
            h4 = (h4_msb << 4) | (h4_lsb & 0x0F)
            self.cal['H4'] = h4 if h4 < 2048 else h4 - 4096
            h5_msb = self.bus.read_byte_data(self.address, 0xE6)
            h5_lsb = self.bus.read_byte_data(self.address, 0xE5)
            h5 = (h5_msb << 4) | (h5_lsb >> 4)
            self.cal['H5'] = h5 if h5 < 2048 else h5 - 4096
            self.cal['H6'] = self.bus.read_byte_data(self.address, 0xE7)
            if self.cal['H6'] > 127: self.cal['H6'] -= 256

    def read_data(self):
        with i2c_lock:
            data = self.bus.read_i2c_block_data(self.address, 0xF7, 8 if self.is_bme else 6)
        pres_raw = (data[0] << 12) | (data[1] << 4) | (data[2] >> 4)
        temp_raw = (data[3] << 12) | (data[4] << 4) | (data[5] >> 4)
        hum_raw = ((data[6] << 8) | data[7]) if self.is_bme else 0

        if temp_raw == 0x80000 or pres_raw == 0x80000:
            return None, None, None

        # Temp compensation
        v1 = (temp_raw / 16384.0 - self.cal['T1'] / 1024.0) * self.cal['T2']
        v2 = ((temp_raw / 131072.0 - self.cal['T1'] / 8192.0) ** 2) * self.cal['T3']
        t_fine = v1 + v2
        temp = t_fine / 5120.0

        # Pressure compensation
        v1 = (t_fine / 2.0) - 64000.0
        v2 = v1 * v1 * self.cal['P6'] / 32768.0
        v2 = v2 + v1 * self.cal['P5'] * 2.0
        v2 = (v2 / 4.0) + (self.cal['P4'] * 65536.0)
        v1 = (self.cal['P3'] * v1 * v1 / 524288.0 + self.cal['P2'] * v1) / 524288.0
        v1 = (1.0 + v1 / 32768.0) * self.cal['P1']
        if v1 < 0.0001:  # Avoid division by zero or extreme values
            pressure = 0
        else:
            p = 1048576.0 - pres_raw
            p = ((p - v2 / 4096.0) * 6250.0) / v1
            v1_p = self.cal['P9'] * p * p / 2147483648.0
            v2_p = p * self.cal['P8'] / 32768.0
            pressure = p + (v1_p + v2_p + self.cal['P7']) / 16.0
            pressure /= 100.0  # hPa

        # Humidity compensation
        if self.is_bme:
            h = t_fine - 76800.0
            h = (hum_raw - (self.cal['H4'] * 64.0 + self.cal['H5'] / 16384.0 * h)) * \
                (self.cal['H2'] / 65536.0 * (1.0 + self.cal['H6'] / 67108864.0 * h * \
                (1.0 + self.cal['H3'] / 67108864.0 * h)))
            h = h * (1.0 - self.cal['H1'] * h / 524288.0)
            humidity = max(0, min(100, h))
        else:
            humidity = 0

        return temp, pressure, humidity

    def close(self):
        pass


CH_PWM1 = 0
CH_HEATER_1 = 1
CH_HEATER_2 = 2
CH_HEATER_3 = 3
CH_HEATER_4 = 4
CH_FAN_1_POWER = 5
CH_FAN_2_POWER = 6
CH_STEPPER_DIR = 7
CH_STEPPER_ENA = 8
CH_PU = 9
CH_PWM2 = 10
CH_RESERVE1 = 11
CH_LED_RED = 12
CH_LED_BLUE = 13
CH_LED_GREEN = 14
CH_SYS_LED = 15


def load_config():
    if HAS_REQUESTS:
        token = os.environ.get("SUPERVISOR_TOKEN")
        if token:
            try:
                resp = requests.get(
                    "http://supervisor/services/mqtt",
                    headers={"Authorization": f"Bearer {token}"},
                    timeout=5,
                )
                if resp.status_code == 200:
                    mqtt_cfg = resp.json()["data"]
                    with open("/data/options.json") as f:
                        opts = json.load(f)
                    opts.update(
                        {
                            "mqtt_host": mqtt_cfg["host"],
                            "mqtt_port": mqtt_cfg["port"],
                            "mqtt_username": mqtt_cfg["username"],
                            "mqtt_password": mqtt_cfg["password"],
                        }
                    )
                    return opts
            except Exception:
                pass

    with open("/data/options.json") as f:
        return json.load(f)


config = load_config()

MQTT_HOST = config["mqtt_host"]
MQTT_PORT = int(config["mqtt_port"])
MQTT_USER = config.get("mqtt_username") or None
MQTT_PASS = config.get("mqtt_password") or None

I2C_BUS = int(config.get("i2c_bus", 1))
PCA_ADDR = int(config["pca_address"], 16)
PCA9539_ADDR = int(config.get("pca9539_address", "0x74"), 16)
PCA9540_ADDR = int(config.get("pca9540_address", "0x70"), 16)
BME_INTERVAL = int(config.get("bme_interval", 30))
PCA_FREQ = int(config.get("pca_frequency", 1000))

DEFAULT_DUTY_CYCLE = int(config.get("default_duty_cycle", 30))
DEFAULT_DUTY_CYCLE = max(0, min(100, DEFAULT_DUTY_CYCLE))

PU_DEFAULT_HZ = float(config.get("pu_default_hz", 10.0))
PU_DEFAULT_HZ = max(0.0, PU_DEFAULT_HZ)

MQTT_DEEP_CLEAN = config.get("mqtt_deep_clean", False)

is_cleaning = False
found_topics = set()
clean_lock = threading.Lock()
CLEAN_PREFIXES = ["pca_", "bme280_", "status_", "pca9539_"]

# Status and Colors
COLOR_OFF = (0, 0, 0)
COLOR_RED = (4095, 0, 0)
COLOR_GREEN = (0, 4095, 0)
COLOR_BLUE = (0, 0, 4095)

system_status = "DIAGNOSTIC"  # OK, ERROR, DIAGNOSTIC
status_lock = threading.Lock()

def set_rgb_color(color_tuple):
    pca.set_duty_12bit(CH_LED_RED, color_tuple[0])
    pca.set_duty_12bit(CH_LED_GREEN, color_tuple[1])
    pca.set_duty_12bit(CH_LED_BLUE, color_tuple[2])

def get_pca9539_pin(pin_idx):
    """Read a specific pin from PCA9539. pin_idx 0-15."""
    if not pca9539:
        return None
    inputs = pca9539.read_inputs()
    return (inputs >> pin_idx) & 1

def hardware_diagnostic():
    global system_status
    logger.info("Starting automated hardware diagnostic...")
    
    # Set status to diagnostic (will be used by sys_led_worker if it's already running)
    with status_lock:
        system_status = "DIAGNOSTIC"
    
    # Ensure Blue LED is ON during diagnostic
    set_rgb_color(COLOR_BLUE)
    
    problems = []
    
    # 1. Test Relays (Heaters 1-4, Fans 1-2 Power)
    # Mapping: Heaters 1-4 -> IO0_0 to IO0_3, Fans 1-2 -> IO0_4 to IO0_5
    # Logic: Low (0) = ON, High (1) = OFF
    test_map = [
        (CH_HEATER_1, 0, "Heater 1"),
        (CH_HEATER_2, 1, "Heater 2"),
        (CH_HEATER_3, 3, "Heater 3"), # Corrected from 2 to 3 if following sequence, wait let's check FEEDBACK_MAP
        (CH_HEATER_4, 3, "Heater 4"),
    ]
    # Wait, let me check the FEEDBACK_MAP I just wrote
    # CH_HEATER_1: 0, CH_HEATER_2: 1, CH_HEATER_3: 2, CH_HEATER_4: 3
    
    relays_to_test = [
        (CH_HEATER_1, "Heater 1"),
        (CH_HEATER_2, "Heater 2"),
        (CH_HEATER_3, "Heater 3"),
        (CH_HEATER_4, "Heater 4"),
        (CH_FAN_1_POWER, "Fan 1 Power"),
        (CH_FAN_2_POWER, "Fan 2 Power"),
    ]
    
    for ch, name in relays_to_test:
        logger.info("Testing %s...", name)
        # Test ON
        if not verified_apply_switch(ch, True, name):
            problems.append(f"{name} ON failure")
        time.sleep(0.2)
        # Test OFF
        if not verified_apply_switch(ch, False, name):
            problems.append(f"{name} OFF failure")
        time.sleep(0.1)

    # 2. Test Stepper ENA, DIR
    stepper_signals = [
        (CH_STEPPER_ENA, "Stepper ENA"),
        (CH_STEPPER_DIR, "Stepper DIR"),
    ]
    
    for ch, name in stepper_signals:
        logger.info("Testing %s...", name)
        # Toggle and check
        verified_apply_switch(ch, True, name)
        time.sleep(0.1)
        verified_apply_switch(ch, False, name)
        time.sleep(0.1)

    # 3. Test PU (Pulse) detection
    logger.info("Testing PU signal feedback...")
    pu_detected = False
    # Send a few pulses and see if we can catch them
    for _ in range(10):
        channel_on(CH_PU)
        time.sleep(0.01)
        if get_pca9539_pin(10) == 1:
            pu_detected = True
        channel_off(CH_PU)
        time.sleep(0.01)
        if pu_detected: break
    
    if not pu_detected:
        logger.warning("PU feedback not detected during diagnostic (check R12/wiring)")
        problems.append("PU feedback failure - check R12/wiring")
    
    if problems:
        logger.error("Hardware diagnostic completed with ERRORS: %s", ", ".join(problems))
        with status_lock:
            system_status = "ERROR"
        set_rgb_color(COLOR_RED) # Briefly show red
        time.sleep(1)
    else:
        logger.info("Hardware diagnostic PASSED.")
        with status_lock:
            system_status = "OK"
        set_rgb_color(COLOR_GREEN) # Briefly show green
        time.sleep(1)

    set_rgb_color(COLOR_OFF)
    return len(problems) == 0


def _topic(component, unique_id, suffix):
    """Generate Home Assistant MQTT topic string."""
    return f"homeassistant/{component}/{unique_id}/{suffix}"


AVAIL_TOPIC = "homeassistant/availability"


TOPIC_PWM1_DUTY_CMD = _topic("number", "pca_pwm1_duty", "set")
TOPIC_PWM1_DUTY_STATE = _topic("number", "pca_pwm1_duty", "state")

TOPIC_HEATER_1_CMD = _topic("switch", "pca_heater_1", "set")
TOPIC_HEATER_1_STATE = _topic("switch", "pca_heater_1", "state")
TOPIC_HEATER_2_CMD = _topic("switch", "pca_heater_2", "set")
TOPIC_HEATER_2_STATE = _topic("switch", "pca_heater_2", "state")
TOPIC_HEATER_3_CMD = _topic("switch", "pca_heater_3", "set")
TOPIC_HEATER_3_STATE = _topic("switch", "pca_heater_3", "state")
TOPIC_HEATER_4_CMD = _topic("switch", "pca_heater_4", "set")
TOPIC_HEATER_4_STATE = _topic("switch", "pca_heater_4", "state")

TOPIC_FAN_1_POWER_CMD = _topic("switch", "pca_fan_1_power", "set")
TOPIC_FAN_1_POWER_STATE = _topic("switch", "pca_fan_1_power", "state")
TOPIC_FAN_2_POWER_CMD = _topic("switch", "pca_fan_2_power", "set")
TOPIC_FAN_2_POWER_STATE = _topic("switch", "pca_fan_2_power", "state")

TOPIC_PWM2_DUTY_CMD = _topic("number", "pca_pwm2_duty", "set")
TOPIC_PWM2_DUTY_STATE = _topic("number", "pca_pwm2_duty", "state")

TOPIC_STEPPER_DIR_CMD = _topic("select", "pca_stepper_dir", "set")
TOPIC_STEPPER_DIR_STATE = _topic("select", "pca_stepper_dir", "state")

TOPIC_STEPPER_ENA_CMD = _topic("switch", "pca_stepper_ena", "set")
TOPIC_STEPPER_ENA_STATE = _topic("switch", "pca_stepper_ena", "state")

TOPIC_PU_ENABLE_CMD = _topic("switch", "pca_pu_enable", "set")
TOPIC_PU_ENABLE_STATE = _topic("switch", "pca_pu_enable", "state")

TOPIC_PU_FREQ_CMD = _topic("number", "pca_pu_freq_hz", "set")
TOPIC_PU_FREQ_STATE = _topic("number", "pca_pu_freq_hz", "state")

TOPIC_BME_CH0_76_TEMP = _topic("sensor", "bme280_ch0_0x76_temperature", "state")
TOPIC_BME_CH0_76_HUM = _topic("sensor", "bme280_ch0_0x76_humidity", "state")
TOPIC_BME_CH0_76_PRESS = _topic("sensor", "bme280_ch0_0x76_pressure", "state")

TOPIC_BME_CH0_77_TEMP = _topic("sensor", "bme280_ch0_0x77_temperature", "state")
TOPIC_BME_CH0_77_HUM = _topic("sensor", "bme280_ch0_0x77_humidity", "state")
TOPIC_BME_CH0_77_PRESS = _topic("sensor", "bme280_ch0_0x77_pressure", "state")

TOPIC_BME_CH1_77_TEMP = _topic("sensor", "bme280_ch1_0x77_temperature", "state")
TOPIC_BME_CH1_77_HUM = _topic("sensor", "bme280_ch1_0x77_humidity", "state")
TOPIC_BME_CH1_77_PRESS = _topic("sensor", "bme280_ch1_0x77_pressure", "state")

TOPIC_PCA9539_INPUTS = "homeassistant/sensor/pca9539_inputs/state"

# Feedback status topics (Binary sensors)
TOPIC_FEEDBACK_ENA = _topic("binary_sensor", "status_ena", "state")
TOPIC_FEEDBACK_DIR = _topic("binary_sensor", "status_dir", "state")
TOPIC_FEEDBACK_PU = _topic("binary_sensor", "status_pu", "state")
TOPIC_FEEDBACK_TAXO1 = _topic("binary_sensor", "status_taxo1", "state")
TOPIC_FEEDBACK_TAXO2 = _topic("binary_sensor", "status_taxo2", "state")
TOPIC_FEEDBACK_RELAY1 = _topic("binary_sensor", "status_relay1", "state")
TOPIC_FEEDBACK_RELAY2 = _topic("binary_sensor", "status_relay2", "state")
TOPIC_FEEDBACK_RELAY3 = _topic("binary_sensor", "status_relay3", "state")
TOPIC_FEEDBACK_RELAY4 = _topic("binary_sensor", "status_relay4", "state")
TOPIC_FEEDBACK_RELAY5 = _topic("binary_sensor", "status_relay5", "state")
TOPIC_FEEDBACK_RELAY6 = _topic("binary_sensor", "status_relay6", "state")

TOPIC_RES2 = _topic("binary_sensor", "status_res2", "state")
TOPIC_RES3 = _topic("binary_sensor", "status_res3", "state")
TOPIC_RES4 = _topic("binary_sensor", "status_res4", "state")


def validate_fixed_mapping():
    channels = [
        CH_PWM1,
        CH_HEATER_1,
        CH_HEATER_2,
        CH_HEATER_3,
        CH_HEATER_4,
        CH_FAN_1_POWER,
        CH_FAN_2_POWER,
        CH_STEPPER_DIR,
        CH_STEPPER_ENA,
        CH_PU,
        CH_PWM2,
        CH_RESERVE1,
        CH_LED_RED,
        CH_LED_BLUE,
        CH_LED_GREEN,
        CH_SYS_LED,
    ]
    for ch in channels:
        if not (0 <= ch <= 15):
            raise ValueError(f"Invalid channel {ch}; expected 0..15")
    if len(set(channels)) != len(channels):
        raise ValueError("Fixed channel mapping contains duplicates")


validate_fixed_mapping()
logger.info("Fixed channels: PWM1=0, Heaters=1-4, Fans=5-6, DIR=7, ENA=8, PU=9, SYS_LED=15")


def channel_on(channel: int):
    pca.set_duty_12bit(channel, 4095)


def channel_off(channel: int):
    pca.set_duty_12bit(channel, 0)


logger.info("Opening I2C bus %s, PCA9685 addr=%s", I2C_BUS, hex(PCA_ADDR))
pca = None
for attempt in range(1, 11):
    try:
        pca = PCA9685(shared_bus, PCA_ADDR)
        pca.set_pwm_freq(PCA_FREQ)
        logger.info("PCA9685 global PWM frequency set to %s Hz", PCA_FREQ)
        break
    except Exception as e:
        logger.warning("Attempt %d/10: Cannot initialize PCA9685 (%s). Retrying in 2s...", attempt, e)
        time.sleep(2)

if pca is None:
    logger.error("Fatal: Failed to initialize PCA9685 after 10 attempts.")
    sys.exit(1)


logger.info("Opening I2C bus %s, PCA9539 addr=%s", I2C_BUS, hex(PCA9539_ADDR))
pca9539 = None
try:
    pca9539 = PCA9539(shared_bus, PCA9539_ADDR)
    logger.info("PCA9539 GPIO expander initialized")
except Exception as e:
    logger.warning("PCA9539 initialization failed: %s. GPIO feedback will be disabled.", e)


logger.info("Opening I2C bus %s, PCA9540B addr=%s", I2C_BUS, hex(PCA9540_ADDR))
pca9540 = None
try:
    pca9540 = PCA9540B(shared_bus, PCA9540_ADDR)
    logger.info("PCA9540B multiplexer initialized")
except Exception as e:
    logger.warning("PCA9540B initialization failed: %s. I2C multiplexing disabled.", e)


def init_bme(channel_code: int, address: int, label: str):
    if not pca9540:
        return None
    try:
        # First select the channel on the mux
        pca9540.select_channel(channel_code)
        time.sleep(0.01)
        # Then initialize the sensor
        sensor = BME280(shared_bus, address)
        logger.info("BME280 sensor initialized on %s at %s", label, hex(address))
        # Deselect after init
        pca9540.deselect_channels()
        return sensor
    except Exception as e:
        logger.warning("BME280 initialization failed on %s at %s: %s", label, hex(address), e)
        # Ensure we deselect even on failure
        try: pca9540.deselect_channels()
        except: pass
        return None


bme_ch0_76 = init_bme(PCA9540B.CH0, 0x76, "CH0")
bme_ch0_77 = init_bme(PCA9540B.CH0, 0x77, "CH0")
bme_ch1_77 = init_bme(PCA9540B.CH1, 0x77, "CH1")


pwm1_value = 0.0
pwm1_lock = threading.Lock()

pwm2_value = 0.0
pwm2_lock = threading.Lock()

heater_1 = False
heater_2 = False
heater_3 = False
heater_4 = False
fan_1_power = False
fan_2_power = False

stepper_dir = "CW"
stepper_ena = False

pu_enabled = False
pu_freq_hz = float(PU_DEFAULT_HZ)
pu_lock = threading.Lock()
pu_thread = None
pu_running = False
pu_is_pulsing = False  # Track if the PU worker is actively generating pulses

sys_led_thread = None
sys_led_running = False
sys_led_lock = threading.Lock()


bme_thread = None
bme_running = False
bme_lock = threading.Lock()


pca9539_thread = None
pca9539_running = False
pca9539_lock = threading.Lock()


def pca9539_worker():
    global pca9539_running
    global heater_1, heater_2, heater_3, heater_4
    global fan_1_power, fan_2_power
    global stepper_ena, stepper_dir
    global pu_enabled
    global pwm1_value, pwm2_value

    last_inputs = None
    
    # Pulse detection state
    taxo1_history = []
    taxo2_history = []
    pu_history = []
    
    # LED control state
    led_blink_state = False

    while pca9539_running:
        if pca9539:
            try:
                inputs = pca9539.read_inputs()
                if inputs != last_inputs:
                    client.publish(TOPIC_PCA9539_INPUTS, json.dumps({"raw": inputs, "hex": hex(inputs)}), retain=True)
                    last_inputs = inputs
                
                # Feedback Logic
                # ON (Problem) if actual doesn't match expected, OFF (No Problem) otherwise
                any_problem = False

                # Relays (IO0_0 to IO0_5) - corrected physical mapping
                relays_states = [heater_1, heater_2, heater_4, heater_3, fan_1_power, fan_2_power]
                relay_topics = [TOPIC_FEEDBACK_RELAY1, TOPIC_FEEDBACK_RELAY2, TOPIC_FEEDBACK_RELAY4, 
                                TOPIC_FEEDBACK_RELAY3, TOPIC_FEEDBACK_RELAY5, TOPIC_FEEDBACK_RELAY6]
                
                for i in range(6):
                    expected_on = relays_states[i]
                    actual_bit = (inputs >> i) & 1
                    is_ok = (expected_on == (actual_bit == 0))
                    if not is_ok: any_problem = True
                    client.publish(relay_topics[i], "ON" if not is_ok else "OFF", retain=True)

                # Stepper
                ena_actual = (inputs >> 8) & 1
                ena_ok = (stepper_ena == (ena_actual == 1))
                if not ena_ok: any_problem = True
                client.publish(TOPIC_FEEDBACK_ENA, "ON" if not ena_ok else "OFF", retain=True)

                # DIR feedback only meaningful when stepper is enabled
                if stepper_ena:
                    dir_actual = (inputs >> 9) & 1
                    dir_expected_high = (stepper_dir == "CW")
                    dir_ok = (dir_expected_high == (dir_actual == 1))
                    if not dir_ok: any_problem = True
                    client.publish(TOPIC_FEEDBACK_DIR, "ON" if not dir_ok else "OFF", retain=True)
                else:
                    # When stepper is disabled, DIR feedback is not meaningful
                    client.publish(TOPIC_FEEDBACK_DIR, "OFF", retain=True)

                # PU feedback only meaningful when stepper is enabled
                if stepper_ena:
                    pu_actual = (inputs >> 10) & 1
                    if not pu_enabled:
                        pu_ok = (pu_actual == 0)
                    else:
                        pu_history.append(pu_actual)
                        if len(pu_history) > 5: pu_history.pop(0)
                        if len(pu_history) >= 3 and all(x == pu_history[0] for x in pu_history):
                            pu_ok = False
                        else:
                            pu_ok = True
                    if not pu_ok: any_problem = True
                    client.publish(TOPIC_FEEDBACK_PU, "ON" if not pu_ok else "OFF", retain=True)
                else:
                    # When stepper is disabled, PU feedback is not meaningful
                    client.publish(TOPIC_FEEDBACK_PU, "OFF", retain=True)

                # TAXO 1
                taxo1_actual = (inputs >> 6) & 1
                if pwm1_value > 0.0:
                    taxo1_history.append(taxo1_actual)
                    if len(taxo1_history) > 5: taxo1_history.pop(0)
                    # If all samples are the same, it's not pulsing
                    if len(taxo1_history) >= 3 and all(x == taxo1_history[0] for x in taxo1_history):
                        taxo1_ok = False
                    else:
                        taxo1_ok = True
                else:
                    taxo1_history = []
                    taxo1_ok = True  # Motor off, no pulses expected
                if not taxo1_ok: any_problem = True
                client.publish(TOPIC_FEEDBACK_TAXO1, "ON" if not taxo1_ok else "OFF", retain=True)

                # TAXO 2
                taxo2_actual = (inputs >> 7) & 1
                if pwm2_value > 0.0:
                    taxo2_history.append(taxo2_actual)
                    if len(taxo2_history) > 5: taxo2_history.pop(0)
                    # If all samples are the same, it's not pulsing
                    if len(taxo2_history) >= 3 and all(x == taxo2_history[0] for x in taxo2_history):
                        taxo2_ok = False
                    else:
                        taxo2_ok = True
                else:
                    taxo2_history = []
                    taxo2_ok = True  # Motor off, no pulses expected
                if not taxo2_ok: any_problem = True
                client.publish(TOPIC_FEEDBACK_TAXO2, "ON" if not taxo2_ok else "OFF", retain=True)

                # Reserve inputs
                res4_actual = (inputs >> 12) & 1
                res3_actual = (inputs >> 13) & 1
                res2_actual = (inputs >> 14) & 1
                client.publish(TOPIC_RES4, "ON" if res4_actual == 1 else "OFF", retain=True)
                client.publish(TOPIC_RES3, "ON" if res3_actual == 1 else "OFF", retain=True)
                client.publish(TOPIC_RES2, "ON" if res2_actual == 1 else "OFF", retain=True)

                # Automatic LED control
                if any_problem:
                    # Blink RED, turn off GREEN
                    led_blink_state = not led_blink_state
                    if led_blink_state:
                        channel_on(CH_LED_RED)
                    else:
                        channel_off(CH_LED_RED)
                    channel_off(CH_LED_GREEN)
                else:
                    # Turn on GREEN, turn off RED
                    channel_on(CH_LED_GREEN)
                    channel_off(CH_LED_RED)
                
                # BLUE LED is dropped
                channel_off(CH_LED_BLUE)

            except Exception as e:
                logger.error("PCA9539 read error: %s", e)
        time.sleep(1.0)


def pca9539_start():
    global pca9539_thread, pca9539_running
    if not pca9539:
        return
    with pca9539_lock:
        if pca9539_thread and pca9539_thread.is_alive():
            return
        pca9539_running = True
        pca9539_thread = threading.Thread(target=pca9539_worker, daemon=True)
        pca9539_thread.start()
    logger.info("PCA9539 expander thread started")


def pca9539_stop():
    global pca9539_thread, pca9539_running
    with pca9539_lock:
        pca9539_running = False
    if pca9539_thread and pca9539_thread.is_alive():
        pca9539_thread.join(timeout=2)
    pca9539_thread = None


def bme_worker():
    global bme_running
    while bme_running:
        # Read CH0
        if pca9540:
            try:
                pca9540.select_channel(PCA9540B.CH0)
                time.sleep(0.01)
                
                # Sensor at 0x76
                if bme_ch0_76:
                    temp, press, hum = bme_ch0_76.read_data()
                    if temp is not None:
                        client.publish(TOPIC_BME_CH0_76_TEMP, f"{temp:.2f}", retain=True)
                        client.publish(TOPIC_BME_CH0_76_PRESS, f"{press:.2f}", retain=True)
                        if bme_ch0_76.is_bme:
                            client.publish(TOPIC_BME_CH0_76_HUM, f"{hum:.2f}", retain=True)
                
                # Sensor at 0x77
                if bme_ch0_77:
                    temp, press, hum = bme_ch0_77.read_data()
                    if temp is not None:
                        client.publish(TOPIC_BME_CH0_77_TEMP, f"{temp:.2f}", retain=True)
                        client.publish(TOPIC_BME_CH0_77_PRESS, f"{press:.2f}", retain=True)
                        if bme_ch0_77.is_bme:
                            client.publish(TOPIC_BME_CH0_77_HUM, f"{hum:.2f}", retain=True)
                
                pca9540.deselect_channels()
            except Exception as e:
                logger.error("BME280 CH0 read error: %s", e)
                try: pca9540.deselect_channels()
                except: pass

        # Read CH1
        if bme_ch1_77 and pca9540:
            try:
                pca9540.select_channel(PCA9540B.CH1)
                time.sleep(0.01)
                temp, press, hum = bme_ch1_77.read_data()
                if temp is not None:
                    client.publish(TOPIC_BME_CH1_77_TEMP, f"{temp:.2f}", retain=True)
                    client.publish(TOPIC_BME_CH1_77_PRESS, f"{press:.2f}", retain=True)
                    if bme_ch1_77.is_bme:
                        client.publish(TOPIC_BME_CH1_77_HUM, f"{hum:.2f}", retain=True)
                
                pca9540.deselect_channels()
            except Exception as e:
                logger.error("BME280 CH1 read error: %s", e)
                try: pca9540.deselect_channels()
                except: pass

        time.sleep(BME_INTERVAL)


def bme_start():
    global bme_thread, bme_running
    if not (bme_ch0_76 or bme_ch0_77 or bme_ch1_77):
        return
    with bme_lock:
        if bme_thread and bme_thread.is_alive():
            return
        bme_running = True
        bme_thread = threading.Thread(target=bme_worker, daemon=True)
        bme_thread.start()
    logger.info("BME280 sensor thread started")


def bme_stop():
    global bme_thread, bme_running
    with bme_lock:
        bme_running = False
    if bme_thread and bme_thread.is_alive():
        bme_thread.join(timeout=2)
    bme_thread = None


def update_pwm1_output_locked():
    global pwm1_value

    visual = float(pwm1_value)
    if visual == 0.0:
        pwm_percent = 100.0
    elif 0.0 < visual <= 10.0:
        pwm_percent = 90.0
    else:
        pwm_percent = 100.0 - visual

    duty = int((pwm_percent / 100.0) * 4095)
    duty = max(0, min(4095, duty))
    pca.set_duty_12bit(CH_PWM1, duty)


def update_pwm2_output_locked():
    global pwm2_value

    visual = float(pwm2_value)
    if visual == 0.0:
        pwm_percent = 100.0
    elif 0.0 < visual <= 10.0:
        pwm_percent = 90.0
    else:
        pwm_percent = 100.0 - visual

    duty = int((pwm_percent / 100.0) * 4095)
    duty = max(0, min(4095, duty))
    pca.set_duty_12bit(CH_PWM2, duty)


# Map PCA9685 channels to PCA9539 feedback pins
# Relays: 0-5 (IO0_0 to IO0_5)
# Stepper: 8-10 (IO1_0 to IO1_2)
# TAXO: 11-12 (IO1_3 to IO1_4)
FEEDBACK_MAP = {
    CH_HEATER_1: 0,
    CH_HEATER_2: 1,
    CH_HEATER_3: 2,
    CH_HEATER_4: 3,
    CH_FAN_1_POWER: 4,
    CH_FAN_2_POWER: 5,
    CH_STEPPER_ENA: 8,
    CH_STEPPER_DIR: 9,
    CH_PU: 10
}

# TAXO pins for monitoring (not direct feedback of PCA channels)
TAXO1_PIN = 11
TAXO2_PIN = 12

def verified_apply_switch(channel: int, state: bool, name: str = "Component"):
    global system_status
    feedback_pin = FEEDBACK_MAP.get(channel)
    
    expected_after = None
    if feedback_pin is not None:
        # 1. Pre-check status
        initial_val = get_pca9539_pin(feedback_pin)
        
        # Determine expected values based on schematic
        # For relays (0-5), Low (0) = ON, High (1) = OFF
        if feedback_pin <= 5:
            expected_after = 0 if state else 1
            initial_desc = "ON" if initial_val == 0 else "OFF"
        else:
            # For ENA/DIR/PU, they follow PCA9685 logic (High=ON, Low=OFF)
            expected_after = 1 if state else 0
            initial_desc = "ON" if initial_val == 1 else "OFF"
            
        logger.debug("Pre-check %s (CH %d): Current status is %s", name, channel, initial_desc)

    # 2. Apply command
    if state:
        channel_on(channel)
    else:
        channel_off(channel)
    
    if feedback_pin is not None:
        # 3. Post-check verification
        time.sleep(0.12) # Wait for hardware response
        final_val = get_pca9539_pin(feedback_pin)
        
        if final_val != expected_after:
            logger.error("VERIFICATION FAILED for %s (CH %d): expected %d, got %d", name, channel, expected_after, final_val)
            with status_lock:
                system_status = "ERROR"
            return False
        else:
            logger.debug("Verification passed for %s", name)
            # If we were in error but this worked, should we clear it? 
            # Usually better to let the diagnostic or a manual reset clear errors.
            
    return True

def apply_switch(channel: int, state: bool):
    # This now uses the verified version
    return verified_apply_switch(channel, state)


def stepper_apply_dir(value: str):
    global stepper_dir, pu_enabled, pu_is_pulsing
    new_dir = "CCW" if value == "CCW" else "CW"
    
    # If the direction is already the same, do nothing
    if new_dir == stepper_dir:
        return

    logger.info("Stepper DIR change requested: %s -> %s. Performing safe switch...", stepper_dir, new_dir)
    
    # 1. Disable pulse generation and wait for current pulse to finish
    was_enabled = False
    with pu_lock:
        was_enabled = pu_enabled
        pu_enabled = False
    
    # Wait for pu_worker to stop pulsing (timeout after 2 seconds)
    start_wait = time.time()
    while pu_is_pulsing and (time.time() - start_wait < 2.0):
        time.sleep(0.01)
    
    # Extra safety wait to ensure driver is ready
    time.sleep(0.05)
    
    # 2. Change DIR signal
    stepper_dir = new_dir
    verified_apply_switch(CH_STEPPER_DIR, stepper_dir == "CW", "Stepper DIR")
    
    # 3. Wait ≥ 50 ms (as required for DM332T setup time)
    time.sleep(0.06)
    
    # 4. Resume pulse generation if it was enabled
    if was_enabled:
        with pu_lock:
            pu_enabled = True
        logger.info("Stepper DIR changed and pulses resumed.")
    else:
        logger.info("Stepper DIR changed.")


def stepper_apply_ena(state: bool):
    global stepper_ena
    stepper_ena = bool(state)
    verified_apply_switch(CH_STEPPER_ENA, stepper_ena, "Stepper ENA")


def pu_worker():
    global pu_running, pu_is_pulsing, system_status
    last_level = None
    last_enabled = None
    pulse_feedback_count = 0

    while pu_running:
        with pu_lock:
            enabled = bool(pu_enabled)
            freq = float(pu_freq_hz)

        if (not enabled) or freq <= 0.0:
            pu_is_pulsing = False
            if last_level is not False:
                channel_off(CH_PU)
                last_level = False
            if last_enabled is not False:
                last_enabled = False
            time.sleep(0.1)
            continue

        if last_enabled is not True:
            last_enabled = True
        
        pu_is_pulsing = True
        half = 0.5 / freq
        
        # Pulse ON
        if last_level is not True:
            channel_on(CH_PU)
            last_level = True
        
        # Small sleep and check feedback
        time.sleep(min(0.005, half * 0.5))
        if get_pca9539_pin(10) == 1:
            pulse_feedback_count += 1
        
        time.sleep(max(0, half - min(0.005, half * 0.5)))
        
        if not pu_running:
            break
            
        # Pulse OFF
        if last_level is not False:
            channel_off(CH_PU)
            last_level = False
        time.sleep(half)

        # Every 100 cycles, verify we are actually getting pulses if enabled
        # This is a bit arbitrary, but prevents log spamming
        # For very low frequencies, we might want to check more often
        if freq > 1.0 and pulse_feedback_count == 0:
            # We should have seen pulses
            logger.warning("PU feedback not detected while pulsing!")
            with status_lock:
                system_status = "ERROR"
        
        pulse_feedback_count = 0

    pu_is_pulsing = False
    channel_off(CH_PU)


def pu_start():
    global pu_thread, pu_running
    with pu_lock:
        if pu_thread and pu_thread.is_alive():
            return
        pu_running = True
        pu_thread = threading.Thread(target=pu_worker, daemon=True)
        pu_thread.start()
    logger.info("CH9 PU worker started")


def pu_stop():
    global pu_thread, pu_running
    with pu_lock:
        pu_running = False
    if pu_thread and pu_thread.is_alive():
        pu_thread.join(timeout=2)
    pu_thread = None
    channel_off(CH_PU)


def sys_led_worker():
    global sys_led_running, system_status
    level = False
    
    while sys_led_running:
        level = not level
        
        # System LED (CH15) always blinks
        if level:
            channel_on(CH_SYS_LED)
        else:
            channel_off(CH_SYS_LED)
            
        # RGB LED status
        with status_lock:
            current_status = system_status
            
        if current_status == "OK":
            # Normal: Blink Green
            if level:
                set_rgb_color(COLOR_GREEN)
            else:
                set_rgb_color(COLOR_OFF)
        elif current_status == "ERROR":
            # Problem: Blink Red
            if level:
                set_rgb_color(COLOR_RED)
            else:
                set_rgb_color(COLOR_OFF)
        elif current_status == "DIAGNOSTIC":
            # Diagnostic: Blue (already set by hardware_diagnostic, but let's be sure)
            set_rgb_color(COLOR_BLUE)
            
        time.sleep(1.0)

    channel_off(CH_SYS_LED)
    set_rgb_color(COLOR_OFF)


def sys_led_start():
    global sys_led_thread, sys_led_running
    with sys_led_lock:
        if sys_led_thread and sys_led_thread.is_alive():
            return
        sys_led_running = True
        sys_led_thread = threading.Thread(target=sys_led_worker, daemon=True)
        sys_led_thread.start()
    logger.info("CH15 system LED blinking started")


def sys_led_stop():
    global sys_led_thread, sys_led_running
    with sys_led_lock:
        sys_led_running = False
    if sys_led_thread and sys_led_thread.is_alive():
        sys_led_thread.join(timeout=2)
    sys_led_thread = None
    channel_off(CH_SYS_LED)


try:
    pca.set_duty_12bit(CH_PWM1, 4095)
    pca.set_duty_12bit(CH_PWM2, 4095)
    channel_off(CH_HEATER_1)
    channel_off(CH_HEATER_2)
    channel_off(CH_HEATER_3)
    channel_off(CH_HEATER_4)
    channel_off(CH_FAN_1_POWER)
    channel_off(CH_FAN_2_POWER)
    channel_off(CH_STEPPER_DIR)
    channel_off(CH_STEPPER_ENA)
    channel_off(CH_PU)
    channel_off(CH_RESERVE1)
    channel_off(CH_LED_RED)
    channel_off(CH_LED_BLUE)
    channel_off(CH_LED_GREEN)
    channel_off(CH_SYS_LED)
except Exception as e:
    logger.error("Fatal: cannot set initial channel states (%s)", e)
    sys.exit(1)


try:
    client = mqtt.Client(callback_api_version=mqtt.CallbackAPIVersion.VERSION2)
except (AttributeError, TypeError):
    client = mqtt.Client()

if MQTT_USER and MQTT_PASS:
    client.username_pw_set(MQTT_USER, MQTT_PASS)


device_info_stepper = {
    "identifiers": ["pca9685_stepper_control"],
    "name": "Stepper Control",
    "model": "PCA9685",
    "manufacturer": "NXP Semiconductors",
}

device_info_heaters = {
    "identifiers": ["pca9685_heaters"],
    "name": "Heaters",
}

device_info_fans = {
    "identifiers": ["pca9685_fans"],
    "name": "Fans Control",
}

device_info_leds = {
    "identifiers": ["pca9685_leds"],
    "name": "Status LEDs",
}

device_info_pca9539 = {
    "identifiers": ["pca9539_expander"],
    "name": "GPIO Feedback",
    "model": "PCA9539",
}

device_info_bme_ch0_76 = {
    "identifiers": ["pca9685_bme280_ch0_0x76"],
    "name": "BME280 CH0 0x76",
    "model": "BME280/BMP280",
}

device_info_bme_ch0_77 = {
    "identifiers": ["pca9685_bme280_ch0_0x77"],
    "name": "BME280 CH0 0x77",
    "model": "BME280/BMP280",
}

device_info_bme_ch1_77 = {
    "identifiers": ["pca9685_bme280_ch1_0x77"],
    "name": "BME280 CH1 0x77",
    "model": "BME280/BMP280",
}

device_info_feedback = {
    "identifiers": ["pca9685_feedback_status"],
    "name": "Hardware Feedback Status",
}

DISCOVERIES = [
    ("number", "pca_pwm1_duty", {
        "name": "FAN 1 Speed Duty",
        "unique_id": "pca_pwm1_duty",
        "command_topic": TOPIC_PWM1_DUTY_CMD,
        "state_topic": TOPIC_PWM1_DUTY_STATE,
        "availability_topic": AVAIL_TOPIC,
        "min": 0,
        "max": 100,
        "step": 1,
        "unit_of_measurement": "%",
        "mode": "slider",
        "device": device_info_fans,
    }, "0"),
    ("switch", "pca_fan_1_power", {
        "name": "FAN 1 Power",
        "unique_id": "pca_fan_1_power",
        "command_topic": TOPIC_FAN_1_POWER_CMD,
        "state_topic": TOPIC_FAN_1_POWER_STATE,
        "availability_topic": AVAIL_TOPIC,
        "payload_on": "ON",
        "payload_off": "OFF",
        "device": device_info_fans,
    }, "OFF"),
    ("number", "pca_pwm2_duty", {
        "name": "FAN 2 Speed Duty",
        "unique_id": "pca_pwm2_duty",
        "command_topic": TOPIC_PWM2_DUTY_CMD,
        "state_topic": TOPIC_PWM2_DUTY_STATE,
        "availability_topic": AVAIL_TOPIC,
        "min": 0,
        "max": 100,
        "step": 1,
        "unit_of_measurement": "%",
        "mode": "slider",
        "device": device_info_fans,
    }, "0"),
    ("switch", "pca_fan_2_power", {
        "name": "FAN 2 Power",
        "unique_id": "pca_fan_2_power",
        "command_topic": TOPIC_FAN_2_POWER_CMD,
        "state_topic": TOPIC_FAN_2_POWER_STATE,
        "availability_topic": AVAIL_TOPIC,
        "payload_on": "ON",
        "payload_off": "OFF",
        "device": device_info_fans,
    }, "OFF"),
    ("switch", "pca_heater_1", {
        "name": "Heater 1",
        "unique_id": "pca_heater_1",
        "command_topic": TOPIC_HEATER_1_CMD,
        "state_topic": TOPIC_HEATER_1_STATE,
        "availability_topic": AVAIL_TOPIC,
        "payload_on": "ON",
        "payload_off": "OFF",
        "device": device_info_heaters,
    }, "OFF"),
    ("switch", "pca_heater_2", {
        "name": "Heater 2",
        "unique_id": "pca_heater_2",
        "command_topic": TOPIC_HEATER_2_CMD,
        "state_topic": TOPIC_HEATER_2_STATE,
        "availability_topic": AVAIL_TOPIC,
        "payload_on": "ON",
        "payload_off": "OFF",
        "device": device_info_heaters,
    }, "OFF"),
    ("switch", "pca_heater_3", {
        "name": "Heater 3",
        "unique_id": "pca_heater_3",
        "command_topic": TOPIC_HEATER_3_CMD,
        "state_topic": TOPIC_HEATER_3_STATE,
        "availability_topic": AVAIL_TOPIC,
        "payload_on": "ON",
        "payload_off": "OFF",
        "device": device_info_heaters,
    }, "OFF"),
    ("switch", "pca_heater_4", {
        "name": "Heater 4",
        "unique_id": "pca_heater_4",
        "command_topic": TOPIC_HEATER_4_CMD,
        "state_topic": TOPIC_HEATER_4_STATE,
        "availability_topic": AVAIL_TOPIC,
        "payload_on": "ON",
        "payload_off": "OFF",
        "device": device_info_heaters,
    }, "OFF"),
    ("select", "pca_stepper_dir", {
        "name": "DIR",
        "unique_id": "pca_stepper_dir",
        "command_topic": TOPIC_STEPPER_DIR_CMD,
        "state_topic": TOPIC_STEPPER_DIR_STATE,
        "availability_topic": AVAIL_TOPIC,
        "options": ["CW", "CCW"],
        "device": device_info_stepper,
    }, None),  # Will handle stepper_dir dynamically
    ("switch", "pca_stepper_ena", {
        "name": "ENA",
        "unique_id": "pca_stepper_ena",
        "command_topic": TOPIC_STEPPER_ENA_CMD,
        "state_topic": TOPIC_STEPPER_ENA_STATE,
        "availability_topic": AVAIL_TOPIC,
        "payload_on": "ON",
        "payload_off": "OFF",
        "device": device_info_stepper,
    }, "OFF"),
    ("switch", "pca_pu_enable", {
        "name": "PU Enable",
        "unique_id": "pca_pu_enable",
        "command_topic": TOPIC_PU_ENABLE_CMD,
        "state_topic": TOPIC_PU_ENABLE_STATE,
        "availability_topic": AVAIL_TOPIC,
        "payload_on": "ON",
        "payload_off": "OFF",
        "device": device_info_stepper,
    }, "OFF"),
    ("number", "pca_pu_freq_hz", {
        "name": "PU Frequency",
        "unique_id": "pca_pu_freq_hz",
        "command_topic": TOPIC_PU_FREQ_CMD,
        "state_topic": TOPIC_PU_FREQ_STATE,
        "availability_topic": AVAIL_TOPIC,
        "min": 0,
        "max": 500,
        "step": 1,
        "unit_of_measurement": "Hz",
        "mode": "slider",
        "device": device_info_stepper,
    }, None),  # Will handle pu_freq_hz dynamically
    # BME280 CH0 0x76
    ("sensor", "bme280_ch0_0x76_temperature", {
        "name": "Temperature CH0 0x76",
        "unique_id": "bme280_ch0_0x76_temperature",
        "state_topic": TOPIC_BME_CH0_76_TEMP,
        "unit_of_measurement": "°C",
        "device_class": "temperature",
        "device": device_info_bme_ch0_76,
    }),
    ("sensor", "bme280_ch0_0x76_humidity", {
        "name": "Humidity CH0 0x76",
        "unique_id": "bme280_ch0_0x76_humidity",
        "state_topic": TOPIC_BME_CH0_76_HUM,
        "unit_of_measurement": "%",
        "device_class": "humidity",
        "device": device_info_bme_ch0_76,
    }),
    ("sensor", "bme280_ch0_0x76_pressure", {
        "name": "Pressure CH0 0x76",
        "unique_id": "bme280_ch0_0x76_pressure",
        "state_topic": TOPIC_BME_CH0_76_PRESS,
        "unit_of_measurement": "hPa",
        "device_class": "pressure",
        "device": device_info_bme_ch0_76,
    }),
    # BME280 CH0 0x77
    ("sensor", "bme280_ch0_0x77_temperature", {
        "name": "Temperature CH0 0x77",
        "unique_id": "bme280_ch0_0x77_temperature",
        "state_topic": TOPIC_BME_CH0_77_TEMP,
        "unit_of_measurement": "°C",
        "device_class": "temperature",
        "device": device_info_bme_ch0_77,
    }),
    ("sensor", "bme280_ch0_0x77_humidity", {
        "name": "Humidity CH0 0x77",
        "unique_id": "bme280_ch0_0x77_humidity",
        "state_topic": TOPIC_BME_CH0_77_HUM,
        "unit_of_measurement": "%",
        "device_class": "humidity",
        "device": device_info_bme_ch0_77,
    }),
    ("sensor", "bme280_ch0_0x77_pressure", {
        "name": "Pressure CH0 0x77",
        "unique_id": "bme280_ch0_0x77_pressure",
        "state_topic": TOPIC_BME_CH0_77_PRESS,
        "unit_of_measurement": "hPa",
        "device_class": "pressure",
        "device": device_info_bme_ch0_77,
    }),
    # BME280 CH1 0x76
    ("sensor", "bme280_ch1_0x76_temperature", {
        "name": "Temperature CH1 0x77",
        "unique_id": "bme280_ch1_0x77_temperature",
        "state_topic": TOPIC_BME_CH1_77_TEMP,
        "unit_of_measurement": "°C",
        "device_class": "temperature",
        "device": device_info_bme_ch1_77,
    }),
    ("sensor", "bme280_ch1_0x76_humidity", {
        "name": "Humidity CH1 0x76",
        "unique_id": "bme280_ch1_0x76_humidity",
        "state_topic": TOPIC_BME_CH1_77_HUM,
        "unit_of_measurement": "%",
        "device_class": "humidity",
        "device": device_info_bme_ch1_77,
    }),
    ("sensor", "bme280_ch1_0x76_pressure", {
        "name": "Pressure CH1 0x76",
        "unique_id": "bme280_ch1_0x76_pressure",
        "state_topic": TOPIC_BME_CH1_77_PRESS,
        "unit_of_measurement": "hPa",
        "device_class": "pressure",
        "device": device_info_bme_ch1_77,
    }),
    # Status Feedback (Binary sensors)
    ("binary_sensor", "status_taxo1", {
        "name": "TAXO 1 Status",
        "unique_id": "status_taxo1",
        "state_topic": TOPIC_FEEDBACK_TAXO1,
        "availability_topic": AVAIL_TOPIC,
        "device_class": "problem",
        "device": device_info_feedback,
    }),
    ("binary_sensor", "status_taxo2", {
        "name": "TAXO 2 Status",
        "unique_id": "status_taxo2",
        "state_topic": TOPIC_FEEDBACK_TAXO2,
        "availability_topic": AVAIL_TOPIC,
        "device_class": "problem",
        "device": device_info_feedback,
    }),
    ("binary_sensor", "status_ena", {
        "name": "Status ENA",
        "unique_id": "status_ena",
        "state_topic": TOPIC_FEEDBACK_ENA,
        "availability_topic": AVAIL_TOPIC,
        "device_class": "problem",
        "device": device_info_feedback,
    }),
    ("binary_sensor", "status_dir", {
        "name": "Status DIR",
        "unique_id": "status_dir",
        "state_topic": TOPIC_FEEDBACK_DIR,
        "availability_topic": AVAIL_TOPIC,
        "device_class": "problem",
        "device": device_info_feedback,
    }),
    ("binary_sensor", "status_pu", {
        "name": "Status PU",
        "unique_id": "status_pu",
        "state_topic": TOPIC_FEEDBACK_PU,
        "availability_topic": AVAIL_TOPIC,
        "device_class": "problem",
        "device": device_info_feedback,
    }),
    ("binary_sensor", "status_relay1", {
        "name": "Status Relay 1",
        "unique_id": "status_relay1",
        "state_topic": TOPIC_FEEDBACK_RELAY1,
        "availability_topic": AVAIL_TOPIC,
        "device_class": "problem",
        "device": device_info_feedback,
    }),
    ("binary_sensor", "status_relay2", {
        "name": "Status Relay 2",
        "unique_id": "status_relay2",
        "state_topic": TOPIC_FEEDBACK_RELAY2,
        "availability_topic": AVAIL_TOPIC,
        "device_class": "problem",
        "device": device_info_feedback,
    }),
    ("binary_sensor", "status_relay3", {
        "name": "Status Relay 3",
        "unique_id": "status_relay3",
        "state_topic": TOPIC_FEEDBACK_RELAY3,
        "availability_topic": AVAIL_TOPIC,
        "device_class": "problem",
        "device": device_info_feedback,
    }),
    ("binary_sensor", "status_relay4", {
        "name": "Status Relay 4",
        "unique_id": "status_relay4",
        "state_topic": TOPIC_FEEDBACK_RELAY4,
        "availability_topic": AVAIL_TOPIC,
        "device_class": "problem",
        "device": device_info_feedback,
    }),
    ("binary_sensor", "status_relay5", {
        "name": "Status Relay 5",
        "unique_id": "status_relay5",
        "state_topic": TOPIC_FEEDBACK_RELAY5,
        "availability_topic": AVAIL_TOPIC,
        "device_class": "problem",
        "device": device_info_feedback,
    }),
    ("binary_sensor", "status_relay6", {
        "name": "Status Relay 6",
        "unique_id": "status_relay6",
        "state_topic": TOPIC_FEEDBACK_RELAY6,
        "availability_topic": AVAIL_TOPIC,
        "device_class": "problem",
        "device": device_info_feedback,
    }),
    ("binary_sensor", "status_res2", {
        "name": "Reserve 2",
        "unique_id": "status_res2",
        "state_topic": TOPIC_RES2,
        "availability_topic": AVAIL_TOPIC,
        "device": device_info_feedback,
    }),
    ("binary_sensor", "status_res3", {
        "name": "Reserve 3",
        "unique_id": "status_res3",
        "state_topic": TOPIC_RES3,
        "availability_topic": AVAIL_TOPIC,
        "device": device_info_feedback,
    }),
    ("binary_sensor", "status_res4", {
        "name": "Reserve 4",
        "unique_id": "status_res4",
        "state_topic": TOPIC_RES4,
        "availability_topic": AVAIL_TOPIC,
        "device": device_info_feedback,
    }),
]


def clear_discovery():
    """Clear old retained discovery messages by publishing empty payloads."""
    # List of unique_ids that were previously used but are now removed
    deprecated = [
        ("sensor", "pca9539_inputs"),
        ("binary_sensor", "status_fan1"),
        ("binary_sensor", "status_fan2"),
    ]
    
    for component, unique_id in deprecated:
        topic = f"homeassistant/{component}/{unique_id}/config"
        client.publish(topic, "", retain=True)
        
    for item in DISCOVERIES:
        component, unique_id = item[0], item[1]
        topic = f"homeassistant/{component}/{unique_id}/config"
        client.publish(topic, "", retain=True)
    logger.info("Old discovery messages cleared")


def publish_discovery():
    """Publish discovery configurations and initial states."""
    clear_discovery()
    for item in DISCOVERIES:
        component, unique_id, payload = item[0], item[1], item[2]
        # 1. Publish discovery config
        config_topic = f"homeassistant/{component}/{unique_id}/config"
        client.publish(config_topic, json.dumps(payload), retain=True)

        # 2. Publish initial state if specified
        if len(item) > 3:
            state_topic = payload.get("state_topic")
            initial_val = item[3]

            # Handle dynamic initial values
            if initial_val is None:
                if unique_id == "pca_stepper_dir":
                    initial_val = stepper_dir
                elif unique_id == "pca_pu_freq_hz":
                    initial_val = str(int(pu_freq_hz))

            if state_topic and initial_val is not None:
                client.publish(state_topic, initial_val, retain=True)

    # 3. Set availability
    client.publish(AVAIL_TOPIC, "online", retain=True)
    logger.info("Discovery messages and initial states published")


def perform_deep_clean():
    """Wipe all found topics during the cleaning phase."""
    global is_cleaning, found_topics
    
    logger.info("Deep clean timer expired. Clearing %d topics...", len(found_topics))
    
    with clean_lock:
        is_cleaning = False
        topics_to_clear = list(found_topics)
        found_topics.clear()
    
    # 1. Clear each found topic
    for topic in topics_to_clear:
        client.publish(topic, "", retain=True)
        # Also try to clear config if it was a state topic, or vice versa
        # Most of our topics follow homeassistant/{component}/{unique_id}/{suffix}
        # If we found a state topic, we should also clear the config topic
        parts = topic.split('/')
        if len(parts) >= 4:
            component = parts[1]
            unique_id = parts[2]
            config_topic = f"homeassistant/{component}/{unique_id}/config"
            if config_topic != topic:
                client.publish(config_topic, "", retain=True)

    # 2. Unsubscribe from the wildcard
    client.unsubscribe("homeassistant/#")
    logger.info("Deep clean completed. Proceeding with normal discovery.")
    
    # 3. Finally publish current discovery
    publish_discovery()


def on_connect(client, userdata, flags, reason_code, properties=None):
    global is_cleaning
    rc = reason_code.value if hasattr(reason_code, "value") else reason_code
    if rc != 0:
        logger.error("MQTT connection failed with code %s", rc)
        return

    # Subscriptions for commands
    client.subscribe(TOPIC_PWM1_DUTY_CMD)
    client.subscribe(TOPIC_FAN_1_POWER_CMD)
    client.subscribe(TOPIC_PWM2_DUTY_CMD)
    client.subscribe(TOPIC_FAN_2_POWER_CMD)
    client.subscribe(TOPIC_HEATER_1_CMD)
    client.subscribe(TOPIC_HEATER_2_CMD)
    client.subscribe(TOPIC_HEATER_3_CMD)
    client.subscribe(TOPIC_HEATER_4_CMD)
    client.subscribe(TOPIC_STEPPER_DIR_CMD)
    client.subscribe(TOPIC_STEPPER_ENA_CMD)
    client.subscribe(TOPIC_PU_ENABLE_CMD)
    client.subscribe(TOPIC_PU_FREQ_CMD)

    if MQTT_DEEP_CLEAN:
        logger.info("Deep clean enabled. Scanning for ghost topics...")
        is_cleaning = True
        client.subscribe("homeassistant/#")
        # Start timer to finish cleaning in 3 seconds
        threading.Timer(3.0, perform_deep_clean).start()
    else:
        publish_discovery()

    logger.info("Connected to MQTT broker %s:%s", MQTT_HOST, MQTT_PORT)


def _payload_to_bool(payload: str) -> bool:
    return payload == "ON"


def on_message(client, userdata, msg):
    global pwm1_value
    global pwm2_value
    global heater_1, heater_2, heater_3, heater_4
    global fan_1_power, fan_2_power
    global pu_enabled, pu_freq_hz
    global is_cleaning, found_topics

    topic = msg.topic
    payload_raw = msg.payload.decode("utf-8", errors="ignore").strip()

    # Handle deep cleaning phase
    if is_cleaning:
        # Check if topic belongs to us
        # Either by availability topic or by prefixes in the unique_id part
        is_ours = False
        if topic == AVAIL_TOPIC:
            is_ours = True
        else:
            parts = topic.split('/')
            if len(parts) >= 3:
                unique_id = parts[2]
                for prefix in CLEAN_PREFIXES:
                    if unique_id.startswith(prefix):
                        is_ours = True
                        break
        
        if is_ours:
            with clean_lock:
                if topic not in found_topics:
                    logger.debug("Found ghost topic to clear: %s", topic)
                    found_topics.add(topic)
        return

    payload = payload_raw
    try:
        if topic == TOPIC_PWM1_DUTY_CMD:
            value = max(0.0, min(100.0, float(payload)))
            with pwm1_lock:
                pwm1_value = value
                update_pwm1_output_locked()
                client.publish(TOPIC_PWM1_DUTY_STATE, str(int(value)), retain=True)
            
            # Auto control Fan Power based on Speed
            new_power = value > 0.0
            if new_power != fan_1_power:
                fan_1_power = new_power
                apply_switch(CH_FAN_1_POWER, fan_1_power)
                client.publish(TOPIC_FAN_1_POWER_STATE, "ON" if fan_1_power else "OFF", retain=True)

        elif topic == TOPIC_PWM2_DUTY_CMD:
            value = max(0.0, min(100.0, float(payload)))
            with pwm2_lock:
                pwm2_value = value
                update_pwm2_output_locked()
                client.publish(TOPIC_PWM2_DUTY_STATE, str(int(value)), retain=True)
            
            # Auto control Fan Power based on Speed
            new_power = value > 0.0
            if new_power != fan_2_power:
                fan_2_power = new_power
                apply_switch(CH_FAN_2_POWER, fan_2_power)
                client.publish(TOPIC_FAN_2_POWER_STATE, "ON" if fan_2_power else "OFF", retain=True)

        elif topic == TOPIC_HEATER_1_CMD:
            heater_1 = _payload_to_bool(payload)
            apply_switch(CH_HEATER_1, heater_1)
            client.publish(TOPIC_HEATER_1_STATE, "ON" if heater_1 else "OFF", retain=True)

        elif topic == TOPIC_HEATER_2_CMD:
            heater_2 = _payload_to_bool(payload)
            apply_switch(CH_HEATER_2, heater_2)
            client.publish(TOPIC_HEATER_2_STATE, "ON" if heater_2 else "OFF", retain=True)

        elif topic == TOPIC_HEATER_3_CMD:
            heater_3 = _payload_to_bool(payload)
            apply_switch(CH_HEATER_3, heater_3)
            client.publish(TOPIC_HEATER_3_STATE, "ON" if heater_3 else "OFF", retain=True)

        elif topic == TOPIC_HEATER_4_CMD:
            heater_4 = _payload_to_bool(payload)
            apply_switch(CH_HEATER_4, heater_4)
            client.publish(TOPIC_HEATER_4_STATE, "ON" if heater_4 else "OFF", retain=True)

        elif topic == TOPIC_FAN_1_POWER_CMD:
            fan_1_power = _payload_to_bool(payload)
            apply_switch(CH_FAN_1_POWER, fan_1_power)
            client.publish(TOPIC_FAN_1_POWER_STATE, "ON" if fan_1_power else "OFF", retain=True)
            
            # Sync Speed with Power
            with pwm1_lock:
                if fan_1_power:
                    if pwm1_value == 0:
                        pwm1_value = float(DEFAULT_DUTY_CYCLE)
                else:
                    pwm1_value = 0.0
                update_pwm1_output_locked()
                client.publish(TOPIC_PWM1_DUTY_STATE, str(int(pwm1_value)), retain=True)

        elif topic == TOPIC_FAN_2_POWER_CMD:
            fan_2_power = _payload_to_bool(payload)
            apply_switch(CH_FAN_2_POWER, fan_2_power)
            client.publish(TOPIC_FAN_2_POWER_STATE, "ON" if fan_2_power else "OFF", retain=True)

            # Sync Speed with Power
            with pwm2_lock:
                if fan_2_power:
                    if pwm2_value == 0:
                        pwm2_value = float(DEFAULT_DUTY_CYCLE)
                else:
                    pwm2_value = 0.0
                update_pwm2_output_locked()
                client.publish(TOPIC_PWM2_DUTY_STATE, str(int(pwm2_value)), retain=True)

        elif topic == TOPIC_STEPPER_DIR_CMD:
            stepper_apply_dir(payload)
            client.publish(TOPIC_STEPPER_DIR_STATE, stepper_dir, retain=True)

        elif topic == TOPIC_STEPPER_ENA_CMD:
            stepper_apply_ena(_payload_to_bool(payload))
            client.publish(TOPIC_STEPPER_ENA_STATE, "ON" if stepper_ena else "OFF", retain=True)

        elif topic == TOPIC_PU_ENABLE_CMD:
            with pu_lock:
                pu_enabled = _payload_to_bool(payload)
            if pu_enabled:
                pu_start()
            client.publish(TOPIC_PU_ENABLE_STATE, "ON" if pu_enabled else "OFF", retain=True)

        elif topic == TOPIC_PU_FREQ_CMD:
            hz = max(0.0, float(payload))
            with pu_lock:
                pu_freq_hz = hz
            client.publish(TOPIC_PU_FREQ_STATE, str(int(hz)), retain=True)

    except Exception:
        logger.exception("Error processing topic=%s payload=%s", topic, payload)


client.on_connect = on_connect
client.on_message = on_message


def safe_shutdown(signum=None, frame=None):
    try:
        bme_stop()
        pca9539_stop()
        sys_led_stop()
        pu_stop()

        with pwm1_lock:
            pwm1_value = 0.0
            update_pwm1_output_locked()

        with pwm2_lock:
            pwm2_value = 0.0
            update_pwm2_output_locked()

        channel_off(CH_HEATER_1)
        channel_off(CH_HEATER_2)
        channel_off(CH_HEATER_3)
        channel_off(CH_HEATER_4)
        channel_off(CH_FAN_1_POWER)
        channel_off(CH_FAN_2_POWER)
        channel_off(CH_STEPPER_DIR)
        channel_off(CH_STEPPER_ENA)
        channel_off(CH_PU)
        channel_off(CH_RESERVE1)
        channel_off(CH_LED_RED)
        channel_off(CH_LED_BLUE)
        channel_off(CH_LED_GREEN)
        channel_off(CH_SYS_LED)

        client.publish(AVAIL_TOPIC, "offline", retain=True)
        client.loop_stop()
        client.disconnect()
        if pca: pca.close()
        if pca9539: pca9539.close()
        if pca9540: pca9540.close()

    except Exception:
        logger.exception("Shutdown error")

    sys.exit(0 if signum is not None else 1)


signal.signal(signal.SIGTERM, safe_shutdown)
signal.signal(signal.SIGINT, safe_shutdown)


# Start signaling and diagnostic
sys_led_start()
if pca9539:
    hardware_diagnostic()
else:
    logger.warning("PCA9539 not available, skipping hardware diagnostic.")
    with status_lock:
        system_status = "OK"

logger.info("Connecting to MQTT %s:%s...", MQTT_HOST, MQTT_PORT)
for attempt in range(1, 11):
    try:
        client.connect(MQTT_HOST, MQTT_PORT, 60)
        client.loop_start()
        time.sleep(2)
        if client.is_connected():
            break
        raise RuntimeError("Timeout")
    except Exception as e:
        if attempt == 10:
            logger.error("MQTT connect failed after retries (%s)", e)
            safe_shutdown()
        time.sleep(1.5 ** (attempt - 1))

bme_start()
pca9539_start()

logger.info("Service running")
logger.setLevel(logging.ERROR)

while True:
    time.sleep(5)
    if not client.is_connected():
        try:
            client.reconnect()
            client.publish(AVAIL_TOPIC, "online", retain=True)
        except Exception:
            logger.exception("MQTT reconnect failed")
