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
    subprocess.run(["modprobe", "i2c-dev"], check=False)
except Exception as e:
    logger.warning("Failed to run modprobe i2c-dev: %s", e)

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
    def __init__(self, bus_num: int, address: int):
        self.address = address
        self.bus = SMBus(bus_num)
        self._write8(MODE1, MODE1_AI)
        self._write8(MODE2, MODE2_OUTDRV)
        time.sleep(0.01)

    def close(self):
        try:
            self.bus.close()
        except Exception:
            pass

    def _write8(self, reg, val):
        self.bus.write_byte_data(self.address, reg, val & 0xFF)

    def _read8(self, reg):
        return self.bus.read_byte_data(self.address, reg)

    def set_pwm_freq(self, freq_hz: float):
        prescale = int(round(OSC_HZ / (4096.0 * float(freq_hz)) - 1.0))
        prescale = max(3, min(255, prescale))
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
        self.bus.write_i2c_block_data(self.address, reg, data)

    def set_duty_12bit(self, channel: int, duty: int):
        duty = int(max(0, min(4095, duty)))
        self.set_pwm(channel, 0, duty)


class BME280:
    def __init__(self, bus_num: int, address: int = 0x76):
        self.address = address
        self.bus = SMBus(bus_num)
        self.cal = {}
        # Check Chip ID (BME280 = 0x60, BMP280 = 0x58)
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
        try:
            self.bus.close()
        except Exception:
            pass


CH_PWM1 = 0
CH_HEATER_1 = 1
CH_HEATER_2 = 2
CH_HEATER_3 = 3
CH_HEATER_4 = 4
CH_FAN_1 = 5
CH_FAN_2 = 6
CH_STEPPER_DIR = 7
CH_STEPPER_ENA = 8
CH_PU = 9
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
BME_ADDR = int(config.get("bme_address", "0x76"), 16)
PCA_FREQ = int(config.get("pca_frequency", 1000))

DEFAULT_DUTY_CYCLE = int(config.get("default_duty_cycle", 30))
DEFAULT_DUTY_CYCLE = max(0, min(100, DEFAULT_DUTY_CYCLE))

PU_DEFAULT_HZ = float(config.get("pu_default_hz", 10.0))
PU_DEFAULT_HZ = max(0.0, PU_DEFAULT_HZ)

AVAIL_TOPIC = "homeassistant/pca9685_pwm/availability"


def _topic(component: str, unique_id: str, suffix: str) -> str:
    return f"homeassistant/{component}/{unique_id}/{suffix}"


TOPIC_PWM1_ENABLE_CMD = _topic("switch", "pca_pwm1_enable", "set")
TOPIC_PWM1_ENABLE_STATE = _topic("switch", "pca_pwm1_enable", "state")

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

TOPIC_FAN_1_CMD = _topic("switch", "pca_fan_1", "set")
TOPIC_FAN_1_STATE = _topic("switch", "pca_fan_1", "state")
TOPIC_FAN_2_CMD = _topic("switch", "pca_fan_2", "set")
TOPIC_FAN_2_STATE = _topic("switch", "pca_fan_2", "state")

TOPIC_STEPPER_DIR_CMD = _topic("select", "pca_stepper_dir", "set")
TOPIC_STEPPER_DIR_STATE = _topic("select", "pca_stepper_dir", "state")

TOPIC_STEPPER_ENA_CMD = _topic("switch", "pca_stepper_ena", "set")
TOPIC_STEPPER_ENA_STATE = _topic("switch", "pca_stepper_ena", "state")

TOPIC_PU_ENABLE_CMD = _topic("switch", "pca_pu_enable", "set")
TOPIC_PU_ENABLE_STATE = _topic("switch", "pca_pu_enable", "state")

TOPIC_PU_FREQ_CMD = _topic("number", "pca_pu_freq_hz", "set")
TOPIC_PU_FREQ_STATE = _topic("number", "pca_pu_freq_hz", "state")

TOPIC_BME_TEMP = _topic("sensor", "bme280_temperature", "state")
TOPIC_BME_HUM = _topic("sensor", "bme280_humidity", "state")
TOPIC_BME_PRESS = _topic("sensor", "bme280_pressure", "state")


def validate_fixed_mapping():
    channels = [
        CH_PWM1,
        CH_HEATER_1,
        CH_HEATER_2,
        CH_HEATER_3,
        CH_HEATER_4,
        CH_FAN_1,
        CH_FAN_2,
        CH_STEPPER_DIR,
        CH_STEPPER_ENA,
        CH_PU,
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
        pca = PCA9685(I2C_BUS, PCA_ADDR)
        pca.set_pwm_freq(PCA_FREQ)
        logger.info("PCA9685 global PWM frequency set to %s Hz", PCA_FREQ)
        break
    except Exception as e:
        logger.warning("Attempt %d/10: Cannot initialize PCA9685 (%s). Retrying in 2s...", attempt, e)
        time.sleep(2)

if pca is None:
    logger.error("Fatal: Failed to initialize PCA9685 after 10 attempts.")
    sys.exit(1)


logger.info("Opening I2C bus %s, BME280 addr=%s", I2C_BUS, hex(BME_ADDR))
bme = None
try:
    bme = BME280(I2C_BUS, BME_ADDR)
    logger.info("BME280 sensor initialized")
except Exception as e:
    logger.warning("BME280 initialization failed: %s. Sensor will be disabled.", e)


pwm1_enabled = False
pwm1_value = 0.0
pwm1_lock = threading.Lock()

heater_1 = False
heater_2 = False
heater_3 = False
heater_4 = False
fan_1 = False
fan_2 = False

stepper_dir = "CW"
stepper_ena = False

pu_enabled = False
pu_freq_hz = float(PU_DEFAULT_HZ)
pu_lock = threading.Lock()
pu_thread = None
pu_running = False

sys_led_thread = None
sys_led_running = False
sys_led_lock = threading.Lock()


bme_thread = None
bme_running = False
bme_lock = threading.Lock()


def bme_worker():
    global bme_running
    while bme_running:
        if bme:
            try:
                temp, press, hum = bme.read_data()
                if temp is not None:
                    client.publish(TOPIC_BME_TEMP, f"{temp:.2f}", retain=True)
                    client.publish(TOPIC_BME_PRESS, f"{press:.2f}", retain=True)
                    if bme.is_bme:
                        client.publish(TOPIC_BME_HUM, f"{hum:.2f}", retain=True)
                else:
                    logger.warning("BME280: Invalid data read (sensor might be busy)")
            except Exception as e:
                logger.error("BME280 read error: %s", e)
        time.sleep(60)


def bme_start():
    global bme_thread, bme_running
    if not bme:
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
    global pwm1_enabled, pwm1_value

    if not pwm1_enabled:
        pca.set_duty_12bit(CH_PWM1, 4095)
        return

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


def apply_switch(channel: int, state: bool):
    if state:
        channel_on(channel)
    else:
        channel_off(channel)


def stepper_apply_dir(value: str):
    global stepper_dir
    stepper_dir = "CCW" if value == "CCW" else "CW"
    apply_switch(CH_STEPPER_DIR, stepper_dir == "CW")


def stepper_apply_ena(state: bool):
    global stepper_ena
    stepper_ena = bool(state)
    apply_switch(CH_STEPPER_ENA, stepper_ena)


def pu_worker():
    global pu_running
    last_level = None
    last_enabled = None

    while pu_running:
        with pu_lock:
            enabled = bool(pu_enabled)
            freq = float(pu_freq_hz)

        if (not enabled) or freq <= 0.0:
            if last_level is not False:
                channel_off(CH_PU)
                last_level = False
            if last_enabled is not False:
                last_enabled = False
            time.sleep(0.1)
            continue

        if last_enabled is not True:
            last_enabled = True

        half = 0.5 / freq
        if last_level is not True:
            channel_on(CH_PU)
            last_level = True
        time.sleep(half)
        if not pu_running:
            break
        if last_level is not False:
            channel_off(CH_PU)
            last_level = False
        time.sleep(half)

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
    global sys_led_running
    level = False
    last_written = None

    while sys_led_running:
        level = not level
        if level != last_written:
            if level:
                channel_on(CH_SYS_LED)
            else:
                channel_off(CH_SYS_LED)
            last_written = level
        time.sleep(1.0)

    channel_off(CH_SYS_LED)


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
    channel_off(CH_HEATER_1)
    channel_off(CH_HEATER_2)
    channel_off(CH_HEATER_3)
    channel_off(CH_HEATER_4)
    channel_off(CH_FAN_1)
    channel_off(CH_FAN_2)
    channel_off(CH_STEPPER_DIR)
    channel_off(CH_STEPPER_ENA)
    channel_off(CH_PU)
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


device_info = {
    "identifiers": ["pca9685_pwm_controller"],
    "name": "PCA9685 PWM Controller",
    "model": "PCA9685",
    "manufacturer": "NXP Semiconductors",
    "sw_version": "0.1.0-fixed-channels",
}


def publish_discovery():
    discoveries = [
        ("switch", "pca_pwm1_enable", {
            "name": "PWM 1 Enable (CH0)",
            "unique_id": "pca_pwm1_enable",
            "command_topic": TOPIC_PWM1_ENABLE_CMD,
            "state_topic": TOPIC_PWM1_ENABLE_STATE,
            "availability_topic": AVAIL_TOPIC,
            "payload_on": "ON",
            "payload_off": "OFF",
            "device": device_info,
        }),
        ("number", "pca_pwm1_duty", {
            "name": "PWM 1 Duty (CH0)",
            "unique_id": "pca_pwm1_duty",
            "command_topic": TOPIC_PWM1_DUTY_CMD,
            "state_topic": TOPIC_PWM1_DUTY_STATE,
            "availability_topic": AVAIL_TOPIC,
            "min": 0,
            "max": 100,
            "step": 1,
            "unit_of_measurement": "%",
            "mode": "slider",
            "device": device_info,
        }),
        ("switch", "pca_heater_1", {
            "name": "Heater 1 (CH1)",
            "unique_id": "pca_heater_1",
            "command_topic": TOPIC_HEATER_1_CMD,
            "state_topic": TOPIC_HEATER_1_STATE,
            "availability_topic": AVAIL_TOPIC,
            "payload_on": "ON",
            "payload_off": "OFF",
            "device": device_info,
        }),
        ("switch", "pca_heater_2", {
            "name": "Heater 2 (CH2)",
            "unique_id": "pca_heater_2",
            "command_topic": TOPIC_HEATER_2_CMD,
            "state_topic": TOPIC_HEATER_2_STATE,
            "availability_topic": AVAIL_TOPIC,
            "payload_on": "ON",
            "payload_off": "OFF",
            "device": device_info,
        }),
        ("switch", "pca_heater_3", {
            "name": "Heater 3 (CH3)",
            "unique_id": "pca_heater_3",
            "command_topic": TOPIC_HEATER_3_CMD,
            "state_topic": TOPIC_HEATER_3_STATE,
            "availability_topic": AVAIL_TOPIC,
            "payload_on": "ON",
            "payload_off": "OFF",
            "device": device_info,
        }),
        ("switch", "pca_heater_4", {
            "name": "Heater 4 (CH4)",
            "unique_id": "pca_heater_4",
            "command_topic": TOPIC_HEATER_4_CMD,
            "state_topic": TOPIC_HEATER_4_STATE,
            "availability_topic": AVAIL_TOPIC,
            "payload_on": "ON",
            "payload_off": "OFF",
            "device": device_info,
        }),
        ("switch", "pca_fan_1", {
            "name": "Fan 1 (CH5)",
            "unique_id": "pca_fan_1",
            "command_topic": TOPIC_FAN_1_CMD,
            "state_topic": TOPIC_FAN_1_STATE,
            "availability_topic": AVAIL_TOPIC,
            "payload_on": "ON",
            "payload_off": "OFF",
            "device": device_info,
        }),
        ("switch", "pca_fan_2", {
            "name": "Fan 2 (CH6)",
            "unique_id": "pca_fan_2",
            "command_topic": TOPIC_FAN_2_CMD,
            "state_topic": TOPIC_FAN_2_STATE,
            "availability_topic": AVAIL_TOPIC,
            "payload_on": "ON",
            "payload_off": "OFF",
            "device": device_info,
        }),
        ("select", "pca_stepper_dir", {
            "name": "Stepper DIR (CH7)",
            "unique_id": "pca_stepper_dir",
            "command_topic": TOPIC_STEPPER_DIR_CMD,
            "state_topic": TOPIC_STEPPER_DIR_STATE,
            "availability_topic": AVAIL_TOPIC,
            "options": ["CW", "CCW"],
            "device": device_info,
        }),
        ("switch", "pca_stepper_ena", {
            "name": "Stepper ENA (CH8)",
            "unique_id": "pca_stepper_ena",
            "command_topic": TOPIC_STEPPER_ENA_CMD,
            "state_topic": TOPIC_STEPPER_ENA_STATE,
            "availability_topic": AVAIL_TOPIC,
            "payload_on": "ON",
            "payload_off": "OFF",
            "device": device_info,
        }),
        ("switch", "pca_pu_enable", {
            "name": "PU Enable (CH9)",
            "unique_id": "pca_pu_enable",
            "command_topic": TOPIC_PU_ENABLE_CMD,
            "state_topic": TOPIC_PU_ENABLE_STATE,
            "availability_topic": AVAIL_TOPIC,
            "payload_on": "ON",
            "payload_off": "OFF",
            "device": device_info,
        }),
        ("number", "pca_pu_freq_hz", {
            "name": "PU Frequency (CH9)",
            "unique_id": "pca_pu_freq_hz",
            "command_topic": TOPIC_PU_FREQ_CMD,
            "state_topic": TOPIC_PU_FREQ_STATE,
            "availability_topic": AVAIL_TOPIC,
            "min": 0,
            "max": 500,
            "step": 1,
            "unit_of_measurement": "Hz",
            "mode": "slider",
            "device": device_info,
        }),
        ("sensor", "bme280_temperature", {
            "name": "BME280 Temperature",
            "unique_id": "bme280_temperature",
            "state_topic": TOPIC_BME_TEMP,
            "unit_of_measurement": "°C",
            "device_class": "temperature",
            "device": device_info,
        }),
        ("sensor", "bme280_humidity", {
            "name": "BME280 Humidity",
            "unique_id": "bme280_humidity",
            "state_topic": TOPIC_BME_HUM,
            "unit_of_measurement": "%",
            "device_class": "humidity",
            "device": device_info,
        }),
        ("sensor", "bme280_pressure", {
            "name": "BME280 Pressure",
            "unique_id": "bme280_pressure",
            "state_topic": TOPIC_BME_PRESS,
            "unit_of_measurement": "hPa",
            "device_class": "pressure",
            "device": device_info,
        }),
    ]

    for component, unique_id, payload in discoveries:
        topic = f"homeassistant/{component}/{unique_id}/config"
        client.publish(topic, json.dumps(payload), retain=True)

    client.publish(AVAIL_TOPIC, "online", retain=True)

    client.publish(TOPIC_PWM1_ENABLE_STATE, "OFF", retain=True)
    client.publish(TOPIC_PWM1_DUTY_STATE, "0", retain=True)
    client.publish(TOPIC_HEATER_1_STATE, "OFF", retain=True)
    client.publish(TOPIC_HEATER_2_STATE, "OFF", retain=True)
    client.publish(TOPIC_HEATER_3_STATE, "OFF", retain=True)
    client.publish(TOPIC_HEATER_4_STATE, "OFF", retain=True)
    client.publish(TOPIC_FAN_1_STATE, "OFF", retain=True)
    client.publish(TOPIC_FAN_2_STATE, "OFF", retain=True)
    client.publish(TOPIC_STEPPER_DIR_STATE, stepper_dir, retain=True)
    client.publish(TOPIC_STEPPER_ENA_STATE, "OFF", retain=True)
    client.publish(TOPIC_PU_ENABLE_STATE, "OFF", retain=True)
    client.publish(TOPIC_PU_FREQ_STATE, str(int(pu_freq_hz)), retain=True)

    logger.info("Discovery messages published")


def on_connect(client, userdata, flags, reason_code, properties=None):
    rc = reason_code.value if hasattr(reason_code, "value") else reason_code
    if rc != 0:
        logger.error("MQTT connection failed with code %s", rc)
        return

    client.subscribe(TOPIC_PWM1_ENABLE_CMD)
    client.subscribe(TOPIC_PWM1_DUTY_CMD)
    client.subscribe(TOPIC_HEATER_1_CMD)
    client.subscribe(TOPIC_HEATER_2_CMD)
    client.subscribe(TOPIC_HEATER_3_CMD)
    client.subscribe(TOPIC_HEATER_4_CMD)
    client.subscribe(TOPIC_FAN_1_CMD)
    client.subscribe(TOPIC_FAN_2_CMD)
    client.subscribe(TOPIC_STEPPER_DIR_CMD)
    client.subscribe(TOPIC_STEPPER_ENA_CMD)
    client.subscribe(TOPIC_PU_ENABLE_CMD)
    client.subscribe(TOPIC_PU_FREQ_CMD)
    publish_discovery()

    logger.info("Connected to MQTT broker %s:%s", MQTT_HOST, MQTT_PORT)


def _payload_to_bool(payload: str) -> bool:
    return payload == "ON"


def on_message(client, userdata, msg):
    global pwm1_enabled, pwm1_value
    global heater_1, heater_2, heater_3, heater_4
    global fan_1, fan_2
    global pu_enabled, pu_freq_hz

    topic = msg.topic
    payload = msg.payload.decode("utf-8").strip()

    try:
        if topic == TOPIC_PWM1_ENABLE_CMD:
            new_state = _payload_to_bool(payload)
            with pwm1_lock:
                if new_state and not pwm1_enabled:
                    pwm1_value = float(DEFAULT_DUTY_CYCLE)
                pwm1_enabled = new_state
                update_pwm1_output_locked()
                client.publish(TOPIC_PWM1_ENABLE_STATE, "ON" if pwm1_enabled else "OFF", retain=True)
                client.publish(TOPIC_PWM1_DUTY_STATE, str(int(pwm1_value if pwm1_enabled else 0)), retain=True)

        elif topic == TOPIC_PWM1_DUTY_CMD:
            value = max(0.0, min(100.0, float(payload)))
            with pwm1_lock:
                pwm1_value = value
                pwm1_enabled = value > 0.0
                update_pwm1_output_locked()
                client.publish(TOPIC_PWM1_ENABLE_STATE, "ON" if pwm1_enabled else "OFF", retain=True)
                client.publish(TOPIC_PWM1_DUTY_STATE, str(int(value)), retain=True)

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

        elif topic == TOPIC_FAN_1_CMD:
            fan_1 = _payload_to_bool(payload)
            apply_switch(CH_FAN_1, fan_1)
            client.publish(TOPIC_FAN_1_STATE, "ON" if fan_1 else "OFF", retain=True)

        elif topic == TOPIC_FAN_2_CMD:
            fan_2 = _payload_to_bool(payload)
            apply_switch(CH_FAN_2, fan_2)
            client.publish(TOPIC_FAN_2_STATE, "ON" if fan_2 else "OFF", retain=True)

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
        sys_led_stop()
        pu_stop()

        with pwm1_lock:
            pwm1_enabled = False
            pwm1_value = 0.0
            update_pwm1_output_locked()

        channel_off(CH_HEATER_1)
        channel_off(CH_HEATER_2)
        channel_off(CH_HEATER_3)
        channel_off(CH_HEATER_4)
        channel_off(CH_FAN_1)
        channel_off(CH_FAN_2)
        channel_off(CH_STEPPER_DIR)
        channel_off(CH_STEPPER_ENA)
        channel_off(CH_PU)
        channel_off(CH_SYS_LED)

        client.publish(AVAIL_TOPIC, "offline", retain=True)
        client.loop_stop()
        client.disconnect()
        pca.close()

    except Exception:
        logger.exception("Shutdown error")

    sys.exit(0 if signum is not None else 1)


signal.signal(signal.SIGTERM, safe_shutdown)
signal.signal(signal.SIGINT, safe_shutdown)


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

sys_led_start()
bme_start()

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
