import time
import json
import threading
import math
from smbus2 import SMBus
import paho.mqtt.client as mqtt

# ---------- PCA9685 low-level driver via smbus2 ----------

MODE1      = 0x00
MODE2      = 0x01
PRESCALE   = 0xFE
LED0_ON_L  = 0x06
ALL_LED_ON_L  = 0xFA

# Bits
MODE1_RESTART = 0x80
MODE1_SLEEP   = 0x10
MODE1_AI      = 0x20  # Auto-increment
MODE2_OUTDRV  = 0x04  # Totem pole

OSC_HZ = 25_000_000  # PCA9685 internal oscillator typical

class PCA9685:
    def __init__(self, bus_num: int, address: int):
        self.address = address
        self.bus = SMBus(bus_num)

        # Init sequence
        self._write8(MODE1, MODE1_AI)           # auto-increment on
        self._write8(MODE2, MODE2_OUTDRV)       # output driver
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
        # prescale = round(osc / (4096*freq)) - 1
        prescale = int(round(OSC_HZ / (4096.0 * float(freq_hz)) - 1.0))
        prescale = max(3, min(255, prescale))  # datasheet bounds-ish

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
        # Simple PWM: ON=0, OFF=duty
        self.set_pwm(channel, 0, duty)

# ---------- Read HA add-on options ----------
with open("/data/options.json", "r") as f:
    config = json.load(f)

MQTT_HOST = config["mqtt_host"]
MQTT_PORT = config["mqtt_port"]
MQTT_USER = config.get("mqtt_username") or None
MQTT_PASS = config.get("mqtt_password") or None

I2C_BUS = int(config.get("i2c_bus", 1))  # optional, default 1
PCA_ADDR = int(config["pca_address"], 16)
PCA_FREQ = int(config["pca_frequency"])

MOTOR_CH = int(config["motor_channel"])
LED0_CH = int(config["led0_channel"])
LED1_CH = int(config["led1_channel"])

INVERT_MOTOR = bool(config["invert_motor"])
MOTOR_MIN_PWM = int(config["motor_min_pwm"])  # percent 0..100

# ---------- Init PCA9685 ----------
print(f"Opening I2C bus {I2C_BUS}, PCA9685 addr={hex(PCA_ADDR)}")
pca = PCA9685(I2C_BUS, PCA_ADDR)
pca.set_pwm_freq(PCA_FREQ)
print(f"PCA9685 frequency set to {PCA_FREQ} Hz")

# ---------- MQTT ----------
client = mqtt.Client()
if MQTT_USER and MQTT_PASS:
    client.username_pw_set(MQTT_USER, MQTT_PASS)

blink_thread = None
blink_running = False
blink_lock = threading.Lock()
current_brightness_led1 = 255

def brightness_to_12bit(brightness_0_255: int) -> int:
    b = int(max(0, min(255, brightness_0_255)))
    return int((b / 255.0) * 4095)

def start_blinking():
    global blink_running
    blink_running = True
    while blink_running:
        pca.set_duty_12bit(LED1_CH, brightness_to_12bit(current_brightness_led1))
        time.sleep(0.5)
        pca.set_duty_12bit(LED1_CH, 0)
        time.sleep(0.5)
    pca.set_duty_12bit(LED1_CH, 0)

def stop_blinking():
    global blink_running, blink_thread
    with blink_lock:
        blink_running = False
        if blink_thread and blink_thread.is_alive():
            blink_thread.join(timeout=2)
        blink_thread = None

def on_connect(client, userdata, flags, rc):
    print(f"Connected to MQTT with result code {rc}")

    client.subscribe("homeassistant/number/motor_pwm/set")
    client.subscribe("homeassistant/light/led0_pwm/set")
    client.subscribe("homeassistant/light/led1_pwm/set")

    discovery_motor = {
        "name": "Motor PWM",
        "unique_id": "pca_motor_pwm",
        "command_topic": "homeassistant/number/motor_pwm/set",
        "state_topic": "homeassistant/number/motor_pwm/state",
        "min": 0,
        "max": 100,
        "step": 1,
        "unit_of_measurement": "%",
        "availability_topic": "homeassistant/number/motor_pwm/availability"
    }
    client.publish("homeassistant/number/pca_motor_pwm/config", json.dumps(discovery_motor), retain=True)

    discovery_led0 = {
        "name": "Test LED 0 PWM",
        "unique_id": "pca_led0_pwm",
        "command_topic": "homeassistant/light/led0_pwm/set",
        "state_topic": "homeassistant/light/led0_pwm/state",
        "schema": "json",
        "brightness": True,
        "availability_topic": "homeassistant/light/led0_pwm/availability"
    }
    client.publish("homeassistant/light/pca_led0_pwm/config", json.dumps(discovery_led0), retain=True)

    discovery_led1 = {
        "name": "Test LED 1 Blink",
        "unique_id": "pca_led1_pwm",
        "command_topic": "homeassistant/light/led1_pwm/set",
        "state_topic": "homeassistant/light/led1_pwm/state",
        "schema": "json",
        "brightness": True,
        "effect": True,
        "effect_list": ["solid", "blink"],
        "availability_topic": "homeassistant/light/led1_pwm/availability"
    }
    client.publish("homeassistant/light/pca_led1_pwm/config", json.dumps(discovery_led1), retain=True)

    client.publish("homeassistant/number/motor_pwm/availability", "online", retain=True)
    client.publish("homeassistant/light/led0_pwm/availability", "online", retain=True)
    client.publish("homeassistant/light/led1_pwm/availability", "online", retain=True)

def on_message(client, userdata, msg):
    global current_brightness_led1, blink_thread
    topic = msg.topic
    payload = msg.payload.decode("utf-8")

    try:
        if topic == "homeassistant/number/motor_pwm/set":
            value = float(payload)  # 0..100 (percent)
            value = max(0.0, min(100.0, value))

            if INVERT_MOTOR:
                if value == 0:
                    pwm_percent = 100.0
                else:
                    pwm_percent = MOTOR_MIN_PWM - (value * MOTOR_MIN_PWM / 100.0)
            else:
                pwm_percent = value

            duty = int((pwm_percent / 100.0) * 4095)
            pca.set_duty_12bit(MOTOR_CH, duty)
            client.publish("homeassistant/number/motor_pwm/state", str(value))
            print(f"Motor set: input={value}% -> pwm={pwm_percent}% duty={duty}")

        elif topic == "homeassistant/light/led0_pwm/set":
            data = json.loads(payload)
            state = data.get("state")

            if state == "ON":
                brightness = int(data.get("brightness", 255))
                pca.set_duty_12bit(LED0_CH, brightness_to_12bit(brightness))
                client.publish("homeassistant/light/led0_pwm/state",
                               json.dumps({"state": "ON", "brightness": brightness}))
            else:
                pca.set_duty_12bit(LED0_CH, 0)
                client.publish("homeassistant/light/led0_pwm/state", json.dumps({"state": "OFF"}))

        elif topic == "homeassistant/light/led1_pwm/set":
            data = json.loads(payload)
            state = data.get("state")
            brightness = int(data.get("brightness", 255))
            effect = data.get("effect", "solid")

            current_brightness_led1 = max(0, min(255, brightness))

            if state == "OFF":
                stop_blinking()
                pca.set_duty_12bit(LED1_CH, 0)
                client.publish("homeassistant/light/led1_pwm/state", json.dumps({"state": "OFF"}))

            elif state == "ON":
                if effect == "blink":
                    stop_blinking()
                    with blink_lock:
                        blink_thread = threading.Thread(target=start_blinking, daemon=True)
                        blink_thread.start()
                else:
                    stop_blinking()
                    pca.set_duty_12bit(LED1_CH, brightness_to_12bit(current_brightness_led1))

                client.publish("homeassistant/light/led1_pwm/state",
                               json.dumps({"state": "ON", "brightness": current_brightness_led1, "effect": effect}))

    except Exception as e:
        print(f"Error processing {topic}: {e}")

client.on_connect = on_connect
client.on_message = on_message

print(f"Connecting MQTT {MQTT_HOST}:{MQTT_PORT} ...")
client.connect(MQTT_HOST, MQTT_PORT, 60)
client.loop_forever()
