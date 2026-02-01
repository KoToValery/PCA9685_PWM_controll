#!/usr/bin/env python3
"""
PCA9685 PWM Controller for Home Assistant
Safety-critical motor control with inverted PWM topology:
  - 100% duty = Motor STOPPED (brake)
  - 90% duty  = Minimum running speed
  - 0% duty   = Maximum speed
"""
import time
import json
import threading
import signal
import sys
import os
import requests
from smbus2 import SMBus
import paho.mqtt.client as mqtt

# ---------- PCA9685 low-level driver via smbus2 ----------
MODE1      = 0x00
MODE2      = 0x01
PRESCALE   = 0xFE
LED0_ON_L  = 0x06

MODE1_RESTART = 0x80
MODE1_SLEEP   = 0x10
MODE1_AI      = 0x20
MODE2_OUTDRV  = 0x04

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

# ---------- Configuration loader with Supervisor API fallback ----------
def load_config():
    """Load config from Supervisor API (preferred) or options.json fallback"""
    token = os.environ.get("SUPERVISOR_TOKEN")
    
    # Try Supervisor API first (more secure, auto-syncs with HA MQTT settings)
    if token:
        try:
            resp = requests.get(
                "http://supervisor/services/mqtt",
                headers={"Authorization": f"Bearer {token}"}
            )
            if resp.status_code == 200:
                mqtt_cfg = resp.json()["data"]
                print("✓ MQTT config loaded from Supervisor API")
                # Load other options from options.json
                with open("/data/options.json") as f:
                    opts = json.load(f)
                # Merge MQTT settings from Supervisor
                opts.update({
                    "mqtt_host": mqtt_cfg["host"],
                    "mqtt_port": mqtt_cfg["port"],
                    "mqtt_username": mqtt_cfg["username"],
                    "mqtt_password": mqtt_cfg["password"]
                })
                return opts
        except Exception as e:
            print(f"⚠ Supervisor API unavailable ({e}), falling back to options.json")
    
    # Fallback to options.json
    with open("/data/options.json") as f:
        print("✓ MQTT config loaded from options.json")
        return json.load(f)

config = load_config()

MQTT_HOST = config["mqtt_host"]
MQTT_PORT = config["mqtt_port"]
MQTT_USER = config.get("mqtt_username") or None
MQTT_PASS = config.get("mqtt_password") or None

I2C_BUS   = int(config.get("i2c_bus", 1))
PCA_ADDR  = int(config["pca_address"], 16)
PCA_FREQ  = int(config["pca_frequency"])

MOTOR_CH  = int(config["motor_channel"])
LED0_CH   = int(config["led0_channel"])
LED1_CH   = int(config.get("led1_channel", 2))  # Optional LED1 support

DEFAULT_DUTY_CYCLE = int(config.get("default_duty_cycle", 30))
if not (0 <= DEFAULT_DUTY_CYCLE <= 100):
    raise ValueError(f"default_duty_cycle must be 0-100, got {DEFAULT_DUTY_CYCLE}")

print(f"Configuration validated:")
print(f"  - Default Duty Cycle: {DEFAULT_DUTY_CYCLE}%")
print(f"  - PWM mapping: Visual 0%→100% (STOP), Visual 1-10%→90% (MIN RUN), Visual 11-100%→89-0% (LINEAR)")
print(f"  - MQTT Broker: {MQTT_HOST}:{MQTT_PORT}")

def brightness_to_12bit(brightness_0_255: int) -> int:
    b = int(max(0, min(255, brightness_0_255)))
    return int((b / 255.0) * 4095)

# ---------- Hardware initialization ----------
print(f"Opening I2C bus {I2C_BUS}, PCA9685 addr={hex(PCA_ADDR)}")
try:
    pca = PCA9685(I2C_BUS, PCA_ADDR)
    pca.set_pwm_freq(PCA_FREQ)
    print(f"PCA9685 frequency set to {PCA_FREQ} Hz")
except Exception as e:
    print(f"✗ Fatal: Cannot initialize PCA9685 ({e})")
    sys.exit(1)

# ---------- SAFETY FIRST: Initialize motor to STOP state ----------
print("→ Initializing motor to SAFE STOP state (100% duty)...")
pca.set_duty_12bit(MOTOR_CH, 4095)  # 100% = STOP in inverted topology
time.sleep(0.1)

# ---------- MQTT setup ----------
# Use v2 API to avoid deprecation warning
client = mqtt.Client(callback_api_version=mqtt.CallbackAPIVersion.VERSION2)
if MQTT_USER and MQTT_PASS:
    client.username_pw_set(MQTT_USER, MQTT_PASS)

# --- Motor globals ---
motor_enabled = False
motor_value = 0.0
motor_lock = threading.Lock()

# --- LED0 blink globals ---
led0_blink_thread = None
led0_blink_running = False
led0_blink_lock = threading.Lock()
led0_brightness = 255

# --- LED1 solid state ---
led1_brightness = 0

def led0_start_blinking():
    global led0_blink_running
    while led0_blink_running:
        if not led0_blink_running:
            break
        pca.set_duty_12bit(LED0_CH, brightness_to_12bit(led0_brightness))
        for _ in range(5):
            if not led0_blink_running:
                break
            time.sleep(0.1)
        if not led0_blink_running:
            break
        pca.set_duty_12bit(LED0_CH, 0)
        for _ in range(5):
            if not led0_blink_running:
                break
            time.sleep(0.1)
    pca.set_duty_12bit(LED0_CH, 0)

def led0_stop_blinking():
    global led0_blink_running, led0_blink_thread
    with led0_blink_lock:
        led0_blink_running = False
        if led0_blink_thread and led0_blink_thread.is_alive():
            led0_blink_thread.join(timeout=2)
        led0_blink_thread = None
    pca.set_duty_12bit(LED0_CH, 0)

def update_motor_pwm():
    """
    Update physical motor PWM with safety mapping (inverted topology):
      - Visual 0%   → 100% PWM (STOP)
      - Visual 1-10% → 90% PWM (MIN RUN threshold)
      - Visual 11-100% → (100 - visual)% PWM (linear acceleration)
    """
    global motor_enabled, motor_value
    
    with motor_lock:
        if not motor_enabled:
            pca.set_duty_12bit(MOTOR_CH, 4095)  # 100% = STOP
            return
        
        visual = motor_value
        
        if visual == 0.0:
            pwm_percent = 100.0
        elif 0.0 < visual <= 10.0:
            pwm_percent = 90.0  # Minimum running threshold
        else:
            pwm_percent = 100.0 - visual  # Linear mapping
        
        duty = int((pwm_percent / 100.0) * 4095)
        duty = max(0, min(4095, duty))
        pca.set_duty_12bit(MOTOR_CH, duty)

def update_led1():
    """Update LED1 solid state"""
    pca.set_duty_12bit(LED1_CH, brightness_to_12bit(led1_brightness))

device_info = {
    "identifiers": ["pca9685_pwm_controller"],
    "name": "PCA9685 PWM Controller",
    "model": "PCA9685",
    "manufacturer": "NXP Semiconductors",
    "sw_version": "0.0.18"
}

def publish_discovery():
    """Publish Home Assistant MQTT discovery messages"""
    base_topics = [
        ("switch", "pca_motor_enable", {
            "name": "Motor Enable",
            "command_topic": "homeassistant/switch/motor_enable/set",
            "state_topic": "homeassistant/switch/motor_enable/state",
            "payload_on": "ON",
            "payload_off": "OFF",
            "availability_topic": "homeassistant/switch/motor_enable/availability"
        }),
        ("number", "pca_motor_pwm", {
            "name": "Motor Speed",
            "command_topic": "homeassistant/number/motor_pwm/set",
            "state_topic": "homeassistant/number/motor_pwm/state",
            "min": 0,
            "max": 100,
            "step": 1,
            "unit_of_measurement": "%",
            "availability_topic": "homeassistant/number/motor_pwm/availability"
        }),
        ("light", "pca_led0_blink", {
            "name": "LED0 Test",
            "command_topic": "homeassistant/light/led0_pwm/set",
            "state_topic": "homeassistant/light/led0_pwm/state",
            "schema": "json",
            "brightness": True,
            "effect": True,
            "effect_list": ["blink", "solid"],
            "availability_topic": "homeassistant/light/led0_pwm/availability"
        }),
        ("light", "pca_led1_solid", {
            "name": "LED1 Indicator",
            "command_topic": "homeassistant/light/led1_pwm/set",
            "state_topic": "homeassistant/light/led1_pwm/state",
            "schema": "json",
            "brightness": True,
            "availability_topic": "homeassistant/light/led1_pwm/availability"
        })
    ]
    
    for component, unique_id, payload in base_topics:
        payload["unique_id"] = unique_id
        payload["device"] = device_info
        topic = f"homeassistant/{component}/{unique_id}/config"
        client.publish(topic, json.dumps(payload), retain=True)
        print(f"  Published discovery: {topic}")

    # Set initial availability and states
    for entity in ["switch/motor_enable", "number/motor_pwm", "light/led0_pwm", "light/led1_pwm"]:
        client.publish(f"homeassistant/{entity}/availability", "online", retain=True)
    
    client.publish("homeassistant/switch/motor_enable/state", "OFF", retain=True)
    client.publish("homeassistant/number/motor_pwm/state", "0", retain=True)
    client.publish("homeassistant/light/led0_pwm/state", json.dumps({"state": "OFF"}), retain=True)
    client.publish("homeassistant/light/led1_pwm/state", json.dumps({"state": "OFF", "brightness": 0}), retain=True)

def on_connect(client, userdata, flags, reason_code, properties):
    if reason_code == 0:
        print(f"✓ Connected to MQTT broker {MQTT_HOST}:{MQTT_PORT}")
        client.subscribe("homeassistant/switch/motor_enable/set")
        client.subscribe("homeassistant/number/motor_pwm/set")
        client.subscribe("homeassistant/light/led0_pwm/set")
        client.subscribe("homeassistant/light/led1_pwm/set")
        publish_discovery()
    else:
        print(f"✗ MQTT connection failed with code {reason_code}")

def on_message(client, userdata, msg):
    global led0_brightness, led0_blink_thread, led1_brightness
    global motor_enabled, motor_value
    topic = msg.topic
    payload = msg.payload.decode("utf-8")

    try:
        if topic == "homeassistant/switch/motor_enable/set":
            new_state = (payload == "ON")
            
            with motor_lock:
                if new_state and not motor_enabled:
                    motor_value = float(DEFAULT_DUTY_CYCLE)
                    motor_enabled = True
                    update_motor_pwm()
                    client.publish("homeassistant/switch/motor_enable/state", "ON", retain=True)
                    client.publish("homeassistant/number/motor_pwm/state", str(motor_value), retain=True)
                
                elif not new_state and motor_enabled:
                    motor_enabled = False
                    motor_value = 0.0
                    update_motor_pwm()
                    client.publish("homeassistant/switch/motor_enable/state", "OFF", retain=True)
                    client.publish("homeassistant/number/motor_pwm/state", "0", retain=True)

        elif topic == "homeassistant/number/motor_pwm/set":
            value = max(0.0, min(100.0, float(payload)))
            
            with motor_lock:
                # Auto-sync switch state based on speed
                if value == 0.0 and motor_enabled:
                    motor_enabled = False
                    update_motor_pwm()
                    client.publish("homeassistant/switch/motor_enable/state", "OFF", retain=True)
                elif value > 0.0 and not motor_enabled:
                    motor_enabled = True
                    client.publish("homeassistant/switch/motor_enable/state", "ON", retain=True)
                
                motor_value = value
                if motor_enabled:
                    update_motor_pwm()
                
                client.publish("homeassistant/number/motor_pwm/state", str(value), retain=True)

        elif topic == "homeassistant/light/led0_pwm/set":
            data = json.loads(payload)
            state = data.get("state", "OFF")
            brightness = max(0, min(255, int(data.get("brightness", 255))))
            effect = data.get("effect", "blink")

            led0_brightness = brightness

            if state == "OFF":
                led0_stop_blinking()
                client.publish(
                    "homeassistant/light/led0_pwm/state",
                    json.dumps({"state": "OFF"}),
                    retain=True
                )
            else:  # ON
                led0_stop_blinking()
                if effect == "blink":
                    with led0_blink_lock:
                        led0_blink_running = True
                        led0_blink_thread = threading.Thread(
                            target=led0_start_blinking,
                            daemon=True,
                            name="LED0BlinkThread"
                        )
                        led0_blink_thread.start()
                    client.publish(
                        "homeassistant/light/led0_pwm/state",
                        json.dumps({
                            "state": "ON",
                            "brightness": led0_brightness,
                            "effect": "blink"
                        }),
                        retain=True
                    )
                else:  # solid
                    pca.set_duty_12bit(LED0_CH, brightness_to_12bit(led0_brightness))
                    client.publish(
                        "homeassistant/light/led0_pwm/state",
                        json.dumps({
                            "state": "ON",
                            "brightness": led0_brightness,
                            "effect": "solid"
                        }),
                        retain=True
                    )

        elif topic == "homeassistant/light/led1_pwm/set":
            data = json.loads(payload)
            state = data.get("state", "OFF")
            brightness = max(0, min(255, int(data.get("brightness", 0))))
            
            led1_brightness = brightness if state == "ON" else 0
            update_led1()
            
            client.publish(
                "homeassistant/light/led1_pwm/state",
                json.dumps({
                    "state": "ON" if state == "ON" and brightness > 0 else "OFF",
                    "brightness": led1_brightness
                }),
                retain=True
            )

    except Exception as e:
        print(f"✗ Error processing {topic}: {e}")

client.on_connect = on_connect
client.on_message = on_message

def safe_shutdown(signum=None, frame=None):
    """CRITICAL: Execute safety shutdown sequence"""
    print("\n" + "="*50)
    print("SHUTDOWN INITIATED - Executing safety sequence...")
    print("="*50)
    
    try:
        # ⚠️ SAFETY FIRST: Set motor to STOP (100% duty)
        print("→ Setting motor to SAFE STOP state (100% duty)...")
        pca.set_duty_12bit(MOTOR_CH, 4095)
        time.sleep(0.2)
        
        # Stop all LED activity
        print("→ Stopping LED activity...")
        led0_stop_blinking()
        pca.set_duty_12bit(LED0_CH, 0)
        pca.set_duty_12bit(LED1_CH, 0)
        
        # Update availability states
        print("→ Setting entities to offline...")
        for entity in ["switch/motor_enable", "number/motor_pwm", "light/led0_pwm", "light/led1_pwm"]:
            client.publish(f"homeassistant/{entity}/availability", "offline", retain=True)
        
        # Disconnect cleanly
        print("→ Disconnecting MQTT...")
        client.loop_stop()
        client.disconnect()
        
        # Release hardware
        print("→ Closing I2C bus...")
        pca.close()
        
        print("="*50)
        print("✓ Shutdown sequence completed safely")
        print("="*50)
        
    except Exception as e:
        print(f"! Shutdown error: {e}")
    
    sys.exit(0 if signum is not None else 1)

# Register shutdown handlers BEFORE connecting
signal.signal(signal.SIGTERM, safe_shutdown)
signal.signal(signal.SIGINT, safe_shutdown)

# ---------- MQTT connection with exponential backoff ----------
print(f"Connecting to MQTT broker {MQTT_HOST}:{MQTT_PORT}...")
max_retries = 10
base_delay = 1.0

for attempt in range(1, max_retries + 1):
    try:
        client.connect(MQTT_HOST, MQTT_PORT, 60)
        client.loop_start()
        # Wait for connection confirmation
        time.sleep(2)
        if client.is_connected():
            print("✓ MQTT connected successfully")
            break
        else:
            raise Exception("Connection timeout")
    except Exception as e:
        print(f"⚠ Connection attempt {attempt}/{max_retries} failed: {e}")
        if attempt == max_retries:
            print("✗ Max retries exceeded. Exiting with motor in SAFE STOP state.")
            safe_shutdown()
        time.sleep(base_delay * (1.5 ** (attempt - 1)))  # Exponential backoff

print("Service running. Awaiting commands...")
print("="*50)

# Keep-alive loop with connection monitoring
while True:
    time.sleep(5)
    if not client.is_connected():
        print("⚠ MQTT disconnected - attempting reconnect...")
        try:
            client.reconnect()
            print("✓ MQTT reconnected")
            # Republish availability after reconnect
            for entity in ["switch/motor_enable", "number/motor_pwm", "light/led0_pwm", "light/led1_pwm"]:
                client.publish(f"homeassistant/{entity}/availability", "online", retain=True)
        except Exception as e:
            print(f"✗ Reconnect failed: {e}")
            time.sleep(10)
