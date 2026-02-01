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

# ---------- Configuration loader ----------
def load_config():
    """Load config from options.json"""
    with open("/data/options.json") as f:
        print("✓ Config loaded from options.json")
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
LED1_CH   = int(config.get("led1_channel", 2))

DEFAULT_DUTY_CYCLE = int(config.get("default_duty_cycle", 30))
if not (0 <= DEFAULT_DUTY_CYCLE <= 100):
    raise ValueError(f"default_duty_cycle must be 0-100, got {DEFAULT_DUTY_CYCLE}")

print(f"Configuration validated:")
print(f"  - Default Duty Cycle: {DEFAULT_DUTY_CYCLE}%")
print(f"  - PWM mapping: Visual 0%→100% (STOP), Visual 1-10%→90% (MIN), Visual 11-100%→89-0% (LINEAR)")
print(f"  - MQTT Broker: {MQTT_HOST}:{MQTT_PORT}")

def brightness_to_12bit(brightness_0_255: int) -> int:
    b = int(max(0, min(255, brightness_0_255)))
    return int((b / 255.0) * 4095)

# ---------- Hardware initialization ----------
print(f"Opening I2C bus {I2C_BUS}, PCA9685 addr={hex(PCA_ADDR)}")
try:
    pca = PCA9685(I2C_BUS, PCA_ADDR)
    pca.set_pwm_freq(PCA_FREQ)
    print(f"✓ PCA9685 frequency set to {PCA_FREQ} Hz")
except Exception as e:
    print(f"✗ Fatal: Cannot initialize PCA9685 ({e})")
    sys.exit(1)

# ---------- SAFETY: Initialize motor to STOP state ----------
print("→ Initializing motor to SAFE STOP state (100% duty)...")
pca.set_duty_12bit(MOTOR_CH, 4095)
time.sleep(0.1)

# ---------- MQTT setup (VERSION1 API) ----------
client = mqtt.Client()
if MQTT_USER and MQTT_PASS:
    client.username_pw_set(MQTT_USER, MQTT_PASS)

# --- Global state ---
motor_enabled = False
motor_value = 0.0
motor_lock = threading.Lock()

led0_blink_thread = None
led0_blink_running = False
led0_blink_lock = threading.Lock()
led0_brightness = 255

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
    Update physical motor PWM with safety mapping:
      - Visual 0%   → 100% PWM (STOP)
      - Visual 1-10% → 90% PWM (MIN RUN)
      - Visual 11-100% → (100-visual)% PWM (LINEAR)
    """
    global motor_enabled, motor_value
    
    if not motor_enabled:
        print(f"  Motor DISABLED → PWM=100% → duty=4095 (STOP)")
        pca.set_duty_12bit(MOTOR_CH, 4095)  # STOP
        return
    
    visual = motor_value
    
    if visual == 0.0:
        pwm_percent = 100.0
    elif 0.0 < visual <= 10.0:
        pwm_percent = 90.0
    else:
        pwm_percent = 100.0 - visual
    
    duty = int((pwm_percent / 100.0) * 4095)
    duty = max(0, min(4095, duty))
    
    print(f"  Motor ENABLED: visual={visual}% → pwm={pwm_percent:.1f}% → duty={duty}")
    pca.set_duty_12bit(MOTOR_CH, duty)

def update_led1():
    """Update LED1 solid state"""
    pca.set_duty_12bit(LED1_CH, brightness_to_12bit(led1_brightness))

device_info = {
    "identifiers": ["pca9685_pwm_controller"],
    "name": "PCA9685 PWM Controller",
    "model": "PCA9685",
    "manufacturer": "NXP Semiconductors",
    "sw_version": "0.0.19"
}

def publish_discovery():
    """Publish Home Assistant MQTT discovery"""
    print("Publishing MQTT discovery messages...")
    
    # Motor switch
    client.publish(
        "homeassistant/switch/pca_motor_enable/config",
        json.dumps({
            "name": "Motor Enable",
            "unique_id": "pca_motor_enable",
            "command_topic": "homeassistant/switch/motor_enable/set",
            "state_topic": "homeassistant/switch/motor_enable/state",
            "payload_on": "ON",
            "payload_off": "OFF",
            "availability_topic": "homeassistant/switch/motor_enable/availability",
            "device": device_info
        }),
        retain=True
    )
    
    # Motor speed slider
    client.publish(
        "homeassistant/number/pca_motor_pwm/config",
        json.dumps({
            "name": "Motor Speed",
            "unique_id": "pca_motor_pwm",
            "command_topic": "homeassistant/number/motor_pwm/set",
            "state_topic": "homeassistant/number/motor_pwm/state",
            "min": 0,
            "max": 100,
            "step": 1,
            "unit_of_measurement": "%",
            "availability_topic": "homeassistant/number/motor_pwm/availability",
            "device": device_info
        }),
        retain=True
    )
    
    # LED0 test light
    client.publish(
        "homeassistant/light/pca_led0_blink/config",
        json.dumps({
            "name": "LED0 Test",
            "unique_id": "pca_led0_blink",
            "command_topic": "homeassistant/light/led0_pwm/set",
            "state_topic": "homeassistant/light/led0_pwm/state",
            "schema": "json",
            "brightness": True,
            "effect": True,
            "effect_list": ["blink", "solid"],
            "availability_topic": "homeassistant/light/led0_pwm/availability",
            "device": device_info
        }),
        retain=True
    )
    
    # LED1 indicator
    client.publish(
        "homeassistant/light/pca_led1_solid/config",
        json.dumps({
            "name": "LED1 Indicator",
            "unique_id": "pca_led1_solid",
            "command_topic": "homeassistant/light/led1_pwm/set",
            "state_topic": "homeassistant/light/led1_pwm/state",
            "schema": "json",
            "brightness": True,
            "availability_topic": "homeassistant/light/led1_pwm/availability",
            "device": device_info
        }),
        retain=True
    )
    
    # Set availability online
    for entity in ["switch/motor_enable", "number/motor_pwm", "light/led0_pwm", "light/led1_pwm"]:
        client.publish(f"homeassistant/{entity}/availability", "online", retain=True)
    
    # Initial states
    client.publish("homeassistant/switch/motor_enable/state", "OFF", retain=True)
    client.publish("homeassistant/number/motor_pwm/state", "0", retain=True)
    client.publish("homeassistant/light/led0_pwm/state", json.dumps({"state": "OFF"}), retain=True)
    client.publish("homeassistant/light/led1_pwm/state", json.dumps({"state": "OFF", "brightness": 0}), retain=True)
    
    print("✓ Discovery messages published")

def on_connect(client, userdata, flags, rc):
    """MQTT connection callback (VERSION1 API)"""
    if rc == 0:
        print(f"✓ Connected to MQTT broker {MQTT_HOST}:{MQTT_PORT}")
        
        # Subscribe to command topics
        client.subscribe("homeassistant/switch/motor_enable/set")
        client.subscribe("homeassistant/number/motor_pwm/set")
        client.subscribe("homeassistant/light/led0_pwm/set")
        client.subscribe("homeassistant/light/led1_pwm/set")
        
        publish_discovery()
    else:
        print(f"✗ MQTT connection failed with code {rc}")

def on_message(client, userdata, msg):
    """MQTT message callback"""
    global led0_brightness, led0_blink_thread, led0_blink_running
    global led1_brightness, motor_enabled, motor_value
    
    topic = msg.topic
    payload = msg.payload.decode("utf-8")
    
    print(f"← Received: {topic} = {payload}")

    try:
        if topic == "homeassistant/switch/motor_enable/set":
            new_state = (payload == "ON")
            
            with motor_lock:
                if new_state and not motor_enabled:
                    # Enable motor with default speed
                    motor_value = float(DEFAULT_DUTY_CYCLE)
                    motor_enabled = True
                    print(f"  Motor ENABLED at {motor_value}%")
                    update_motor_pwm()
                    client.publish("homeassistant/switch/motor_enable/state", "ON", retain=True)
                    client.publish("homeassistant/number/motor_pwm/state", str(motor_value), retain=True)
                
                elif not new_state and motor_enabled:
                    # Disable motor
                    motor_enabled = False
                    motor_value = 0.0
                    print("  Motor DISABLED")
                    update_motor_pwm()
                    client.publish("homeassistant/switch/motor_enable/state", "OFF", retain=True)
                    client.publish("homeassistant/number/motor_pwm/state", "0", retain=True)

        elif topic == "homeassistant/number/motor_pwm/set":
            value = max(0.0, min(100.0, float(payload)))
            print(f"  Slider changed to {value}%")
            
            with motor_lock:
                motor_value = value
                
                # Auto-sync switch state
                if value == 0.0 and motor_enabled:
                    motor_enabled = False
                    print("  Auto-disabling motor (slider=0)")
                    update_motor_pwm()  # Update PWM BEFORE publishing state
                    client.publish("homeassistant/switch/motor_enable/state", "OFF", retain=True)
                elif value > 0.0 and not motor_enabled:
                    motor_enabled = True
                    print("  Auto-enabling motor (slider>0)")
                    client.publish("homeassistant/switch/motor_enable/state", "ON", retain=True)
                    update_motor_pwm()  # Update PWM after enabling
                elif motor_enabled:
                    # Motor already enabled, just update PWM
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
                            daemon=True
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
        import traceback
        traceback.print_exc()

client.on_connect = on_connect
client.on_message = on_message

def safe_shutdown(signum=None, frame=None):
    """CRITICAL: Execute safety shutdown"""
    print("\n" + "="*50)
    print("SHUTDOWN - Executing safety sequence...")
    print("="*50)
    
    try:
        print("→ Motor to SAFE STOP (100% duty)...")
        pca.set_duty_12bit(MOTOR_CH, 4095)
        time.sleep(0.2)
        
        print("→ Stopping LEDs...")
        led0_stop_blinking()
        pca.set_duty_12bit(LED0_CH, 0)
        pca.set_duty_12bit(LED1_CH, 0)
        
        print("→ Setting offline...")
        for entity in ["switch/motor_enable", "number/motor_pwm", "light/led0_pwm", "light/led1_pwm"]:
            client.publish(f"homeassistant/{entity}/availability", "offline", retain=True)
        
        print("→ Disconnecting MQTT...")
        client.loop_stop()
        client.disconnect()
        
        print("→ Closing I2C...")
        pca.close()
        
        print("="*50)
        print("✓ Shutdown complete")
        print("="*50)
        
    except Exception as e:
        print(f"! Shutdown error: {e}")
    
    sys.exit(0 if signum is not None else 1)

signal.signal(signal.SIGTERM, safe_shutdown)
signal.signal(signal.SIGINT, safe_shutdown)

# ---------- Connect with retry ----------
print(f"Connecting to MQTT {MQTT_HOST}:{MQTT_PORT}...")
max_retries = 10

for attempt in range(1, max_retries + 1):
    try:
        client.connect(MQTT_HOST, MQTT_PORT, 60)
        client.loop_start()
        time.sleep(2)
        if client.is_connected():
            print("✓ MQTT connected")
            break
        else:
            raise Exception("Connection timeout")
    except Exception as e:
        print(f"⚠ Attempt {attempt}/{max_retries} failed: {e}")
        if attempt == max_retries:
            print("✗ Max retries exceeded")
            safe_shutdown()
        time.sleep(2 ** attempt)

print("="*50)
print("Service running. Awaiting commands...")
print("="*50)

# Keep-alive loop
while True:
    time.sleep(5)
    if not client.is_connected():
        print("⚠ MQTT disconnected - reconnecting...")
        try:
            client.reconnect()
            print("✓ Reconnected")
            for entity in ["switch/motor_enable", "number/motor_pwm", "light/led0_pwm", "light/led1_pwm"]:
                client.publish(f"homeassistant/{entity}/availability", "online", retain=True)
        except Exception as e:
            print(f"✗ Reconnect failed: {e}")
