#!/usr/bin/env python3
"""
PCA9685 PWM Controller for Home Assistant - RELAY FIXED VERSION
Safety-critical motor control with relay pulse support (50-100ms fixed timing)
"""
import time
import json
import threading
import signal
import sys
import os

try:
    import requests
    HAS_REQUESTS = True
except ImportError:
    HAS_REQUESTS = False
    print("⚠ requests module not available - using options.json only")

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
    if HAS_REQUESTS:
        token = os.environ.get("SUPERVISOR_TOKEN")
        if token:
            try:
                resp = requests.get(
                    "http://supervisor/services/mqtt",
                    headers={"Authorization": f"Bearer {token}"},
                    timeout=5
                )
                if resp.status_code == 200:
                    mqtt_cfg = resp.json()["data"]
                    print("✓ MQTT config loaded from Supervisor API")
                    with open("/data/options.json") as f:
                        opts = json.load(f)
                    opts.update({
                        "mqtt_host": mqtt_cfg["host"],
                        "mqtt_port": mqtt_cfg["port"],
                        "mqtt_username": mqtt_cfg["username"],
                        "mqtt_password": mqtt_cfg["password"]
                    })
                    return opts
            except Exception as e:
                print(f"⚠ Supervisor API unavailable ({e}), falling back to options.json")
    
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
LED1_CH   = int(config.get("led1_channel", 2))

# NEW: Relay configuration - fixed timing (milliseconds)
RELAY_CH = int(config.get("relay_channel", LED0_CH))  # Default: same as LED0
RELAY_PULSE_MS = float(config.get("relay_pulse_ms", 50))  # Fixed pulse duration (50-100ms)

DEFAULT_DUTY_CYCLE = int(config.get("default_duty_cycle", 30))
if not (0 <= DEFAULT_DUTY_CYCLE <= 100):
    raise ValueError(f"default_duty_cycle must be 0-100, got {DEFAULT_DUTY_CYCLE}")

print(f"Configuration validated:")
print(f"  - Motor Channel: {MOTOR_CH}")
print(f"  - LED0 Channel: {LED0_CH}, LED1 Channel: {LED1_CH}")
print(f"  - Relay Channel: {RELAY_CH}")
print(f"  - Relay Pulse: {RELAY_PULSE_MS}ms (FIXED, independent of PWM freq)")
print(f"  - PWM Frequency: {PCA_FREQ}Hz (only affects motor, not relay timing)")
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

print("→ Initializing motor to SAFE STOP state (100% duty)...")
pca.set_duty_12bit(MOTOR_CH, 4095)
time.sleep(0.1)

# ---------- MQTT setup ----------
try:
    client = mqtt.Client(callback_api_version=mqtt.CallbackAPIVersion.VERSION2)
    MQTT_V2 = True
except (AttributeError, TypeError):
    client = mqtt.Client()
    MQTT_V2 = False
    print("⚠ Using MQTT v1 API (paho-mqtt < 2.0)")

if MQTT_USER and MQTT_PASS:
    client.username_pw_set(MQTT_USER, MQTT_PASS)

# --- Globals ---
motor_enabled = False
motor_value = 0.0
motor_lock = threading.Lock()

led0_blink_thread = None
led0_blink_running = False
led0_blink_lock = threading.Lock()
led0_brightness = 255
led0_relay_mode = False  # True = relay pulse mode, False = normal LED blink

led1_brightness = 0

# ---------- Topic Definitions ----------
SWITCH_TOPIC_CMD = "homeassistant/switch/pca_motor_enable/set"
SWITCH_TOPIC_STATE = "homeassistant/switch/pca_motor_enable/state"
SWITCH_TOPIC_AVAIL = "homeassistant/switch/pca_motor_enable/availability"

NUMBER_TOPIC_CMD = "homeassistant/number/pca_motor_pwm/set"
NUMBER_TOPIC_STATE = "homeassistant/number/pca_motor_pwm/state"
NUMBER_TOPIC_AVAIL = "homeassistant/number/pca_motor_pwm/availability"

LED0_TOPIC_CMD = "homeassistant/light/pca_led0_blink/set"
LED0_TOPIC_STATE = "homeassistant/light/pca_led0_blink/state"
LED0_TOPIC_AVAIL = "homeassistant/light/pca_led0_blink/availability"

LED1_TOPIC_CMD = "homeassistant/light/pca_led1_solid/set"
LED1_TOPIC_STATE = "homeassistant/light/pca_led1_solid/state"
LED1_TOPIC_AVAIL = "homeassistant/light/pca_led1_solid/availability"

RELAY_TOPIC_CMD = "homeassistant/switch/pca_relay/set"
RELAY_TOPIC_STATE = "homeassistant/switch/pca_relay/state"

def led0_start_blinking():
    """Blinking thread - handles both LED and Relay modes"""
    global led0_blink_running
    
    if led0_relay_mode:
        # RELAY MODE: Fixed 50-100ms ON, 500ms OFF (independent of PWM frequency)
        pulse_duration = RELAY_PULSE_MS / 1000.0  # Convert ms to seconds
        off_duration = 0.5  # 500ms between pulses
        
        print(f"RELAY MODE started on ch {LED0_CH}: {RELAY_PULSE_MS}ms ON / 500ms OFF")
        
        while led0_blink_running:
            if not led0_blink_running: 
                break
            
            # Turn ON 100% (4095) - this creates continuous HIGH output
            pca.set_duty_12bit(LED0_CH, 4095)
            time.sleep(pulse_duration)  # Hold for fixed time (e.g. 50ms)
            
            if not led0_blink_running: 
                break
                
            # Turn OFF
            pca.set_duty_12bit(LED0_CH, 0)
            time.sleep(off_duration)
            
    else:
        # NORMAL LED MODE: Variable brightness, 500ms period
        print(f"LED MODE started on ch {LED0_CH}: brightness {led0_brightness}/255")
        while led0_blink_running:
            if not led0_blink_running: 
                break
            pca.set_duty_12bit(LED0_CH, brightness_to_12bit(led0_brightness))
            for _ in range(5):  # 500ms ON
                if not led0_blink_running: 
                    break
                time.sleep(0.1)
            if not led0_blink_running: 
                break
            pca.set_duty_12bit(LED0_CH, 0)
            for _ in range(5):  # 500ms OFF
                if not led0_blink_running: 
                    break
                time.sleep(0.1)
    
    # Cleanup
    pca.set_duty_12bit(LED0_CH, 0)
    print(f"Blinking stopped on ch {LED0_CH}")

def led0_stop_blinking():
    global led0_blink_running, led0_blink_thread, led0_relay_mode  # <-- ПОПРАВЕНО: добавен global
    with led0_blink_lock:
        led0_blink_running = False
        if led0_blink_thread and led0_blink_thread.is_alive():
            led0_blink_thread.join(timeout=2)
        led0_blink_thread = None
    pca.set_duty_12bit(LED0_CH, 0)

def trigger_relay_single_pulse():
    """Single relay pulse - non-blocking, runs in separate thread"""
    pulse_duration = RELAY_PULSE_MS / 1000.0
    
    print(f"Relay pulse on ch {RELAY_CH}: {RELAY_PULSE_MS}ms")
    pca.set_duty_12bit(RELAY_CH, 4095)  # 100% ON
    time.sleep(pulse_duration)          # Fixed 50-100ms
    pca.set_duty_12bit(RELAY_CH, 0)     # OFF
    print("Relay pulse complete")

def update_motor_pwm():
    """Update physical motor PWM - MUST be called with motor_lock held"""
    global motor_enabled, motor_value
    
    if not motor_enabled:
        pca.set_duty_12bit(MOTOR_CH, 4095)  # 100% = STOP (inverted logic)
        print("  → Motor PWM: 100% duty (SAFE STOP)")
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
    pca.set_duty_12bit(MOTOR_CH, duty)
    print(f"  → Motor PWM: visual={visual:.0f}% → {pwm_percent:.0f}% duty → {duty}")

def update_led1():
    """Update LED1 solid state"""
    pca.set_duty_12bit(LED1_CH, brightness_to_12bit(led1_brightness))

device_info = {
    "identifiers": ["pca9685_pwm_controller"],
    "name": "PCA9685 PWM Controller",
    "model": "PCA9685",
    "manufacturer": "NXP Semiconductors",
    "sw_version": "0.0.28-relay-fix"  # Обновена версия
}

def publish_discovery():
    """Publish HA discovery"""
    discoveries = [
        ("switch", "pca_motor_enable", {
            "name": "Motor Enable",
            "unique_id": "pca_motor_enable",
            "command_topic": SWITCH_TOPIC_CMD,
            "state_topic": SWITCH_TOPIC_STATE,
            "availability_topic": SWITCH_TOPIC_AVAIL,
            "payload_on": "ON",
            "payload_off": "OFF",
            "device": device_info
        }),
        ("number", "pca_motor_pwm", {
            "name": "Motor Speed",
            "unique_id": "pca_motor_pwm",
            "command_topic": NUMBER_TOPIC_CMD,
            "state_topic": NUMBER_TOPIC_STATE,
            "availability_topic": NUMBER_TOPIC_AVAIL,
            "min": 0,
            "max": 100,
            "step": 1,
            "unit_of_measurement": "%",
            "device": device_info
        }),
        ("light", "pca_led0_blink", {
            "name": "LED0 Test",
            "unique_id": "pca_led0_blink",
            "command_topic": LED0_TOPIC_CMD,
            "state_topic": LED0_TOPIC_STATE,
            "availability_topic": LED0_TOPIC_AVAIL,
            "schema": "json",
            "brightness": True,
            "effect": True,
            "effect_list": ["blink", "solid", "relay_pulse"],  # relay_pulse = fixed 50-100ms
            "device": device_info
        }),
        ("light", "pca_led1_solid", {
            "name": "LED1 Indicator",
            "unique_id": "pca_led1_solid",
            "command_topic": LED1_TOPIC_CMD,
            "state_topic": LED1_TOPIC_STATE,
            "availability_topic": LED1_TOPIC_AVAIL,
            "schema": "json",
            "brightness": True,
            "device": device_info
        }),
        ("switch", "pca_relay", {
            "name": "Relay Pulse",
            "unique_id": "pca_relay",
            "command_topic": RELAY_TOPIC_CMD,
            "state_topic": RELAY_TOPIC_STATE,
            "availability_topic": SWITCH_TOPIC_AVAIL,
            "payload_on": "TRIGGER",
            "payload_off": "OFF",
            "device": device_info
        })
    ]
    
    for component, unique_id, payload in discoveries:
        topic = f"homeassistant/{component}/{unique_id}/config"
        client.publish(topic, json.dumps(payload), retain=True)
    
    print("✓ Discovery messages published")
    
    # Initial states
    for topic in [SWITCH_TOPIC_AVAIL, NUMBER_TOPIC_AVAIL, LED0_TOPIC_AVAIL, LED1_TOPIC_AVAIL]:
        client.publish(topic, "online", retain=True)
    
    client.publish(SWITCH_TOPIC_STATE, "OFF", retain=True)
    client.publish(NUMBER_TOPIC_STATE, "0", retain=True)
    client.publish(LED0_TOPIC_STATE, json.dumps({"state": "OFF"}), retain=True)
    client.publish(LED1_TOPIC_STATE, json.dumps({"state": "OFF", "brightness": 0}), retain=True)
    client.publish(RELAY_TOPIC_STATE, "OFF", retain=True)

def on_connect(client, userdata, flags, reason_code, properties=None):
    """MQTT connection callback"""
    rc = reason_code.value if hasattr(reason_code, 'value') else reason_code
    
    if rc == 0:
        print(f"✓ Connected to MQTT broker {MQTT_HOST}:{MQTT_PORT}")
        client.subscribe(SWITCH_TOPIC_CMD)
        client.subscribe(NUMBER_TOPIC_CMD)
        client.subscribe(LED0_TOPIC_CMD)
        client.subscribe(LED1_TOPIC_CMD)
        client.subscribe(RELAY_TOPIC_CMD)
        publish_discovery()
    else:
        print(f"✗ MQTT connection failed with code {rc}")

def on_message(client, userdata, msg):
    """MQTT message callback"""
    global motor_enabled, motor_value
    global led0_brightness, led0_blink_thread, led0_blink_running, led1_brightness
    global led0_relay_mode  # <-- ПОПРАВЕНО: това липсваше и беше причината да не работи!
    
    topic = msg.topic
    payload = msg.payload.decode("utf-8")
    print(f"← MQTT [{topic}]: {payload[:60]}")

    try:
        if topic == SWITCH_TOPIC_CMD:
            new_state = (payload == "ON")
            
            with motor_lock:
                if new_state and not motor_enabled:
                    motor_value = float(DEFAULT_DUTY_CYCLE)
                    motor_enabled = True
                    update_motor_pwm()
                    client.publish(SWITCH_TOPIC_STATE, "ON", retain=True)
                    client.publish(NUMBER_TOPIC_STATE, str(int(motor_value)), retain=True)
                    print(f"→ Motor ENABLED at {int(motor_value)}%")
                
                elif not new_state and motor_enabled:
                    motor_enabled = False
                    motor_value = 0.0
                    update_motor_pwm()
                    client.publish(SWITCH_TOPIC_STATE, "OFF", retain=True)
                    client.publish(NUMBER_TOPIC_STATE, "0", retain=True)
                    print("→ Motor DISABLED (SAFE STOP)")

        elif topic == NUMBER_TOPIC_CMD:
            value = max(0.0, min(100.0, float(payload)))
            
            with motor_lock:
                motor_value = value
                should_be_enabled = (value > 0.0)
                
                if should_be_enabled != motor_enabled:
                    motor_enabled = should_be_enabled
                    client.publish(SWITCH_TOPIC_STATE, "ON" if should_be_enabled else "OFF", retain=True)
                    print(f"→ Auto-sync switch: {'ON' if should_be_enabled else 'OFF'}")
                
                update_motor_pwm()
                client.publish(NUMBER_TOPIC_STATE, str(int(value)), retain=True)
                
                if value == 0.0:
                    print("→ CRITICAL: Slider=0 → SAFE STOP commanded")

        elif topic == LED0_TOPIC_CMD:
            data = json.loads(payload)
            state = data.get("state", "OFF")
            brightness = max(0, min(255, int(data.get("brightness", 255))))
            effect = data.get("effect", "blink")

            led0_brightness = brightness
            led0_stop_blinking()

            if state == "OFF":
                client.publish(LED0_TOPIC_STATE, json.dumps({"state": "OFF"}), retain=True)
            else:
                if effect == "relay_pulse":
                    led0_relay_mode = True  # Сега това ще работи понеже има global декларация!
                    print(f"→ Setting relay_mode = True (effect: {effect})")  # Debug лог
                    with led0_blink_lock:
                        led0_blink_running = True
                        led0_blink_thread = threading.Thread(
                            target=led0_start_blinking,
                            daemon=True
                        )
                        led0_blink_thread.start()
                    state_payload = {
                        "state": "ON", 
                        "brightness": 255, 
                        "effect": "relay_pulse"
                    }
                    print(f"→ Auto-relay mode ON (continuous {RELAY_PULSE_MS}ms pulses)")
                
                elif effect == "blink":
                    led0_relay_mode = False
                    print(f"→ Setting relay_mode = False (effect: {effect})")  # Debug лог
                    with led0_blink_lock:
                        led0_blink_running = True
                        led0_blink_thread = threading.Thread(
                            target=led0_start_blinking,
                            daemon=True
                        )
                        led0_blink_thread.start()
                    state_payload = {
                        "state": "ON", 
                        "brightness": led0_brightness, 
                        "effect": "blink"
                    }
                else:  # solid
                    led0_relay_mode = False
                    pca.set_duty_12bit(LED0_CH, brightness_to_12bit(led0_brightness))
                    state_payload = {
                        "state": "ON", 
                        "brightness": led0_brightness, 
                        "effect": "solid"
                    }
                
                client.publish(LED0_TOPIC_STATE, json.dumps(state_payload), retain=True)

        elif topic == LED1_TOPIC_CMD:
            data = json.loads(payload)
            state = data.get("state", "OFF")
            brightness = max(0, min(255, int(data.get("brightness", 0))))
            
            led1_brightness = brightness if state == "ON" else 0
            update_led1()
            
            state_payload = {
                "state": "ON" if state == "ON" and brightness > 0 else "OFF",
                "brightness": led1_brightness
            }
            client.publish(LED1_TOPIC_STATE, json.dumps(state_payload), retain=True)

        elif topic == RELAY_TOPIC_CMD:
            if payload == "TRIGGER":
                # Single pulse in separate thread (non-blocking)
                threading.Thread(target=trigger_relay_single_pulse, daemon=True).start()
                client.publish(RELAY_TOPIC_STATE, "ON", retain=True)
                time.sleep(0.05)  # Short display time for state
                client.publish(RELAY_TOPIC_STATE, "OFF", retain=True)
            elif payload == "OFF":
                pca.set_duty_12bit(RELAY_CH, 0)
                client.publish(RELAY_TOPIC_STATE, "OFF", retain=True)

    except Exception as e:
        print(f"✗ Error processing {topic}: {e}")
        import traceback
        traceback.print_exc()

client.on_connect = on_connect
client.on_message = on_message

def safe_shutdown(signum=None, frame=None):
    print("\n" + "="*50)
    print("SHUTDOWN - Executing safety sequence...")
    print("="*50)
    
    try:
        print("→ Motor to SAFE STOP (100% duty)...")
        pca.set_duty_12bit(MOTOR_CH, 4095)
        time.sleep(0.2)
        
        print("→ Stopping LEDs/Relay...")
        led0_stop_blinking()
        pca.set_duty_12bit(LED0_CH, 0)
        pca.set_duty_12bit(LED1_CH, 0)
        pca.set_duty_12bit(RELAY_CH, 0)
        
        print("→ Setting offline...")
        for topic in [SWITCH_TOPIC_AVAIL, NUMBER_TOPIC_AVAIL, LED0_TOPIC_AVAIL, LED1_TOPIC_AVAIL]:
            client.publish(topic, "offline", retain=True)
        
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

# ---------- Connect with retries ----------
print(f"Connecting to MQTT {MQTT_HOST}:{MQTT_PORT}...")
for attempt in range(1, 11):
    try:
        client.connect(MQTT_HOST, MQTT_PORT, 60)
        client.loop_start()
        time.sleep(2)
        if client.is_connected():
            print("✓ MQTT connected")
            break
        raise Exception("Timeout")
    except Exception as e:
        print(f"⚠ Attempt {attempt}/10: {e}")
        if attempt == 10:
            print("✗ Max retries - exiting with motor in SAFE STOP")
            safe_shutdown()
        time.sleep(1.5 ** (attempt - 1))

print("="*50)
print("✅ Service running - awaiting commands")
print("="*50)

while True:
    time.sleep(5)
    if not client.is_connected():
        print("⚠ MQTT disconnected - reconnecting...")
        try:
            client.reconnect()
            print("✓ Reconnected")
            for topic in [SWITCH_TOPIC_AVAIL, NUMBER_TOPIC_AVAIL, LED0_TOPIC_AVAIL, LED1_TOPIC_AVAIL]:
                client.publish(topic, "online", retain=True)
        except Exception as e:
            print(f"✗ Reconnect failed: {e}")
