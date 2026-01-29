import time
import json
import threading
import signal
import sys
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

# ---------- Read HA add-on options ----------
with open("/data/options.json", "r") as f:
    config = json.load(f)

MQTT_HOST = config["mqtt_host"]
MQTT_PORT = config["mqtt_port"]
MQTT_USER = config.get("mqtt_username") or None
MQTT_PASS = config.get("mqtt_password") or None

I2C_BUS   = int(config.get("i2c_bus", 1))
PCA_ADDR  = int(config["pca_address"], 16)
PCA_FREQ  = int(config["pca_frequency"])

MOTOR_CH  = int(config["motor_channel"])
LED0_CH   = int(config["led0_channel"])  # LED0 = Blink test

DEFAULT_DUTY_CYCLE = int(config.get("default_duty_cycle", 30))  # default visual duty when enabling

# Validate configuration
if not (0 <= DEFAULT_DUTY_CYCLE <= 100):
    raise ValueError(f"default_duty_cycle must be 0-100, got {DEFAULT_DUTY_CYCLE}")

print(f"Configuration validated:")
print(f"  - Default Duty Cycle: {DEFAULT_DUTY_CYCLE}%")
print(f"  - PWM mapping: Visual 0%→100% (STOP), Visual 10%→90% (START), Visual 100%→0% (MAX)")

def brightness_to_12bit(brightness_0_255: int) -> int:
    b = int(max(0, min(255, brightness_0_255)))
    return int((b / 255.0) * 4095)

print(f"Opening I2C bus {I2C_BUS}, PCA9685 addr={hex(PCA_ADDR)}")
pca = PCA9685(I2C_BUS, PCA_ADDR)
pca.set_pwm_freq(PCA_FREQ)
print(f"PCA9685 frequency set to {PCA_FREQ} Hz")

# ---------- MQTT ----------
client = mqtt.Client()
if MQTT_USER and MQTT_PASS:
    client.username_pw_set(MQTT_USER, MQTT_PASS)

# --- Motor globals ---
motor_enabled = False
motor_value = 0.0  # Visual value 0..100

# --- LED0 blink globals ---
led0_blink_thread = None
led0_blink_running = False
led0_blink_lock = threading.Lock()
led0_brightness = 255

def led0_start_blinking():
    global led0_blink_running
    led0_blink_running = True
    while led0_blink_running:
        if not led0_blink_running:  # Double-check before turning on
            break
        pca.set_duty_12bit(LED0_CH, brightness_to_12bit(led0_brightness))
        
        # Sleep in small chunks to respond faster to stop signal
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
    
    # Ensure LED is off when stopping
    pca.set_duty_12bit(LED0_CH, 0)

def led0_stop_blinking():
    global led0_blink_running, led0_blink_thread
    with led0_blink_lock:
        led0_blink_running = False
        if led0_blink_thread and led0_blink_thread.is_alive():
            led0_blink_thread.join(timeout=2)
        led0_blink_thread = None
    # Extra safety: ensure LED is off
    pca.set_duty_12bit(LED0_CH, 0)

def update_motor_pwm():
    """Update physical motor PWM based on enabled state and visual value
    
    Hardware behavior:
    - PWM 0% → Motor MAX speed
    - PWM 90% → Motor START (minimum)
    - PWM 100% → Motor STOP
    
    Visual mapping:
    - Visual 0% → Physical 100% (STOP)
    - Visual 1-10% → Physical 90% (MIN/START) - snapped
    - Visual 11-100% → Physical 89-0% (LINEAR to MAX)
    """
    global motor_enabled, motor_value
    
    if not motor_enabled:
        # Motor disabled - 100% PWM = STOP
        pca.set_duty_12bit(MOTOR_CH, 4095)
        return
    
    visual = motor_value
    
    # Snap visual 1-10% to physical 90% (START threshold)
    if 0.0 < visual <= 10.0:
        pwm_percent = 90.0  # START/MIN
    elif visual == 0:
        pwm_percent = 100.0  # STOP
    else:
        # Linear mapping for visual 11-100% → physical 89-0%
        # visual 11% → 89%, visual 100% → 0%
        pwm_percent = 100.0 - visual
    
    duty = int((pwm_percent / 100.0) * 4095)
    duty = max(0, min(4095, duty))
    pca.set_duty_12bit(MOTOR_CH, duty)

def on_connect(client, userdata, flags, rc):
    print(f"Connected to MQTT with result code {rc}")

    # Device information for grouping all entities together
    device_info = {
        "identifiers": ["pca9685_pwm_controller"],
        "name": "PCA9685 PWM Controller",
        "model": "PCA9685",
        "manufacturer": "NXP Semiconductors",
        "sw_version": "0.0.14"
    }

    # Subscriptions
    client.subscribe("homeassistant/switch/motor_enable/set")
    client.subscribe("homeassistant/number/motor_pwm/set")
    client.subscribe("homeassistant/light/led0_pwm/set")

    # Motor enable switch
    discovery_motor_switch = {
        "name": "Motor Enable",
        "unique_id": "pca_motor_enable",
        "command_topic": "homeassistant/switch/motor_enable/set",
        "state_topic": "homeassistant/switch/motor_enable/state",
        "payload_on": "ON",
        "payload_off": "OFF",
        "availability_topic": "homeassistant/switch/motor_enable/availability",
        "device": device_info
    }
    client.publish("homeassistant/switch/pca_motor_enable/config", json.dumps(discovery_motor_switch), retain=True)

    # Motor number (slider)
    discovery_motor = {
        "name": "Motor PWM",
        "unique_id": "pca_motor_pwm",
        "command_topic": "homeassistant/number/motor_pwm/set",
        "state_topic": "homeassistant/number/motor_pwm/state",
        "min": 0,
        "max": 100,
        "step": 1,
        "unit_of_measurement": "%",
        "availability_topic": "homeassistant/number/motor_pwm/availability",
        "device": device_info
    }
    client.publish("homeassistant/number/pca_motor_pwm/config", json.dumps(discovery_motor), retain=True)

    # LED0 = Blink test (effect blink/solid)
    discovery_led0_blink = {
        "name": "LED0 Blink Test",
        "unique_id": "pca_led0_blink",
        "command_topic": "homeassistant/light/led0_pwm/set",
        "state_topic": "homeassistant/light/led0_pwm/state",
        "schema": "json",
        "brightness": True,
        "effect": True,
        "effect_list": ["blink", "solid"],
        "availability_topic": "homeassistant/light/led0_pwm/availability",
        "device": device_info
    }
    client.publish("homeassistant/light/pca_led0_pwm/config", json.dumps(discovery_led0_blink), retain=True)

    # Availability
    client.publish("homeassistant/switch/motor_enable/availability", "online", retain=True)
    client.publish("homeassistant/number/motor_pwm/availability", "online", retain=True)
    client.publish("homeassistant/light/led0_pwm/availability", "online", retain=True)
    
    # Initial states
    client.publish("homeassistant/switch/motor_enable/state", "OFF", retain=True)
    client.publish("homeassistant/number/motor_pwm/state", "0", retain=True)

def on_message(client, userdata, msg):
    global led0_brightness, led0_blink_thread, motor_enabled, motor_value
    topic = msg.topic
    payload = msg.payload.decode("utf-8")

    try:
        if topic == "homeassistant/switch/motor_enable/set":
            motor_enabled = (payload == "ON")
            
            if motor_enabled:
                # Motor enabled - load default duty cycle and update PWM
                motor_value = float(DEFAULT_DUTY_CYCLE)
                update_motor_pwm()
                client.publish("homeassistant/switch/motor_enable/state", "ON")
                client.publish("homeassistant/number/motor_pwm/state", str(motor_value))
            else:
                # Motor disabled - set slider to 0 and PWM to maximum (inverted = stopped)
                motor_value = 0.0
                update_motor_pwm()
                client.publish("homeassistant/switch/motor_enable/state", "OFF")
                client.publish("homeassistant/number/motor_pwm/state", "0")
        
        elif topic == "homeassistant/number/motor_pwm/set":
            value = float(payload)  # 0..100
            value = max(0.0, min(100.0, value))
            motor_value = value
            
            # Auto-sync: slider 0 → switch OFF, slider >0 → switch ON
            if value == 0.0 and motor_enabled:
                motor_enabled = False
                client.publish("homeassistant/switch/motor_enable/state", "OFF")
                update_motor_pwm()
            elif value > 0.0 and not motor_enabled:
                motor_enabled = True
                client.publish("homeassistant/switch/motor_enable/state", "ON")
                update_motor_pwm()
            elif motor_enabled:
                # Just update PWM if already enabled
                update_motor_pwm()
            
            client.publish("homeassistant/number/motor_pwm/state", str(value))

        elif topic == "homeassistant/light/led0_pwm/set":
            data = json.loads(payload)
            state = data.get("state")
            brightness = int(data.get("brightness", 255))
            effect = data.get("effect", "blink")  # default blink

            led0_brightness = max(0, min(255, brightness))

            if state == "OFF":
                led0_stop_blinking()
                pca.set_duty_12bit(LED0_CH, 0)
                client.publish("homeassistant/light/led0_pwm/state", json.dumps({"state": "OFF"}))

            elif state == "ON":
                if effect == "blink":
                    led0_stop_blinking()
                    with led0_blink_lock:
                        led0_blink_thread = threading.Thread(target=led0_start_blinking, daemon=True)
                        led0_blink_thread.start()
                    client.publish(
                        "homeassistant/light/led0_pwm/state",
                        json.dumps({"state": "ON", "brightness": led0_brightness, "effect": "blink"})
                    )
                else:
                    led0_stop_blinking()
                    pca.set_duty_12bit(LED0_CH, brightness_to_12bit(led0_brightness))
                    client.publish(
                        "homeassistant/light/led0_pwm/state",
                        json.dumps({"state": "ON", "brightness": led0_brightness, "effect": "solid"})
                    )

    except Exception as e:
        print(f"Error processing {topic}: {e}")

client.on_connect = on_connect
client.on_message = on_message

# ---------- Safe shutdown handling ----------
def safe_shutdown(signum=None, frame=None):
    """Safe shutdown: stop motor, turn off LEDs, close connections"""
    print("\n========================================")
    print("SHUTDOWN SIGNAL RECEIVED - Cleaning up...")
    print("========================================")
    
    try:
        # Stop motor (set to safe state)
        print("Stopping motor (setting to safe state)...")
        pca.set_duty_12bit(MOTOR_CH, 4095)  # Full PWM = motor stopped in inverted mode
        
        # Stop LED blinking
        print("Stopping LED0 blinking...")
        led0_stop_blinking()
        
        # Turn off all PWM channels
        print("Turning off all PWM channels...")
        for ch in range(16):
            pca.set_duty_12bit(ch, 0)
        
        # Set MQTT availability to offline
        print("Setting MQTT availability to offline...")
        client.publish("homeassistant/switch/motor_enable/availability", "offline", retain=True)
        client.publish("homeassistant/number/motor_pwm/availability", "offline", retain=True)
        client.publish("homeassistant/light/led0_pwm/availability", "offline", retain=True)
        
        # Disconnect MQTT
        print("Disconnecting MQTT...")
        client.loop_stop()
        client.disconnect()
        
        # Close I2C bus
        print("Closing I2C bus...")
        pca.close()
        
        print("========================================")
        print("Cleanup completed successfully")
        print("========================================")
        
    except Exception as e:
        print(f"Error during shutdown: {e}")
    
    sys.exit(0)

# Register signal handlers
signal.signal(signal.SIGTERM, safe_shutdown)  # Docker stop
signal.signal(signal.SIGINT, safe_shutdown)   # Ctrl+C

print(f"Connecting MQTT {MQTT_HOST}:{MQTT_PORT} ...")
client.connect(MQTT_HOST, MQTT_PORT, 60)
client.loop_start()  # Non-blocking loop in background thread

# Keep alive loop with periodic checks
try:
    print("Service running. Press Ctrl+C to stop.")
    while True:
        time.sleep(1)
        
        # Check if MQTT is still connected
        if not client.is_connected():
            print("MQTT disconnected, attempting reconnect...")
            try:
                client.reconnect()
            except Exception as e:
                print(f"Reconnect failed: {e}")
        
except KeyboardInterrupt:
    safe_shutdown()
except Exception as e:
    print(f"Fatal error: {e}")
    safe_shutdown()
