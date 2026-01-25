import os
import time
import json
import threading
import paho.mqtt.client as mqtt


os.environ["BLINKA_FORCEBOARD"] = "RASPBERRY_PI_5"
os.environ["BLINKA_FORCECHIP"] = "BCM2712"

import board
import busio
from adafruit_pca9685 import PCA9685


# Четене на опции от /data/options.json
with open('/data/options.json') as f:
    config = json.load(f)

MQTT_HOST = config['mqtt_host']
MQTT_PORT = config['mqtt_port']
MQTT_USER = config.get('mqtt_username', None)
MQTT_PASS = config.get('mqtt_password', None)
PCA_ADDR = int(config['pca_address'], 16)
PCA_FREQ = config['pca_frequency']
MOTOR_CH = config['motor_channel']
LED0_CH = config['led0_channel']
LED1_CH = config['led1_channel']
INVERT_MOTOR = config['invert_motor']
MOTOR_MIN_PWM = config['motor_min_pwm']

# Инициализация на I2C и PCA9685 (оптимизирано за Pi 5 в Docker)
print("Initializing I2C bus...")
try:
    # Опит за автоматична инициализация
    i2c_bus = board.I2C()
    print("I2C initialized using board.I2C()")
except Exception as e:
    print(f"board.I2C() failed: {e}")
    # Fallback: директно отваряне на I2C Bus 1
    try:
        i2c_bus = busio.I2C(board.SCL, board.SDA)
        print("I2C initialized using busio.I2C(SCL, SDA)")
    except Exception as e2:
        print(f"busio.I2C() also failed: {e2}")
        raise

print(f"Connecting to PCA9685 at address {hex(PCA_ADDR)}...")
pca = PCA9685(i2c_bus, address=PCA_ADDR)
pca.frequency = PCA_FREQ
print(f"PCA9685 initialized at {PCA_FREQ}Hz")

# MQTT клиент
client = mqtt.Client()
if MQTT_USER and MQTT_PASS:
    client.username_pw_set(MQTT_USER, MQTT_PASS)

# Глобални променливи за blinking
blink_thread = None
blink_running = False
blink_lock = threading.Lock()
current_brightness_led1 = 0

def start_blinking():
    """Thread function за blinking на LED1"""
    global blink_running, current_brightness_led1
    print("Starting blink effect...")
    blink_running = True
    while blink_running:
        try:
            # On: Задай brightness
            pwm_value = int((current_brightness_led1 / 255) * 4095)
            pca.channels[LED1_CH].duty_cycle = pwm_value
            time.sleep(0.5)
            # Off
            if blink_running:  # Проверка преди изключване
                pca.channels[LED1_CH].duty_cycle = 0
                time.sleep(0.5)
        except Exception as e:
            print(f"Error in blink loop: {e}")
            break
    # Изключи LED при излизане от loop
    try:
        pca.channels[LED1_CH].duty_cycle = 0
    except:
        pass
    print("Blink effect stopped")

def stop_blinking():
    """Спира blinking thread-а безопасно"""
    global blink_running, blink_thread, blink_lock
    with blink_lock:
        if blink_running:
            print("Stopping blink effect...")
            blink_running = False
            if blink_thread and blink_thread.is_alive():
                blink_thread.join(timeout=2)

def on_connect(client, userdata, flags, rc):
    print(f"Connected to MQTT broker with result code {rc}")
    
    # Субскрайб към теми
    client.subscribe("homeassistant/number/motor_pwm/set")
    client.subscribe("homeassistant/light/led0_pwm/set")
    client.subscribe("homeassistant/light/led1_pwm/set")
    print("Subscribed to MQTT topics")
    
    # MQTT Discovery за мотора
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
    
    # MQTT Discovery за LED0 (PWM light)
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
    
    # MQTT Discovery за LED1 (с effects: solid и blink)
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
    
    # Публикуване на availability
    client.publish("homeassistant/number/motor_pwm/availability", "online", retain=True)
    client.publish("homeassistant/light/led0_pwm/availability", "online", retain=True)
    client.publish("homeassistant/light/led1_pwm/availability", "online", retain=True)
    print("MQTT discovery messages published")

def on_message(client, userdata, msg):
    topic = msg.topic
    payload = msg.payload.decode('utf-8')
    
    try:
        if topic == "homeassistant/number/motor_pwm/set":
            # Мотор логика
            value = float(payload)
            if INVERT_MOTOR:
                if value == 0:
                    pwm_percent = 100
                else:
                    pwm_percent = MOTOR_MIN_PWM - (value * MOTOR_MIN_PWM / 100)
            else:
                pwm_percent = value
            
            pwm_value = int(pwm_percent / 100 * 4095)
            pwm_value = max(0, min(4095, pwm_value))  # Clamp to 0-4095
            pca.channels[MOTOR_CH].duty_cycle = pwm_value
            client.publish("homeassistant/number/motor_pwm/state", str(value))
            print(f"Set motor PWM to {pwm_percent:.1f}% (input: {value}%, duty: {pwm_value})")
        
        elif topic == "homeassistant/light/led0_pwm/set":
            # LED0 PWM светлина
            data = json.loads(payload)
            state = data.get('state')
            
            if state == 'ON':
                brightness = data.get('brightness', 255)
                pwm_value = int((brightness / 255) * 4095)
                pca.channels[LED0_CH].duty_cycle = pwm_value
                client.publish("homeassistant/light/led0_pwm/state", 
                             json.dumps({"state": "ON", "brightness": brightness}))
                print(f"LED0 ON at brightness {brightness}")
            elif state == 'OFF':
                pca.channels[LED0_CH].duty_cycle = 0
                client.publish("homeassistant/light/led0_pwm/state", json.dumps({"state": "OFF"}))
                print("LED0 OFF")
        
        elif topic == "homeassistant/light/led1_pwm/set":
            # LED1 с blink ефект
            data = json.loads(payload)
            state = data.get('state')
            brightness = data.get('brightness', 255)
            effect = data.get('effect', 'solid')
            
            global current_brightness_led1, blink_thread, blink_lock
            current_brightness_led1 = brightness
            
            if state == 'OFF':
                stop_blinking()
                pca.channels[LED1_CH].duty_cycle = 0
                client.publish("homeassistant/light/led1_pwm/state", json.dumps({"state": "OFF"}))
                print("LED1 OFF")
                
            elif state == 'ON':
                if effect == 'blink':
                    stop_blinking()
                    with blink_lock:
                        blink_thread = threading.Thread(target=start_blinking, daemon=True)
                        blink_thread.start()
                    print(f"LED1 blinking at brightness {brightness}")
                    
                elif effect == 'solid':
                    stop_blinking()
                    pwm_value = int((brightness / 255) * 4095)
                    pca.channels[LED1_CH].duty_cycle = pwm_value
                    print(f"LED1 solid at brightness {brightness}")
                
                client.publish("homeassistant/light/led1_pwm/state", 
                             json.dumps({"state": "ON", "brightness": brightness, "effect": effect}))
    
    except Exception as e:
        print(f"Error processing message on {topic}: {e}")

def on_disconnect(client, userdata, rc):
    print(f"Disconnected from MQTT broker with code {rc}")
    if rc != 0:
        print("Unexpected disconnect, will attempt reconnect...")

# Налагане на callbacks
client.on_connect = on_connect
client.on_message = on_message
client.on_disconnect = on_disconnect

# Свързване към MQTT
print(f"Connecting to MQTT broker at {MQTT_HOST}:{MQTT_PORT}...")
try:
    client.connect(MQTT_HOST, MQTT_PORT, 60)
    print("Starting MQTT loop...")
    client.loop_forever()
except KeyboardInterrupt:
    print("\nShutting down...")
    stop_blinking()
    client.disconnect()
except Exception as e:
    print(f"Fatal error: {e}")
    stop_blinking()
    raise
