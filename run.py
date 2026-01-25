import os
import time
import json
import threading
import paho.mqtt.client as mqtt
from board import SCL, SDA
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

# Инициализация на I2C и PCA9685
i2c_bus = busio.I2C(SCL, SDA)
pca = PCA9685(i2c_bus, address=PCA_ADDR)
pca.frequency = PCA_FREQ

# MQTT клиент
client = mqtt.Client()
if MQTT_USER and MQTT_PASS:
    client.username_pw_set(MQTT_USER, MQTT_PASS)

# Глобални променливи за blinking
blink_thread = None
blink_running = False
current_brightness_led1 = 0  # За да помни brightness по време на blink

def start_blinking():
    global blink_running, current_brightness_led1
    blink_running = True
    while blink_running:
        # On: Задай brightness
        pwm_value = int((current_brightness_led1 / 255) * 4095)
        pca.channels[LED1_CH].duty_cycle = pwm_value
        time.sleep(0.5)  # Blink rate: 0.5 sec on
        # Off
        pca.channels[LED1_CH].duty_cycle = 0
        time.sleep(0.5)  # 0.5 sec off
    pca.channels[LED1_CH].duty_cycle = 0  # Изключи накрая

def stop_blinking():
    global blink_running, blink_thread
    blink_running = False
    if blink_thread and blink_thread.is_alive():
        blink_thread.join()

def on_connect(client, userdata, flags, rc):
    print(f"Connected to MQTT with result code {rc}")
    # Субскрайб към теми
    client.subscribe("homeassistant/number/motor_pwm/set")
    client.subscribe("homeassistant/light/led0_pwm/set")
    client.subscribe("homeassistant/light/led1_pwm/set")
    
    # Discovery за мотора (без промяна)
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
    
    # Discovery за LED0 (PWM light)
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
    
    # Discovery за LED1 (с effects: solid и blink)
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
    
    # Availability
    client.publish("homeassistant/number/motor_pwm/availability", "online", retain=True)
    client.publish("homeassistant/light/led0_pwm/availability", "online", retain=True)
    client.publish("homeassistant/light/led1_pwm/availability", "online", retain=True)

def on_message(client, userdata, msg):
    topic = msg.topic
    payload = msg.payload.decode('utf-8')
    
    if topic == "homeassistant/number/motor_pwm/set":
        # Без промяна (мотор логика)
        try:
            value = float(payload)
            if INVERT_MOTOR:
                if value == 0:
                    pwm_percent = 100
                else:
                    pwm_percent = MOTOR_MIN_PWM - (value * MOTOR_MIN_PWM / 100)
            else:
                pwm_percent = value
            pwm_value = int(pwm_percent / 100 * 4095)
            pca.channels[MOTOR_CH].duty_cycle = pwm_value
            client.publish("homeassistant/number/motor_pwm/state", str(value))
            print(f"Set motor PWM to {pwm_percent}% (value: {value})")
        except Exception as e:
            print(f"Error setting motor: {e}")
    
    elif topic == "homeassistant/light/led0_pwm/set":
        # Без промяна (LED0 PWM)
        try:
            data = json.loads(payload)
            state = data.get('state')
            if state == 'ON':
                brightness = data.get('brightness', 255)
                pwm_value = int((brightness / 255) * 4095)
                pca.channels[LED0_CH].duty_cycle = pwm_value
                client.publish("homeassistant/light/led0_pwm/state", json.dumps({"state": "ON", "brightness": brightness}))
            elif state == 'OFF':
                pca.channels[LED0_CH].duty_cycle = 0
                client.publish("homeassistant/light/led0_pwm/state", json.dumps({"state": "OFF"}))
        except Exception as e:
            print(f"Error setting LED0: {e}")
    
    elif topic == "homeassistant/light/led1_pwm/set":
        try:
            data = json.loads(payload)
            state = data.get('state')
            brightness = data.get('brightness', 255)
            effect = data.get('effect', 'solid')
            
            global current_brightness_led1, blink_thread
            current_brightness_led1 = brightness
            
            if state == 'OFF':
                stop_blinking()
                pca.channels[LED1_CH].duty_cycle = 0
                client.publish("homeassistant/light/led1_pwm/state", json.dumps({"state": "OFF"}))
            elif state == 'ON':
                if effect == 'blink':
                    stop_blinking()  # Спря предишния, ако има
                    blink_thread = threading.Thread(target=start_blinking)
                    blink_thread.start()
                elif effect == 'solid':
                    stop_blinking()
                    pwm_value = int((brightness / 255) * 4095)
                    pca.channels[LED1_CH].duty_cycle = pwm_value
                client.publish("homeassistant/light/led1_pwm/state", json.dumps({"state": "ON", "brightness": brightness, "effect": effect}))
        except Exception as e:
            print(f"Error setting LED1: {e}")

client.on_connect = on_connect
client.on_message = on_message

client.connect(MQTT_HOST, MQTT_PORT, 60)
client.loop_forever()
