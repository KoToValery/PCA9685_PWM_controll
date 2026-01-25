
FROM python:3.11-slim


# Инсталираме само абсолютно необходимите build tools и i2c-tools
RUN apt-get update && apt-get install -y --no-install-recommends \
    build-essential \
    python3-dev \
    i2c-tools \
    && rm -rf /var/lib/apt/lists/*

# Инсталираме Python библиотеките
RUN pip install --no-cache-dir \
    adafruit-circuitpython-pca9685 \
    adafruit-blinka \
    paho-mqtt \
    rpi.gpio

# Настройки за Blinka
ENV BLINKA_FORCE_BOARD=RASPBERRY_PI_5
ENV BLINKA_FORCE_CHIP=BCM2712
# Важно: Задаваме generic linux I2C, за да избегнем libgpiod dependency
ENV BLINKA_FORCE_I2C=1

COPY run.py /
CMD [ "python3", "-u", "/run.py" ]
