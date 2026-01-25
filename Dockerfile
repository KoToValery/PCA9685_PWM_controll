# Използваме Debian версията на HA base image (Bookworm)
# Това е важно, за да работи apt-get командата ти
FROM homeassistant/aarch64-base-debian:bookworm

# Инсталираме Python и dependencies
# Тъй като това е "base" image, а не "python" image, трябва да инсталираме Python изрично
RUN apt-get update && apt-get install -y --no-install-recommends \
    python3 \
    python3-pip \
    python3-venv \
    python3-dev \
    build-essential \
    i2c-tools \
    libi2c-dev \
    && rm -rf /var/lib/apt/lists/*

# Създаваме виртуална среда (задължително за Debian 12+)
ENV VIRTUAL_ENV=/opt/venv
RUN python3 -m venv $VIRTUAL_ENV
ENV PATH="$VIRTUAL_ENV/bin:$PATH"

# Инсталираме Python библиотеките във venv-а
RUN pip install --no-cache-dir \
    adafruit-circuitpython-pca9685 \
    adafruit-blinka \
    paho-mqtt \
    rpi.gpio

# Настройки за Blinka (Pi 5)
ENV BLINKA_FORCE_BOARD=RASPBERRY_PI_5
ENV BLINKA_FORCE_CHIP=BCM2712
ENV BLINKA_FORCE_I2C=1

COPY run.py /
CMD [ "python3", "-u", "/run.py" ]
