
FROM python:3.11-slim


# Инсталиране на системни dependencies за Raspberry Pi 5
RUN apt-get update && apt-get install -y --no-install-recommends \
    gcc \
    libc-dev \
    linux-libc-dev \
    i2c-tools \
    python3-dev \
    libgpiod2 \
    libgpiod-dev \
    && rm -rf /var/lib/apt/lists/*

# Инсталиране на Python библиотеки
RUN pip install --no-cache-dir \
    adafruit-circuitpython-pca9685 \
    adafruit-blinka \
    paho-mqtt

# Форсиране на Board тип за Raspberry Pi 5 (заобикаля хардуерна детекция в Docker)
ENV BLINKA_FORCE_BOARD=RASPBERRY_PI_5
ENV BLINKA_FORCE_CHIP=BCM2712

COPY run.py /
CMD [ "python3", "-u", "/run.py" ]
