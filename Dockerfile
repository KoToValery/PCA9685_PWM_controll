FROM python:3.11-slim

# I2C tooling + препоръчани зависимости за Blinka на Bookworm/RPi5
RUN apt-get update && apt-get install -y --no-install-recommends \
    i2c-tools \
    libgpiod-dev \
    python3-libgpiod \
  && rm -rf /var/lib/apt/lists/*

# Python libs (без venv, без build-essential)
RUN pip install --no-cache-dir \
    adafruit-blinka \
    adafruit-circuitpython-pca9685 \
    paho-mqtt \
    rpi-lgpio \
  && pip uninstall -y RPi.GPIO || true

# ВАЖНО: правилните имена на env променливите (без underscore)
ENV BLINKA_FORCEBOARD=RASPBERRY_PI_5
ENV BLINKA_FORCECHIP=BCM2712

COPY run.py /
CMD ["python3", "-u", "/run.py"]
