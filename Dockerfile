FROM python:3.11-slim

# Инсталирай системни зависимости за I2C и компилация
RUN apk add --no-cache \
    gcc \
    musl-dev \
    linux-headers \
    i2c-tools

# Инсталирай Python пакети
RUN pip install --no-cache-dir \
    adafruit-circuitpython-pca9685 \
    paho-mqtt

COPY run.py /
CMD [ "python", "-u", "/run.py" ]
