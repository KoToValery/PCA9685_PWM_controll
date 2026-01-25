
FROM homeassistant/aarch64-base-python:3.12


RUN apt-get update && apt-get install -y --no-install-recommends \
    gcc \
    libc-dev \
    linux-libc-dev \
    i2c-tools \
    python3-dev \
    && rm -rf /var/lib/apt/lists/*


RUN pip install --no-cache-dir \
    adafruit-circuitpython-pca9685 \
    paho-mqtt

COPY run.py /
CMD [ "python", "-u", "/run.py" ]
