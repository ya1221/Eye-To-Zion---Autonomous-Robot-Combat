FROM python:3.11-slim-bookworm

ENV PYTHONUNBUFFERED=1

# libportaudio2 is the runtime lib sounddevice binds to; alsa-utils gives us
# arecord -l inside the container for device debugging
RUN apt-get update && apt-get install -y \
    libportaudio2 \
    alsa-utils \
    && rm -rf /var/lib/apt/lists/*

COPY requirements.txt /tmp/requirements.txt
RUN pip3 install --no-cache-dir -r /tmp/requirements.txt

WORKDIR /app

# Copy application files (overridden by volume mount in compose for live edits)
# trigger.py here is an intentional self-contained duplicate of
# ../../onboard/audio_processor/trigger.py -- see that file's docstring.
COPY . /app/

ENTRYPOINT ["python3", "record_impacts.py"]
