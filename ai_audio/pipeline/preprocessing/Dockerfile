FROM python:3.11-slim-bookworm

ENV PYTHONUNBUFFERED=1

# librosa's STFT/mel path needs a C compiler for a couple of its deps on
# arm64 wheels that don't ship prebuilt, plus libsndfile for its I/O backend
RUN apt-get update && apt-get install -y \
    build-essential \
    libsndfile1 \
    && rm -rf /var/lib/apt/lists/*

COPY requirements.txt /tmp/requirements.txt
RUN pip3 install --no-cache-dir -r /tmp/requirements.txt

WORKDIR /app

# Copy application files (overridden by volume mount in compose for live edits)
COPY . /app/

ENTRYPOINT ["python3", "make_features.py"]
