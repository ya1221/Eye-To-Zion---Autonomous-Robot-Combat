FROM python:3.11-slim-bookworm

ENV PYTHONUNBUFFERED=1

COPY requirements.txt /tmp/requirements.txt
RUN pip3 install --no-cache-dir -r /tmp/requirements.txt

WORKDIR /app

# Copy application files (overridden by volume mount in compose for live edits)
COPY . /app/

ENTRYPOINT ["python3", "train.py"]
