#!/bin/sh

# 1. Start InfluxDB in the background (&)
influxdb3 serve --node-id=node0 --object-store=file --data-dir=/var/lib/influxdb3/data &
PID=$!

# 2. Wait 5 seconds for the server to wake up
echo "Waiting for InfluxDB to start..."
sleep 5

# 3. Create the Database automatically
echo "Creating 'sensor' database..."
influxdb3 database create sensor

# 4. Keep the container running
wait $PID