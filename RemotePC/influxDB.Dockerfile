FROM influxdb:3-core

# Switch to root temporarily to create the directory structures
USER root

# Pre-create the directory and explicitly assign ownership to InfluxDB's non-root user (1500)
RUN mkdir -p /var/lib/influxdb3/data && \
    chown -R 1500:1500 /var/lib/influxdb3/data

# Switch back to the safe, unprivileged user context
USER 1500   