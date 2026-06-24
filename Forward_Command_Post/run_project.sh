#!/bin/bash

xhost +local:docker > /dev/null 2>&1

docker compose up -d
