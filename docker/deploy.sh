#!/usr/bin/env bash

# See https://docs.docker.com/v17.12/docker-cloud/builds/push-images/

sudo docker login
sudo docker build -t hydro .
sudo docker tag hydro diceengineering/hydro:latest
sudo docker push diceengineering/hydro
