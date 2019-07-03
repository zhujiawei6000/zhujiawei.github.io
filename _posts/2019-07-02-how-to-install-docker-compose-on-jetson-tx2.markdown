---
layout: post
title:  "How to Install Docker Compose on Jetson TX2"
date:   2019-06-28 15:31:00
categories: main
brief: "This is my approach to install docker compose on Jetson TX2."
---

# Prerequisite
JetPack 3.3 or above
    
# Install Docker
1. install dependencies
```shell
sudo apt-get update
sudo apt-get install \
    apt-transport-https \
    ca-certificates \
    curl \
    gnupg-agent \
    software-properties-common
```
2. add apt-get source
```shell
curl -fsSL https://download.docker.com/linux/ubuntu/gpg | sudo apt-key add -
sudo apt-key fingerprint 0EBFCD88
sudo add-apt-repository \
"deb [arch=arm64] https://download.docker.com/linux/ubuntu \
    $(lsb_release -cs) \
    stable"
```
3. install
```shell
sudo apt-get update
sudo apt-get install docker-ce docker-ce-cli containerd.io
```
4. add user to docker group (optional)
```shell
sudo usermod -aG docker nvidia
```
    
# Install Docker Compose
1. install dependencies
```shell
sudo apt install python-pip libffi-dev libssl-dev
```
Notes: if you encounter `cannot locate python-pip` error, this may due to missing some apt-get source. You can edit /etc/apt/source.list, and append 'universe multiverse' to each line contains url.
2. install
```shell
sudo pip install docker-compose
```


