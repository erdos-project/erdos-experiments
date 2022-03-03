#!/bin/bash

# Install Rust
curl --proto '=https' --tlsv1.2 -sSf https://sh.rustup.rs | sh

# Install docker, maven
sudo apt install docker.io docker-compose maven

# Install pandas
pip3 install pandas