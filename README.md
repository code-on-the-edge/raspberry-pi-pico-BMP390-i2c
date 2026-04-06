# Raspberry Pi Pico BMP390 i2c

# Raspberry Pi Pico 1 and 2 Setup

```bash
# Install Pico SDK
mkdir ~/pico
cd ~/pico
git clone https://github.com/raspberrypi/pico-sdk.git --branch master
cd pico-sdk
git submodule update --init
cd ..
git clone https://github.com/raspberrypi/pico-examples.git --branch master

# Install dependencies
sudo apt update
sudo apt install cmake gcc-arm-none-eabi libnewlib-arm-none-eabi build-essential
sudo apt install g++ libstdc++-arm-none-eabi-newlib

# Configure environment
echo "export PICO_SDK_PATH=$HOME/pico-sdk" >> ~/.bashrc
source ~/.bashrc
```
# Picotool
```bash
# Clone Repo
git clone https://github.com/raspberrypi/picotool
cd picotool

# Build Picotool
mkdir build
cd build
cmake ..
make

# Check Installation
./picotool help
```

# Flash with Picotool
```bash
cd path/to/project/folder/
mkdir build
cd build
cmake ..
make
sudo /home/$USER/picotool/build/picotool load -f bmp390.uf2
```
# References
Getting started with Raspberry Pi Pico-series
* https://pip-assets.raspberrypi.com/categories/610-raspberry-pi-pico/documents/RP-008276-DS-1-getting-started-with-pico.pdf?disposition=inline

Picotool
* https://github.com/raspberrypi/picotool
