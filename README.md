# Lorawan Node For Raspberry Pi with TTN Mapper

While LoraWan nodes for ttn Mapper are readily available for Arduino, They are hard to come by for Rasperry Pi. This code here is based on [LoRaWAN-device-GPS-Raspberry-Pi](https://github.com/thomaswesenberg/LoRaWAN-device-GPS-RaspBerry-Pi) specially the part [How to configure a RaspBerry Pi Zero](https://github.com/thomaswesenberg/LoRaWAN-device-GPS-RaspBerry-Pi/blob/master/raspberry-setup.txt) and the examples from [Ernst de vreede](https://github.com/ernstdevreede/lmic_pi). I used RFM95W and hooked it up with wires just as described in [this page](https://www.hackster.io/ChrisSamuelson/lora-raspberry-pi-single-channel-gateway-cheap-d57d36).


Please note that you need a file called myconfig.h with your DevEUI, NwkSKey, AppSKey, DevAddr
