# Hyggelig IoT Modbus

The ESP32 gateway reads temperature and humidity from the DHT11 ESP32 over Modbus TCP and samples the MQ-135 on GPIO34. It publishes Sparkplug B over MQTT/TLS. The gateway is edge node `esp32-01` and sends MQ-135 readings as `NDATA`. The DHT11 is device `esp32-02` and its readings are sent as `DDATA` after `DBIRTH`. The DHT11 server firmware is on the `ModBus-Server` branch.

Copy `src/wifi_local.example.h` to `src/wifi_local.h` and set the gateway's Wi-Fi details before uploading with PlatformIO. The local AP details must match the DHT11 server. Git ignores `src/wifi_local.h`.
