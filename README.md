# Hyggelig IoT Modbus

ESP32-gatewayen læser temperatur og luftfugtighed fra DHT11-enheden via Modbus TCP og aflæser MQ-135 på GPIO34. Gatewayen sender målingerne som Sparkplug B over MQTT/TLS. Gatewayen er edge node `esp32-01` og sender MQ-135-målinger som `NDATA`. DHT11 er enhed `esp32-02`, og dens målinger sendes som `DDATA` efter `DBIRTH`. Firmwaren til DHT11-serveren ligger på branchen `ModBus-Server`.

Kopiér `src/wifi_local.example.h` til `src/wifi_local.h`, og udfyld gatewayens Wi‑Fi-oplysninger før upload med PlatformIO. Indstillingerne for det lokale adgangspunkt skal matche DHT11-serverens. Git ignorerer `src/wifi_local.h`.

Gatewayen deler én 2,4 GHz-radio mellem hotspottet og det lokale Modbus-netværk. Ved opstart søger den efter det konfigurerede hotspot. Hvis den finder det, bruger det lokale adgangspunkt samme kanal. Den serielle log viser genforbindelsesforsøg og fejlkoder ved afbrydelser. Wi‑Fi-forbindelsen kan stadig være ustabil, så der kan være huller mellem nye målinger i QuestDB.
