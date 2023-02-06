# HelloRestCar2
Requirements:

-Arduino Uno WiFi REV2
-4tronix Initio 4WD Robocar
-Library:  WiFiNINA, NewPing, ArduinoHttpClient, SPI



Add SSID.h with your ssid information:

#define SSID      "Your gateway id"

#define KEY       "Your gateway WPA pass"

Description:

Car makes http-requests to webserver for fetching actions it's going to execute next.
It must implement all actions which are defined on web server backend client api.
Default implementation fo webserver is visible on https://robo.sukelluspaikka.fi
