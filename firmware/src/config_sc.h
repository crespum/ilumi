/*
  ===========================================================================================================================================
                                            Modify all parameters below to suit you environment
  ===========================================================================================================================================
*/
int kRetain = 0;                                              // Retain mqtt messages (0 for off, 1 for on)
int kUpdFreq = 1;                                             // Update frequency in Mintes to check for mqtt connection. Defualt 1 min.
int kRetries = 10;                                            // WiFi retry count (10 default). Increase if not connecting to your WiFi.
int QOS = 0;                                                  // QOS level for all mqtt messages. (0 or 1)

#define NONE                                                  // Set to NONE, TEMP, or WS (Cannot be blank)
                                                              // NONE for standard Sonoff relay only ON / OFF (default)
                                                              // TEMP for DHT11/22 Support on Pin 5 of header (GPIO 14)
                                                              // WS for External Wallswitch Support on Pin 5 of header (GPIO 14)

#define MQTT_SERVER      "192.168.0.100"                      // Your mqtt server ip address
#define MQTT_PORT        1883                                 // Your mqtt port
#define MQTT_TOPIC       "home/ring/living_room/1/#"            // Base mqtt topic
#define MQTT_USER        "secret"                          // mqtt username
#define MQTT_PASS        "secret"                          // mqtt password

#define WIFI_SSID        "secret"                           // Your WiFi ssid
#define WIFI_PASS        "secret"                      // Your WiFi password
/*
  ===========================================================================================================================================
*/
