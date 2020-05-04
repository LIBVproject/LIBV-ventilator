# LIBV Open Source Ventilator

Check out our [website](https://www.bvmvent.org/) for updates.


### Sending Data to IoT Module
```
// Example of sending data to IoT module
// See https://arduinojson.org/ for more info on ArduinoJson

// Library: ArduinoJson by Benoit Blanchon
#include <ArduinoJson.h>

void setup() {
    Serial1.begin(9600);    // To IoT module
    while (!Serial1) continue;
}

void loop() {
    StaticJsonDocument<200> doc;     // Set to number of bytes big enough to hold the data
    doc["uptime_hours"] = 0.1;
    serializeJson(doc, Serial1);
}
```