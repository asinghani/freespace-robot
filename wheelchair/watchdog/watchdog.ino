
// Startup signal = 431
// Normal comms = 66

#define CAN_H A1
#define RELAY 8
#define SIGNAL_LED 13

#define ACTIVITY_THRESH 35 // < 35 for more than 5 seconds = shutoff
#define TIMEOUT_MS 300

#define STARTUP_THRESH 120 // > 120 = startup
#define STARTUP_END_THRESH 100 // < 100 = safe

unsigned long lastBlink = 0;

void setup() {
    Serial.begin(9600);
    pinMode(RELAY, OUTPUT);
    pinMode(SIGNAL_LED, OUTPUT);

    while(true) {
        // Disabled state
        digitalWrite(RELAY, LOW);
        digitalWrite(SIGNAL_LED, HIGH);

        while(analogRead(CAN_H) < STARTUP_THRESH) {// Wait until startup
          //Serial.println(analogRead(CAN_H));
          //delay(3);
        }
        while(analogRead(CAN_H) > STARTUP_END_THRESH); // Wait until startup ended



        // Enabled state
        digitalWrite(RELAY, HIGH);
        unsigned long lastActive = millis();

        while(true) {
            if(analogRead(CAN_H) > ACTIVITY_THRESH) {
                lastActive = millis();
            }
            
            if(millis() - lastActive > TIMEOUT_MS) {
                break;
            }

            // Blink LED
            if(millis() - lastBlink > 250) {
                digitalWrite(SIGNAL_LED, !digitalRead(SIGNAL_LED));
                lastBlink = millis();
            }
        }
    }
}

void loop() { }
