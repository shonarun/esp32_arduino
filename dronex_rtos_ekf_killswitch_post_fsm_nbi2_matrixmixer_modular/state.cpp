#include "state.h"
#include <Arduino.h>

volatile FlightState current_state = STATE_BOOT;
volatile bool emergency_kill = true; 
volatile bool is_armed = false;
volatile bool is_flying = false;
volatile bool calibrate_done = false;

void setFlightState(FlightState new_state) {
    if (current_state == new_state) return;

    current_state = new_state;
    Serial.print("SYSTEM STATE CHANGED TO: ");
    switch (current_state) {
        case STATE_BOOT:          Serial.println("BOOT"); break;
        case STATE_STANDBY:       Serial.println("STANDBY"); break;
        case STATE_ARMED_TAKEOFF: Serial.println("ARMED_TAKEOFF"); break;
        case STATE_FLYING:        Serial.println("FLYING"); break;
        case STATE_KILLED:        Serial.println("KILLED"); break;
    }
}