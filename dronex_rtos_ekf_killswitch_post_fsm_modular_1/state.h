#pragma once

enum FlightState {
  STATE_BOOT,
  STATE_STANDBY,
  STATE_ARMED_TAKEOFF,
  STATE_FLYING,
  STATE_KILLED
};

extern volatile FlightState current_state;
extern volatile bool emergency_kill;
extern volatile bool is_armed;
extern volatile bool is_flying;
extern volatile bool calibrate_done;

void setFlightState(FlightState new_state);