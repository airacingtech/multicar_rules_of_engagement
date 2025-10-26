# Laguna Seca Multicar Rules of Engagement
The goal is to exercise multi-car passing where every vehicle runs active cruise control and follows negotiated speed profiles without race control intervention.

## Summary of the Rules
- Passing is allowed only on certified straights that provide enough lateral clearance for both cars to hold their lanes.
- Race control will coordinate passes between two vehicles: an attacker and a defender.
- **If any vehicle reports `ct_state = CT_STATE_EMERGENCY_SHUTDOWN`, every car within radio reach must come to an immediate stop until that state is clear.**[^race-control]

## AVST Position message
```Python
std_msgs/Header header        # Standard ROS header (stamp drives relative timing)
uint8   car_id                # Car ID [ - ]
uint8   heartbeat             # Rolling heartbeat counter; drop rate exposes link quality [ - ]

float64 lat                   # Vehicle longitude, VectorNav vehicle frame origin (rear axle centre) [ dd.dd ]
float64 lon                   # Vehicle latitude, VectorNav vehicle frame origin (rear axle centre) [ dd.dd ]
float32 alt                   # Vehicle altitude (ellipsoid), VectorNav vehicle frame origin (rear axle centre) [ m ]
float32 heading               # Vehicle heading, GPS style, North = 0, East = 90 [ deg ]
float32 v_north               # North velocity from VN INS solution (ENU frame) [ m/s ]
float32 v_east                # East velocity from VN INS solution (ENU frame) [ m/s ]
float32 v_up                  # Up velocity from VN INS solution (ENU frame) [ m/s ]
uint32 gps_tow_ms             # GPS time-of-week in milliseconds (GNSS1 output from VN-310) [ ms ]
int8    ct_state              # Vehicle intent aligned with expanded raptor ct_state enumerations [ enum ]

# Vehicle state constants
int8 CT_STATE_UNKNOWN = 0
int8 CT_STATE_CAUTION = 8
int8 CT_STATE_NOMINAL = 9
int8 CT_STATE_SAFE_STOP = 10
int8 CT_STATE_EMERGENCY_SHUTDOWN = 12
int8 CT_STATE_DEFENDING_STANDBY = 20
int8 CT_STATE_DEFENDING_ENGAGED = 21
int8 CT_STATE_ATTACKING_CATCHUP = 25
int8 CT_STATE_ATTACKING_OVERTAKING = 26
```

### Implementation guidance
- Feed `lat`/`lon`/`alt`/`heading` and `v_north`/`v_east`/`v_up` directly from the VectorNav INS ENU outputs, using an identical configuration across teams at 100 Hz to keep trajectories aligned even when GNSS quality dips.
- Down-convert VN-310 GPS time-of-week to milliseconds for `gps_tow_ms`; a 1 ms resolution still yields ≤10 cm temporal alignment error at 100 m/s.
- Publish the AVST message on the same 100 Hz schedule for all cars so perception teams can treat it as a ground-truth baseline.
- Use the ct_state enumerations above (caution, nominal, safe stop, emergency shutdown, defending standby/engaged, attacking catchup/overtaking); treat values outside this set as `CT_STATE_UNKNOWN`. Reference: https://docs.google.com/spreadsheets/d/1k7i-BM3IfjrjvFqjKOQNnPPz1IUEVOhj/edit?usp=sharing&ouid=115064437266663800799&rtpof=true&sd=true

### Emergency stop coordination
- Emergency stops trigger only on a transition into `CT_STATE_EMERGENCY_SHUTDOWN`; each listener latches the initiating `car_id` and message `header.stamp` as the stop event.
- Vehicles remain stopped until they receive a newer message from that initiator reporting a non-emergency state, or an explicit clear directive.
- Messages that repeat the same stop event after the clear are ignored so delayed packets do not cause phantom stops.

[^race-control]: The AVST flow assumes an active human race control monitoring the event. Race control must be prepared to halt the field if the transponder system fails and to red-stop trailing cars when a leading transponder-equipped vehicle leaves the track or stops unexpectedly. Teams should evaluate additional edge cases and recognise the system's limitations; they may run their own perception stacks, but cannot assume that other entrants do so. Participation requires a working ACC that respects the transponder-provided distances.
