# Humanoid pilot control sequence

This runnable sequence animates the G1+O6 humanoid through three control states: ready on controls, left hand pushing the throttle forward, and right hand deflecting the control stick.

- Source model: `models\derived\pilot\g1_o6_cockpit_action.xml`
- Duration: 3.00 s
- Frames: 72
- GIF: `reports\screenshots\pilot_control_sequence\pilot_control_sequence.gif`
- Throttle travel: 0.120 m
- Stick pitch delta: -0.220 rad
- Stick roll delta: -0.120 rad
- Max hand/control tracking error: 0.0097 m

The sequence is deterministic kinematic playback from IK keyframes. It is suitable for visualization and integration tests; forceful contact-control policy training remains a separate task.
