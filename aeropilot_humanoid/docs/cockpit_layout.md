# Adjustable cockpit mechanism seed

Phase 4 creates an independently testable seat, two-axis passive centering stick, non-centering linear throttle, opposed rudder pedals, independent toe brakes, heel supports, platform, and overview camera.

- Passive simulation: 5.002 s
- Joints / actuators / equalities: 7 / 0 / 1
- Behavior checks: `{"brakes_release": true, "finite_state": true, "rudder_remains_opposed": true, "rudder_returns_to_center": true, "stick_returns_to_center": true, "throttle_holds_position": true}`

All layout values are search seeds/ranges, not a final reachability claim. Phase 5 must optimize them against G1+O6 joint margins and collisions.
