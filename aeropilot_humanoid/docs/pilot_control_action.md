# Humanoid pilot-control action scene

This scene combines the sourced Unitree G1 humanoid, dual LinkerHand O6 hands, and the passive cockpit stick/throttle mechanisms into a keyframed control-action storyboard.

- Scene: `models\scenes\pilot_control_action.xml`
- Model: `models\derived\pilot\g1_o6_cockpit_action.xml`
- Local FBX reference: `3d66.com_JDH5455235936.fbx`
- Keyframes: `pilot_ready_on_controls, left_hand_pushes_throttle_forward, left_thumb_toggles_throttle_switch, right_hand_deflects_control_stick`
- Maximum hand-to-control target error: 0.0097 m
- Validation threshold: 0.0300 m
- Screenshots: `reports\screenshots\pilot_action\01_pilot_ready_on_controls.png, reports\screenshots\pilot_action\02_left_hand_pushes_throttle_forward.png, reports\screenshots\pilot_action\03_left_thumb_toggles_throttle_switch.png, reports\screenshots\pilot_action\04_right_hand_deflects_control_stick.png`

The FBX file is converted into split OBJ meshes and imported as articulated MuJoCo bodies. The generated keyframes drive the converted model's throttle and stick joints directly. The result proves a centimetre-scale geometric action pose, not a trained contact-rich grasping controller.
