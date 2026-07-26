# LinkerHand O6 joint map

Both hands were parsed independently from their official URDFs. They are not mirrored derivatives.

## Left O6

- Root link: `lh_hand_base_link`
- Links / joints: 12 / 11
- Independent inputs / passive mimic joints: 6 / 5
- URDF mass: 0.113856113 kg
- Raw MuJoCo dynamic mass: 0.046562589 kg

| Joint | Parent | Child | Axis | Range (rad) | Role / coupling |
|---|---|---|---|---|---|
| lh_thumb_cmc_yaw | lh_hand_base_link | lh_thumb_metacarpals_base2 | [0.0, 0.0, 1.0] | [0.0, 1.3] | independent_input |
| lh_thumb_cmc_pitch | lh_thumb_metacarpals_base2 | lh_thumb_metacarpals | [0.0, 1.0, 0.0] | [0.0, 0.58] | independent_input |
| lh_thumb_ip | lh_thumb_metacarpals | lh_thumb_distal | [0.0, 1.0, 0.0] | [0.0, 1.08] | passive_mimic → lh_thumb_cmc_pitch × 2.29 |
| lh_index_mcp_pitch | lh_hand_base_link | lh_index_proximal | [0.0, 1.0, 0.0] | [0.0, 1.6] | independent_input |
| lh_index_dip | lh_index_proximal | lh_index_distal | [0.0, 1.0, 0.0] | [0.0, 1.43] | passive_mimic → lh_index_mcp_pitch × 0.89 |
| lh_middle_mcp_pitch | lh_hand_base_link | lh_middle_proximal | [0.0, 1.0, 0.0] | [0.0, 1.6] | independent_input |
| lh_middle_dip | lh_middle_proximal | lh_middle_distal | [0.0, 1.0, 0.0] | [0.0, 1.43] | passive_mimic → lh_middle_mcp_pitch × 0.89 |
| lh_ring_mcp_pitch | lh_hand_base_link | lh_ring_proximal | [0.0, 1.0, 0.0] | [0.0, 1.6] | independent_input |
| lh_ring_dip | lh_ring_proximal | lh_ring_distal | [0.0, 1.0, 0.0] | [0.0, 1.43] | passive_mimic → lh_ring_mcp_pitch × 0.89 |
| lh_pinky_mcp_pitch | lh_hand_base_link | lh_pinky_proximal | [0.0, 1.0, 0.0] | [0.0, 1.6] | independent_input |
| lh_pinky_dip | lh_pinky_proximal | lh_pinky_distal | [0.0, 1.0, 0.0] | [0.0, 1.43] | passive_mimic → lh_pinky_mcp_pitch × 0.89 |

## Right O6

- Root link: `rh_hand_base_link`
- Links / joints: 12 / 11
- Independent inputs / passive mimic joints: 6 / 5
- URDF mass: 0.113956201 kg
- Raw MuJoCo dynamic mass: 0.046555422 kg

| Joint | Parent | Child | Axis | Range (rad) | Role / coupling |
|---|---|---|---|---|---|
| rh_thumb_cmc_yaw | rh_hand_base_link | rh_thumb_metacarpals_base2 | [0.0, 0.0, -1.0] | [0.0, 1.36] | independent_input |
| rh_thumb_cmc_pitch | rh_thumb_metacarpals_base2 | rh_thumb_metacarpals | [0.0, 1.0, 0.0] | [0.0, 0.58] | independent_input |
| rh_thumb_ip | rh_thumb_metacarpals | rh_thumb_distal | [0.0, 1.0, 0.0] | [0.0, 1.08] | passive_mimic → rh_thumb_cmc_pitch × 1.86 |
| rh_index_mcp_pitch | rh_hand_base_link | rh_index_proximal | [0.0, 1.0, 0.0] | [0.0, 1.6] | independent_input |
| rh_index_dip | rh_index_proximal | rh_index_distal | [0.0, 1.0, 0.0] | [0.0, 1.43] | passive_mimic → rh_index_mcp_pitch × 0.89 |
| rh_middle_mcp_pitch | rh_hand_base_link | rh_middle_proximal | [0.0, 1.0, 0.0] | [0.0, 1.6] | independent_input |
| rh_middle_dip | rh_middle_proximal | rh_middle_distal | [0.0, 1.0, 0.0] | [0.0, 1.43] | passive_mimic → rh_middle_mcp_pitch × 0.89 |
| rh_ring_mcp_pitch | rh_hand_base_link | rh_ring_proximal | [0.0, 1.0, 0.0] | [0.0, 1.6] | independent_input |
| rh_ring_dip | rh_ring_proximal | rh_ring_distal | [0.0, 1.0, 0.0] | [0.0, 1.43] | passive_mimic → rh_ring_mcp_pitch × 0.89 |
| rh_pinky_mcp_pitch | rh_hand_base_link | rh_pinky_proximal | [0.0, 1.0, 0.0] | [0.0, 1.6] | independent_input |
| rh_pinky_dip | rh_pinky_proximal | rh_pinky_distal | [0.0, 1.0, 0.0] | [0.0, 1.43] | passive_mimic → rh_pinky_mcp_pitch × 0.89 |

## Integration finding

Raw URDF compilation succeeds, but the fixed root is fused into the world and its inertia is not retained as an attachable rigid body. The root-preserving Phase 2 conversion must also recreate all five mimic constraints per hand. The asymmetric thumb multipliers (left 2.29, right 1.86) must not be mirrored or averaged.
