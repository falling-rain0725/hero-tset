# Vision transport and frame layout

This document records the low-level interface and the packet layouts provided to this module. The application layer is intentionally not migrated in this change.

## Controller to vision

- Command ID: `0x0002`; all multi-byte integers and floats are little-endian.
- `data_len` is `46` (`11 * 4 + 2`); total frame size is `54` bytes.
- Frame layout: SOF `0xA5`, data length, CRC8 over the first 3 bytes, command ID, two flag bytes, 11 floats, CRC16.
- Float order: `yaw`, `pitch`, `roll`, reserved zero, reserved zero, `yaw_vel`, `pitch_vel`, `game_progress`, `current_HP`, `current_base_hp`, `current_enemy_base_hp`.
- Flags byte 0: self-color in bits 0–3 and work mode in bits 4–7. Flags byte 1: bullet speed.
- CRC16 covers bytes 0–51; its two result bytes occupy offsets 52–53.

`VisionSend(const Vision_Send_s *data)` accepts the semantic values. It inserts the two reserved zero floats, encodes the fields, builds the frame, and calls the selected UART or USB transport.

## Vision to controller

- Command ID: `0x0001`; `data_len` is `34`; total frame size is `42` bytes.
- Float order: `pitch`, `yaw`, `delta_pitch`, `delta_yaw`, `target_distance`, `vel_x`, `vel_y`, `angle_yaw`.
- The target state is carried in the low four bits of flags byte 0: 0 no target, 1 converging, 2 ready to fire.
- CRC8 covers the first 3 bytes. CRC16 covers bytes 0–39; its two result bytes occupy offsets 40–41.

The stream parser accepts arbitrary input chunks, validates framing and checksums, requires the expected command and payload size, and publishes a receive structure only when all eight floats are finite. `VisionGetLatest()` returns the most recent valid structure and reports daemon online state.

## Integration status

The existing application callers still use the previous setter-based `VisionSend` interface and need to migrate separately to the pointer-based data structure. This change updates only `modules/master_machine`.
