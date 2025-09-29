# msg/leaf_health_events.msg
uint64 timestamp                # [us]
uint32 remote_event_id          # MAVLink EVENT.id from the inbound message
uint8  ext_severity             # lower 4 bits of MAVLink EVENT.log_levels (0..7)
uint8[5] arguments              # first 5 bytes of EVENT.arguments (enough to display context)
