class _Srv(object):
    pass


# the service types robot_stream.REMOTE asks for (hal's read-only services)
battery_read = board_read = ext_read = led_reply = move_status = servo_offset_read = servo_read = value_get = \
    version_read = _Srv
