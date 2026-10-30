# Local START/END segment markers (mainv4)

The active dashboard is robot_debug.py (Tkinter / Bluetooth Classic SPP).

During an active CSV run, click MOC START [1] or press 1 to mark the start of a difficult segment. Click MOC END [0] or press 0 for its end. Repeat alternating START and END as often as needed in one run. These marker controls do not start or stop the robot.

Markers are recorded locally by the Python application via the existing CSV writer queue. No MARK command is sent to the ESP32. No firmware ACK, LED indication, or firmware reflash is necessary. Telemetry from ESP32 continues independently.

CSV columns: event_type, event_id, segment_id, event_status. Accepted marker rows have event_status=LOCAL and a host_iso timestamp from the laptop. Device t_ms and encoder fields are blank on marker rows because the event is not measured on-device. Telemetry rows retain their original t_ms, encoder, ADC, and PID values. Analyze marked windows using host_iso, allowing for Bluetooth receive latency.

Markers require an active CSV recording session. Invalid duplicate START or END commands are ignored. CSV writer flushes periodically and closes on STOP. If the queue rejects a marker, the application reports a failure and does not advance the segment counter.

Keyboard shortcuts work when the dashboard has focus and text-entry widgets do not have focus.
