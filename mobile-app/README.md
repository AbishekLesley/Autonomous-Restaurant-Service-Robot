# Restaurant Robot React UI - V21 Ultrasonic Obstacle

Adds:
- Staff obstacle card with front/left/right readings.
- Robot icon red boundary glow while obstacle stop/help is active.
- Bypass for current journey from Staff page.
- Technical page ultrasonic sensor enable/disable and bypass tools.
- Technical page Hall read/update buttons.

Current repository integration:
- ESP32: `../firmware/esp32/esp32_robot_api_led_control_v13.ino`
- STM32: `../firmware/stm32/Core/Src/main.c`


## V24 LED control fix

The LED modal now keeps local slider, color, brightness, speed, pattern, zone, and link-side edits while the dashboard continues polling `/api/status`. Press **Apply settings** to send the selected values to the ESP32. Background status refreshes no longer reset the controls to defaults.
