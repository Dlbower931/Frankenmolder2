# Pi Pico 2W Setup for Injection Molding Machine

This guide explains how to set up the Pi Pico 2W for high-frequency communication with the Raspberry Pi in your injection molding machine.

## Hardware Connections

### Pi Pico 2W Pinout for Extruder Control (Minimal)

```
GPIO 2  -> Extruder Motor Control (PWM)
LED     -> Status LED (built-in)
```

### Power and Communication
- **USB-C**: Connect to Raspberry Pi for serial communication
- **3.3V Power**: Can be powered via USB or external 3.3V supply
- **Ground**: Connect to common ground with Raspberry Pi

## Software Setup

### 1. Install MicroPython on Pi Pico 2W

1. Download the latest MicroPython firmware for Pi Pico 2W from [micropython.org](https://micropython.org/download/RPI_PICO_W/)
2. Hold BOOTSEL button while connecting USB to enter bootloader mode
3. Copy the `.uf2` file to the mounted drive
4. The Pi Pico will restart with MicroPython installed

### 2. Upload the Firmware

1. Connect Pi Pico 2W to your computer via USB
2. Use a MicroPython IDE (like Thonny) to upload `pico_firmware/main.py`
3. Save it as `main.py` on the Pi Pico (it will run automatically on boot)

### 3. Test the Connection

1. Connect Pi Pico 2W to Raspberry Pi via USB
2. Start the ROS system: `docker-compose up`
3. Run the test script: `python3 test_pico_commands.py`

## Communication Protocol

### Message Format
All messages are JSON-formatted strings sent over USB serial:

```json
{
  "type": "command_type",
  "timestamp": 1234567890.123,
  "data": {
    "parameter1": "value1",
    "parameter2": 123.45
  }
}
```

### Command Types (Minimal)

#### From Raspberry Pi to Pi Pico:
- `HANDSHAKE` - Initial connection
- `RUN` - Start the extruder motor
- `STOP` - Stop the extruder motor
- `SET_RPM` - Set target RPM `{ "rpm": <float> }`

#### From Pi Pico to Raspberry Pi:
- `handshake_ack` - Acknowledge connection
- `status` - `{ running, target_rpm, actual_rpm }`
- `error` - Error messages

## ROS Topics

### Published by Pi Pico Communication Node:
- `/pico/status` - General status messages
- `/pico/connected` - Connection status (Bool)
- `/extruder/actual_rpm` - Current measured RPM (Float32)

### Subscribed by Pi Pico Communication Node:
- `/extruder/run` - Bool (True = run, False = stop)
- `/extruder/target_rpm` - Float32 (desired RPM)

## Operation

- Serial Baud Rate: 115,200 bps
- Status Updates: 10 Hz (`actual_rpm`, `target_rpm`, `running`)
- Automatic Reconnection on USB reconnect

## Troubleshooting

### Connection Issues
1. Check USB cable connection
2. Verify Pi Pico 2W is powered (LED should be on)
3. Check if device appears in `/dev/ttyUSB*` or `/dev/ttyACM*`
4. Ensure proper permissions for serial devices

### Communication Issues
1. Check baud rate settings (115,200)
2. Verify JSON message format
3. Check ROS node logs for errors
4. Test with simple handshake command

### Hardware Issues
1. Verify pin connections
2. Check power supply (3.3V)
3. Test sensors with multimeter
4. Check for loose connections

## Safety Features

- **Emergency Stop**: Immediate shutdown of all motors
- **Error Handling**: Graceful handling of communication errors
- **Status Monitoring**: Continuous sensor feedback
- **Automatic Reconnection**: Reconnects if communication is lost

## Performance Optimization

For maximum performance:
1. Use high-quality USB cable
2. Keep Pi Pico close to Raspberry Pi
3. Minimize electrical interference
4. Use dedicated power supply if needed
5. Monitor system resources (CPU, memory)

## Next Steps

1. **Calibrate Sensors**: Adjust ADC readings for your specific sensors
2. **Tune Control Loops**: Optimize injection speed and pressure control
3. **Add Safety Limits**: Implement maximum pressure and temperature limits
4. **Logging**: Add detailed logging for process monitoring
5. **Integration**: Connect with your existing extruder temperature control
