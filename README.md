## Hardware Required
- ESP32-CAM module
- 16x2 LCD display with I2C backpack (PCF8574)
- Jumper wires
- Breadboard (optional)
- 5V power supply for ESP32-CAM

## LCD I2C Wiring

### ESP32-CAM to LCD I2C Connections:
```
ESP32-CAM    ->    LCD I2C Module
GPIO 14      ->    SDA
GPIO 12      ->    SCL  
5V           ->    VCC
GND          ->    GND
```

### Pin Configuration:
- **SDA (Data)**: GPIO 14
- **SCL (Clock)**: GPIO 12
- **Power**: 5V and GND

## I2C Address
- Default I2C address: `0x27`
- If your LCD doesn't work, try: `0x3F`

## Required Libraries
Install these libraries in Arduino IDE:
1. **LiquidCrystal_I2C** by Frank de Brabander
2. **ArduinoJson** by Benoit Blanchon
3. **ESP32** board package

## Installation Steps:
1. Connect hardware according to wiring diagram
2. Install required libraries
3. Upload the code to ESP32-CAM
4. Power on and check Serial Monitor for debugging

## Testing LCD:
If the LCD doesn't display anything:
1. Check wiring connections
2. Try different I2C address (0x3F instead of 0x27)
3. Use I2C scanner to find the correct address

## Features:
- Only displays sign detection results with >80% accuracy
- Shows "Low confidence" message for results <80%
- Real-time status updates on LCD
- Serial monitor logging for debugging
