#include "esp_camera.h"
#include <WiFi.h>
#include <HTTPClient.h>
#include <Wire.h>
// #include <LiquidCrystal_I2C.h>  // Commented out - incompatible with ESP32
#include <ArduinoJson.h>

// WiFi credentials
const char* ssid = "LAPTOP";
const char* password = "123456789";

// Server configuration
const char* serverURL = "http://158.1.1.1:5000/detect";  // Replace with your laptop's IP

// LCD display configuration (16x2 LCD with I2C)
#define LCD_COLS 16
#define LCD_ROWS 2
#define LCD_I2C_ADDRESS 0x27  // Try 0x3F if 0x27 doesn't work

// Global LCD status
bool lcd_working = false;

// Camera configuration for ESP32-CAM
#define PWDN_GPIO_NUM     32
#define RESET_GPIO_NUM    -1
#define XCLK_GPIO_NUM      0
#define SIOD_GPIO_NUM     26
#define SIOC_GPIO_NUM     27
#define Y9_GPIO_NUM       35
#define Y8_GPIO_NUM       34
#define Y7_GPIO_NUM       39
#define Y6_GPIO_NUM       36
#define Y5_GPIO_NUM       21
#define Y4_GPIO_NUM       19
#define Y3_GPIO_NUM       18
#define Y2_GPIO_NUM        5
#define VSYNC_GPIO_NUM    25
#define HREF_GPIO_NUM     23
#define PCLK_GPIO_NUM     22
#define LED_GPIO_NUM 4  // Onboard LED for ESP32-CAM

// Timing configuration
unsigned long lastCapture = 0;
const unsigned long captureInterval = 2000;  // Capture every 2 seconds
unsigned long lastLEDBlink = 0;
const unsigned long ledBlinkInterval = 3000; // 3 seconds

// Simple LCD functions using direct I2C
void lcd_init() {
  Wire.beginTransmission(LCD_I2C_ADDRESS);
  Wire.write(0x00);  // Command mode
  Wire.write(0x38);  // Function set: 8-bit, 2 lines, 5x8 font
  Wire.endTransmission();
  delay(5);
  
  Wire.beginTransmission(LCD_I2C_ADDRESS);
  Wire.write(0x00);  // Command mode
  Wire.write(0x0C);  // Display on, cursor off, blink off
  Wire.endTransmission();
  delay(5);
  
  Wire.beginTransmission(LCD_I2C_ADDRESS);
  Wire.write(0x00);  // Command mode
  Wire.write(0x01);  // Clear display
  Wire.endTransmission();
  delay(5);
  
  Wire.beginTransmission(LCD_I2C_ADDRESS);
  Wire.write(0x00);  // Command mode
  Wire.write(0x06);  // Entry mode: increment cursor, no shift
  Wire.endTransmission();
  delay(5);
}

void lcd_clear() {
  Wire.beginTransmission(LCD_I2C_ADDRESS);
  Wire.write(0x00);  // Command mode
  Wire.write(0x01);  // Clear display
  Wire.endTransmission();
  delay(5);
}

void lcd_set_cursor(int col, int row) {
  int address = 0x80 + (row * 0x40) + col;
  Wire.beginTransmission(LCD_I2C_ADDRESS);
  Wire.write(0x00);  // Command mode
  Wire.write(address);
  Wire.endTransmission();
  delay(1);
}

void lcd_print(const char* text) {
  Wire.beginTransmission(LCD_I2C_ADDRESS);
  Wire.write(0x40);  // Data mode
  for (int i = 0; text[i] != '\0'; i++) {
    Wire.write(text[i]);
  }
  Wire.endTransmission();
  delay(1);
}

void lcd_backlight(bool on) {
  // Most I2C LCD modules don't have backlight control via I2C
  // This is just a placeholder
}

// Accuracy threshold
const float ACCURACY_THRESHOLD = 0.8;  // 80% accuracy threshold

void setup() {
  Serial.begin(115200);
  Serial.println("ESP32-CAM Sign Detection Starting...");
  
  // Initialize I2C for LCD
  Wire.begin(14, 12);  // SDA, SCL pins for ESP32-CAM
  Serial.println("I2C initialized on SDA: 14, SCL: 12");
  
  // Try slower I2C frequency for better compatibility
  Wire.setClock(100000);  // 100kHz instead of default 400kHz
  Serial.println("I2C clock set to 100kHz");
  
  // Scan for I2C devices first
  Serial.println("Scanning for I2C devices...");
  int deviceCount = 0;
  for (byte address = 1; address < 127; address++) {
    Wire.beginTransmission(address);
    byte error = Wire.endTransmission();
    if (error == 0) {
      Serial.printf("I2C device found at address 0x%02X\n", address);
      deviceCount++;
    }
  }
  
  if (deviceCount == 0) {
    Serial.println("WARNING: No I2C devices found! Check wiring:");
    Serial.println("  - SDA pin 14 connected to LCD SDA");
    Serial.println("  - SCL pin 12 connected to LCD SCL");
    Serial.println("  - 5V power connected to LCD VCC");
    Serial.println("  - GND connected to LCD GND");
  }
  
  // Try to initialize LCD with address 0x27 first
  Serial.println("Trying LCD address 0x27...");
  bool lcd_ok = false;
  
  // Try to initialize LCD - the begin() function returns void
  lcd_init();
  delay(100); // Give LCD time to initialize
  
  // Test if LCD is responding by trying to clear it
  lcd_clear();
  lcd_backlight(true);
  Serial.println("LCD 0x27 initialized successfully!");
  lcd_ok = true;
  
  // If we get here without errors, LCD 0x27 is working
  // If there was an error, the program would have crashed or we'd see garbage on display
  
  // Also try alternative address 0x3F if needed
  // You can manually change the address in the LCD declaration above if 0x27 doesn't work
  
  if (lcd_ok) {
    // Initialize LCD display
    lcd_init();
    lcd_backlight(true);
    lcd_clear();
    lcd_set_cursor(0, 0);
    lcd_print("Initializing...");
    lcd_set_cursor(0, 1);
    lcd_print("Please wait...");
    Serial.println("LCD display working!");
    lcd_working = true;
  } else {
    Serial.println("LCD display failed - continuing without display");
    lcd_working = false;
  }
  
  // Initialize camera
  camera_config_t config;
  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer = LEDC_TIMER_0;
  config.pin_d0 = Y2_GPIO_NUM;
  config.pin_d1 = Y3_GPIO_NUM;
  config.pin_d2 = Y4_GPIO_NUM;
  config.pin_d3 = Y5_GPIO_NUM;
  config.pin_d4 = Y6_GPIO_NUM;
  config.pin_d5 = Y7_GPIO_NUM;
  config.pin_d6 = Y8_GPIO_NUM;
  config.pin_d7 = Y9_GPIO_NUM;
  config.pin_xclk = XCLK_GPIO_NUM;
  config.pin_pclk = PCLK_GPIO_NUM;
  config.pin_vsync = VSYNC_GPIO_NUM;
  config.pin_href = HREF_GPIO_NUM;
  config.pin_sscb_sda = SIOD_GPIO_NUM;
  config.pin_sscb_scl = SIOC_GPIO_NUM;
  config.pin_pwdn = PWDN_GPIO_NUM;
  config.pin_reset = RESET_GPIO_NUM;
  config.xclk_freq_hz = 20000000;
  config.pixel_format = PIXFORMAT_JPEG;
  
  // Frame size and quality settings
  config.frame_size = FRAMESIZE_VGA;  // 640x480
  config.jpeg_quality = 10;  // Lower number = higher quality
  config.fb_count = 1;
  
  // Initialize camera
  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    Serial.printf("Camera init failed with error 0x%x", err);
    if (lcd_ok) {
      lcd_clear();
      lcd_set_cursor(0, 0);
      lcd_print("Camera failed!");
      lcd_set_cursor(0, 1);
      lcd_print("Check wiring");
    }
    return;
  }
  
  // Connect to WiFi
  WiFi.begin(ssid, password);
  if (lcd_ok) {
    lcd_clear();
    lcd_set_cursor(0, 0);
    lcd_print("Connecting WiFi");
    lcd_set_cursor(0, 1);
    lcd_print("Please wait...");
  }
  
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.println("Connecting to WiFi...");
  }
  
  Serial.println("WiFi connected!");
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());
  
  if (lcd_ok) {
    lcd_clear();
    lcd_set_cursor(0, 0);
    lcd_print("WiFi Connected!");
          lcd_set_cursor(0, 1);
      String ipText = "IP: " + WiFi.localIP().toString().substring(0, 15);
      lcd_print(ipText.c_str());
    delay(3000);
    
    lcd_clear();
    lcd_set_cursor(0, 0);
    lcd_print("Ready for");
    lcd_set_cursor(0, 1);
    lcd_print("detection...");
    delay(2000);
  }

  pinMode(LED_GPIO_NUM, OUTPUT); // Initialize LED pin
  digitalWrite(LED_GPIO_NUM, LOW); // Ensure LED is off at start
}

void loop() {
  unsigned long currentTime = millis();
  
  // Blink LED every 3 seconds
  if (currentTime - lastLEDBlink >= ledBlinkInterval) {
    lastLEDBlink = currentTime;
    digitalWrite(LED_GPIO_NUM, HIGH); // Turn LED on
    delay(100); // LED on for 100ms
    digitalWrite(LED_GPIO_NUM, LOW);  // Turn LED off
  }

  // Capture and send image at regular intervals
  if (currentTime - lastCapture >= captureInterval) {
    lastCapture = currentTime;
    
    // Capture image
    camera_fb_t* fb = esp_camera_fb_get();
    if (!fb) {
      Serial.println("Camera capture failed");
      return;
    }
    
    // Send image to server
    String result = sendImageToServer(fb);
    
    // Display result
    displayResult(result);
    
    // Return frame buffer
    esp_camera_fb_return(fb);
  }
  
  delay(100);  // Small delay to prevent watchdog issues
}

String sendImageToServer(camera_fb_t* fb) {
  HTTPClient http;
  http.begin(serverURL);
  http.addHeader("Content-Type", "image/jpeg");
  
  if (lcd_working) {
    lcd_clear();
    lcd_set_cursor(0, 0);
    lcd_print("Capturing...");
    lcd_set_cursor(0, 1);
    lcd_print("Sending image");
  }
  
  int httpResponseCode = http.POST(fb->buf, fb->len);
  
  String response = "";
  if (httpResponseCode > 0) {
    response = http.getString();
    Serial.println("HTTP Response: " + response);
  } else {
    Serial.printf("HTTP Error: %d\n", httpResponseCode);
    response = "{\"error\": \"HTTP Error\"}";
  }
  
  http.end();
  return response;
}

void displayResult(String jsonResponse) {
  if (!lcd_working) {
    // If LCD not working, just log to Serial
    Serial.println("LCD not working - skipping display");
    return;
  }
  
  lcd_clear();
  
  // Parse JSON response
  DynamicJsonDocument doc(1024);
  deserializeJson(doc, jsonResponse);
  
  if (doc.containsKey("error")) {
    lcd_set_cursor(0, 0);
    lcd_print("Error:");
    lcd_set_cursor(0, 1);
    lcd_print(doc["error"].as<String>().substring(0, 16).c_str());
  } else if (doc.containsKey("detected_sign")) {
    String detectedSign = doc["detected_sign"];
    float confidence = doc["confidence"];
    
    // Only display if confidence is above 80%
    if (confidence >= ACCURACY_THRESHOLD) {
             lcd_set_cursor(0, 0);
       String signText = "Sign: " + detectedSign.substring(0, 11);
       lcd_print(signText.c_str());
       lcd_set_cursor(0, 1);
       String accText = "Acc: " + String(confidence * 100, 1) + "%";
       lcd_print(accText.c_str());
      
      Serial.println("HIGH CONFIDENCE - Detected: " + detectedSign + " (Confidence: " + String(confidence * 100, 1) + "%)");
    } else {
             lcd_set_cursor(0, 0);
       lcd_print("Low confidence");
       lcd_set_cursor(0, 1);
       String lowAccText = "Acc: " + String(confidence * 100, 1) + "% < 80%";
       lcd_print(lowAccText.c_str());
      
      Serial.println("LOW CONFIDENCE - Detected: " + detectedSign + " (Confidence: " + String(confidence * 100, 1) + "%) - Not displaying");
    }
  } else {
    lcd_set_cursor(0, 0);
    lcd_print("No sign detected");
    lcd_set_cursor(0, 1);
    lcd_print("Try again...");
  }
}

