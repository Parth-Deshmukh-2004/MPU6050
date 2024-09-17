#include <WiFi.h>
#include <WiFiClient.h>
#include <MPU6050.h>
#include <Wire.h>

// Set your mobile hotspot SSID and password
const char* ssid = "Parth Deshmukh";  // Replace with your mobile hotspot SSID
const char* password = "parth4002";  // Replace with your mobile hotspot password

WiFiServer server(80);

MPU6050 mpu;

void setup() {
  Serial.begin(115200);

  // Initialize MPU6050
  Wire.begin();
  mpu.initialize();

  // Connect to your mobile hotspot
  WiFi.begin(ssid, password);
  Serial.println("Connecting to WiFi...");

  // Wait until the ESP32 is connected to the Wi-Fi network
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.print(".");
  }

  // Once connected, print the IP address
  Serial.println("");
  Serial.println("Connected to WiFi!");
  Serial.print("ESP32 IP Address: ");
  Serial.println(WiFi.localIP());

  server.begin();  // Start server
  Serial.println("Server started");
}

void loop() {
  WiFiClient client = server.available();  // Check if a client has connected
  
  if (client) {
    Serial.println("New Client Connected");
    while (client.connected()) {
      // Read MPU6050 data
      float accelX = mpu.getAccelerationX();
      float accelY = mpu.getAccelerationY();
      float accelZ = mpu.getAccelerationZ();
      float gyroX = mpu.getRotationX();
      float gyroY = mpu.getRotationY();
      float gyroZ = mpu.getRotationZ();

      // Prepare data to send
      String message = "AccelX: " + String(accelX) + ", AccelY: " + String(accelY) +
                       ", AccelZ: " + String(accelZ) + ", GyroX: " + String(gyroX) +
                       ", GyroY: " + String(gyroY) + ", GyroZ: " + String(gyroZ);
      
      // Send data to client
      client.println(message);
      delay(1000);
    }
    client.stop();
    Serial.println("Client disconnected");
  }
}
