#include <WiFi.h>

// Motor control pins for L298N
const int frontMotor1 = 19;
const int frontMotor2 = 18;
const int backMotor1 = 23;
const int backMotor2 = 32;

// Wi-Fi connection credentials
const char* ssid = "Parth Deshmukh";
const char* password = "parth4002";

// Server details (IP of the sender ESP32)
const char* host = "192.168.230.254";  // Change this to match the IP address of the sending ESP32

// Movement thresholds for controlling the car
const float accelThreshold = 0.5;  // Threshold for acceleration
const float gyroThreshold = 0.5;   // Threshold for gyroscope
const float flatHandThreshold = 0.2; // Threshold to detect flat hand position

// Timing variables for stopping the car
unsigned long flatHandStartTime = 0;
const unsigned long stopDelay = 2000;  // Stop after 2 seconds of flat hand

// PWM ranges
const int maxPWM = 255;  // Maximum speed for PWM
const int minPWM = 100;  // Minimum speed to prevent motor stalling

void setup() {
  Serial.begin(115200);

  // Initialize motor control pins as outputs
  pinMode(frontMotor1, OUTPUT);
  pinMode(frontMotor2, OUTPUT);
  pinMode(backMotor1, OUTPUT);
  pinMode(backMotor2, OUTPUT);

  // Stop all motors initially
  stopMotors();

  // Connect to Wi-Fi
  connectToWiFi();
}

void loop() {
  WiFiClient client;

  if (client.connect(host, 80)) {  // Connect to the server (sender ESP32)
    Serial.println("Connected to sender");

    while (client.connected()) {
      if (client.available()) {
        String line = client.readStringUntil('\n');
        Serial.println("Received data: " + line);  // Print the received MPU6050 data

        // Parse the data from the sender ESP32
        float accelX, accelY, accelZ, gyroX, gyroY, gyroZ;
        sscanf(line.c_str(), "AccelX: %f, AccelY: %f, AccelZ: %f, GyroX: %f, GyroY: %f, GyroZ: %f", &accelX, &accelY, &accelZ, &gyroX, &gyroY, &gyroZ);

        // Control the car based on the parsed MPU6050 data
        controlCar(accelY, gyroZ);
      }
    }
    client.stop();
    Serial.println("Disconnected from sender");
  } else {
    Serial.println("Connection to sender failed, retrying...");
  }

  delay(500);  // Retry every 0.5 seconds if disconnected
}

// Function to control the car movement based on the MPU6050 data
void controlCar(float accelY, float gyroZ) {
  // Flat hand detection (i.e., accelY close to 0)
  if (abs(accelY) < flatHandThreshold) {
    // If hand stays flat for more than the stopDelay, stop the car
    if (flatHandStartTime == 0) {
      flatHandStartTime = millis();  // Start timer when flat hand is detected
    } else if (millis() - flatHandStartTime >= stopDelay) {
      stopMotors();  // Stop the car if the flat hand is held too long
      return;  // No need to check other movements if we stop
    }
  } else {
    // Reset the flat hand timer if hand leaves the flat position
    flatHandStartTime = 0;
  }

  // Control forward/backward motion based on Y-axis acceleration
  if (accelY > accelThreshold) {
    moveForward();
  } else if (accelY < -accelThreshold) {
    moveBackward();
  } else {
    stopMotors();  // Stop if no significant movement
  }

  // Smoothly control steering (left/right) based on Z-axis rotation (gyroZ)
  if (gyroZ > gyroThreshold) {
    smoothTurnRight(gyroZ);
  } else if (gyroZ < -gyroThreshold) {
    smoothTurnLeft(gyroZ);
  } else {
    stopFrontMotors();
  }
}

// Wi-Fi connection function
void connectToWiFi() {
  Serial.println("Connecting to WiFi...");
  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    Serial.print(".");
    delay(1000);
  }

  Serial.println("\nConnected to WiFi!");
  Serial.print("IP Address: ");
  Serial.println(WiFi.localIP());
}

// Functions to control the car's motors

// Move the car forward
void moveForward() {
  digitalWrite(backMotor1, HIGH);
  digitalWrite(backMotor2, LOW);
  Serial.println("Moving Forward");
}

// Move the car backward
void moveBackward() {
  digitalWrite(backMotor1, LOW);
  digitalWrite(backMotor2, HIGH);
  Serial.println("Moving Backward");
}

// Smooth turn left using PWM
void smoothTurnLeft(float gyroZ) {
  int pwmValue = map(abs(gyroZ), gyroThreshold, 10.0, minPWM, maxPWM);  // Map the gyroZ value to PWM range
  pwmValue = constrain(pwmValue, minPWM, maxPWM);  // Ensure PWM stays within min-max limits

  analogWrite(frontMotor1, pwmValue);
  analogWrite(frontMotor2, LOW);
  Serial.print("Turning Left with PWM: ");
  Serial.println(pwmValue);
}

// Smooth turn right using PWM
void smoothTurnRight(float gyroZ) {
  int pwmValue = map(abs(gyroZ), gyroThreshold, 10.0, minPWM, maxPWM);  // Map the gyroZ value to PWM range
  pwmValue = constrain(pwmValue, minPWM, maxPWM);  // Ensure PWM stays within min-max limits

  analogWrite(frontMotor1, LOW);
  analogWrite(frontMotor2, pwmValue);
  Serial.print("Turning Right with PWM: ");
  Serial.println(pwmValue);
}

// Stop all motors
void stopMotors() {
  digitalWrite(frontMotor1, LOW);
  digitalWrite(frontMotor2, LOW);
  digitalWrite(backMotor1, LOW);
  digitalWrite(backMotor2, LOW);
  Serial.println("Motors Stopped");
}

// Stop only the front motors (steering)
void stopFrontMotors() {
  digitalWrite(frontMotor1, LOW);
  digitalWrite(frontMotor2, LOW);
  Serial.println("Front Motors Stopped");
}
