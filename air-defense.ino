#include <Servo.h>
#include <SoftwareSerial.h>

// Pin definisi
#define trigPin 2
#define echoPin 3
#define servoHorizontalPin 9
#define servoVerticalPin 10

// Servo
Servo servoHorizontal;
Servo servoVertical;

// Variabel untuk jarak dan waktu
long duration;
float distance;
int targetDistance = 20; // Target jarak yang diinginkan untuk pelacakan objek (dalam cm)

// Set point (kondisi default) untuk kedua servo
int defaultHorizontalPosition = 90; // Posisi tengah untuk horizontal (set point)
int defaultVerticalPosition = 90;   // Posisi tengah untuk vertikal (set point)

// PID controller parameters untuk horizontal dan vertical
float Kp = 2.0;  // Increased for better response
float Ki = 0.1;  // Reduced to prevent windup
float Kd = 1.0;  // Reduced for stability

// PID variables for horizontal servo
float previousErrorHorizontal = 0;
float integralHorizontal = 0;
float derivativeHorizontal = 0;
float outputHorizontal = 0;

// PID variables for vertical servo  
float previousErrorVertical = 0;
float integralVertical = 0;
float derivativeVertical = 0;
float outputVertical = 0;

// Variabel untuk memonitor interferensi (misalnya, perubahan jarak yang tiba-tiba)
int interferedDistanceThreshold = 50; // Jika jarak objek lebih besar dari ini, dianggap interferensi
bool isInterfered = false;

// Non-blocking delay variables
unsigned long previousMillis = 0;
unsigned long interval = 100; // Interval untuk pembaruan sistem (100ms)
unsigned long sensorInterval = 60; // Interval untuk pembacaan sensor (60ms untuk stabilitas)

// Filter eksponensial untuk noise
float filteredDistance = 0.0;
const float alpha = 0.2; // Increased for better filtering

// Initialize Bluetooth (using pins 6,7 to avoid conflict with ultrasonic sensor)
SoftwareSerial bluetooth(6, 7);  // Pin RX dan TX untuk Bluetooth HC-05

// Delta time for PID calculation
unsigned long previousTime = 0;
float deltaTime = 0;

// Current servo positions
int currentHorizontalPos = 90;
int currentVerticalPos = 90;

void setup() {
  // Inisialisasi pin
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);

  // Attach servo ke pin
  servoHorizontal.attach(servoHorizontalPin);
  servoVertical.attach(servoVerticalPin);

  // Inisialisasi servo ke posisi tengah (set point)
  servoHorizontal.write(defaultHorizontalPosition);
  servoVertical.write(defaultVerticalPosition);
  currentHorizontalPos = defaultHorizontalPosition;
  currentVerticalPos = defaultVerticalPosition;

  // Output ke Serial Monitor
  Serial.begin(9600);
  
  // Initialize Bluetooth
  bluetooth.begin(9600);

  // Initialize time
  previousTime = millis();

  // Menampilkan pesan awal di Serial Monitor
  Serial.println("Object Tracking System Enabled");
  Serial.println("Commands: kpp, kpm, kip, kim, kdp, kdm");
}

void loop() {
  unsigned long currentMillis = millis();
  static unsigned long previousSensorMillis = 0;

  // Calculate delta time for PID
  deltaTime = (currentMillis - previousTime) / 1000.0; // Convert to seconds
  if (deltaTime <= 0) deltaTime = 0.1; // Prevent division by zero

  // Cek perintah Bluetooth
  if (bluetooth.available()) {
    String command = bluetooth.readStringUntil('\n');  
    command.trim(); // Remove whitespace
    handleBluetoothCommand(command);  
  }

  // Cek perintah Serial Monitor
  if (Serial.available()) {
    String command = Serial.readStringUntil('\n');
    command.trim();
    handleBluetoothCommand(command);
  }

  if (currentMillis - previousMillis >= interval) {
    previousMillis = currentMillis;

    // Jika waktu pembacaan sensor telah tercapai
    if (currentMillis - previousSensorMillis >= sensorInterval) {
      previousSensorMillis = currentMillis;
      
      // Mengambil jarak menggunakan sensor ultrasonik
      distance = readUltrasonicDistance();

      // Filter eksponensial untuk mengurangi noise
      if (filteredDistance == 0.0) {
        filteredDistance = distance; // Initialize filter
      } else {
        filteredDistance = alpha * distance + (1 - alpha) * filteredDistance;
      }

      // Menampilkan jarak ke serial monitor
      Serial.print("Raw Distance: ");
      Serial.print(distance);
      Serial.print(" cm | Filtered: ");
      Serial.print(filteredDistance);
      Serial.println(" cm");

      // Cek apakah ada interferensi
      checkInterference();
    }

    // Update servo positions and control
    updateServoControl();
    
    // Update previous time for next PID calculation
    previousTime = currentMillis;
  }
}

// Fungsi untuk membaca sensor ultrasonik dengan error handling
float readUltrasonicDistance() {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  duration = pulseIn(echoPin, HIGH, 30000); // 30ms timeout
  
  if (duration == 0) {
    return filteredDistance; // Return last valid reading if timeout
  }
  
  float dist = duration * 0.034 / 2;  // Menghitung jarak dalam cm
  
  // Validate reading (typical HC-SR04 range: 2-400cm)
  if (dist < 2 || dist > 400) {
    return filteredDistance; // Return last valid reading
  }
  
  return dist;
}

// Fungsi untuk mengecek interferensi
void checkInterference() {
  if (filteredDistance > interferedDistanceThreshold || filteredDistance < 2) {
    if (!isInterfered) {
      Serial.println("Interference detected! Moving to default position...");
      // Reset PID integrals when switching modes
      integralHorizontal = 0;
      integralVertical = 0;
    }
    isInterfered = true;
  } else {
    if (isInterfered) {
      Serial.println("Object detected. Starting tracking...");
      // Reset PID integrals when switching modes
      integralHorizontal = 0;
      integralVertical = 0;
    }
    isInterfered = false;
  }
}

// Fungsi untuk update kontrol servo
void updateServoControl() {
  if (isInterfered) {
    // Gerakkan servo kembali ke posisi default menggunakan PID
    moveToDefaultPosition();
  } else {
    // Pelacakan objek
    trackObject();
  }
  
  // Display current status
  displayStatus();
}

// Fungsi untuk menghitung PID untuk horizontal
void calculatePIDHorizontal(float error) {
  // Proportional
  float proportional = Kp * error;
  
  // Integral dengan anti-windup
  integralHorizontal += error * deltaTime;
  integralHorizontal = constrain(integralHorizontal, -50, 50);  // Anti-windup
  float integral = Ki * integralHorizontal;
  
  // Derivative
  if (deltaTime > 0) {
    derivativeHorizontal = (error - previousErrorHorizontal) / deltaTime;
  } else {
    derivativeHorizontal = 0;
  }
  float derivative = Kd * derivativeHorizontal;
  
  // Calculate output
  outputHorizontal = proportional + integral + derivative;
  outputHorizontal = constrain(outputHorizontal, -30, 30); // Limit output for smooth movement
  
  // Apply output to servo
  int newPosition = currentHorizontalPos + (int)outputHorizontal;
  newPosition = constrain(newPosition, 0, 180);
  
  if (newPosition != currentHorizontalPos) {
    servoHorizontal.write(newPosition);
    currentHorizontalPos = newPosition;
  }
  
  previousErrorHorizontal = error;
}

// Fungsi untuk menghitung PID untuk vertical
void calculatePIDVertical(float error) {
  // Proportional
  float proportional = Kp * error;
  
  // Integral dengan anti-windup
  integralVertical += error * deltaTime;
  integralVertical = constrain(integralVertical, -50, 50);  // Anti-windup
  float integral = Ki * integralVertical;
  
  // Derivative
  if (deltaTime > 0) {
    derivativeVertical = (error - previousErrorVertical) / deltaTime;
  } else {
    derivativeVertical = 0;
  }
  float derivative = Kd * derivativeVertical;
  
  // Calculate output
  outputVertical = proportional + integral + derivative;
  outputVertical = constrain(outputVertical, -30, 30); // Limit output for smooth movement
  
  // Apply output to servo
  int newPosition = currentVerticalPos + (int)outputVertical;
  newPosition = constrain(newPosition, 0, 180);
  
  if (newPosition != currentVerticalPos) {
    servoVertical.write(newPosition);
    currentVerticalPos = newPosition;
  }
  
  previousErrorVertical = error;
}

// Fungsi untuk menggerakkan servo ke posisi default (set point)
void moveToDefaultPosition() {
  // Error untuk horizontal servo
  float errorHorizontal = defaultHorizontalPosition - currentHorizontalPos;
  calculatePIDHorizontal(errorHorizontal);

  // Error untuk vertical servo  
  float errorVertical = defaultVerticalPosition - currentVerticalPos;
  calculatePIDVertical(errorVertical);
}

// Fungsi untuk melacak objek
void trackObject() {
  // Logika tracking yang diperbaiki
  // Untuk horizontal: jika objek terlalu dekat, gerakkan servo untuk "menjauh"
  // Untuk vertical: sesuaikan berdasarkan jarak untuk menjaga objek di center
  
  float distanceError = targetDistance - filteredDistance;
  
  // Horizontal tracking (pan left/right based on distance)
  float errorHorizontal = distanceError * 0.5; // Scale factor untuk horizontal movement
  calculatePIDHorizontal(errorHorizontal);

  // Vertical tracking (tilt up/down based on distance)  
  float errorVertical = -distanceError * 0.3; // Scale factor untuk vertical movement (negative for correct direction)
  calculatePIDVertical(errorVertical);
}

// Fungsi untuk menampilkan status sistem
void displayStatus() {
  Serial.print("Mode: ");
  Serial.print(isInterfered ? "DEFAULT" : "TRACKING");
  Serial.print(" | Target: ");
  Serial.print(targetDistance);
  Serial.print(" cm | Current: ");
  Serial.print(filteredDistance);
  Serial.print(" cm | Servos - H: ");
  Serial.print(currentHorizontalPos);
  Serial.print("° V: ");
  Serial.print(currentVerticalPos);
  Serial.print("° | PID: P=");
  Serial.print(Kp);
  Serial.print(" I=");
  Serial.print(Ki);
  Serial.print(" D=");
  Serial.println(Kd);
}

// Fungsi untuk menangani perintah Bluetooth/Serial
void handleBluetoothCommand(String command) {
  command.toLowerCase(); // Convert to lowercase for consistency
  
  if (command == "kpp") {
    Kp += 0.1;
    Kp = constrain(Kp, 0, 10); // Prevent excessive values
    Serial.print("Kp increased to: ");
    Serial.println(Kp, 2);
    bluetooth.print("Kp: ");
    bluetooth.println(Kp, 2);
  } 
  else if (command == "kpm") {
    Kp -= 0.1;
    Kp = constrain(Kp, 0, 10);
    Serial.print("Kp decreased to: ");
    Serial.println(Kp, 2);
    bluetooth.print("Kp: ");
    bluetooth.println(Kp, 2);
  } 
  else if (command == "kdp") {
    Kd += 0.1;
    Kd = constrain(Kd, 0, 5);
    Serial.print("Kd increased to: ");
    Serial.println(Kd, 2);
    bluetooth.print("Kd: ");
    bluetooth.println(Kd, 2);
  } 
  else if (command == "kdm") {
    Kd -= 0.1;
    Kd = constrain(Kd, 0, 5);
    Serial.print("Kd decreased to: ");
    Serial.println(Kd, 2);
    bluetooth.print("Kd: ");
    bluetooth.println(Kd, 2);
  } 
  else if (command == "kip") {
    Ki += 0.01; // Smaller increment for Ki
    Ki = constrain(Ki, 0, 1);
    Serial.print("Ki increased to: ");
    Serial.println(Ki, 3);
    bluetooth.print("Ki: ");
    bluetooth.println(Ki, 3);
  } 
  else if (command == "kim") {
    Ki -= 0.01;
    Ki = constrain(Ki, 0, 1);
    Serial.print("Ki decreased to: ");
    Serial.println(Ki, 3);
    bluetooth.print("Ki: ");
    bluetooth.println(Ki, 3);
  }
  else if (command == "reset") {
    // Reset PID parameters to default
    Kp = 2.0;
    Ki = 0.1;
    Kd = 1.0;
    integralHorizontal = 0;
    integralVertical = 0;
    Serial.println("PID parameters reset to default");
    bluetooth.println("PID reset to default");
  }
  else if (command == "status") {
    // Display current status
    displayStatus();
  }
}