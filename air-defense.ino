

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
int distance;
int targetDistance = 20; // Target jarak yang diinginkan untuk pelacakan objek (dalam cm)

// Set point (kondisi default) untuk kedua servo
int defaultHorizontalPosition = 90; // Posisi tengah untuk horizontal (set point)
int defaultVerticalPosition = 90;   // Posisi tengah untuk vertikal (set point)

// PID controller parameters untuk horizontal dan vertical
float Kp = 1.0;
float Ki = 1.0;
float Kd = 2.0;

float previousErrorHorizontal = 0;
float previousErrorVertical = 0;
float integralHorizontal = 0;
float integralVertical = 0;
float derivativeHorizontal, derivativeVertical;
float outputHorizontal, outputVertical;

// Variabel untuk memonitor interferensi (misalnya, perubahan jarak yang tiba-tiba)
int interferedDistanceThreshold = 30; // Jika jarak objek lebih besar dari ini, dianggap interferensi
bool isInterfered = false;

// Non-blocking delay variables
unsigned long previousMillis = 0;
unsigned long interval = 100; // Interval untuk pembaruan sistem (100ms)
unsigned long sensorInterval = 50; // Interval untuk pembacaan sensor

// Filter eksponensial untuk noise
float filteredDistance = 0.0;
const float alpha = 0.1; // Faktor penghalusan (0.1 untuk sedikit penghalusan)

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

  // Output ke Serial Monitor
  Serial.begin(9600);
}

void loop() {
  unsigned long currentMillis = millis();
  static unsigned long previousSensorMillis = 0;

  if (currentMillis - previousMillis >= interval) {
    previousMillis = currentMillis;

    // Jika waktu pembacaan sensor telah tercapai
    if (currentMillis - previousSensorMillis >= sensorInterval) {
      previousSensorMillis = currentMillis;
      
      // Mengambil jarak menggunakan sensor ultrasonik
      digitalWrite(trigPin, LOW);
      delayMicroseconds(2);
      digitalWrite(trigPin, HIGH);
      delayMicroseconds(10);
      digitalWrite(trigPin, LOW);

      duration = pulseIn(echoPin, HIGH);
      distance = duration * 0.034 / 2;  // Menghitung jarak dalam cm

      // Filter eksponensial
      filteredDistance = alpha * distance + (1 - alpha) * filteredDistance;

      // Menampilkan jarak ke serial monitor
      Serial.print("Filtered Distance: ");
      Serial.print(filteredDistance);
      Serial.println(" cm");

      // Cek apakah ada interferensi, misalnya objek tiba-tiba jauh
      if (filteredDistance > interferedDistanceThreshold) {
        isInterfered = true; // Ada interferensi
      } else {
        isInterfered = false; // Tidak ada interferensi
      }

      // Menampilkan status interferensi di Serial Monitor
      if (isInterfered) {
        Serial.println("Interference detected! Moving to default set point...");
      } else {
        Serial.println("No interference. Following object...");
      }
    }

    // Menampilkan set point di Serial Monitor
    Serial.print("Default Horizontal Position: ");
    Serial.print(defaultHorizontalPosition);
    Serial.print(" | Current Horizontal Position: ");
    Serial.println(servoHorizontal.read());

    Serial.print("Default Vertical Position: ");
    Serial.print(defaultVerticalPosition);
    Serial.print(" | Current Vertical Position: ");
    Serial.println(servoVertical.read());

    // PID Control untuk horizontal dan vertical
    if (isInterfered) {
      // Gerakkan servo kembali ke posisi default menggunakan PID
      moveToDefaultPosition();
    } else {
      // Pelacakan objek
      trackObject();
    }
  }
}

// Fungsi untuk menghitung PID untuk horizontal
void calculatePIDHorizontal(float errorHorizontal) {
  integralHorizontal += errorHorizontal;
  integralHorizontal = constrain(integralHorizontal, -100, 100);  // Batasi nilai integral
  derivativeHorizontal = errorHorizontal - previousErrorHorizontal;
  outputHorizontal = Kp * errorHorizontal + Ki * integralHorizontal + Kd * derivativeHorizontal;
  int horizontalPosition = servoHorizontal.read() + outputHorizontal;
  horizontalPosition = constrain(horizontalPosition, 0, 180); // Membatasi agar servo tidak bergerak melebihi batas
  servoHorizontal.write(horizontalPosition);
  previousErrorHorizontal = errorHorizontal;
}

// Fungsi untuk menghitung PID untuk vertical
void calculatePIDVertical(float errorVertical) {
  integralVertical += errorVertical;
  integralVertical = constrain(integralVertical, -100, 100);  // Batasi nilai integral
  derivativeVertical = errorVertical - previousErrorVertical;
  outputVertical = Kp * errorVertical + Ki * integralVertical + Kd * derivativeVertical;
  int verticalPosition = servoVertical.read() + outputVertical;
  verticalPosition = constrain(verticalPosition, 0, 180); // Membatasi agar servo tidak bergerak melebihi batas
  servoVertical.write(verticalPosition);
  previousErrorVertical = errorVertical;
}

// Fungsi untuk menggerakkan servo ke posisi default (set point)
void moveToDefaultPosition() {
  // Menghitung error dan PID untuk pergerakan servo horizontal menuju posisi default
  float errorHorizontal = defaultHorizontalPosition - servoHorizontal.read();
  calculatePIDHorizontal(errorHorizontal);

  // Menghitung error dan PID untuk pergerakan servo vertical menuju posisi default
  float errorVertical = defaultVerticalPosition - servoVertical.read();
  calculatePIDVertical(errorVertical);
}

// Fungsi untuk melacak objek
void trackObject() {
  // Menghitung error dan PID untuk pergerakan servo horizontal mengikuti jarak objek
  float errorHorizontal = targetDistance - filteredDistance;
  calculatePIDHorizontal(errorHorizontal);

  // Menghitung error dan PID untuk pergerakan servo vertical mengikuti jarak objek
  float errorVertical = filteredDistance - targetDistance;
  calculatePIDVertical(errorVertical);
}
