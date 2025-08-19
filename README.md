# BLUETOOTH-OBSTACLE-AVOIDANCE
BLUETOOTH+OBSTACLE AVOIDANCE
#include <Servo.h>
#include <NewPing.h>
#include <LiquidCrystal_I2C.h>

#define SERVO_PIN 3
#define ULTRASONIC_SENSOR_TRIG 11
#define ULTRASONIC_SENSOR_ECHO 12
#define MAX_AUTO_SPEED 150           // Auto Mode
#define MAX_MANUAL_SPEED 255         // Manual Mode
#define DISTANCE_TO_CHECK 30

// Right motor
int enableRightMotor = 5;
int rightMotorPin1 = 7;
int rightMotorPin2 = 8;

// Left motor
int enableLeftMotor = 6;
int leftMotorPin1 = 9;
int leftMotorPin2 = 10;

NewPing mySensor(ULTRASONIC_SENSOR_TRIG, ULTRASONIC_SENSOR_ECHO, 400);
Servo myServo;

// LCD
LiquidCrystal_I2C lcd(0x27, 16, 2);

char mode = 'M'; // 'M' = Manual, 'A' = Auto

void setup() {
  Serial.begin(9600);

  pinMode(enableRightMotor, OUTPUT);
  pinMode(rightMotorPin1, OUTPUT);
  pinMode(rightMotorPin2, OUTPUT);
  pinMode(enableLeftMotor, OUTPUT);
  pinMode(leftMotorPin1, OUTPUT);
  pinMode(leftMotorPin2, OUTPUT);

  myServo.attach(SERVO_PIN);
  myServo.write(90);

  lcd.init();
  lcd.backlight();

  rotateMotor(0,0);
  Serial.println("🔌 Ready");

  displayModeWithStarkIEEE("Manual");
}

void loop() {
  // التبديل بين الوضعين
  if (Serial.available() > 0) {
    char cmd = Serial.read();
    Serial.print("Received: "); Serial.println(cmd);

    if (cmd == 'M') {
      mode = 'M';
      rotateMotor(0,0);
      displayModeWithStarkIEEE("Manual");
    } else if (cmd == 'A') {
      mode = 'A';
      rotateMotor(0,0);
      displayModeWithStarkIEEE("Auto");
    }

    // أوامر Manual Mode
    if (mode == 'M') {
      switch(cmd) {
        case 'F': rotateMotor(MAX_MANUAL_SPEED, MAX_MANUAL_SPEED); break;
        case 'B': rotateMotor(-MAX_MANUAL_SPEED, -MAX_MANUAL_SPEED); break;
        case 'L': rotateMotor(-MAX_MANUAL_SPEED, MAX_MANUAL_SPEED); break;
        case 'R': rotateMotor(MAX_MANUAL_SPEED, -MAX_MANUAL_SPEED); break;
        case 'S': rotateMotor(0,0); break;
      }
    }
  }

  // Auto Mode
  if (mode == 'A') {
    int distance = mySensor.ping_cm();

    if (distance > 0 && distance < DISTANCE_TO_CHECK) {
      rotateMotor(0,0); delay(500);
      rotateMotor(-MAX_AUTO_SPEED, -MAX_AUTO_SPEED); delay(200);
      rotateMotor(0,0); delay(500);

      myServo.write(180); delay(500);
      int distanceLeft = mySensor.ping_cm();

      myServo.write(0); delay(500);
      int distanceRight = mySensor.ping_cm();

      myServo.write(90); delay(500);

      if(distanceLeft == 0) rotateMotor(MAX_AUTO_SPEED, -MAX_AUTO_SPEED);
      else if(distanceRight == 0) rotateMotor(-MAX_AUTO_SPEED, MAX_AUTO_SPEED);
      else if(distanceLeft >= distanceRight) rotateMotor(MAX_AUTO_SPEED, -MAX_AUTO_SPEED);
      else rotateMotor(-MAX_AUTO_SPEED, MAX_AUTO_SPEED);

      delay(200);
      rotateMotor(0,0); delay(200);
    } else {
      rotateMotor(MAX_AUTO_SPEED, MAX_AUTO_SPEED);
    }
  }
}

// دالة عرض المود في منتصف السطر الأول، و STARK ... IEEE في السطر الثاني
void displayModeWithStarkIEEE(String modeText) {
  lcd.clear();

  // السطر الأول: المود في المنتصف
  int lenMode = modeText.length();
  int spacesMode = (16 - lenMode) / 2;
  lcd.setCursor(spacesMode,0);
  lcd.print(modeText);

  // السطر الثاني: STARK في البداية و IEEE في النهاية
  lcd.setCursor(0,1);
  lcd.print("STARK");
  lcd.setCursor(16 - 4,1);
  lcd.print("IEEE");
}

void rotateMotor(int rightMotorSpeed, int leftMotorSpeed) {
  if (rightMotorSpeed < 0) { digitalWrite(rightMotorPin1, LOW); digitalWrite(rightMotorPin2, HIGH);}
  else if (rightMotorSpeed > 0) { digitalWrite(rightMotorPin1, HIGH); digitalWrite(rightMotorPin2, LOW);}
  else { digitalWrite(rightMotorPin1, LOW); digitalWrite(rightMotorPin2, LOW); }

  if (leftMotorSpeed < 0) { digitalWrite(leftMotorPin1, LOW); digitalWrite(leftMotorPin2, HIGH);}
  else if (leftMotorSpeed > 0) { digitalWrite(leftMotorPin1, HIGH); digitalWrite(leftMotorPin2, LOW);}
  else { digitalWrite(leftMotorPin1, LOW); digitalWrite(leftMotorPin2, LOW); }

  analogWrite(enableRightMotor, abs(rightMotorSpeed));
  analogWrite(enableLeftMotor, abs(leftMotorSpeed));
}
