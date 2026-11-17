/********************************************************************
 * MOBILE ROBOT MOTOR CONTROLLER
 * Board       : Arduino Micro / Pro Micro (ATmega32U4)
 * Motor Driver: L298N
 * Communication: USB Serial
 * Baud Rate   : 57600
 *
 * Serial commands:
 * m <left_speed> <right_speed>
 * e
 * r
 * p
 ********************************************************************/

// ================= MOTOR PIN DEFINITIONS =================

// Right motor
#define RIGHT_MOTOR_FORWARD   6
#define RIGHT_MOTOR_BACKWARD  5
#define RIGHT_MOTOR_ENABLE    9

// Left motor
#define LEFT_MOTOR_FORWARD    16
#define LEFT_MOTOR_BACKWARD   14
#define LEFT_MOTOR_ENABLE     10

// ================= ENCODER PIN DEFINITIONS =================

// Left encoder
#define LEFT_ENCODER_A  2
#define LEFT_ENCODER_B  3

// Right encoder
#define RIGHT_ENCODER_A 0
#define RIGHT_ENCODER_B 1

// ================= ENCODER VARIABLES =================

volatile long leftEncoderCount = 0;
volatile long rightEncoderCount = 0;

volatile uint8_t leftEncoderState = 0;
volatile uint8_t rightEncoderState = 0;

// Quadrature transition lookup table
const int8_t encoderTable[16] = {
   0, -1,  1,  0,
   1,  0,  0, -1,
  -1,  0,  0,  1,
   0,  1, -1,  0
};

// Change these to -1 if an encoder reports reversed counts.
const int LEFT_ENCODER_DIRECTION = -1;
const int RIGHT_ENCODER_DIRECTION = 1;

// ================= MOTOR CONTROL =================

void setLeftMotor(int speed) {

  speed = constrain(speed, -255, 255);

  if (speed > 0) {

    digitalWrite(LEFT_MOTOR_FORWARD, HIGH);
    digitalWrite(LEFT_MOTOR_BACKWARD, LOW);

  }
  else if (speed < 0) {

    digitalWrite(LEFT_MOTOR_FORWARD, LOW);
    digitalWrite(LEFT_MOTOR_BACKWARD, HIGH);

    speed = -speed;
  }
  else {

    digitalWrite(LEFT_MOTOR_FORWARD, LOW);
    digitalWrite(LEFT_MOTOR_BACKWARD, LOW);
  }

  analogWrite(LEFT_MOTOR_ENABLE, speed);
}


void setRightMotor(int speed) {

  speed = constrain(speed, -255, 255);

  if (speed > 0) {

    digitalWrite(RIGHT_MOTOR_FORWARD, HIGH);
    digitalWrite(RIGHT_MOTOR_BACKWARD, LOW);

  }
  else if (speed < 0) {

    digitalWrite(RIGHT_MOTOR_FORWARD, LOW);
    digitalWrite(RIGHT_MOTOR_BACKWARD, HIGH);

    speed = -speed;
  }
  else {

    digitalWrite(RIGHT_MOTOR_FORWARD, LOW);
    digitalWrite(RIGHT_MOTOR_BACKWARD, LOW);
  }

  analogWrite(RIGHT_MOTOR_ENABLE, speed);
}


void setMotorSpeed(int leftSpeed, int rightSpeed) {

  setLeftMotor(leftSpeed);
  setRightMotor(rightSpeed);
}


// ================= ENCODER INTERRUPTS =================

void updateLeftEncoder() {

  uint8_t state = (digitalRead(LEFT_ENCODER_A) << 1) |
                   digitalRead(LEFT_ENCODER_B);

  uint8_t index = (leftEncoderState << 2) | state;

  leftEncoderCount += encoderTable[index] *
                      LEFT_ENCODER_DIRECTION;

  leftEncoderState = state;
}


void updateRightEncoder() {

  uint8_t state = (digitalRead(RIGHT_ENCODER_A) << 1) |
                   digitalRead(RIGHT_ENCODER_B);

  uint8_t index = (rightEncoderState << 2) | state;

  rightEncoderCount += encoderTable[index] *
                       RIGHT_ENCODER_DIRECTION;

  rightEncoderState = state;
}


// ================= ENCODER READ =================

void readEncoders(long &leftCount, long &rightCount) {

  noInterrupts();

  leftCount = leftEncoderCount;
  rightCount = rightEncoderCount;

  interrupts();
}


// ================= RESET ENCODERS =================

void resetEncoders() {

  noInterrupts();

  leftEncoderCount = 0;
  rightEncoderCount = 0;

  leftEncoderState =
    (digitalRead(LEFT_ENCODER_A) << 1) |
     digitalRead(LEFT_ENCODER_B);

  rightEncoderState =
    (digitalRead(RIGHT_ENCODER_A) << 1) |
     digitalRead(RIGHT_ENCODER_B);

  interrupts();
}


// ================= SERIAL COMMAND HANDLER =================

char inputBuffer[50];
uint8_t inputIndex = 0;


void processCommand(char *command) {

  int leftSpeed;
  int rightSpeed;

  // MOTOR COMMAND
  if (command[0] == 'm') {

    if (sscanf(command, "m %d %d",
               &leftSpeed, &rightSpeed) == 2) {

      setMotorSpeed(leftSpeed, rightSpeed);

      Serial.println("OK");
    }
    else {

      Serial.println("ERROR");
    }
  }

  // ENCODER READ COMMAND
  else if (command[0] == 'e') {

    long leftCount;
    long rightCount;

    readEncoders(leftCount, rightCount);

    Serial.print(leftCount);
    Serial.print(" ");
    Serial.println(rightCount);
  }

  // RESET ENCODERS
  else if (command[0] == 'r') {

    resetEncoders();

    Serial.println("OK");
  }

  // PING
  else if (command[0] == 'p') {

    Serial.println("PONG");
  }

  else {

    Serial.println("Invalid Command");
  }
}


// ================= SERIAL INPUT =================

void serialEvent() {

  while (Serial.available()) {

    char incoming = Serial.read();

    if (incoming == '\n' || incoming == '\r') {

      if (inputIndex > 0) {

        inputBuffer[inputIndex] = '\0';

        processCommand(inputBuffer);

        inputIndex = 0;
      }
    }

    else if (inputIndex < sizeof(inputBuffer) - 1) {

      inputBuffer[inputIndex++] = incoming;
    }
  }
}


// ================= SETUP =================

void setup() {

  Serial.begin(57600);

  // Motor pins
  pinMode(LEFT_MOTOR_FORWARD, OUTPUT);
  pinMode(LEFT_MOTOR_BACKWARD, OUTPUT);
  pinMode(LEFT_MOTOR_ENABLE, OUTPUT);

  pinMode(RIGHT_MOTOR_FORWARD, OUTPUT);
  pinMode(RIGHT_MOTOR_BACKWARD, OUTPUT);
  pinMode(RIGHT_MOTOR_ENABLE, OUTPUT);

  // Encoder pins
  pinMode(LEFT_ENCODER_A, INPUT_PULLUP);
  pinMode(LEFT_ENCODER_B, INPUT_PULLUP);

  pinMode(RIGHT_ENCODER_A, INPUT_PULLUP);
  pinMode(RIGHT_ENCODER_B, INPUT_PULLUP);

  // Stop motors
  setMotorSpeed(0, 0);

  // Initialize encoder states
  leftEncoderState =
    (digitalRead(LEFT_ENCODER_A) << 1) |
     digitalRead(LEFT_ENCODER_B);

  rightEncoderState =
    (digitalRead(RIGHT_ENCODER_A) << 1) |
     digitalRead(RIGHT_ENCODER_B);

  // Encoder interrupts
  attachInterrupt(
    digitalPinToInterrupt(LEFT_ENCODER_A),
    updateLeftEncoder,
    CHANGE
  );

  attachInterrupt(
    digitalPinToInterrupt(LEFT_ENCODER_B),
    updateLeftEncoder,
    CHANGE
  );

  attachInterrupt(
    digitalPinToInterrupt(RIGHT_ENCODER_A),
    updateRightEncoder,
    CHANGE
  );

  attachInterrupt(
    digitalPinToInterrupt(RIGHT_ENCODER_B),
    updateRightEncoder,
    CHANGE
  );

  Serial.println("ROS MOTOR CONTROLLER READY");
}


// ================= MAIN LOOP =================

void loop() {

  serialEvent();

}
