#include <arduino.h>
#include <Adafruit_PWMServoDriver.h>
#include <Wire.h>
#include <SPI.h>

const int SERVO_1 = 0;  // Base Rotation
const int SERVO_2 = 1;  // Arm 1 Rotation
const int SERVO_3 = 2;  // Arm 2 Rotation
const int SERVO_4 = 3;  // Arm 3 Rotation
const int SERVO_5a = 4; // Arm 4 Rotation a
const int SERVO_5b = 5; // Arm 4 Rotation b
const int SERVO_6 = 6;  // Gripper

const int SERVO_1_MIN = 115; // Min Pulse MG 995
const int SERVO_1_MAX = 580; // Max Pulse MG 995
const int SERVO_2_MIN = 120; // Min Pulse MG 996R
const int SERVO_2_MAX = 560; // Max Pulse MG 996R 
const int SERVO_3_MIN = 120; // Min Pulse MG 996R
const int SERVO_3_MAX = 560; // Max Pulse MG 996R
const int SERVO_4_MIN = 120; // Min Pulse MG 996R
const int SERVO_4_MAX = 560; // Max Pulse MG 996R
const int SERVO_5_MIN = 100; // Min Pulse SG90 Tower Pro
const int SERVO_5_MAX = 570; // Max Pulse SG90 Tower Pro
const int SERVO_6_MIN = 100; // Min Pulse SG90 Tower Pro
const int SERVO_6_MAX = 570; // Max Pulse SG90 Tower Pro

const int SERVO_1_MIN_ANGLE = 0; // Min Anglle for Base Rotation
const int SERVO_1_MAX_ANGLE = 180; // Max Angle for Base Rotation
const int SERVO_2_MIN_ANGLE = 0; // Min Angle for Arm 1 Rotation
const int SERVO_2_MAX_ANGLE = 180; // Max Angle for Arm 1 Rotation
const int SERVO_3_MIN_ANGLE = 0; // Min Angle for Arm 2 Rotation
const int SERVO_3_MAX_ANGLE = 180; // Max Angle for Arm 2 Rotation
const int SERVO_4_MIN_ANGLE = 0; // Min Angle for Arm 3 Rotation
const int SERVO_4_MAX_ANGLE = 180; // Max Angle for Arm 3 Rotation
const int SERVO_5_MIN_ANGLE = 0; // Min Angle for Arm 4 Rotation
const int SERVO_5_MAX_ANGLE = 180; // Max Angle for Arm 4 Rotation
const int SERVO_6_MIN_ANGLE = 135; // Min Angle for Gripper
const int SERVO_6_MAX_ANGLE = 175; // Max Angle for Gripper

// Parking Positions for Servos
const int SERVO_1_PARK = 90; // Base Rotation
const int SERVO_2_PARK = 90; // Arm 1 Rotation
const int SERVO_3_PARK = 90; // Arm 2 Rotation
const int SERVO_4_PARK = 90; // Arm 3 Rotation
const int SERVO_5a_PARK = 90; // Arm 4 Rotation
const int SERVO_5b_PARK = SERVO_5a_PARK; // Arm 4 Rotation
const int SERVO_6_PARK = 180; // Gripper

// Converting Angle to motor pulses
const int SERVO_1_MIN_PULSE = map(SERVO_1_MIN_ANGLE, 0, 180, SERVO_1_MIN, SERVO_1_MAX);
const int SERVO_1_MAX_PULSE = map(SERVO_1_MAX_ANGLE, 0, 180, SERVO_1_MIN, SERVO_1_MAX);
const int SERVO_2_MIN_PULSE = map(SERVO_2_MIN_ANGLE, 0, 180, SERVO_2_MIN, SERVO_2_MAX);
const int SERVO_2_MAX_PULSE = map(SERVO_2_MAX_ANGLE, 0, 180, SERVO_2_MIN, SERVO_2_MAX);
const int SERVO_3_MIN_PULSE = map(SERVO_3_MIN_ANGLE, 0, 180, SERVO_3_MIN, SERVO_3_MAX);
const int SERVO_3_MAX_PULSE = map(SERVO_3_MAX_ANGLE, 0, 180, SERVO_3_MIN, SERVO_3_MAX);
const int SERVO_4_MIN_PULSE = map(SERVO_4_MIN_ANGLE, 0, 180, SERVO_4_MIN, SERVO_4_MAX);
const int SERVO_4_MAX_PULSE = map(SERVO_4_MAX_ANGLE, 0, 180, SERVO_4_MIN, SERVO_4_MAX);
const int SERVO_5_MIN_PULSE = map(SERVO_5_MIN_ANGLE, 0, 180, SERVO_5_MIN, SERVO_5_MAX);
const int SERVO_5_MAX_PULSE = map(SERVO_5_MAX_ANGLE, 0, 180, SERVO_5_MIN, SERVO_5_MAX);
const int SERVO_6_MIN_PULSE = map(SERVO_6_MIN_ANGLE, 0, 180, SERVO_6_MIN, SERVO_6_MAX);
const int SERVO_6_MAX_PULSE = map(SERVO_6_MAX_ANGLE, 0, 180, SERVO_6_MIN, SERVO_6_MAX);

const int SERVO_1_PARK_PULSE = map(SERVO_1_PARK, 0, 180, SERVO_1_MIN, SERVO_1_MAX);
const int SERVO_2_PARK_PULSE = map(SERVO_2_PARK, 0, 180, SERVO_2_MIN, SERVO_2_MAX);
const int SERVO_3_PARK_PULSE = map(SERVO_3_PARK, 0, 180, SERVO_3_MIN, SERVO_3_MAX);
const int SERVO_4_PARK_PULSE = map(SERVO_4_PARK, 0, 180, SERVO_4_MIN, SERVO_4_MAX);
const int SERVO_5a_PARK_PULSE = map(SERVO_5a_PARK, 0, 180, SERVO_5_MIN, SERVO_5_MAX);
const int SERVO_5b_PARK_PULSE = map(SERVO_5b_PARK, 0, 180, SERVO_5_MIN, SERVO_5_MAX);
const int SERVO_6_PARK_PULSE = map(SERVO_6_PARK, 0, 180, SERVO_6_MIN, SERVO_6_MAX);

int SERVO_1_POS = SERVO_1_PARK_PULSE;
int SERVO_2_POS = SERVO_2_PARK_PULSE;
int SERVO_3_POS = SERVO_3_PARK_PULSE;
int SERVO_4_POS = SERVO_4_PARK_PULSE;
int SERVO_5a_POS = SERVO_5a_PARK_PULSE;
int SERVO_5b_POS = SERVO_5b_PARK_PULSE;
int SERVO_6_POS = SERVO_6_PARK_PULSE;

//============================================================================================
// Defining the PWM Driver
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(); // Uses Slave Address 0x40

void setup() {
    Serial.begin(115200);
    pwm.begin();
    pwm.setPWMFreq(60); // Analog servos run at ~60 Hz updates
    Serial.println("Servos Configured and Ready to Rock and Roll......");
    Serial.println("Awaiting Orders");    
}

void IRAM_ATTR Move_Robot(float Angle1, float Angle2, float Angle3, float Angle4, float Angle5, float GripperState) {
  // Angle1 - Base Rotation (90 Degree of motor is set as zero in actual)
  // Angle2 - Arm 1 Rotation
  // Angle3 - Arm 2 Rotation
  // Angle4 - Arm 3 Rotation
  // Angle5 - Arm 4 Rotation - Gripper Orientation Control
  // GripperState - Gripper Open/Close (0 or 1)

  
  // Adjusting the zero position
  Angle1 += 90;
  Angle1 = constrain(Angle1, SERVO_1_MIN_ANGLE, SERVO_1_MAX_ANGLE); // Constrain to the set limits
  Angle2 = constrain(Angle2, SERVO_2_MIN_ANGLE, SERVO_2_MAX_ANGLE); // Constrain to the set limits
  Angle3 = constrain(Angle3, SERVO_3_MIN_ANGLE, SERVO_3_MAX_ANGLE); // Constrain to the set limits
  Angle4 = constrain(Angle4, SERVO_4_MIN_ANGLE, SERVO_4_MAX_ANGLE); // Constrain to the set limits
  Angle5 = constrain(Angle5, SERVO_5_MIN_ANGLE, SERVO_5_MAX_ANGLE); // Constrain to the set limits
  GripperState = constrain(GripperState, 0, 1); // Constrain to the set limits

  int Target_Pulse_1 = map(Angle1, 0, 180, SERVO_1_MIN_PULSE, SERVO_1_MAX_PULSE); // Pulse Signal for Base Rotation
  int Target_Pulse_2 = map(Angle2, 0, 180, SERVO_2_MIN_PULSE, SERVO_2_MAX_PULSE); // Pulse Signal for Arm 1 Rotation
  int Target_Pulse_3 = map(Angle3, 0, 180, SERVO_3_MIN_PULSE, SERVO_3_MAX_PULSE); // Pulse Signal for Arm 2 Rotation
  int Target_Pulse_4 = map(Angle4, 0, 180, SERVO_4_MIN_PULSE, SERVO_4_MAX_PULSE); // Pulse Signal for Arm 3 Rotation
  int Target_Pulse_5 = map(Angle5, 0, 180, SERVO_5_MIN_PULSE, SERVO_5_MAX_PULSE); // Pulse Signal for Arm 4 Rotation
  int Target_Pulse_6 = GripperState == 1 ? SERVO_6_MAX_PULSE : SERVO_6_MIN_PULSE; // Pulse Signal for Gripper Open/Close
  pwm.setPWM(SERVO_5a, 0, Target_Pulse_5);
  pwm.setPWM(SERVO_5b, 0, Target_Pulse_5);
  pwm.setPWM(SERVO_6, 0, Target_Pulse_6);

  // Get current positions
  int start_pulse_1 = SERVO_1_POS;
  int start_pulse_2 = SERVO_2_POS;
  int start_pulse_3 = SERVO_3_POS;
  int start_pulse_4 = SERVO_4_POS;
  int start_pulse_5a = SERVO_5a_POS;
  int start_pulse_5b = SERVO_5b_POS;
  int start_pulse_6 = SERVO_6_POS;

  // Get current positions
  SERVO_5a_POS = Target_Pulse_5;
  SERVO_5b_POS = Target_Pulse_5;
  SERVO_6_POS = Target_Pulse_6;

  // Calculate deltas
  int delta1 = Target_Pulse_1 - start_pulse_1;
  int delta2 = Target_Pulse_2 - start_pulse_2;
  int delta3 = Target_Pulse_3 - start_pulse_3;
  int delta4 = Target_Pulse_4 - start_pulse_4;

  // Find maximum absolute delta
  int max_diff = max(abs(delta1), max(abs(delta2), max(abs(delta3),abs(delta4))));

  if (max_diff == 0) {
    delay(500);
    Serial.println("");
    Serial.println("Done");
    return;
  }
// Motion parameters
const int step_delay = 3; //overall speed (ms per step)
  
// Smooth movement with acceleration/deceleration
  for (int i = 0; i <= max_diff; i++) {
    float t = (float)i / max_diff;
    // Smoothstep easing function (3t² - 2t³)
    float eased_t = t * t * (3.0 - 2.0 * t);

    // Calculate intermediate positions
    int current1 = start_pulse_1 + round(delta1 * eased_t);
    int current2 = start_pulse_2 + round(delta2 * eased_t);
    int current3 = start_pulse_3 + round(delta3 * eased_t);
    int current4 = start_pulse_4 + round(delta4 * eased_t);

    // Update servo positions
    pwm.setPWM(SERVO_1, 0, current1);
    pwm.setPWM(SERVO_2, 0, current2);
    pwm.setPWM(SERVO_3, 0, current3);
    pwm.setPWM(SERVO_4, 0, current4);

    // Update global position trackers
    SERVO_1_POS = current1;
    SERVO_2_POS = current2;
    SERVO_3_POS = current3;
    SERVO_4_POS = current4;
    delay(step_delay);
  }
    Serial.println("");
    Serial.println("Done");
  }
  
  void loop() {
    //take user input with decimal places
    float Angle1, Angle2, Angle3, Angle4, Angle5;
    int GripperState;
    static String inputString = ""; 
    static bool inputComplete = false;

  while (Serial.available() > 0) {
    char incomingChar = Serial.read();

    if (incomingChar == '\n') {
      inputComplete = true;
      Serial.println("Enter the angles and gripper state separated by spaces : ");
      break;
    } else {
      inputString += incomingChar; 
    }
  }

    if (inputComplete) {
        inputString.trim();
        Serial.print(inputString);
        if (inputString.length() > 0) {
            int firstSpaceIndex = inputString.indexOf(' ');
            int secondSpaceIndex = inputString.indexOf(' ', firstSpaceIndex + 1);
            int thirdSpaceIndex = inputString.indexOf(' ', secondSpaceIndex + 1);
            int fourthSpaceIndex = inputString.indexOf(' ', thirdSpaceIndex + 1);
            int fifthSpaceIndex = inputString.indexOf(' ', fourthSpaceIndex + 1);
            
            Angle1 = inputString.substring(0, firstSpaceIndex).toFloat();
            Angle2 = inputString.substring(firstSpaceIndex + 1, secondSpaceIndex).toFloat();
            Angle3 = inputString.substring(secondSpaceIndex + 1, thirdSpaceIndex).toFloat();
            Angle4 = inputString.substring(thirdSpaceIndex + 1, fourthSpaceIndex).toFloat();
            Angle5 = inputString.substring(fourthSpaceIndex + 1, fifthSpaceIndex).toFloat();
            GripperState = inputString.substring(fifthSpaceIndex + 1).toInt();
            
            Move_Robot(Angle1, Angle2, Angle3, Angle4, Angle5, GripperState);

            //print angles
            Serial.print("User Values : [");
            Serial.print(Angle1);
            Serial.print(" ");
            Serial.print(Angle2);
            Serial.print(" ");
            Serial.print(Angle3);
            Serial.print(" ");
            Serial.print(Angle4);
            Serial.print(" ");
            Serial.print(Angle5);
            Serial.print(" ");
            Serial.print(GripperState);
            Serial.println("]");
        }
    inputString = "";
    inputComplete = false;
    }
}