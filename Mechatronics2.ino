#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Encoder.h>

#include "robot_arm.hpp"

using namespace mech2;

#define DEG2RAD(x) x * 0.01745329F

// OLED Properties
#define OLED_WIDTH 128
#define OLED_HEIGHT 64
#define OLED_ADDR  0x3C

// Robot arm enable pin
#define ROBOT_ARM_ENABLE_PIN 13

// Arduino ports for base motor
#define BASE_MOTOR_PIN_1 22
#define BASE_MOTOR_PIN_2 23
#define BASE_MOTOR_PIN_3 24
#define BASE_MOTOR_PIN_4 25
#define BASE_MOTOR_PIN_EN 53

// Arduino ports for puncher motor
#define PUNCH_MOTOR_PIN_1 9
#define PUNCH_MOTOR_PIN_2 10
#define PUNCH_MOTOR_PIN_EN 52

// Arduino ports for arm motor
#define ARM_MOTOR_PIN_1 5
#define ARM_MOTOR_PIN_2 6
#define ARM_MOTOR_PIN_EN 51

// Arm motor encoder feedback
#define ARM_MOTOR_ENCODER_A 2
#define ARM_MOTOR_ENCODER_B 3

// Convert encoder position to degrees
#define ARM_ENCODER_POS2RAD(x) x * 0.09817477F

// Gear ratios for the motors
#define BASE_MOTOR_GEAR_RATIO 3.5F
#define PUNCH_MOTOR_GEAR_RATIO 1.0F
#define ARM_MOTOR_GEAR_RATIO 11250.0F

// Motors
#define NUM_MOTORS 3
Motor * motors[NUM_MOTORS];

// Available points of travel
// Calibration
const float ORIGIN[NUM_MOTORS] = {0, 0, 0};
// Phase 1
const float ENGAGE[NUM_MOTORS] = {0, -1, 0};
const float PRESS[NUM_MOTORS] = {0, -1, DEG2RAD(-35)};
const float PICKUP[NUM_MOTORS] = {0, -1, 0};
// Phase 2
const float SPIN[NUM_MOTORS] = {DEG2RAD(180), -0.5, 0};
// Phase 3
const float DROP1[NUM_MOTORS] = {DEG2RAD(180), 0.25, DEG2RAD(-10)};
const float DROP2[NUM_MOTORS] = {DEG2RAD(180), 0.25, DEG2RAD(-10)};
// Return to origin (w/ drift offset)
const float ORIGIN2[NUM_MOTORS] = {0, 0.25, 0};

// Single motion commands
const float LOAD_PUNCHER[NUM_MOTORS] = {0, 3, 0};
const float UNLOAD_PUNCHER[NUM_MOTORS] = {0, -5, 0};
const float ARM_UP[NUM_MOTORS] = {0, 0, DEG2RAD(60)};
const float ARM_DOWN[NUM_MOTORS] = {0, 0, DEG2RAD(-60)};
const float SPIN_LEFT[NUM_MOTORS] = {DEG2RAD(360), 0, 0};
const float SPIN_RIGHT[NUM_MOTORS] = {DEG2RAD(-360), 0, 0};

// Path points
#define PATH_SIZE 8
float * path[PATH_SIZE] = {ORIGIN, ENGAGE, PRESS, PICKUP, SPIN, DROP1, DROP2, ORIGIN2};

// Single motion path
//#define PATH_SIZE 1
//float * path[PATH_SIZE] = {LOAD_PUNCHER};

// Robot arm
RobotArm * robot_arm;

// OLED Display
Adafruit_SSD1306 display(OLED_WIDTH, OLED_HEIGHT);

// Feedback for puncher
float punch = 0.0F;

// Feedback for PID
float arm_angle = 0.0F;

// Encoder class
Encoder encoder(ARM_MOTOR_ENCODER_A, ARM_MOTOR_ENCODER_B);

// Function to print to display
void print(String body, int size = 1);

// Function to convert a vector to a string
String vec2str(float * vec, int size);

// Setup the robot arm
void setup() {

  // Start Serial
  Serial.begin(9600);
  Serial.flush();
  Serial.println("Serial started on 9600");

  // Begin the OLED Display
  display.begin(SSD1306_SWITCHCAPVCC, OLED_ADDR);
  display.clearDisplay();
  delay(1000);
  print("OLED initialized");
  delay(250);

  // Initialize the motors
  // Base Motor
  motors[0] = new StepperMotor(BASE_MOTOR_PIN_1, BASE_MOTOR_PIN_2, BASE_MOTOR_PIN_3, BASE_MOTOR_PIN_4, BASE_MOTOR_PIN_EN, BASE_MOTOR_GEAR_RATIO);
  // Punch Motor
  motors[1] = new DCMotor(PUNCH_MOTOR_PIN_1, PUNCH_MOTOR_PIN_2, PUNCH_MOTOR_PIN_EN, PUNCH_MOTOR_GEAR_RATIO, punch);
  // Arm Motor
  motors[2] = new DCMotor(ARM_MOTOR_PIN_1, ARM_MOTOR_PIN_2, ARM_MOTOR_PIN_EN, ARM_MOTOR_GEAR_RATIO, arm_angle);
  print("Motors initialized");
  delay(250);
    
  // Initialize the robot arm
  robot_arm = new RobotArm(motors, NUM_MOTORS, path, PATH_SIZE);
  print("Robot Arm initialized");
  delay(250);

  // Initialize start pin
  pinMode(ROBOT_ARM_ENABLE_PIN, INPUT);
  
}

unsigned long time = 0;
#define MICRO2SEC(x) x * 10E-6F
void loop() {

  // Update arm position
  arm_angle = ARM_ENCODER_POS2RAD(encoder.read());

  // Update punch position
  punch += motors[1]->spin_direction * MICRO2SEC(micros() - time);

  // Update time
  time = micros();

  // Stop if finished
  if (robot_arm->finished()) {
    Serial.println("Path finished.");
    print("Path \nfinished", 2);
    delay(5000);
    return;
  }

  // Update enabled pin
  if (digitalRead(ROBOT_ARM_ENABLE_PIN)) {
    robot_arm->enabled = !robot_arm->enabled;
    delay(100);
  }

  // Check if enabled
  if (!robot_arm->enabled) {
    robot_arm->pause();
    print("Waiting for start \nsignal...");
    return;
  }

  // Get current state
  float * state = robot_arm->getState();
  float * goal = robot_arm->getGoal();

  // Print state and goal
  print("Path: " + String(robot_arm->path_index) + "/" + PATH_SIZE + "\n"  
        "S: " + vec2str(state, NUM_MOTORS) + "\n" +
        "G: " + vec2str(goal, NUM_MOTORS) + "\n\n\n" + 
        "Time: " + String(time));

  // Spin motors
  robot_arm->tick();

}

void print(String body, int size = 1) {
  display.clearDisplay();

  // Header
  display.setTextSize(2);
  display.setTextColor(WHITE);
  display.setCursor(0, 0);
  display.println("Robot Arm");

  // BODY
  display.setTextSize(size);
  display.setTextColor(WHITE);
  display.setCursor(0, 17);
  display.println(body);

  display.display();

  //Serial.println(body);
}

String vec2str(float * vec, int size) {
  String str = size > 0 ? String(vec[0]) : String();
  for (int i = 1; i < size; i++)
    str += " " + String(vec[i]);
  return str;
}
