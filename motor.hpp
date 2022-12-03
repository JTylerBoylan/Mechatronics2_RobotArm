#ifndef ROBOT_ARM_MOTOR_HPP
#define ROBOT_ARM_MOTOR_HPP

#include <Arduino.h>

// Custom mod function
#define mod(a,b) a%b >= 0 ? a%b : (a%b)+b

// Threshold for goal determination
#define ANGULAR_RESOLUTION 0.02F

// Stepper motor steps
#define NUMBER_OF_STEPS 4
#define STEP_INCREMENT_ANGLE 0.03141593F
const int FullSteps[NUMBER_OF_STEPS] = {0b0101, 0b1001, 0b1010, 0b0110};

// Stepper motor timing delay
#define STEPPER_UPDATE_DELAY 0

// DC motor speed
#define DC_MOTOR_DUTY 255

namespace mech2 {

    // Abstract Motor class
    class Motor {

      public:

        // Number of driver pins
        int pins_size;

        // Motor driver pins
        int * pins;

        // Pin to enable/disable driver
        int enable_pin;

        // Gear ratio
        float gear_ratio;

        //  CCW: -1, CW: 1, None: 0
        int spin_direction;

        // Get motor current position
        virtual float getPosition() = 0;

        // Spin the motor towards a given position
        virtual void spin(float position) = 0;

        // Check if current motor state is at a given position
        virtual bool at(float position) = 0;

        // Stop the motor
        virtual void stop() = 0;

        // Destructor
        ~Motor() {
          delete[] pins;
        }
    };

    // Stepper motor class
    class StepperMotor : public Motor {

      public:

        // Step position
        int step;

        // Timing
        unsigned long time;

        // Constructor
        StepperMotor(int pinA, int pinB, int pinC, int pinD, int pinEN, float gr) {

          // Set member variables
          pins_size = 4;
          pins = new int[pins_size]{pinA, pinB, pinC, pinD};
          enable_pin = pinEN;

          gear_ratio = gr;
          step = 0;

          spin_direction = 0;
                 
          // Set all pins to ouput
          for (int i = 0; i < pins_size; i++)
            pinMode(pins[i], OUTPUT);
            
          // Set enable to output-pulldown
          pinMode(enable_pin, OUTPUT);
          digitalWrite(enable_pin, LOW);
        }

        float getPosition() {
          // Convert step to position
          return STEP_INCREMENT_ANGLE * step / gear_ratio;
        }

        void spin(float position) {

          // Enable driver
          digitalWrite(enable_pin, HIGH);

          // Frequency control
          float dt = micros() - time;
          time = micros();
          if (dt/1000 <= STEPPER_UPDATE_DELAY)
            return;
          
          // Update position
          float error = position - getPosition();
          int spin_direction = error >= 0 ? 1 : -1;
          step += spin_direction;

          // Write to pins          
          for (int i = 0; i < pins_size; i++)
            digitalWrite(pins[i], FullSteps[mod(step, NUMBER_OF_STEPS)] & (0b1 << i));

        }

        bool at(float position) {
          // Stepper motor goal criteria
          return abs(position - getPosition()) < STEP_INCREMENT_ANGLE / 2.0F;
        }

        void stop() {
          // Disable motor driver
          digitalWrite(enable_pin, LOW);
          spin_direction = 0;
        }

    };

    // DC Motor class
    class DCMotor : public Motor {

      public:
   
        // Pointer to motor's feedback value
        float * feedback;

        // Constructor
        DCMotor(int pinA, int pinB, int pinEN, float gr, float &fdbck) {

          // Set member variables
          pins_size = 2;
          pins = new int[pins_size]{pinA, pinB};
          enable_pin = pinEN;
          
          feedback = &fdbck;
          
          gear_ratio = gr;

          spin_direction = 0;
                 
          // Set all pins to output
          for (int i = 0; i < pins_size; i++)
            pinMode(pins[i], OUTPUT);

          // Set enable pin to output-pulldown
          pinMode(enable_pin, OUTPUT);
          digitalWrite(enable_pin, LOW);
        }

        float getPosition() {
          // Convert feedback to actual position
          return *feedback / gear_ratio;
        }

        void spin(float position) {

          // Enable motor driver
          digitalWrite(enable_pin, HIGH);

          // Set duty based on goal direction
          spin_direction = position - getPosition() > 0 ? 1 : -1;
          int duty = spin_direction * DC_MOTOR_DUTY;
          int dutyA = duty >= 0 ? duty : 0;
          int dutyB = duty < 0 ? -duty : 0;

          // Write duties to pins
          analogWrite(pins[0], dutyA);
          analogWrite(pins[1], dutyB);
        }

        bool at(float position) {
          // DC Motor goal criteria
          return abs(position - getPosition()) < ANGULAR_RESOLUTION / 2.0F;
        }

        void stop() {
          // Disable motor driver
          analogWrite(enable_pin, LOW);
          // Set duties to 0
          analogWrite(pins[0], LOW);
          analogWrite(pins[1], LOW);
          spin_direction = 0;
        }

    };

}

#endif