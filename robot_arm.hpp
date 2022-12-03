#ifndef ROBOT_ARM_HPP
#define ROBOT_ARM_HPP

#include "motor.hpp"

namespace mech2 {

  // Robot Arm class
  class RobotArm {

    public:

      int motors_size;
      Motor ** motors;

      int path_size;
      float ** path;
      int path_index;

      bool enabled;

      float * state;

      // Constructor
      RobotArm(Motor ** _motors, int num_motors, float ** _path, int num_points) {
        
        // Set member variables

        motors = _motors;
        motors_size = num_motors;

        path = _path;
        path_size = num_points;

        path_index = 0;

        enabled = false;
        
        state = new float[num_motors];

      }

      // Robot arm clock cycle
      void tick() { 

        // Goal check
        if (goalReached())
          path_index++;

        // Path finished check
        if (finished()) {
          pause();
          return;
        }

        // Spin motors
        float * goal = getGoal();
        for (int m = 0; m < motors_size; m++) {
          if (motors[m]->at(goal[m]))
            motors[m]->stop();
          else
            motors[m]->spin(goal[m]);
        }
   
      }

      // Get current state of all motors
      float * getState() {
        for (int m = 0; m < motors_size; m++)
          state[m] = motors[m]->getPosition();
        return state;
      }

      // Get goal state
      float * getGoal() {
        return path[path_index];
      }

      // Finished?
      bool finished() {
        return path_index == path_size;
      }

      // Stop the motors
      void pause() {
        for (int m = 0; m < motors_size; m++)
          motors[m]->stop();
      }

      // Destructor
      ~RobotArm() {
        delete[] state;
      }

    private:

      // Check if goal has been reached
      bool goalReached() {
        float * goal = getGoal();
        bool at_goal = true;
        for (int m = 0; m < motors_size; m++)
          at_goal *= motors[m]->at(goal[m]);
        return at_goal;
      }

  };

}

#endif