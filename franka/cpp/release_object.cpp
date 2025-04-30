// Copyright (c) 2023 Franka Robotics GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#include <iostream>
#include <sstream>
#include <string>
#include <thread>

#include <franka/exception.h>
#include <franka/gripper.h>

/**
 * @example grasp_object.cpp
 * An example showing how to control FRANKA's gripper.
 */

int main(int argc, char** argv) {
  if (argc != 2) {
    std::cerr << "Usage: ./grasp_object <gripper-hostname>" << std::endl;
    return -1;
  }

  try {
    franka::Gripper gripper(argv[1]);
    // Check for the maximum grasping width.
    franka::GripperState gripper_state = gripper.readOnce();
    
    std::cout << "Grasped object, will release it now." << std::endl;
    gripper.move(gripper_state.max_width, 0.1);
    gripper.stop();
  } catch (franka::Exception const& e) {
    std::cout << e.what() << std::endl;
    return -1;
  }

  return 0;
}
