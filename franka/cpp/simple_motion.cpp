// Copyright (c) 2023 Franka Robotics GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#include <array>
#include <atomic>
#include <cmath>
#include <functional>
#include <iostream>
#include <iterator>
#include <mutex>
#include <thread>

#include <franka/duration.h>
#include <franka/exception.h>
#include <franka/model.h>
#include <franka/rate_limiting.h>
#include <franka/robot.h>

#include "common.h"

namespace
{
  template <class T, size_t N>
  std::ostream &operator<<(std::ostream &ostream, const std::array<T, N> &array)
  {
    ostream << "[";
    std::copy(array.cbegin(), array.cend() - 1, std::ostream_iterator<T>(ostream, ","));
    std::copy(array.cend() - 1, array.cend(), std::ostream_iterator<T>(ostream));
    ostream << "]";
    return ostream;
  }
} // anonymous namespace

/**
 * @example joint_impedance_control.cpp
 * An example showing a joint impedance type control that executes a Cartesian motion in the shape
 * of a circle. The example illustrates how to use the internal inverse kinematics to map a
 * Cartesian trajectory to joint space. The joint space target is tracked by an impedance control
 * that additionally compensates coriolis terms using the libfranka model library. This example
 * also serves to compare commanded vs. measured torques. The results are printed from a separate
 * thread to avoid blocking print functions in the real-time loop.
 */



int main(int argc, char **argv)
{
  // Check whether the required arguments were passed.
  if (argc != 6)
  {
    std::cerr << "Usage: " << argv[0] << " <robot-ip> <x-coord> <y-coord> <z-coord> <exec_time>"  << std::endl;
    std::cout << "Executing default home motion" << std::endl;
    try
    {
      franka::Robot robot(argv[1]);
      goHome(robot);
    }
    catch (const franka::Exception &ex)
    {
      std::cerr << ex.what() << std::endl;
    }
    return -1;
  }

  double time = 0.0;
  double tf = 5.0;
  double xf = 0.0;
  double yf = 0.0;
  double zf = 0.0;
  const double print_rate = 10.0;
  const double sphere_radius = 0.855;
  franka::RobotState robot_state;
  std::array<double, 16> initial_pose;
  // Initialize data fields for the print thread.
  struct
  {
    std::mutex mutex;
    bool has_data;
    franka::RobotState robot_state;
    double time;
    std::array<double, 7> tau_d_last;
    std::array<double, 7> gravity;
  } print_data{};
  std::atomic_bool running{true};


  try {
    xf = std::stod(argv[2]);
    yf = std::stod(argv[3]);
    zf = std::stod(argv[4]);
    tf = std::stod(argv[5]);
  } catch (const std::invalid_argument& ia) {
    std::cerr << "Invalid argument: " << ia.what() << std::endl;
    return -1;
  }

  // check if xf, yf, zf are within the sphere
  if (pow(xf, 2) + pow(yf, 2) + pow(zf, 2) > pow(sphere_radius, 2)){
    std::cerr << "Invalid argument: (" << xf << "," << yf << "," << zf << ") is outside the sphere of radius " << sphere_radius << std::endl;
    return -1;
  }else{
    std::cout << "Valid argument: (" << xf << "," << yf << "," << zf << ") is inside the sphere of radius " << sphere_radius << std::endl;
  }

  // zf must be greater than 0.05
  if (zf < 0.015){
    std::cerr << "Invalid argument: z must be greater than 0.01" << std::endl;
    return -1;
  }


  // Start print thread.
  std::thread print_thread([print_rate, &print_data, &running]()
                           {
    while (running) {
      // Sleep to achieve the desired print rate.
      std::this_thread::sleep_for(
          std::chrono::milliseconds(static_cast<int>((1.0 / print_rate * 1000.0))));

      // Try to lock data to avoid read write collisions.
      if (print_data.mutex.try_lock() && print_data.has_data) {
        franka::CartesianPose pose_desired = print_data.robot_state.O_T_EE_c;
        // print pose_desired one element at a time
        std::cout << "Pose desired: [";
        for (size_t i = 0; i < 15; i++) {
          std::cout << pose_desired.O_T_EE[i] << ", ";
        }
        std::cout << pose_desired.O_T_EE[15] << "]" << std::endl;
        std::cout << "Time: " << print_data.time << std::endl;
        print_data.has_data = false;
        print_data.mutex.unlock();
      }
    }
    std::cout << "End printing" << std::endl; });

  try
  {
    // Connect to robot.
    franka::Robot robot(argv[1]);
    setDefaultBehavior(robot);

    // Set additional parameters always before the control loop, NEVER in the control loop!
    // Set collision behavior.
    robot.setCollisionBehavior(
        {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}}, {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}},
        {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}}, {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}},
        {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}}, {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}},
        {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}}, {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}});

    franka::Model model = robot.loadModel();

    robot_state = robot.readOnce();
    // retrieve robot state
    if (print_data.mutex.try_lock())
    {
      print_data.has_data = true;
      print_data.robot_state = robot_state;
      print_data.mutex.unlock();
    }

    auto trajectory_callback = [=,  &time, &xf, &yf, &zf, &tf, &initial_pose, &print_data, &running](
                                   const franka::RobotState &robot_state,
                                   franka::Duration period) -> franka::CartesianPose
    {
      // Update time.
      time += period.toSec();

      if (time == 0.0){
        // Read the initial pose to start the motion from in the first time step.
        initial_pose = robot_state.O_T_EE_c;
      }

      franka::CartesianPose pose_desired = initial_pose;
      franka::CartesianPose start_pose = initial_pose;
      double t2 = pow(time, 2);
      double t3 = pow(time, 3);
      double tf2 = pow(tf, 2);
      double tf3 = pow(tf, 3);
      
      double x0 = pose_desired.O_T_EE[12];
      double a2 = 3 * (xf - x0) / tf2;
      double a3 = -2 * (xf - x0) / tf3;
      double xt = x0 + a2*t2 +  a3*t3;
      double vx = 2*a2*time + 3*a3*t2;

      double y0 = pose_desired.O_T_EE[13];
      a2 = 3 * (yf - y0) / tf2;
      a3 = -2 * (yf - y0) / tf3;
      double yt = y0 + a2*t2 +  a3*t3;
      double vy = 2*a2*time + 3*a3*t2;

      double z0 = pose_desired.O_T_EE[14];
      a2 = 3 * (zf - z0) / tf2;
      a3 = -2 * (zf - z0) / tf3;
      double zt = z0 + a2*t2 +  a3*t3;
      double vz = 2*a2*time + 3*a3*t2;
      

      pose_desired.O_T_EE[12] = xt;
      pose_desired.O_T_EE[13] = yt;
      pose_desired.O_T_EE[14] = zt;

      bool stop = fabs(vx) < 0.0001 && fabs(vy) < 0.0001 && fabs(vz) < 0.0001 && time > 1.0;

      if (time >= tf || stop)
      {
        running = false;
        std::cout << time << ": End of motion ............." << std::endl;
        return franka::MotionFinished(pose_desired);
      }

      // Update data to print.
      if (print_data.mutex.try_lock()){
        print_data.has_data = true;
        print_data.robot_state = robot_state;
        print_data.time = time;
        print_data.mutex.unlock();
      }else{
        double x = start_pose.O_T_EE[12];
        double y = start_pose.O_T_EE[13];
        double z = start_pose.O_T_EE[14];
        std::cout << time << ": Init Pose: (" << x << "," << y << "," << z << "), Curr Pose: (" << xt << "," << yt << "," << zt << "), Curr Vel: (" << vx << "," << vy << "," <<  vz << ")"<<  std::endl;
      }
      return pose_desired;
    };

    robot.control(trajectory_callback);
  }
  catch (const franka::Exception &ex)
  {
    running = false;
    std::cerr << ex.what() << std::endl;
  }

  if (print_thread.joinable())
  {
    print_thread.join();
  }
  return 0;
}
