

#include <franka/robot.h>
#include <franka/gripper.h>


//create a main function
int main() {

    // declare a variable to store ip
    std::string ip = "192.168.1.131";

    //franka::Gripper gripper("<fci-ip>");
    franka::Robot robot(ip);
    robot.automaticErrorRecovery();

    return 0;
}