// sudo apt-get install libcpprest-dev
// g++ -std=c++11  motion_server.cpp -o float_api -lcpprest -lssl -lcrypto -lboost_system
// sudo apt install libcpprest-dev 
// test: 
// curl -X POST http://192.168.1.200:34568 -H "Content-Type: application/json" -d "[0.5, 0, 0.3]"
//

#include <cpprest/http_listener.h>
#include <cpprest/json.h>
#include <iomanip>  // For std::setprecision
#include "common.h"

using namespace web;
using namespace web::http;
using namespace web::http::experimental::listener;

class RobotHandler {
    // Declare robot as a member variable
    franka::Robot robot;
    franka::Gripper gripper;
public:
    // FIXME: Add a try-catch block to catch exceptions thrown by the robot
    RobotHandler(const std::string& ip_addr) : robot(ip_addr), gripper(ip_addr) { // Initialize robot in the initializer list
    }
    void initialize() {     
       	goHome(robot);
        gripper.homing();
        
        franka::Model model = robot.loadModel();
        const franka::RobotState& robot_state = robot.readOnce();
        std::array<double, 16> initial_pose = robot_state.O_T_EE; //robot_state.O_T_EE_c;
        for(int i = 0; i < 16; i++)
        {
            std::cout << initial_pose[i] << ", ";
            if (i % 4 == 3)
            {
                std::cout << std::endl;
            }
        }
        getRotationAngles();
    }

    
    std::tuple<double, double, double> getRotationAngles(const std::array<double, 16>* custom_pose = nullptr, bool verbose = true) {
        std::array<double, 16> pose;
        if (custom_pose) {
            pose = *custom_pose;
        } else {
            const franka::RobotState& robot_state = robot.readOnce();
            pose = robot_state.O_T_EE;
        }
        
        double Beta = atan2(-pose[2], sqrt(pow(pose[0], 2) + pow(pose[1], 2)));
        double Alpha = 0.0;
        double Gamma = 0.0;
        double cosBeta = cos(Beta);
        if (abs(cosBeta) < 1e-5) {
            if (Beta > 0){
                Beta = M_PI_2;
                Gamma = atan2(pose[4], pose[5]);
            }else{
                Beta = -M_PI_2;
                Gamma = -atan2(pose[4], pose[5]);
            }
        }else{
            Alpha = atan2(pose[1]/cosBeta, pose[0]/cosBeta);
            Gamma = atan2(pose[6]/cosBeta, pose[10]/cosBeta);
        }
        
        if (verbose) {
            std::cout << std::fixed << std::setprecision(2);
            std::cout << "Alpha(Z): " << Alpha << ", Beta(Y): " << Beta << ", Gamma(X): " << Gamma << " radians" << std::endl;
            // in degrees
            std::cout << "Alpha(Z): " << Alpha * 180 / M_PI << ", Beta(Y): " << Beta * 180 / M_PI << ", Gamma(X): " << Gamma * 180 / M_PI << " degrees" << std::endl;
            //std::cout << "cosBeta: " << cos(Beta) << std::endl;
            // xyz coordinates
            std::cout << "X: " << pose[12] << ", Y: " << pose[13] << ", Z: " << pose[14] << " meters" <<  std::endl;

        }
        return std::make_tuple(Alpha, Beta, Gamma);
    }


    bool isValidCartesianPose(const std::vector<float>& numbers) {
        const double sphere_radius = 0.855;
        double xf = numbers[0];
        double yf = numbers[1];
        double zf = numbers[2];
        if (pow(xf, 2) + pow(yf, 2) + pow(zf, 2) > pow(sphere_radius, 2)){
            throw std::runtime_error("The desired position is outside the workspace.");
            return false;
        }
        if (zf < 0.015){
            throw std::runtime_error("The desired position is too close to the table.");
            return false;
        }
        return true;
    }

    // moveToCartesian accepts three float numbers in a vector
    std::vector<double> moveToCartesian(const std::vector<float> &numbers)
    {
        // vector of 3 double initialized to all zeros
        std::vector<double> final_coords(6, 0.0);
        if (numbers.size() < 3)
        {
            throw std::runtime_error("Please provide at least 3 float numbers (x,y,z,t).");
            return final_coords;
        }
        if (!isValidCartesianPose(numbers))
        {
            return final_coords;
        }
        double xf = numbers[0];
        double yf = numbers[1];
        double zf = numbers[2];
        std::array<double, 16> initial_pose;
        std::array<double, 16> final_pose;
        double time = 0.0;
        double tf = 0.0;
        double Alpha = 0.0, Beta = 0.0, Gamma = 0.0l;
        double deltaAlpha = 0.0, deltaBeta = 0.0, deltaGamma = 0.0;
        double Alphaf = 0.0, Betaf = 0.0, Gammaf = 0.0;
        bool is_rotation = false;
        // Define a default constant for time
        const double DEFAULT_MOTION_TIME = 5.0;

        if (numbers.size() < 4)
        {
            tf = DEFAULT_MOTION_TIME; // Use the default time duration
        }
        else if (numbers.size() >= 4)
        {
            tf = numbers[3];
            // Check if tf is less than the default time
            if (tf < DEFAULT_MOTION_TIME)
            {
            tf = DEFAULT_MOTION_TIME;
            }
        }

        if (numbers.size() >= 5)
        {
            deltaAlpha = numbers[4];
            // must be between -180 and 180
            if (deltaAlpha < -180 || deltaAlpha > 180)
            {
                deltaAlpha = 0.0;
                // print error
                std::cerr << "Error: deltaAlpha must be between -180 and 180 degrees." << std::endl;
            }
            //in radians
            deltaAlpha = deltaAlpha * M_PI / 180;
            is_rotation = true;
        }

        if (numbers.size() >= 6)
        {
            deltaBeta = numbers[5];
            // must be between -180 and 180
            if (deltaBeta < -180 || deltaBeta > 180)
            {
                deltaBeta = 0.0;
                // print error
                std::cerr << "Error: deltaBeta must be between -180 and 180 degrees." << std::endl;
            }
            //in radians
            deltaBeta = deltaBeta * M_PI / 180;
        }
        if (numbers.size() >= 7)
        {
            deltaGamma = numbers[6];
            // must be between -180 and 180
            if (deltaGamma < -180 || deltaGamma > 180)
            {
                deltaGamma = 0.0;
                // print error
                std::cerr << "Error: deltaGamma must be between -180 and 180 degrees." << std::endl;
            }
            //in radians
            deltaGamma = deltaGamma * M_PI / 180;
        }

        try
        {
            auto trajectory_callback = [this, xf, yf, zf, tf, is_rotation, deltaAlpha, deltaBeta, deltaGamma, 
                                        &time, &Alpha, &Beta, &Gamma, &Alphaf, &Betaf, &Gammaf, &initial_pose, &final_pose](
                                           const franka::RobotState &robot_state,
                                           franka::Duration period) -> franka::CartesianPose
            {
                time += period.toSec();
                
                if (time == 0.0)
                {
                    // Read the initial pose to start the motion from in the first time step.
                    initial_pose = robot_state.O_T_EE;
                    for(int i = 0; i < 16; i++)
                    {
                        std::cout << initial_pose[i] << ", ";
                        if (i % 4 == 3)
                        {
                            std::cout << std::endl;
                        }
                    }
                    if (is_rotation){
                        // get the rotation angles
                        std::tuple<double, double, double> angles = getRotationAngles(&initial_pose, false);
                        Alpha = std::get<0>(angles);
                        Beta = std::get<1>(angles);
                        Gamma = std::get<2>(angles);
                        
                        // Calculate final rotation angles
                        Alphaf = Alpha + deltaAlpha;
                        Betaf = Beta + deltaBeta;
                        Gammaf = Gamma + deltaGamma;
                    }
                }

                // cubic polynomial trajectory
                franka::CartesianPose pose_desired = initial_pose;
                double t2 = pow(time, 2);
                double t3 = pow(time, 3);
                double tf2 = pow(tf, 2);
                double tf3 = pow(tf, 3);

                double x0 = pose_desired.O_T_EE[12];
                double a2 = 3 * (xf - x0) / tf2;
                double a3 = -2 * (xf - x0) / tf3;
                double xt = x0 + a2 * t2 + a3 * t3;
                double vx = 2 * a2 * time + 3 * a3 * t2;

                double y0 = pose_desired.O_T_EE[13];
                a2 = 3 * (yf - y0) / tf2;
                a3 = -2 * (yf - y0) / tf3;
                double yt = y0 + a2 * t2 + a3 * t3;
                double vy = 2 * a2 * time + 3 * a3 * t2;

                double z0 = pose_desired.O_T_EE[14];
                a2 = 3 * (zf - z0) / tf2;
                a3 = -2 * (zf - z0) / tf3;
                double zt = z0 + a2 * t2 + a3 * t3;
                double vz = 2 * a2 * time + 3 * a3 * t2;
                bool stop = fabs(vx) < 0.0001 && fabs(vy) < 0.0001 && fabs(vz) < 0.0001 && time > 1.0;

                pose_desired.O_T_EE[12] = xt;
                pose_desired.O_T_EE[13] = yt;
                pose_desired.O_T_EE[14] = zt;

                if(is_rotation){
                    // Rotation
                    a2 = 3 * (Alphaf - Alpha) / tf2;
                    a3 = -2 * (Alphaf - Alpha) / tf3;
                    double Alphat = Alpha + a2 * t2 + a3 * t3;
                    double vAlpha = 2 * a2 * time + 3 * a3 * t2;

                    a2 = 3 * (Betaf - Beta) / tf2;
                    a3 = -2 * (Betaf - Beta) / tf3;
                    double Betat = Beta + a2 * t2 + a3 * t3;
                    double vBeta = 2 * a2 * time + 3 * a3 * t2;

                    a2 = 3 * (Gammaf - Gamma) / tf2;
                    a3 = -2 * (Gammaf - Gamma) / tf3;
                    double Gammat = Gamma + a2 * t2 + a3 * t3;
                    double vGamma = 2 * a2 * time + 3 * a3 * t2;

                    // Rotation matrix
                    pose_desired.O_T_EE[0] = cos(Alphat) * cos(Betat);
                    pose_desired.O_T_EE[1] = sin(Alphat) * cos(Betat);
                    pose_desired.O_T_EE[2] = -sin(Betat);
                    pose_desired.O_T_EE[4] = cos(Alphat) * sin(Betat) * sin(Gammat) - sin(Alphat) * cos(Gammat);
                    pose_desired.O_T_EE[5] = sin(Alphat) * sin(Betat) * sin(Gammat) + cos(Alphat) * cos(Gammat);
                    pose_desired.O_T_EE[6] = cos(Betat) * sin(Gammat);
                    pose_desired.O_T_EE[8] = cos(Alphat) * sin(Betat) * cos(Gammat) + sin(Alphat) * sin(Gammat);
                    pose_desired.O_T_EE[9] = sin(Alphat) * sin(Betat) * cos(Gammat) - cos(Alphat) * sin(Gammat);
                    pose_desired.O_T_EE[10] = cos(Betat) * cos(Gammat);
                    stop = stop && fabs(vAlpha) < 0.0001 && fabs(vBeta) < 0.0001 && fabs(vGamma) < 0.0001;
                }

                if (time >= tf || stop)
                {
                    std::cout << std::endl << time << "sec : End of motion ............." << std::endl;
                    final_pose = pose_desired.O_T_EE;
                    return franka::MotionFinished(pose_desired);
                }

                return pose_desired;
            };

            robot.control(trajectory_callback);
        }
        catch (const franka::Exception &ex)
        {
            std::cerr << ex.what() << std::endl;
        }

        // Initialize final_pose with the robot's final pose
        final_pose = robot.readOnce().O_T_EE;
        // call getRotationAngles with final_pose
        std::tuple<double, double, double> final_angles = getRotationAngles();
        final_coords[0] = final_pose[12];
        final_coords[1] = final_pose[13];
        final_coords[2] = final_pose[14];
        final_coords[3] = std::get<0>(final_angles);
        final_coords[4] = std::get<1>(final_angles);
        final_coords[5] = std::get<2>(final_angles);
        return final_coords;
    }

    std::string closeGripper(const std::vector<float> &numbers) {
        try {    
            double grasping_width = numbers[0];
            // Check for the maximum grasping width.
            franka::GripperState gripper_state = gripper.readOnce();
                if (gripper_state.max_width < grasping_width) {
                return "Object is too large for the current fingers on the gripper: " + std::to_string(gripper_state.max_width);
            }

            // Grasp the object.
            //bool grasp(double width,
            //double speed,
            //double force,
            //double epsilon_inner = 0.005,
            //double epsilon_outer = 0.005) const;
            
            if (!gripper.grasp(grasping_width, 0.1, 0.0, 0.01, 0.01)) {
                return "Failed to grasp object.";
            }

            // Wait 3s and check afterwards, if the object is still grasped.
            std::this_thread::sleep_for(std::chrono::duration<double, std::milli>(100));

            gripper_state = gripper.readOnce();
            if (!gripper_state.is_grasped) {
                return "Object lost.";
            }
        } catch (franka::Exception const& e) {
            return e.what();
        }
        return "Object grasped successfully.";
    }

    std::string openGripper(const std::vector<float> &numbers)
    {
        try
        {
            franka::GripperState gripper_state = gripper.readOnce();
            double speed = numbers[0];
            std::cout << "Grasped object, will release it now." << std::endl;
            //bool move(double width, double speed) const;

            gripper.move(gripper_state.max_width, speed);
            gripper.stop();
        }
        catch (franka::Exception const &e)
        {
            return e.what();
        }
        return "Gripper opened successfully.";
    }
};

class RobotRestAPI {
public:
    RobotRestAPI(utility::string_t url, const std::string& robot_ip) 
        : m_listener(url), robotHandler(robot_ip) {
        m_listener.support(methods::POST, std::bind(&RobotRestAPI::handle_post, this, std::placeholders::_1));
        robotHandler.initialize(); // Initialize the robot
    }

    pplx::task<void> open() { return m_listener.open(); }
    pplx::task<void> close() { return m_listener.close(); }

private:
    void handle_post(http_request request) {
        request
            .extract_json()
            .then([](json::value body)
                  {
                      std::map<std::string, std::vector<float>> data;
                      for (const auto& item : body.as_object())
                      {
                          if (!item.second.is_array())
                          {
                              throw std::runtime_error("All values must be arrays.");
                          }
                          std::vector<float> numbers;
                          for (const auto& num : item.second.as_array())
                          {
                              if (!num.is_number())
                              {
                                  throw std::runtime_error("All array elements must be numbers.");
                              }
                              numbers.push_back(static_cast<float>(num.as_double()));
                          }
                          data[item.first] = numbers;
                      }
                      return data;
                  })
            .then([this](std::map<std::string, std::vector<float>> data)
                  {
                json::value response;
                for (const auto& item : data)
                {
                    const std::string& key = item.first;
                    const std::vector<float>& numbers = item.second;

                    if (key == "moveToCartesian") {
                        std::vector<double> final_coords = robotHandler.moveToCartesian(numbers);
                        // convert the final_coords to a json array
                        for (int i = 0; i < 6; ++i) {
                            response[key][i] = json::value::number(final_coords[i]);
                        }
                    } else if (key == "closeGripper") {
                        std::string message = robotHandler.closeGripper(numbers);
                        response[key] = json::value::string(message);
                    } else if (key == "openGripper") {
                        std::string message = robotHandler.openGripper(numbers);
                        response[key] = json::value::string(message);
                    }else {
                        std::cout << "Invalid command: " << key << std::endl;
                        response["Response"] = json::value::string("Invalid command.");
                    }
                }
                return response; })
            .then([=](json::value response)
                  { request.reply(status_codes::OK, response); })
            .then([=](pplx::task<void> t)
                  {
                try {
                    t.get();
                }
                catch (const std::exception &e) {
                    request.reply(status_codes::BadRequest, json::value::string(e.what()));
                } });
    }

    http_listener m_listener;
    RobotHandler robotHandler; // Add RobotHandler as a member variable
};;


#include <iostream> // Add this include for std::cerr and std::cout

int main(int argc, char* argv[]) {
    std::string port = "34568";
    std::string address = "http://0.0.0.0:" + port;
    utility::string_t utility_address = utility::conversions::to_string_t(address);
    std::string robot_ip = "192.168.2.100";

    if (argc == 2) {
        robot_ip = argv[1];
    } 

    RobotRestAPI api(utility_address, robot_ip);
    
    try {
        api.open().wait();
        std::cout << "Listening at: " << address << std::endl;
        std::cout << "Press ENTER to exit." << std::endl;
        std::string line;
        std::getline(std::cin, line);
        api.close().wait();
    }
    catch (std::exception const & e) {
        std::cout << e.what() << std::endl;
    }

    return 0;
}
