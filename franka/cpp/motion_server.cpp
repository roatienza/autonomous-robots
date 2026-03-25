// sudo apt-get install libcpprest-dev
// g++ -std=c++11  bdm_motion_server.cpp -o bdm_motion_server -lcpprest -lssl -lcrypto -lboost_system
//
// Changes from motion_server.cpp:
//   - Constructor now calls automaticErrorRecovery() on startup to clear any stale error state
//   - moveToCartesian() catches franka::Exception, calls automaticErrorRecovery() to
//     unlock the robot, then returns HTTP 400 with a "collision_recovery:" prefix so
//     the Python side can decide what motion to perform next.
//   - initialize() uses the same goHome() prompt as motion_server (press Enter before moving).

#include <cpprest/http_listener.h>
#include <cpprest/json.h>
#include <iomanip>
#include <mutex>
#include "common.h"

using namespace web;
using namespace web::http;
using namespace web::http::experimental::listener;

class RobotHandler {
    franka::Robot robot;
    franka::Gripper gripper;
    std::mutex robot_mutex;  // libfranka is not thread-safe; serialize all robot calls
public:
    RobotHandler(const std::string& ip_addr) : robot(ip_addr), gripper(ip_addr) {
        // Clear any pre-existing error state left over from a previous run
        try {
            robot.automaticErrorRecovery();
        } catch (const franka::Exception& e) {
            // No error to recover from — this is fine
            std::cerr << "Startup error recovery (may be benign): " << e.what() << std::endl;
        }
        // Note: setDefaultBehavior() intentionally NOT called here.
        // It overrides Desk-configured force thresholds with values that may be
        // too low for the robot's current pose, triggering a false collision stop.
    }

    void initialize() {
        // Same startup behaviour as motion_server: prompt user before moving.
        // If the arm can't reach home (obstacle, joint limit), abort with a clear message.
        try {
            goHome(robot);
        } catch (const franka::Exception& e) {
            throw std::runtime_error(
                std::string("initialize(): failed to move to home position: ") + e.what());
        }
        gripper.homing();

        franka::Model model = robot.loadModel();
        const franka::RobotState& robot_state = robot.readOnce();
        std::array<double, 16> initial_pose = robot_state.O_T_EE;
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
            std::cout << "Alpha(Z): " << Alpha * 180 / M_PI << ", Beta(Y): " << Beta * 180 / M_PI << ", Gamma(X): " << Gamma * 180 / M_PI << " degrees" << std::endl;
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
        std::lock_guard<std::mutex> lock(robot_mutex);
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
        const double DEFAULT_MOTION_TIME = 5.0;

        if (numbers.size() < 4)
        {
            tf = DEFAULT_MOTION_TIME;
        }
        else if (numbers.size() >= 4)
        {
            tf = numbers[3];
            if (tf < DEFAULT_MOTION_TIME)
            {
            tf = DEFAULT_MOTION_TIME;
            }
        }

        if (numbers.size() >= 5)
        {
            deltaAlpha = numbers[4];
            if (deltaAlpha < -90 || deltaAlpha > 90)
            {
                deltaAlpha = 0.0;
                std::cerr << "Error: deltaAlpha must be between -90 and 90 degrees." << std::endl;
            }
            deltaAlpha = deltaAlpha * M_PI / 180;
            is_rotation = true;
        }

        if (numbers.size() >= 6)
        {
            deltaBeta = numbers[5];
            if (deltaBeta < -90 || deltaBeta > 90)
            {
                deltaBeta = 0.0;
                std::cerr << "Error: deltaBeta must be between -90 and 90 degrees." << std::endl;
            }
            deltaBeta = deltaBeta * M_PI / 180;
        }
        if (numbers.size() >= 7)
        {
            deltaGamma = numbers[6];
            if (deltaGamma < -90 || deltaGamma > 90)
            {
                deltaGamma = 0.0;
                std::cerr << "Error: deltaGamma must be between -90 and 90 degrees." << std::endl;
            }
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
                        std::tuple<double, double, double> angles = getRotationAngles(&initial_pose, false);
                        Alpha = std::get<0>(angles);
                        Beta = std::get<1>(angles);
                        Gamma = std::get<2>(angles);

                        Alphaf = Alpha + deltaAlpha;
                        Betaf = Beta + deltaBeta;
                        Gammaf = Gamma + deltaGamma;
                    }
                }

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
            std::cerr << "franka::Exception during motion: " << ex.what() << std::endl;

            // Unlock the robot so it can accept new commands.
            // The Python side is responsible for any recovery motion after this.
            try {
                std::cerr << "Attempting automatic error recovery..." << std::endl;
                robot.automaticErrorRecovery();
                std::cerr << "Robot unlocked. Returning 400 to caller." << std::endl;
            } catch (const franka::Exception& recovery_ex) {
                std::cerr << "Error recovery failed: " << recovery_ex.what() << std::endl;
                throw std::runtime_error(
                    std::string("collision_recovery_failed: original=") + ex.what() +
                    " recovery=" + recovery_ex.what());
            }

            // The "collision_recovery:" prefix lets mainutilsedgrasper.py detect this case.
            throw std::runtime_error(std::string("collision_recovery: ") + ex.what());
        }

        final_pose = robot.readOnce().O_T_EE;
        std::tuple<double, double, double> final_angles = getRotationAngles();
        final_coords[0] = final_pose[12];
        final_coords[1] = final_pose[13];
        final_coords[2] = final_pose[14];
        final_coords[3] = std::get<0>(final_angles) * 180.0 / M_PI;
        final_coords[4] = std::get<1>(final_angles) * 180.0 / M_PI;
        final_coords[5] = std::get<2>(final_angles) * 180.0 / M_PI;
        return final_coords;
    }

    std::string closeGripper(const std::vector<float> &numbers) {
        std::lock_guard<std::mutex> lock(robot_mutex);
        try {
            double grasping_width = numbers[0];
            franka::GripperState gripper_state = gripper.readOnce();
            if (gripper_state.max_width < grasping_width) {
                return "Object is too large for the current fingers on the gripper: " + std::to_string(gripper_state.max_width);
            }
            if (!gripper.grasp(grasping_width, 0.1, 0.0, 0.01, 0.01)) {
                return "Failed to grasp object.";
            }
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
        std::lock_guard<std::mutex> lock(robot_mutex);
        try
        {
            franka::GripperState gripper_state = gripper.readOnce();
            double speed = numbers[0];
            std::cout << "Grasped object, will release it now." << std::endl;
            gripper.move(gripper_state.max_width, speed);
            gripper.stop();
        }
        catch (franka::Exception const &e)
        {
            return e.what();
        }
        return "Gripper opened successfully.";
    }

    // Returns current end-effector pose: [x, y, z, alpha_deg, beta_deg, gamma_deg].
    // Used by Python to obtain the actual current rotation before issuing a reset move,
    // since after a collision the arm may have stopped at an unknown intermediate angle.
    std::vector<double> readState()
    {
        std::lock_guard<std::mutex> lock(robot_mutex);
        const franka::RobotState& state = robot.readOnce();
        std::array<double, 16> pose = state.O_T_EE;
        auto [alpha, beta, gamma] = getRotationAngles(&pose, false);
        return {pose[12], pose[13], pose[14],
                alpha * 180.0 / M_PI,
                beta  * 180.0 / M_PI,
                gamma * 180.0 / M_PI};
    }
};

class RobotRestAPI {
public:
    RobotRestAPI(utility::string_t url, const std::string& robot_ip)
        : m_listener(url), robotHandler(robot_ip) {
        m_listener.support(methods::POST, std::bind(&RobotRestAPI::handle_post, this, std::placeholders::_1));
        robotHandler.initialize();
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
                        for (int i = 0; i < 6; ++i) {
                            response[key][i] = json::value::number(final_coords[i]);
                        }
                    } else if (key == "closeGripper") {
                        std::string message = robotHandler.closeGripper(numbers);
                        response[key] = json::value::string(message);
                    } else if (key == "openGripper") {
                        std::string message = robotHandler.openGripper(numbers);
                        response[key] = json::value::string(message);
                    } else if (key == "readState") {
                        std::vector<double> state = robotHandler.readState();
                        for (int i = 0; i < 6; ++i) {
                            response[key][i] = json::value::number(state[i]);
                        }
                    } else {
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
    RobotHandler robotHandler;
};


#include <iostream>

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
