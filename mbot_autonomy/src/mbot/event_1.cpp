#include <utils/lcm_config.h>
#include <mbot/mbot_channels.h>
#include <mbot_lcm_msgs/path2D_t.hpp>
#include <lcm/lcm-cpp.hpp>
#include <iostream>
#include <unistd.h>
#include <cmath>
#include <chrono>
using namespace std::chrono;


int main(int argc, char** argv)
{
    std::cout << "Executing event 1. " << "\n";
    auto start = high_resolution_clock::now();
    // std::cout << "Executing the checkpoint 1 path. " << "\n";

    mbot_lcm_msgs::path2D_t path;
    path.path.resize(9);

    mbot_lcm_msgs::pose2D_t nextPose;


    float easy_path[12][2] = {   {0.00, 0.00},
                                {0.61, 0.00},
                                {0.61, 0.61},
                                {1.22, 0.61},
                                {1.22, 0.00},
                                {1.83, 0.00},
                                {1.83, 1.22},
                                {0.00, 1.22},
                                {0.00, 0.00}}; 
                                // {0.00, 0.10}}; //slow_speed

    for(int i = 0; i < 9; i++){
        nextPose.x = easy_path[i][0];
        nextPose.y = easy_path[i][1];
        nextPose.theta = 0.0f;
        path.path[i] = nextPose;
    }

    path.path.insert(path.path.begin(), nextPose);

    path.path_length = path.path.size();

    lcm::LCM lcmInstance(MULTICAST_URL);
    std::cout << "publish to: " << CONTROLLER_PATH_CHANNEL << std::endl;
    lcmInstance.publish(CONTROLLER_PATH_CHANNEL, &path);
    sleep(1);

    return 0;
    auto stop = high_resolution_clock::now();
    auto duration = duration_cast<microseconds>(stop - start);
    std::cout << duration.count() << std::endl;
}
