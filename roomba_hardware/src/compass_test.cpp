#include <ros/ros.h>
#include <unistd.h>
#include <vector>
#include <iostream>
#include <cmath>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <std_msgs/Float32.h>

const int kIntCtrlAddr = 0x09;
const int kStatusAddr = 0x08;

const int kMeasureDurationUs = 8000;

struct Calibration {
    std::array<std::array<double,3>,3> soft;
    std::array<double,3> hard;
};


class Compass {

    private:
    int devId;
    ros::Publisher compass_pub, compass_raw_pub, heading_pub;
    std::vector<double> hard_, soft_;

    public:
    Compass(ros::NodeHandle& nh) {

        ros::NodeHandle priv_nh("~");

        XmlRpc::XmlRpcValue calibrations;
        priv_nh.getParam("~calibrations", calibrations);

        for (auto entry: calibrations) 
        {
            auto calibration = entry.second;
            Calibration c;
            for(int i=0; i<3; i++)
                c.hard[i] = calibration["hard"][i];
            for(int y=0; y<3; y++)
                for(int x=0; x<3; x++)
                    c.soft[y][x] = calibration["soft"][y][x];
        }

        // compass_raw_pub = nh.advertise<geometry_msgs::Vector3Stamped>("/compass_raw", 1);
        // compass_pub = nh.advertise<geometry_msgs::Vector3>("/compass", 1);
        // heading_pub = nh.advertise<std_msgs::Float32>("/heading", 1);
    }

};

int main (int argc, char **argv)
{
    ros::init(argc, argv, "compass_test");
    ros::NodeHandle nh;
    Compass compass = Compass(nh);

    // try {
    //     compass.spin();
    // } catch (const std::runtime_error& ex) {
    //     ROS_FATAL_STREAM("[Compass] Runtime error: " << ex.what());
    //     return 1;
    // }
    
    return 0;
}