#include <ros/ros.h>
#include <unistd.h>
#include <vector>
#include <iostream>
#include <math.h>
#include <wiringPiI2C.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <std_msgs/Float32.h>

const int kIntCtrlAddr = 0x09;
const int kStatusAddr = 0x08;

const int kMeasureDurationUs = 8000;

struct Calibration {
    std::string name;
    std::array<std::array<double,3>,3> soft;
    std::array<double,3> hard;
};

geometry_msgs::Vector3Stamped to_msg(std::array<float, 3> array)
{
    geometry_msgs::Vector3Stamped msg;
    msg.header.stamp = ros::Time::now();
    msg.vector.x = array[0];
    msg.vector.y = array[1];
    msg.vector.z = array[2];

    return msg;
}


class Compass {

    private:
    int devId;
    ros::Publisher compass_pub, compass_raw_pub, heading_pub;
    std::vector<Calibration> calibrations;

    public:
    Compass(ros::NodeHandle& nh) {
        devId = wiringPiI2CSetup(0x30);

        ros::NodeHandle priv_nh("~");
        XmlRpc::XmlRpcValue calibration_params;
        priv_nh.getParam("calibrations", calibration_params);
        
        for (auto entry: calibration_params) 
        {
            auto calibration = entry.second;
            Calibration c;
            c.name = entry.first;
            for(int i=0; i<3; i++)
                c.hard[i] = calibration["hard"][i];
            for(int y=0; y<3; y++)
                for(int x=0; x<3; x++)
                    c.soft[y][x] = calibration["soft"][y][x];
            calibrations.push_back(c);
        }

        ROS_INFO("Num calibrations: %d", calibrations.size());

        compass_raw_pub = nh.advertise<geometry_msgs::Vector3Stamped>("/compass_raw", 1);
        compass_pub = nh.advertise<geometry_msgs::Vector3>("/compass", 1);
        heading_pub = nh.advertise<std_msgs::Float32>("/heading", 1);
    }

    void spin() {
        ros::Rate rate(30); // ROS Rate at 30Hz
        while (ros::ok()) {
            // Set and measure with positive polarity
            writeReg(devId, kIntCtrlAddr, 1<<3);
            auto setRead = measure();

            // Reset and measure with negative polarity
            writeReg(devId, kIntCtrlAddr, 1<<4);
            auto resetRead = measure();

            // Compute the common component between them
            std::array<float, 3> measurement;
            for(int i=0; i<3; i++)
                measurement[i] = setRead[i] - resetRead[i];


            compass_raw_pub.publish(to_msg(measurement));

            for(Calibration calibration: calibrations)
            {
                std::array<float, 3> shifted;
                for(int i=0; i<3; i++)
                    shifted[i] = measurement[i] - calibration.hard[i];

                std::array<float, 3> corrected;
                for(int x=0; x<3; x++)
                {
                    corrected[x] = 0;
                    for(int y=0; y<3; y++)
                        corrected[x] += shifted[y] * calibration.soft[y][x];
                }

                float radius_error = sqrt(corrected[0] * corrected[0] + corrected[1] * corrected[1]) - 1;
                float total_distance = sqrt(radius_error * radius_error + corrected[2] * corrected[2]);

                ROS_INFO("%s error: %f", calibration.name.c_str(), total_distance);
            }

            // float offsetX = measurement.vector.x - hard_[0];
            // float offsetY = measurement.vector.y - hard_[1];

            // float x = soft_[0] * offsetX + soft_[2] * offsetY;
            // float y = soft_[1] * offsetX + soft_[3] * offsetY;

            // measurement.vector.x = x;
            // measurement.y = y;
            // measurement.z = 0;
            // compass_pub.publish(measurement);

            // ROS_INFO("%f", x*x + y*y);

            // if(x*x + y*y > 1.2 || x*x + y*y < 0.8) {
            //     ROS_ERROR_THROTTLE(1, "[Compass] Impossible magnetometer value! If this continues, recalibrate magnetometer");
            // } else {
            //     std_msgs::Float32 msg;
            //     msg.data = atan2(y, x);
            //     heading_pub.publish(msg);
            // }
            rate.sleep();
        }
    }


    private:
    void writeReg(int fd, int address, int value) {
        int result = wiringPiI2CWriteReg8(fd, address, value);

        if (result == -1) {
            throw std::runtime_error("Register write failed");
        }
    }

    std::array<float, 3> measure() {
        std::array<float, 3> measurement;

        // Initiate measurement
        writeReg(devId, kIntCtrlAddr, 1);

        // Wait until status register says done
        usleep(kMeasureDurationUs);
        while (!(wiringPiI2CReadReg8(devId, kStatusAddr) & 1)) {}

        // Read values from 0x00 - 0x06
        wiringPiI2CWrite(devId, 0);
        for(int i=0; i<3; i++)
            measurement[i] = (wiringPiI2CRead(devId) << 8) + wiringPiI2CRead(devId);

        return measurement;
    }
};

int main (int argc, char **argv)
{
    ros::init(argc, argv, "compass");
    ros::NodeHandle nh;
    Compass compass = Compass(nh);

    try {
        compass.spin();
    } catch (const std::runtime_error& ex) {
        ROS_FATAL_STREAM("[Compass] Runtime error: " << ex.what());
        return 1;
    }
    
    return 0;
}