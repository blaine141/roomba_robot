#include <ros/ros.h>
#include <unistd.h>
#include <wiringPiI2C.h>
#include <geometry_msgs/Vector3.h>

const int kIntCtrlAddr = 0x09;
const int kStatusAddr = 0x08;

const int kMeasureDurationUs = 8000;


class Compass {

    private:
    int devId;
    ros::Publisher compass_pub;

    public:
    Compass(ros::NodeHandle& nh) {
        devId = wiringPiI2CSetup(0x30);

        compass_pub = nh.advertise<geometry_msgs::Vector3>("/compass", 1);
    }

    void spin() {
        ros::Rate rate(20); // ROS Rate at 20Hz
        while (ros::ok()) {
            // Set and measure with positive polarity
            writeReg(devId, kIntCtrlAddr, 1<<3);
            geometry_msgs::Vector3 setRead = measure();

            // Reset and measure with negative polarity
            writeReg(devId, kIntCtrlAddr, 1<<4);
            geometry_msgs::Vector3 resetRead = measure();

            // Compute the common component between them
            geometry_msgs::Vector3 measurement;
            measurement.x = setRead.x - resetRead.x;
            measurement.y = setRead.y - resetRead.y;
            measurement.z = setRead.z - resetRead.z;

            // Publish measurement and sleep
            compass_pub.publish(measurement);
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

    geometry_msgs::Vector3 measure() {
        geometry_msgs::Vector3 msg;

        // Initiate measurement
        writeReg(devId, kIntCtrlAddr, 1);

        // Wait until status register says done
        usleep(kMeasureDurationUs);
        while (!(wiringPiI2CReadReg8(devId, kStatusAddr) & 1)) {}

        // Read values from 0x00 - 0x06
        wiringPiI2CWrite(devId, 0);
        msg.x = (wiringPiI2CRead(devId) << 8) + wiringPiI2CRead(devId);
        msg.y = (wiringPiI2CRead(devId) << 8) + wiringPiI2CRead(devId);
        msg.z = (wiringPiI2CRead(devId) << 8) + wiringPiI2CRead(devId);
        return msg;
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