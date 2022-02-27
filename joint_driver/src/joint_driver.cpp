#include <ros/ros.h>
#include <serial/serial.h>
#include <std_msgs/String.h>
#include <std_msgs/Empty.h>
#include <sensor_msgs/JointState.h>

struct test
{
    float base_motor;
    float elbow_motor1;
    float elbow_motor2;
    float wrist_motor1;
    float wrist_motor2;
    float gripper_motor;
};

serial::Serial ser;

void write_callback(const sensor_msgs::JointState &msg)
{
    int i;
    for (i=0; i<6; i++){
        printf("%f\n",msg.position[i]);
    }
    struct test data;
    data.base_motor=msg.position[0];
    data.elbow_motor1 = msg.position[1];
    data.elbow_motor2 = msg.position[2];
    data.wrist_motor1 = msg.position[3] ;
    data.wrist_motor2 = msg.position[4];
    data.gripper_motor = msg.position[5];
    ser.write((const uint8_t*) &data, sizeof(struct test)); 
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "serial_example_node");
    ros::NodeHandle nh;
    struct test data;
    ros::Subscriber write_sub = nh.subscribe("joint_states", 1000, write_callback); 
    ros::Publisher read_pub = nh.advertise<sensor_msgs::JointState>("read", 1000);

    try
    {
        ser.setPort("/dev/ttyACM0");
        ser.setBaudrate(115200);
        serial::Timeout to = serial::Timeout::simpleTimeout(10);
        ser.setTimeout(to);
        ser.open();
    }
    catch (serial::IOException &e)
    {
        ROS_ERROR_STREAM("Unable to open port ");
        return -1;
    }

    if (ser.isOpen())
    {
        ROS_INFO_STREAM("Serial Port initialized");
    }
    else
    {
        ROS_INFO_STREAM("Serial Port not active");
    }

    // gelen datayi 57.32 ile carpmayi unutma. henuz gelen data yok
    ros::Rate loop_rate(5);
    //char str[98];
    uint8_t junk = '5';
    ser.write((uint8_t *)&junk, 1);

    ros::spin();
}