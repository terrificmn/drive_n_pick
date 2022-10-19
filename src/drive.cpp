#include <ros/ros.h>
#include <geometry_msgs/Twist.h>


class DriveNPick {
private:
    ros::NodeHandle nh;
    ros::Publisher pub_cmd;
    ros::AsyncSpinner spinner;
    int sequence = 0;
    ros::Time current_time;

protected:
    ros::Time start_time, end_time;
    

public:
    DriveNPick() : spinner(0) { 
        ROS_INFO("init Drive_and_Pick_it_up");
        spinner.start();

        pub_cmd = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 10);
        this->drive();
    }

    ~DriveNPick() {}

    void setStartTime() {
        this->start_time = ros::Time().now();
    }

    void drive() {
        double store_linear_x = 0.0;
        double distance = 0.0;
        ros::Duration duration_time;
        ros::Duration sec_duration_time;
        this->setStartTime();
        ROS_INFO("start time set!");
        
        ros::Rate lr(10);

        while(ros::ok()) {
            geometry_msgs::Twist cmd;
            if(this->sequence == 0) {
                cmd.linear.x = 0.3;
                distance = cmd.linear.x * duration_time.toSec();

                // ROS_INFO("distance: %lf", distance);
                // ROS_INFO("total time: %lf", duration_time.toSec());
                
                if(distance > 1.5) {
                    ROS_INFO("robot stopped");
                    this->sequence = 1;
                    distance = 0.0; //reset
                    cmd.linear.x = 0.0;
                    this->current_time = ros::Time::now();
                    this->setStartTime();
                }

            } else if(this->sequence == 1) {
                cmd.linear.x = 0.0;
                cmd.angular.z = 0.3;
                ROS_INFO("robot rotate");
                
                ROS_INFO("total time: %lf", duration_time.toSec());
                if(duration_time.toSec() > 3.0) {
                    
                    cmd.angular.z = 0.0;
                    this->sequence = 2;
                    this->setStartTime();
                    ROS_INFO("robot rotation stopped");
                }

            }  else if(this->sequence == 2) {
                cmd.linear.x = 0.1;
                cmd.angular.z = 0.3;
                ROS_INFO("robot rotate and moving");

                distance = cmd.linear.x * duration_time.toSec();

                ROS_INFO("distance: %lf", distance);

                if(distance > 1.0) {
                    ROS_INFO("robot stopped");
                    this->sequence = 3;
                    distance = 0.0; //reset
                    cmd.linear.x = 0.0;
                    this->current_time = ros::Time::now();
                    this->setStartTime();
                }

            }

            this->end_time = ros::Time().now();
            duration_time = this->end_time - this->start_time;

            this->pub_cmd.publish(cmd);
            lr.sleep();
        }
    }

};


int main(int argc, char** argv) {
    ros::init(argc, argv, "driving_and_picking_up_node");

    DriveNPick DriveObj;
    

    ros::waitForShutdown();
    ros::shutdown();
    return 0;
}
