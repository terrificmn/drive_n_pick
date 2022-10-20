#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/LaserScan.h>
#include <tf/tf.h>
#include <ros/package.h>

class DriveNPick {
private:
    ros::NodeHandle nh;
    ros::Publisher pub_cmd;
    ros::Subscriber sub_odom;
    ros::Subscriber sub_scan;
    ros::AsyncSpinner spinner;
    int sequence = 0;
    ros::Time current_time;
    nav_msgs::Odometry odom_msg;
    sensor_msgs::LaserScan scan_msg;
    double scan_front = 100.0; // init

    double yaw;


protected:
    ros::Time start_time, end_time;
    

public:
    DriveNPick() : spinner(0) { 
        ROS_INFO("init Drive_and_Pick_it_up");
        spinner.start();

        pub_cmd = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 10);
        sub_odom = nh.subscribe<nav_msgs::Odometry>("/md_odom", 10, &DriveNPick::odomCb, this);
        sub_scan = nh.subscribe<sensor_msgs::LaserScan>("/scan", 10, &DriveNPick::scanCb, this);
        this->drive();   
    }

    ~DriveNPick() {}

    void setStartTime() {
        this->start_time = ros::Time().now();
        ROS_INFO("start time set!");
        // ROS_WARN("starttime: %lf", this->start_time.toSec());
    }

    void drive() {
        double distance = 0.0;
        ros::Duration duration_time;
        ros::Duration(0.3).sleep(); // prevent from saving 0 to start_time
        std::string pkgpath = ros::package::getPath("drive_n_pick");
        this->setStartTime();
        
        ros::Rate lr(10);

        while(ros::ok()) {
            geometry_msgs::Twist cmd;
            if(this->sequence == 0) {
                cmd.linear.x = 0.4;
                cmd.angular.z = 0.0;  //0.4
                distance = cmd.linear.x * duration_time.toSec();

                // ROS_INFO("distance: %lf", distance);
                // ROS_INFO("total time: %lf", duration_time.toSec());
                
                if(distance > 1.54 || (this->scan_front < 0.2)) {   /// distance 2.35
                    ROS_INFO("robot stopped");
                    this->sequence = 1;
                    distance = 0.0; //reset
                    cmd.linear.x = 0.0;
                    cmd.angular.z = 0.0;
                    this->setStartTime();
                }

            } else if(this->sequence == 1) {
                cmd.linear.x = 0.0;
                cmd.angular.z = 0.5; // 0.0
                distance = cmd.linear.x * duration_time.toSec();

                // ROS_INFO("distance: %lf", distance);
                // ROS_INFO("total time: %lf", duration_time.toSec());
                ROS_INFO("yaw: %lf", this->yaw);
                
                if(this->yaw > 1.51) {
                    ROS_INFO("robot stopped");
                    this->sequence = 2;
                    distance = 0.0; //reset
                    cmd.linear.x = 0.0;
                    cmd.angular.z = 0.0;
                    this->setStartTime();
                }

            } else if(this->sequence == 2) {
                cmd.linear.x = 0.3;
                cmd.angular.z = 0.0; // 0.0
                distance = cmd.linear.x * duration_time.toSec();

                // ROS_INFO("distance: %lf", distance);
                // ROS_INFO("total time: %lf", duration_time.toSec());
                
                if(distance > 1.91 || (this->scan_front < 0.18)) {
                    ROS_INFO("robot stopped");
                    this->sequence = 3;
                    distance = 0.0; //reset
                    cmd.linear.x = 0.0;
                    cmd.angular.z = 0.0;
                    this->setStartTime();
                }

            }
             ///// waiting 
             else if(this->sequence == 3) {
                cmd.linear.x = 0.0;
                cmd.angular.z = 0.0;
                ROS_INFO("xArm execute");
                // ROS_INFO("total time: %lf", duration_time.toSec());

                ROS_INFO("yaw : %lf", this->yaw);

                if(duration_time.toSec() > 2.0) {
                    /// xarm execute
                    std::string py_cmd = "python3 " + pkgpath + "/py/x_pick_it_up.py";
                    system(py_cmd.c_str());
                    cmd.angular.z = 0.0;
                    distance = 0.0; //reset
                    this->sequence++;
                    this->setStartTime();
                    ROS_WARN("xarm now executed");
                }
            
            }  else if(this->sequence == 4) {
                cmd.linear.x = 0.0;
                cmd.angular.z = -0.6;
                ROS_INFO("robot rotate and moving");

                distance = cmd.linear.x * duration_time.toSec();

                ROS_INFO("yaw: %lf", this->yaw);
                if(this->yaw < 0.27) {
                    ROS_INFO("robot stopped");
                    this->sequence++;
                    distance = 0.0; //reset
                    cmd.linear.x = 0.0;
                    cmd.angular.z = 0.0;
                    this->current_time = ros::Time::now();
                    this->setStartTime();
                }
            } 
            else if(this->sequence == 5) {
                cmd.linear.x = 0.3;
                cmd.angular.z = 0.0;
                ROS_INFO("robot moving");

                distance = cmd.linear.x * duration_time.toSec();

                ROS_INFO("distance: %lf", distance);

                if(distance > 1.0 || (this->scan_front < 0.2)) {
                    ROS_INFO("robot stopped");
                    this->sequence++;
                    distance = 0.0; //reset
                    cmd.linear.x = 0.0;
                    cmd.angular.z = 0.0;
                    this->setStartTime();
                }

            } else if(this->sequence == 6) {
                cmd.linear.x = 0.0;
                cmd.angular.z = -0.6;
                ROS_INFO("robot rotate");

                // ROS_INFO("distance: %lf", distance);
                ROS_INFO("yaw: %lf", this->yaw);

                if(this->yaw < -1.51 || (this->scan_front < 0.2)) {
                    ROS_INFO("robot stopped");
                    this->sequence++;
                    distance = 0.0; //reset
                    cmd.linear.x = 0.0;
                    cmd.angular.z = 0.0;
                    this->setStartTime();
                }
            } 
             else if(this->sequence == 7) {
                cmd.linear.x = 0.3;
                cmd.angular.z = 0.0;
                ROS_INFO("robot rotate and moving");

                distance = cmd.linear.x * duration_time.toSec();

                ROS_INFO("distance: %lf", distance);

                if(distance > 1.6 || (this->scan_front < 1.0)) {
                    ROS_INFO("robot stopped");
                    this->sequence++;
                    distance = 0.0; //reset
                    cmd.linear.x = 0.0;
                    cmd.angular.z = 0.0;
                    this->setStartTime();
                }

            }  else if(this->sequence == 8) {
                std::string py_cmd = "python3 " + pkgpath + "/py/x_put_it_down.py";
                system(py_cmd.c_str());  ///blocking
                this->sequence = 10 ;
            }

            this->end_time = ros::Time().now();
            duration_time = this->end_time - this->start_time;

            this->pub_cmd.publish(cmd);
            lr.sleep();
        }
    }

    void odomCb(const nav_msgs::Odometry::ConstPtr &msg) {
        this->odom_msg.pose.pose.orientation.x = msg->pose.pose.orientation.x;
        this->odom_msg.pose.pose.orientation.y = msg->pose.pose.orientation.y;
        this->odom_msg.pose.pose.orientation.z = msg->pose.pose.orientation.z;
        this->odom_msg.pose.pose.orientation.w = msg->pose.pose.orientation.w;
        this->yaw = this->quaternionToYaw();
    }

    double quaternionToYaw() {
        // quartnion to r p y
        tf::Quaternion q(
            this->odom_msg.pose.pose.orientation.x,
            this->odom_msg.pose.pose.orientation.y,
            this->odom_msg.pose.pose.orientation.z,
            this->odom_msg.pose.pose.orientation.w
        );

        tf::Matrix3x3 m(q);
        double r, p, y;
        m.getRPY(r, p, y); //yaw's value updated right away

        return y;
    }

    void scanCb(const sensor_msgs::LaserScan::ConstPtr &msg) {
        // ROS_INFO("Scan %f", msg->ranges[365]);  //360  // almost front 
        // this->scan_front = msg->ranges[365];
        this->scan_front = (msg->ranges[361]+msg->ranges[362]+msg->ranges[363]+msg->ranges[364]+msg->ranges[365]) /5;
        // ROS_INFO("Scan %f", this->scan_front); 
    }

};


int main(int argc, char** argv) {
    ros::init(argc, argv, "driving_and_picking_up_node");

    DriveNPick DriveObj;

    ros::waitForShutdown();
    ros::shutdown();
    return 0;
}
