#include <ros/ros.h>
#include <mav_trajectory_generation_example/example_planner.h>

#include <iostream>
#include <functional>

class TestPlanner
{
public:
    TestPlanner(ros::NodeHandle &nh) : nh_(nh), planner_(nh)
    {
        timer_ = nh_.createTimer(ros::Duration(3.5), &TestPlanner::timer_cb, this);
        timer_.start();
    }

    void timer_cb(const ros::TimerEvent &)
    {
        Eigen::Vector3d position, velocity;
        // TODO: Add reference subscriber
        // double x = reference_odom.pose.pose.position.x;
        // double y = reference_odom.pose.pose.position.y;
        // double z = reference_odom.pose.pose.position.z;
        // double v_x = reference_odom.twist.twist.linear.x;
        // double v_y = reference_odom.twist.twist.linear.y;
        // double v_z = reference_odom.twist.twist.linear.z;

        // double x = 2.0, y = 2.0, z = 2.0;
        // double v_x = 0.0, v_y = 0.0, v_z = 0.0;

        double t = ros::Time::now().sec;
        double x = std::cos(0.3 * t), y = std::sin(0.3 * t), z = 2.0;
        double v_x = -0.3 * std::sin(0.3 * t), v_y = 0.3 * std::cos(0.3 * t), v_z = 0.0;

        position << x, y, z;
        velocity << v_x, v_y, v_z;

        planner_.planTrajectory(position, velocity, &trajectory_);
        planner_.publishTrajectory(trajectory_);
    }

private:
    ros::NodeHandle &nh_;
    ros::Timer timer_;
    ExamplePlanner planner_;
    mav_trajectory_generation::Trajectory trajectory_;
};

// nav_msgs::Odometry reference_odom;
// void odomCallback(const nav_msgs::Odometry::ConstPtr &msg)
// {
//     reference_odom = *msg;
// }


int main(int argc, char **argv)
{
    ros::init(argc, argv, "planner_test_node");
    ros::NodeHandle nh;
    TestPlanner testPlanner(nh);
    ROS_INFO("Initialized trajectory planner test node.");
    ros::spin();

    return 0;
}