#include "ros/ros.h"
#include "trajectory_msgs/JointTrajectory.h"
#include "hmh_kinematic/Common.h"
#include "hmh_kinematic/DH_config.h"
#include "hmh_kinematic/FT.h"
#include "hmh_kinematic/kinematic.h"
// 函数声明，用于生成JointTrajectory消息
trajectory_msgs::JointTrajectory generateJointTrajectory(const std::vector<double> &joint_angles);

int main(int argc, char *argv[])
{
        ros::init(argc, argv, "ur5_pub");
        ros::NodeHandle nh;
        ros::Publisher pub = nh.advertise<trajectory_msgs::JointTrajectory>("/pos_joint_traj_controller/command", 10);
        ros::Rate rate(100);

        double tt = 30.0;
        double dt = 0.01;
        std::vector<double> TT(tt / dt);
        for (size_t i = 0; i <= TT.size(); i++)
                TT[i] = i * dt;

        for (size_t i = 0; i <= TT.size(); i++)
        {
                // 假设你有一个函数来获取或计算关节角，这里我们直接使用一个静态数组
                std::vector<double> joint_angles = {30, 30+i*0.01, 30, 30, 30, 30};

                // 使用generateJointTrajectory函数生成轨迹消息
                trajectory_msgs::JointTrajectory joint_states = generateJointTrajectory(joint_angles);

                std::cout << "i: " <<i << std::endl;

                // 发布轨迹消息
                pub.publish(joint_states);
                ros::spinOnce();
                rate.sleep();
        }
        return 0;
}

// 函数定义，用于生成JointTrajectory消息
trajectory_msgs::JointTrajectory generateJointTrajectory(const std::vector<double> &joint_angles)
{
        trajectory_msgs::JointTrajectory joint_states;
        joint_states.header.stamp = ros::Time::now();
        joint_states.header.frame_id = "world"; // base_link
        joint_states.joint_names = {"shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint", "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"};

        trajectory_msgs::JointTrajectoryPoint point;
        point.time_from_start = ros::Duration(0.5); // 假设每个点持续1秒
        std::vector<double> joint_angles_degrees;
        for (double angle : joint_angles)
        {
                joint_angles_degrees.push_back(angle * (M_PI / 180.0));
        }
        point.positions = joint_angles_degrees; // 使用传入的关节角
        joint_states.points.push_back(point);

        return joint_states;
}