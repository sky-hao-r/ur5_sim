#include "ros/ros.h"
#include "trajectory_msgs/JointTrajectory.h"
#include "hmh_kinematic/Common.h"
#include "hmh_kinematic/DH_config.h"
#include"hmh_kinematic/FT.h"
#include"hmh_kinematic/kinematic.h"
#include"hmh_kinematic/traj_plan.h"
#include <fstream>
#include <vector>
#include <iostream>
// 函数声明，用于生成JointTrajectory消息
trajectory_msgs::JointTrajectory generateJointTrajectory(const std::vector<double> &joint_angles);

int main(int argc, char *argv[])
{ 
        ros::init(argc, argv, "ur5_pub");
        ros::NodeHandle nh;
        ros::Publisher pub = nh.advertise<trajectory_msgs::JointTrajectory>("/pos_joint_traj_controller/command", 10);
        ros::Rate rate(100);
        JointState q;
        q << 33.9295, 25.5053, -55.5289, -81.0342, -33.9295, 0; // 初始位置
        double tt = 30.0;
        double dt = 0.01;
        std::vector<double> TT(tt / dt);
        for (size_t i = 0; i <= TT.size(); i++)
                TT[i] = i * dt;
        Eigen::MatrixXd P(TT.size() + 1, 6);

        for (size_t i = 0;i<=TT.size();i++)
        {
                double t = TT[i];
                std::cout << "t: " << t << std::endl;
                // Kinematics kinematics(UR5_MDH::alpha, UR5_MDH::a, UR5_MDH::theta, UR5_MDH::d);
                Kinematics kinematics(UR3_MDH::alpha, UR3_MDH::a, UR3_MDH::theta, UR3_MDH::d);

                // Position pos = TrajPlan::traj_plan_scene(t,tt);
                Position xyzrpy_d;
                JointsVelocity dxyzrpy_d;
                Direction xyth_d, dxyth_d;
                // TrajPlan::traj_plan_scene(t, tt, xyzrpy_d, dxyzrpy_d, xyth_d, dxyth_d);
                TrajPlan::traj_plan_ee_scene(t, tt, xyzrpy_d, dxyzrpy_d, xyth_d, dxyth_d);

                Position pos = xyzrpy_d;

                // std::cout << "pos: " << pos << std::endl;
                q=q* M_PI / 180;
                // TrajPlan::Traj_plan_car(t, tt, pos);
                TransMat T = kinematics.FR(pos);
                JointState joints = kinematics.Get_OptionalIK(T,q);
                q = joints * 180 / M_PI;
                P.row(i) = pos.transpose(); // 将关节角存储到矩阵中


                // // 假设你有一个函数来获取或计算关节角，这里我们直接使用一个静态数组
                std::vector<double> joint_angles = {q[0], q[1], q[2] , q[3] , q[4], q[5] };

                // // 使用generateJointTrajectory函数生成轨迹消息
                trajectory_msgs::JointTrajectory joint_states = generateJointTrajectory(joint_angles);


                // // 发布轨迹消息
                pub.publish(joint_states);
                ros::spinOnce();
                rate.sleep();
        }

        // 将Q矩阵保存到txt文件
        std::ofstream outFile("P_1.txt");
        if (outFile.is_open())
        {
                for (int i = 0; i < P.rows(); i++)
                {
                        for (int j = 0; j < P.cols(); j++)
                        {
                                outFile << P(i, j) << " ";
                        }
                        outFile << std::endl;
                }
                outFile.close();
                ROS_INFO("P_1.txt 文件保存成功。");
        }
        else
        {
                ROS_ERROR("无法打开文件 P_1.txt。");
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
        point.time_from_start = ros::Duration(0.01); // 假设每个点持续1秒
        std::vector<double> joint_angles_degrees;
        for (double angle : joint_angles)
        {
                joint_angles_degrees.push_back(angle * (M_PI / 180.0));
        }
        point.positions = joint_angles_degrees; // 使用传入的关节角
        joint_states.points.push_back(point);

        return joint_states;
}