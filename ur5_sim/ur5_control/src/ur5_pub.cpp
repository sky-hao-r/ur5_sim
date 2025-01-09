#include "ros/ros.h"
#include "trajectory_msgs/JointTrajectory.h"
#include "hmh_kinematic/Common.h"
#include "hmh_kinematic/DH_config.h"
#include "hmh_kinematic/FT.h"
#include "hmh_kinematic/kinematic.h"
#include "hmh_kinematic/traj_plan.h"
#include <fstream>
#include <vector>
#include <iostream>


int main(int argc, char *argv[])
{
        ros::init(argc, argv, "ur5_pub");
        ros::NodeHandle nh;
        ros::Rate rate(100);
        JointState q;
        q << 50, -30, -90, -60, -49, 0; // 初始位置
        double tt = 30.0;
        double dt = 0.01;
        std::vector<double> TT(tt / dt);
        for (size_t i = 0; i <= TT.size(); i++)
                TT[i] = i * dt;
        Eigen::MatrixXd Q(TT.size() + 1, 6);

        while (ros::ok())
        {
        // for (size_t i = 0; i <= TT.size(); i++)
        // {
        //         double t = TT[i];
                Kinematics kinematics(UR5_MDH::alpha, UR5_MDH::a, UR5_MDH::theta, UR5_MDH::d);
                Position pos;
                JointsVelocity vel;
                Direction dir;
                Direction dir1;
                TrajPlan::traj_plan_ee8(0, tt, pos,vel,dir,dir1);
                TransMat T = kinematics.FR(pos);
                Eigen::MatrixXd Q = kinematics.Ik_pieer(T) * 180 / M_PI;
                JointState joints = kinematics.Get_OptionalIK(T, q);
                 joints * 180 / M_PI;
                std::cout<<"Q:\n"<<Q<<std::endl;
                std::cout << "q:\n " << joints*180 / M_PI << std::endl;
                // Q.row(i) = q.transpose(); // 将关节角存储到矩阵中

                ros::spinOnce();
                rate.sleep();
        // }
        }
        // // 将Q矩阵保存到txt文件
        // std::ofstream outFile("Q_matrix.txt");
        // if (outFile.is_open())
        // {
        //         for (int i = 0; i < Q.rows(); i++)
        //         {
        //                 for (int j = 0; j < Q.cols(); j++)
        //                 {
        //                         outFile << Q(i, j) << " ";
        //                 }
        //                 outFile << std::endl;
        //         }
        //         outFile.close();
        //         ROS_INFO("Q_matrix.txt 文件保存成功。");
        // }
        // else
        // {
        //         ROS_ERROR("无法打开文件 Q_matrix.txt。");
        // }
        return 0;
}
