// 测试 车臂先到目标点，再进行轨迹规划，通过逆运动学计算

#include "ros/ros.h"
#include "hmh_kinematic/kinematic.h"
#include "hmh_kinematic/Common.h"
#include "hmh_kinematic/DH_config.h"
#include "hmh_kinematic/FT.h"
#include "hmh_kinematic/traj_plan.h"
#include "geometry_msgs/Vector3.h"
#include "geometry_msgs/Twist.h"
#include <trajectory_msgs/JointTrajectory.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <sensor_msgs/JointState.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>

int main(int argc, char *argv[])
{
        ros::init(argc, argv, "demo01_test1");
        ros::NodeHandle nh;
        ros::Rate rate(100);

        /*
                创建节点发布器
        */


        ros::Publisher xyth_d_pub = nh.advertise<geometry_msgs::Vector3>("xyth_d", 10);

        ros::Publisher xyzrpy_d_pub = nh.advertise<geometry_msgs::Vector3>("xyzrpy_d", 10);

        // 初始化数据并创建储存矩阵B
        double tt = 30.0;
        double dt = 0.01;
        std::vector<double> TT(tt / dt);
        for (size_t i = 0; i <= TT.size(); ++i)
                TT[i] = i * dt;
        Eigen::MatrixXd XYTH(3, TT.size() + 1); // 存放车的实际位置
        XYTH.setZero();
        Eigen::MatrixXd XYZRPY(6, TT.size() + 1); // 存放机械臂的实际末端位姿
        XYZRPY.setZero();
        Eigen::MatrixXd Q(8, TT.size() + 1);
        Q.setZero();
        JointsVelocity xyzrpy_WE;                   // 机械臂的实际末端位姿
        Eigen::MatrixXd XYZRPY_d(6, TT.size() + 1); // 存放规划的所有机械臂的末端位移
        Eigen::MatrixXd XYTH_d(3, TT.size() + 1);   // 存放规划的所有车的位
        for (size_t i = 0; i <= TT.size(); ++i)
        {
                double t = TT[i];
                Position xyzrpy_d;
                JointsVelocity dxyzrpy_d;
                Direction xyth_d, dxyth_d;
                // TrajPlan::traj_plan_scene(t, tt, xyzrpy_d, dxyzrpy_d, xyth_d, dxyth_d);
                // TrajPlan::traj_plan_ee8(t, tt, xyzrpy_d, dxyzrpy_d, xyth_d, dxyth_d);
                TrajPlan::traj_plan_2circle(t, tt, xyzrpy_d, dxyzrpy_d, xyth_d, dxyth_d);
                // TrajPlan::traj_plan_ee8(t, tt, xyzrpy_d, dxyzrpy_d, xyth_d, dxyth_d);
                XYZRPY_d.col(i) = xyzrpy_d; // 存放规划机械臂的末端位移
                XYTH_d.col(i) = xyth_d;     // 存放规划车的位置
                Position pos_e, pos_b;


                XYZRPY.col(i) = xyzrpy_WE; // 存放机械臂的末端位移

                geometry_msgs::Vector3 xyth_d_msg;
                xyth_d_msg.x = xyth_d[0];
                xyth_d_msg.y = xyth_d[1];
                xyth_d_msg.z = xyth_d[2];
                xyth_d_pub.publish(xyth_d_msg);

                geometry_msgs::Vector3 xyzrpy_d_msgs;
                xyzrpy_d_msgs.x = xyzrpy_d[0];
                xyzrpy_d_msgs.y = xyzrpy_d[1];
                xyzrpy_d_msgs.z = xyzrpy_d[2];
                xyzrpy_d_pub.publish(xyzrpy_d_msgs);

                ros::spinOnce();
                rate.sleep();
        }

        return 0;
}
