#include "lec5_hw/visualizer.hpp"
#include "lec5_hw/trajectory.hpp"

#include <ros/ros.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseStamped.h>

#include <cmath>
#include <iostream>
#include <vector>

struct Config
{
    std::string targetTopic;
    double clickHeight;
    std::vector<double> initialVel;
    std::vector<double> initialAcc;
    std::vector<double> terminalVel;
    std::vector<double> terminalAcc;
    double allocationSpeed;
    double allocationAcc;
    int maxPieceNum;

    Config(const ros::NodeHandle &nh_priv)
    {
        nh_priv.getParam("TargetTopic", targetTopic);
        nh_priv.getParam("ClickHeight", clickHeight);
        nh_priv.getParam("InitialVel", initialVel);
        nh_priv.getParam("InitialAcc", initialAcc);
        nh_priv.getParam("TerminalVel", terminalVel);
        nh_priv.getParam("TerminalAcc", terminalAcc);
        nh_priv.getParam("AllocationSpeed", allocationSpeed);
        nh_priv.getParam("AllocationAcc", allocationAcc);
        nh_priv.getParam("MaxPieceNum", maxPieceNum);
    }
};

double timeTrapzVel(const double dist,
                    const double vel,
                    const double acc)
{
    const double t = vel / acc;
    const double d = 0.5 * acc * t * t;

    if (dist < d + d)
    {
        return 2.0 * sqrt(dist / acc);
    }
    else
    {
        return 2.0 * t + (dist - 2.0 * d) / vel;
    }
}

void minimumJerkTrajGen(
    // Inputs:
    const int pieceNum,
    const Eigen::Vector3d &initialPos,
    const Eigen::Vector3d &initialVel,
    const Eigen::Vector3d &initialAcc,
    const Eigen::Vector3d &terminalPos,
    const Eigen::Vector3d &terminalVel,
    const Eigen::Vector3d &terminalAcc,
    const Eigen::Matrix3Xd &intermediatePositions,
    const Eigen::VectorXd &timeAllocationVector,
    // Outputs:
    Eigen::MatrixX3d &coefficientMatrix)
{
    // coefficientMatrix is a matrix with 6*piece num rows and 3 columes
    // As for a polynomial c0+c1*t+c2*t^2+c3*t^3+c4*t^4+c5*t^5,
    // each 6*3 sub-block of coefficientMatrix is
    // --              --
    // | c0_x c0_y c0_z |
    // | c1_x c1_y c1_z |
    // | c2_x c2_y c2_z |
    // | c3_x c3_y c3_z |
    // | c4_x c4_y c4_z |
    // | c5_x c5_y c5_z |
    // --              --
    // Please computed coefficientMatrix of the minimum-jerk trajectory
    // in this function

    // ------------------------ Put your solution below ------------------------
    const int N = 6;
    ROS_INFO("pieceNum : %d\n", pieceNum);
    std::cout<<"timeAllocationVector : "<<timeAllocationVector.transpose()<<std::endl;

    Eigen::MatrixXd M = Eigen::MatrixXd::Zero(6 * pieceNum, 6 * pieceNum);

    Eigen::MatrixXd F0 = Eigen::MatrixXd::Zero(3, 6);
    F0 << 1, 0, 0, 0, 0, 0,
          0, 1, 0, 0, 0, 0,
          0, 0, 2, 0, 0, 0;
    std::cout<<"F0 : \n"<<F0<<std::endl;

    double T = timeAllocationVector(0);
    double t = T;
    double t2 = t * t;
    double t3 = t2 * t;
    double t4 = t3 * t;
    double t5 = t4 * t;
    Eigen::MatrixXd EM = Eigen::MatrixXd::Zero(3, 6);
    EM << 1, t,  t2,   t3,    t4,    t5,
            0, 1, 2*t, 3*t2,  4*t3,  5*t4,
            0, 0,   2,  6*t, 12*t2, 20*t3;
    std::cout<<"EM : \n"<<EM<<std::endl;

    M.block<3, 6>(0, 0) = F0;
    M.block<3, 6>(M.rows() - 3, M.cols() - 6) = EM;

    Eigen::MatrixXd Fi = Eigen::MatrixXd::Zero(6, 6);
    Fi << 0,  0, 0, 0, 0, 0,
          -1, 0, 0, 0, 0, 0,
          0, -1, 0, 0, 0, 0,
          0, 0, -2, 0, 0, 0,
          0, 0, 0, -6, 0, 0,
          0, 0, 0, 0, -24, 0;
    
    auto ei = [&](double T) {
        double t = T;
        double t2 = t * t;
        double t3 = t2 * t;
        double t4 = t3 * t;
        double t5 = t4 * t;
        Eigen::MatrixXd Ei = Eigen::MatrixXd::Zero(6, 6);
        Ei <<   1,  t,  t2,     t3,     t4,     t5,
                1,  t,  t2,     t3,     t4,     t5,
                0,  1, 2*t,   3*t2,   4*t3,   5*t4,
                0,  0,   2,    6*t,  12*t2,  20*t3,
                0,  0,   0,      6,   24*t,  60*t2,
                0,  0,   0,      0,     24,  120*t;
        return Ei;
    };

    for (size_t i = 1; i < pieceNum; i++)
    {
        M.block<6, 6>((i-1)*6+3, (i-1)*6+6) = Fi;

        Eigen::MatrixXd Ei = ei(timeAllocationVector(i));
        M.block<6, 6>((i-1)*6+3, (i-1)*6) = Ei;
    }

    std::cout<<"M \n"<<M<<std::endl;

    Eigen::MatrixXd b = Eigen::MatrixXd::Zero(6 * pieceNum, 3);
    // initial
    b.row(0) = initialPos;
    b.row(1) = initialVel;
    b.row(2) = initialAcc;
    // terminal
    b.row(b.rows() - 3) = terminalPos;
    b.row(b.rows() - 2) = terminalVel;
    b.row(b.rows() - 1) = terminalAcc;
    // inermediate
    for (int i = 1; i < pieceNum; i++) {
        Eigen::MatrixXd Di = Eigen::MatrixXd::Zero(6, 3);
        Di.row(0) = intermediatePositions.col(i-1).transpose();
        b.block<6, 3>((i-1)*6+3, 0) = Di;
    }
    std::cout<<"b \n"<<b<<std::endl;

    Eigen::JacobiSVD<Eigen::MatrixXd> svd(M, Eigen::ComputeThinU | Eigen::ComputeThinV);
    auto x = svd.solve(b);
    std::cout<<"x \n"<<x<<std::endl;
    coefficientMatrix = x;
    
    // ------------------------ Put your solution above ------------------------
}

class ClickGen
{
private:
    Config config;

    ros::NodeHandle nh;
    ros::Subscriber targetSub;

    Visualizer visualizer;

    Eigen::Matrix3Xd positions;
    Eigen::VectorXd times;
    int positionNum;
    Trajectory<5> traj;

public:
    ClickGen(const Config &conf,
             ros::NodeHandle &nh_)
        : config(conf),
          nh(nh_),
          visualizer(nh),
          positions(3, config.maxPieceNum + 1),
          times(config.maxPieceNum),
          positionNum(0)
    {
        targetSub = nh.subscribe(config.targetTopic, 1,
                                 &ClickGen::targetCallBack, this,
                                 ros::TransportHints().tcpNoDelay());
    }

    void targetCallBack(const geometry_msgs::PoseStamped::ConstPtr &msg)
    {
        if (positionNum > config.maxPieceNum)
        {
            positionNum = 0;
            traj.clear();
        }

        positions(0, positionNum) = msg->pose.position.x;
        positions(1, positionNum) = msg->pose.position.y;
        positions(2, positionNum) = std::fabs(msg->pose.orientation.z) * config.clickHeight;

        if (positionNum > 0)
        {
            const double dist = (positions.col(positionNum) - positions.col(positionNum - 1)).norm();
            times(positionNum - 1) = timeTrapzVel(dist, config.allocationSpeed, config.allocationAcc);
        }

        ++positionNum;

        if (positionNum > 1)
        {
            const int pieceNum = positionNum - 1;
            const Eigen::Vector3d initialPos = positions.col(0);
            const Eigen::Vector3d initialVel(config.initialVel[0], config.initialVel[1], config.initialVel[2]);
            const Eigen::Vector3d initialAcc(config.initialAcc[0], config.initialAcc[1], config.initialAcc[2]);
            const Eigen::Vector3d terminalPos = positions.col(pieceNum);
            const Eigen::Vector3d terminalVel(config.terminalVel[0], config.terminalVel[1], config.terminalVel[2]);
            const Eigen::Vector3d terminalAcc(config.terminalAcc[0], config.terminalAcc[1], config.terminalAcc[2]);
            const Eigen::Matrix3Xd intermediatePositions = positions.middleCols(1, pieceNum - 1);
            const Eigen::VectorXd timeAllocationVector = times.head(pieceNum);

            std::cout<<"positions : \n"<<positions<<std::endl;

            std::cout<<"intermediatePositions : \n"<<intermediatePositions<<std::endl;

            Eigen::MatrixX3d coefficientMatrix = Eigen::MatrixXd::Zero(6 * pieceNum, 3);

            minimumJerkTrajGen(pieceNum,
                               initialPos, initialVel, initialAcc,
                               terminalPos, terminalVel, terminalAcc,
                               intermediatePositions,
                               timeAllocationVector,
                               coefficientMatrix);

            traj.clear();
            traj.reserve(pieceNum);
            for (int i = 0; i < pieceNum; i++)
            {
                traj.emplace_back(timeAllocationVector(i),
                                  coefficientMatrix.block<6, 3>(6 * i, 0).transpose().rowwise().reverse());
            }
        }

        visualizer.visualize(traj, positions.leftCols(positionNum));

        return;
    }
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "click_gen_node");
    ros::NodeHandle nh_;
    ClickGen clickGen(Config(ros::NodeHandle("~")), nh_);
    ros::spin();
    return 0;
}
