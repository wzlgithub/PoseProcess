/*************************************************************************
	> File Name: PoseProcess.h
	> Author: 
	> Mail: 
	> Created Time: 2017年09月21日 星期四 10时11分11秒
 ************************************************************************/

#ifndef _POSEPROCESS_H
#define _POSEPROCESS_H
#include<iostream>
#include<Eigen/Dense>
#include<Eigen/Core>
#include<Eigen/Geometry>
#include<vector>
#include <opencv2/opencv.hpp>

namespace EigenCV {


enum EulerType
{
    XYZ,
    ZYX,
    ZYZ,
};



class Pose
{
public:
    //default: (trans,quat)=(0, 0, 0, 0, 0, 0, 1)
    Pose();
    //copy constructor
    Pose(const Pose& Pose);
    Pose(const Eigen::Vector3d& trans, const Eigen::Quaterniond& quat );
    Pose(const Eigen::Vector3d& trans, const Eigen::Matrix3d& rot);
    Pose(const Eigen::Vector3d& trans, const Eigen::AngleAxisd& angleaxis);
    //the euler angle is degree default.. sequnce is always rotx, roty, rotz, euler_type is XYZ default
    Pose(const Eigen::Vector3d& trans, const std::vector<double>& euler, EulerType euler_type=XYZ, bool isdegree=true);
    Pose(const cv::Mat& trans, const cv::Mat& rot);
    //trans is 0 default
    Pose(const Eigen::Quaterniond& quat );
    //trans is 0 default
    Pose(const Eigen::Matrix3d& rot);
    //trans is 0 default
    Pose(const Eigen::AngleAxisd& angleaxis);

    Pose(const std::vector<double>& euler, EulerType euler_type=XYZ, bool isdegree=true);
    //trans is 0 default
    Pose(const cv::Mat& rot);
    Pose& operator=(const Pose& p);
    ~Pose();
    //    this * pose
    Pose MultiplyPose(const Pose& pose) const;
    //    pose * this
    Pose MultiplyLeft(const Pose& pose) const;
    //   pose invert
    Pose invert() const;
    /*return Matrix4d pose type[ rotation, trans
                                                            0,  0,  0,  1]*/
    Eigen::Matrix4d to_matpose() const;
    /*return trans*/
    Eigen::Vector3d get_trans() const;
    /*retrun quaternion*/
    Eigen::Quaterniond get_quat() const ;
    //eigen pose ------->(cv::Mat trans, cv::Mat rot)
    void pose2cvmat(cv::Mat& trans,cv::Mat& rot) const;
    //retrun (transx, transy, transz, x, y, z, w)
    std::vector<double> vec() const;
    /*show pose type[ rotation, trans
                                          0,  0,  0,  1]*/
    void show_matpose() const;
    /*(trans[3], euler[3])*/
    std::vector<double> to_eulerpose(EulerType euler_type) const;
    /*trans , angle, axis*/
    std::vector<double> to_angleaxispose() const;
    friend std::ostream& operator<<(std::ostream& out, const Pose& pose);
private:
    Eigen::Vector3d trans;
    Eigen::Quaterniond quat;
    Eigen::Matrix4d compute_matpose() const;
    Eigen::Vector3d quat2euler(EulerType euler_type) const;
    Eigen::AngleAxisd quat2angleaxis() const;
};

}

#endif
