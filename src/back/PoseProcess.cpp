/*************************************************************************
    > File Name: PoseProcess.cpp
    > Author:
    > Mail:
    > Created Time: 2017年09月09日 星期六 16时56分06秒
 ************************************************************************/

#include "PoseProcess.h"

namespace EigenCV {

Pose::Pose()
{
    this->trans=Eigen::Vector3d(0,0,0);
    this->quat=Eigen::Quaterniond(1,0,0,0);
}

Pose::Pose(const Pose& pose)
{
    this->trans=pose.trans;
    this->quat=pose.quat;
}

Pose& Pose::operator=(const Pose& p)
{
    this->trans=p.trans;
    this->quat=p.quat;
    return *this;
}

Pose::Pose(const Eigen::Vector3d& trans, const Eigen::Quaterniond& quat)
{
    this->trans=trans;
    this->quat=quat;
}

Pose::Pose(const Eigen::Vector3d& trans, const Eigen::Matrix3d& rot)
{
    this->trans=trans;
    this->quat=rot;
}

Pose::Pose(const Eigen::Vector3d& trans, const Eigen::AngleAxisd& angleaxis)
{
    this->trans=trans;
    this->quat=angleaxis;
}

Pose::Pose(const Eigen::Vector3d& trans, const std::vector<double>& euler, EulerType euler_type, bool isdegree)
{
    std::vector<double> euler_rad;
    if(isdegree)
    {
        euler_rad.push_back(euler[0]/180*M_PI);
        euler_rad.push_back(euler[1]/180*M_PI);
        euler_rad.push_back(euler[2]/180*M_PI);
    }
    else
        euler_rad = euler;
    switch (euler_type) {
    case XYZ:
    {
        Eigen::AngleAxisd aax_1(euler_rad[0], Eigen::Vector3d(1, 0, 0));
        Eigen::AngleAxisd aay_1(euler_rad[1], Eigen::Vector3d(0, 1, 0));
        Eigen::AngleAxisd aaz_1(euler_rad[2], Eigen::Vector3d(0, 0, 1));
        this->trans=trans;
        this->quat=aax_1 * aay_1 * aaz_1;;
        break;
    }
    case ZYX:
    {
        Eigen::AngleAxisd aax_2(euler_rad[0], Eigen::Vector3d(1, 0, 0));
        Eigen::AngleAxisd aay_2(euler_rad[1], Eigen::Vector3d(0, 1, 0));
        Eigen::AngleAxisd aaz_2(euler_rad[2], Eigen::Vector3d(0, 0, 1));
        this->trans=trans;
        this->quat=aaz_2 * aay_2 * aax_2;;
        break;
    }
    case ZYZ:
    {
        Eigen::AngleAxisd aaz1_3(euler_rad[0], Eigen::Vector3d(0, 0, 1));
        Eigen::AngleAxisd aay_3(euler_rad[1], Eigen::Vector3d(0, 1, 0));
        Eigen::AngleAxisd aaz2_3(euler_rad[2], Eigen::Vector3d(0, 0, 1));
        this->trans=trans;
        this->quat=aaz1_3 * aay_3 * aaz2_3;;
        break;
    }
    default:
        break;
    }
}

Pose::Pose(const std::vector<double>& euler, EulerType euler_type, bool isdegree)
{
    std::vector<double> euler_rad;
    if(isdegree)
    {
        euler_rad.push_back(euler[0]/180*M_PI);
        euler_rad.push_back(euler[1]/180*M_PI);
        euler_rad.push_back(euler[2]/180*M_PI);
    }
    else
        euler_rad = euler;
    switch (euler_type) {
    case XYZ:
    {
        Eigen::AngleAxisd aax_1(euler_rad[0], Eigen::Vector3d(1, 0, 0));
        Eigen::AngleAxisd aay_1(euler_rad[1], Eigen::Vector3d(0, 1, 0));
        Eigen::AngleAxisd aaz_1(euler_rad[2], Eigen::Vector3d(0, 0, 1));
        this->trans=Eigen::Vector3d(0,0,0);
        this->quat=aax_1 * aay_1 * aaz_1;
        break;
    }
    case ZYX:
    {
        Eigen::AngleAxisd aax_2(euler_rad[0], Eigen::Vector3d(1, 0, 0));
        Eigen::AngleAxisd aay_2(euler_rad[1], Eigen::Vector3d(0, 1, 0));
        Eigen::AngleAxisd aaz_2(euler_rad[2], Eigen::Vector3d(0, 0, 1));
        this->trans=Eigen::Vector3d(0,0,0);
        this->quat=aaz_2 * aay_2 * aax_2;
        break;
    }

    case ZYZ:
    {
        Eigen::AngleAxisd aaz1_3(euler_rad[0], Eigen::Vector3d(0, 0, 1));
        Eigen::AngleAxisd aay_3(euler_rad[1], Eigen::Vector3d(0, 1, 0));
        Eigen::AngleAxisd aaz2_3(euler_rad[2], Eigen::Vector3d(0, 0, 1));
        this->trans=Eigen::Vector3d(0,0,0);
        this->quat=aaz1_3 * aay_3 * aaz2_3;
        break;
    }
    default:
        break;
    }
}

Pose::Pose(const cv::Mat& trans, const cv::Mat& rot)
{
    cv::Mat temp_trans=trans.clone();
    Eigen::Map<Eigen::Vector3d> temp_trans1(temp_trans.ptr<double>(0));
    cv::Mat rot_transpose = rot.t();
    Eigen::Map<Eigen::Matrix3d> temp_rot(rot_transpose.ptr<double>(0));
    this->trans=temp_trans1;
    this->quat=temp_rot;
}

Pose::Pose(const Eigen::Quaterniond& quat )
{
    this->trans=Eigen::Vector3d(0,0,0);
    this->quat=quat;
}

Pose::Pose(const Eigen::Matrix3d& rot)
{
    this->trans=Eigen::Vector3d(0,0,0);
    this->quat=rot;
}

Pose::Pose(const Eigen::AngleAxisd& angleaxis)
{\
    this->trans=Eigen::Vector3d(0,0,0);
    this->quat=angleaxis;
}

Pose::Pose(const cv::Mat& rot)
{
    cv::Mat rot_transpose = rot.t();
    Eigen::Map<Eigen::Matrix3d> temp_rot(rot_transpose.ptr<double>(0));
    this->trans=Eigen::Vector3d(0,0,0);
    this->quat=temp_rot;
}



Pose::~Pose()
{

}

Pose Pose::MultiplyPose(const Pose& pose) const
{
    Pose ans;
    ans.trans=this->quat*pose.trans + this->trans;
    ans.quat=this->quat*pose.quat;
    return ans;
}

Pose Pose::MultiplyLeft(const Pose& pose) const
{
    Pose ans;
    ans.trans=pose.quat*this->trans + pose.trans;
    ans.quat=pose.quat*this->quat;
    return ans;
}

Pose Pose::invert() const
{
    Pose ans;
    ans.trans=this->quat.conjugate()*this->trans*(-1);
    ans.quat=this->quat.conjugate();
    return ans;
}

Eigen::Matrix4d Pose::to_matpose() const
 {
    return compute_matpose();
 }

void Pose::pose2cvmat(cv::Mat& trans,cv::Mat& rot) const
{
    trans=(cv::Mat_<double>(3,1)<<this->trans[0],this->trans[1], this->trans[2]);
    Eigen::Matrix3d rot_temp;
    rot_temp=this->quat.matrix();
    rot=(cv::Mat_<double>(3,3)<<rot_temp(0,0),rot_temp(0,1),rot_temp(0,2),rot_temp(1,0),rot_temp(1,1),rot_temp(1,2),rot_temp(2,0),rot_temp(2,1),rot_temp(2,2));
}

Eigen::Vector3d  Pose::get_trans() const
{
    return this->trans;
}

Eigen::Quaterniond Pose::get_quat() const
{
    return this->quat;
}

std::vector<double> Pose::to_eulerpose(EulerType euler_type) const
{
    Eigen::Vector3d euler=quat2euler(euler_type);
    std::vector<double> euler_pose;
    euler_pose.push_back(this->trans[0]);
    euler_pose.push_back(this->trans[1]);
    euler_pose.push_back(this->trans[2]);
    euler_pose.push_back(euler[0]/M_PI * 180);
    euler_pose.push_back(euler[1]/M_PI * 180);
    euler_pose.push_back(euler[2]/M_PI * 180);
    return euler_pose;
}

/*angle, axis*/
std::vector<double> Pose::to_angleaxispose() const
{
    Eigen::AngleAxisd angleaxis = quat2angleaxis();
    std::vector<double> angleaxis_pose;
    angleaxis_pose.push_back(this->trans[0]);
    angleaxis_pose.push_back(this->trans[1]);
    angleaxis_pose.push_back(this->trans[2]);
    angleaxis_pose.push_back(angleaxis.angle());
    angleaxis_pose.push_back(angleaxis.axis()[0]);
    angleaxis_pose.push_back(angleaxis.axis()[1]);
    angleaxis_pose.push_back(angleaxis.axis()[2]);
    return angleaxis_pose;
}

std::vector<double> Pose::vec() const
{
    std::vector<double> res;
    res.push_back(this->trans[0]);
    res.push_back(this->trans[1]);
    res.push_back(this->trans[2]);
    res.push_back(this->quat.x());
    res.push_back(this->quat.y());
    res.push_back(this->quat.z());
    res.push_back(this->quat.w());
    return res;
}

void Pose::show_matpose() const
{
    std::cout<<compute_matpose()<<std::endl;
}

Eigen::Matrix4d Pose::compute_matpose() const
{
    Eigen::Matrix4d matpose;
    Eigen::Matrix3d rot;
    rot=this->quat.matrix();
    matpose.block<3,3>(0,0)=rot;
    matpose.block<3,1>(0,3)=this->trans;
    matpose.block<1,4>(3,0)<<0,0,0,1;
    return matpose;
}

/* output transx,transy,transz, rotx, roty, rotz, rotw   */
std::ostream& operator<<(std::ostream& out, const Pose& pose)
{
    out<<pose.trans[0]<<" "
          <<pose.trans[1]<<" "
          <<pose.trans[2]<<" "
          <<pose.quat.x()<<" "
          <<pose.quat.y()<<" "
          <<pose.quat.z()<<" "
          <<pose.quat.w()<<'\n';
}


//quat----->
Eigen::Vector3d Pose::quat2euler(EulerType euler_type) const
{
    Eigen::Vector3d euler;
    switch(euler_type)
    {
    case XYZ:
        euler=this->quat.matrix().eulerAngles(0,1,2);
        break;
    case ZYX:
        euler=this->quat.matrix().eulerAngles(2,1,0);
        break;
    case ZYZ:
        euler=this->quat.matrix().eulerAngles(2,1,2);
        break;
    }
    return euler;
}

Eigen::AngleAxisd Pose::quat2angleaxis() const
{
    Eigen::AngleAxisd angleaxis;
    angleaxis=quat;
    return angleaxis;
}


}

