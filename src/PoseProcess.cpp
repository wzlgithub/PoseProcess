/*************************************************************************
    > File Name: PoseProcess.cpp
    > Author:
    > Mail:
    > Created Time: 2017年09月09日 星期六 16时56分06秒
 ************************************************************************/

#include "PoseProcess.h"

namespace EigenCV {

template <class T>
Pose<T>::Pose()
{
    this->trans=Eigen::Matrix<T,3,1>(T(0),T(0),T(0));
    this->quat=Eigen::Quaternion<T>(1,0,0,0);
}

template <class T>
Pose<T>::Pose(const Pose& pose)
{
    this->trans=pose.trans;
    this->quat=pose.quat;
}

template <class T>
Pose<T>& Pose<T>::operator=(const Pose<T>& p)
{
    this->trans=p.trans;
    this->quat=p.quat;
    return *this;
}

template <class T>
Pose<T>::Pose(const Eigen::Matrix<T,3,1>& trans, const Eigen::Quaternion<T>& quat)
{
    this->trans=trans;
    this->quat=quat;
}

template <class T>
Pose<T>::Pose(const Eigen::Matrix<T,3,1>& trans, const Eigen::Matrix<T,3,3>& rot)
{
    this->trans=trans;
    this->quat=rot;
}

template <class T>
Pose<T>::Pose(const Eigen::Matrix<T,3,1>& trans, const Eigen::AngleAxis<T>& angleaxis)
{
    this->trans=trans;
    this->quat=angleaxis;
}

template <class T>
Pose<T>::Pose(const Eigen::Matrix<T,3,1>& trans, const std::vector<T>& euler, EulerType euler_type, bool isdegree)
{
    std::vector<T> euler_rad;
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
        Eigen::AngleAxis<T> aax_1(euler_rad[0], Eigen::Matrix<T,3,1>(1, 0, 0));
        Eigen::AngleAxis<T> aay_1(euler_rad[1], Eigen::Matrix<T,3,1>(0, 1, 0));
        Eigen::AngleAxis<T> aaz_1(euler_rad[2], Eigen::Matrix<T,3,1>(0, 0, 1));
        this->trans=trans;
        this->quat=aax_1 * aay_1 * aaz_1;;
        break;
    }
    case ZYX:
    {
        Eigen::AngleAxis<T> aax_2(euler_rad[0], Eigen::Matrix<T,3,1>(1, 0, 0));
        Eigen::AngleAxis<T> aay_2(euler_rad[1], Eigen::Matrix<T,3,1>(0, 1, 0));
        Eigen::AngleAxis<T> aaz_2(euler_rad[2], Eigen::Matrix<T,3,1>(0, 0, 1));
        this->trans=trans;
        this->quat=aaz_2 * aay_2 * aax_2;;
        break;
    }
    case ZYZ:
    {
        Eigen::AngleAxis<T> aaz1_3(euler_rad[0], Eigen::Matrix<T,3,1>(0, 0, 1));
        Eigen::AngleAxis<T> aay_3(euler_rad[1], Eigen::Matrix<T,3,1>(0, 1, 0));
        Eigen::AngleAxis<T> aaz2_3(euler_rad[2], Eigen::Matrix<T,3,1>(0, 0, 1));
        this->trans=trans;
        this->quat=aaz1_3 * aay_3 * aaz2_3;;
        break;
    }
    default:
        break;
    }
}

template <class T>
Pose<T>::Pose(const std::vector<T>& euler, EulerType euler_type, bool isdegree)
{
    std::vector<T> euler_rad;
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
        Eigen::AngleAxis<T> aax_1(euler_rad[0], Eigen::Matrix<T,3,1>(1, 0, 0));
        Eigen::AngleAxis<T> aay_1(euler_rad[1], Eigen::Matrix<T,3,1>(0, 1, 0));
        Eigen::AngleAxis<T> aaz_1(euler_rad[2], Eigen::Matrix<T,3,1>(0, 0, 1));
        this->trans=Eigen::Vector3d(0,0,0);
        this->quat=aax_1 * aay_1 * aaz_1;
        break;
    }
    case ZYX:
    {
        Eigen::AngleAxis<T> aax_2(euler_rad[0], Eigen::Matrix<T,3,1>(1, 0, 0));
        Eigen::AngleAxis<T> aay_2(euler_rad[1], Eigen::Matrix<T,3,1>(0, 1, 0));
        Eigen::AngleAxis<T> aaz_2(euler_rad[2], Eigen::Matrix<T,3,1>(0, 0, 1));
        this->trans=Eigen::Matrix<T,3,1>(0,0,0);
        this->quat=aaz_2 * aay_2 * aax_2;
        break;
    }

    case ZYZ:
    {
        Eigen::AngleAxis<T> aaz1_3(euler_rad[0], Eigen::Matrix<T,3,1>(0, 0, 1));
        Eigen::AngleAxis<T> aay_3(euler_rad[1], Eigen::Matrix<T,3,1>(0, 1, 0));
        Eigen::AngleAxis<T> aaz2_3(euler_rad[2], Eigen::Matrix<T,3,1>(0, 0, 1));
        this->trans=Eigen::Matrix<T,3,1>(0,0,0);
        this->quat=aaz1_3 * aay_3 * aaz2_3;
        break;
    }
    default:
        break;
    }
}

template <class T>
Pose<T>::Pose(const cv::Mat& trans, const cv::Mat& rot)
{
    cv::Mat temp_trans=trans.clone();
    Eigen::Map<Eigen::Matrix<T,3,1>> temp_trans1(temp_trans.ptr<T>(0));
    cv::Mat rot_transpose = rot.t();
    Eigen::Map<Eigen::Matrix<T,3,3>> temp_rot(rot_transpose.ptr<T>(0));
    this->trans=temp_trans1;
    this->quat=temp_rot;
}

template <class T>
Pose<T>::Pose(const Eigen::Quaternion<T>& quat )
{
    this->trans=Eigen::Matrix<T,3,1>(0,0,0);
    this->quat=quat;
}

template <class T>
Pose<T>::Pose(const Eigen::Matrix<T,3,3>& rot)
{
    this->trans=Eigen::Matrix<T,3,1>(0,0,0);
    this->quat=rot;
}

template <class T>
Pose<T>::Pose(const Eigen::AngleAxis<T>& angleaxis)
{\
    this->trans=Eigen::Matrix<T,3,1>(0,0,0);
    this->quat=angleaxis;
}

template <class T>
Pose<T>::Pose(const cv::Mat& rot)
{
    cv::Mat rot_transpose = rot.t();
    Eigen::Map<Eigen::Matrix<T,3,3>> temp_rot(rot_transpose.ptr<T>(0));
    this->trans=Eigen::Vector3d(0,0,0);
    this->quat=temp_rot;
}


template <class T>
Pose<T>::~Pose()
{

}

template <class T>
Pose<T> Pose<T>::MultiplyPose(const Pose<T>& pose) const
{
    Pose ans;
    ans.trans=this->quat*pose.trans + this->trans;
    ans.quat=this->quat*pose.quat;
    return ans;
}

template <class T>
Pose<T> Pose<T>::MultiplyLeft(const Pose<T>& pose) const
{
    Pose ans;
    ans.trans=pose.quat*this->trans + pose.trans;
    ans.quat=pose.quat*this->quat;
    return ans;
}

template <class T>
Pose<T> Pose<T>::invert() const
{
    Pose ans;
    ans.trans=this->quat.conjugate()*this->trans*(-1);
    ans.quat=this->quat.conjugate();
    return ans;
}

template <class T>
Eigen::Matrix<T,4,4> Pose<T>::to_matpose() const
 {
    return compute_matpose();
 }

template <class T>
void Pose<T>::pose2cvmat(cv::Mat& trans,cv::Mat& rot) const
{
    trans=(cv::Mat_<T>(3,1)<<this->trans[0],this->trans[1], this->trans[2]);
    Eigen::Matrix<T,3,3> rot_temp;
    rot_temp=this->quat.matrix();
    rot=(cv::Mat_<T>(3,3)<<rot_temp(0,0),rot_temp(0,1),rot_temp(0,2),rot_temp(1,0),rot_temp(1,1),rot_temp(1,2),rot_temp(2,0),rot_temp(2,1),rot_temp(2,2));
}

template <class T>
Eigen::Matrix<T,3,1>  Pose<T>::get_trans() const
{
    return this->trans;
}

template <class T>
Eigen::Quaternion<T> Pose<T>::get_quat() const
{
    return this->quat;
}

template <class T>
std::vector<T> Pose<T>::to_eulerpose(EulerType euler_type) const
{
    Eigen::Matrix<T,3,1> euler=quat2euler(euler_type);
    std::vector<T> euler_pose;
    euler_pose.push_back(this->trans[0]);
    euler_pose.push_back(this->trans[1]);
    euler_pose.push_back(this->trans[2]);
    euler_pose.push_back(euler[0]/M_PI * 180);
    euler_pose.push_back(euler[1]/M_PI * 180);
    euler_pose.push_back(euler[2]/M_PI * 180);
    return euler_pose;
}

/*angle, axis*/
template <class T>
std::vector<T> Pose<T>::to_angleaxispose() const
{
    Eigen::AngleAxis<T> angleaxis = quat2angleaxis();
    std::vector<T> angleaxis_pose;
    angleaxis_pose.push_back(this->trans[0]);
    angleaxis_pose.push_back(this->trans[1]);
    angleaxis_pose.push_back(this->trans[2]);
    angleaxis_pose.push_back(angleaxis.angle());
    angleaxis_pose.push_back(angleaxis.axis()[0]);
    angleaxis_pose.push_back(angleaxis.axis()[1]);
    angleaxis_pose.push_back(angleaxis.axis()[2]);
    return angleaxis_pose;
}

template <class T>
std::vector<T> Pose<T>::vec() const
{
    std::vector<T> res;
    res.push_back(this->trans[0]);
    res.push_back(this->trans[1]);
    res.push_back(this->trans[2]);
    res.push_back(this->quat.x());
    res.push_back(this->quat.y());
    res.push_back(this->quat.z());
    res.push_back(this->quat.w());
    return res;
}

template <class T>
void Pose<T>::show_matpose() const
{
    std::cout<<compute_matpose()<<std::endl;
}

template <class T>
Eigen::Matrix<T,4,4> Pose<T>::compute_matpose() const
{
    Eigen::Matrix<T,4,4> matpose;
    Eigen::Matrix<T,3,3> rot;
    rot=this->quat.matrix();
    matpose.block<3,3>(0,0)=rot;
    matpose.block<3,1>(0,3)=this->trans;
    matpose.block<1,4>(3,0)<<T(0),T(0),T(0),T(1);
    return matpose;
}

template <class T>
/* output transx,transy,transz, rotx, roty, rotz, rotw   */
std::ostream& operator<<(std::ostream& out, const Pose<T>& pose)
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
template <class T>
Eigen::Matrix<T,3,1> Pose<T>::quat2euler(EulerType euler_type) const
{
    Eigen::Matrix<T,3,1> euler;
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

template <class T>
Eigen::AngleAxis<T> Pose<T>::quat2angleaxis() const
{
    Eigen::AngleAxis<T> angleaxis;
    angleaxis=quat;
    return angleaxis;
}


}

