/*************************************************************************
	> File Name: main.cpp
	> Author: 
	> Mail: 
	> Created Time: 2017年09月21日 星期四 10时56分26秒
 ************************************************************************/

#include<iostream>
#include "PoseProcess.h"
#include<Eigen/Dense>
#include<Eigen/Core>
#include<Eigen/Geometry>
#include <opencv2/opencv.hpp>
#include <vector>
using namespace std;
using namespace Eigen;
using namespace cv;

int main()
{
    AngleAxisd t_V1(M_PI/18, Vector3d(1, 0, 0));
    AngleAxisd t_V2(M_PI/9, Vector3d(0, 1, 0));
    AngleAxisd t_V3(M_PI/6, Vector3d(0, 0, 1));
    EigenCV::Pose p1(t_V1);
    EigenCV::Pose p2(t_V2);
    EigenCV::Pose p3(t_V3);
    EigenCV::Pose p4=p1.MultiplyPose(p2).MultiplyPose(p3);
    cout<<(t_V1 * t_V2 * t_V3).matrix()<<endl;
    p4.show_matpose();
    cout<<"rot x,y,z"<<p4;

     vector<double> rot=p4.to_eulerpose(EigenCV::XYZ);
     cout<<"rot: "<<rot[3]<<endl;
     cout<<"rot: "<<rot[4]<<endl;
     cout<<"rot: "<<rot[5]<<endl;

     vector<double> euler;
     euler.push_back(10);
     euler.push_back(20);
     euler.push_back(30);
     EigenCV::Pose p(euler);
     cout<<p;
     vector<double> rot1=p.to_eulerpose(EigenCV::XYZ);
     cout<<"rot: "<<rot1[3]<<endl;
     cout<<"rot: "<<rot1[4]<<endl;
     cout<<"rot: "<<rot1[5]<<endl;

    return 0;
}
