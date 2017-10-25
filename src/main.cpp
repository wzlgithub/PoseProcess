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
    AngleAxisd t_V1(M_PI, Vector3d(0, 0, 1));
    EigenCV::Pose<double> p1(t_V1);
    cout<<p1.get_quat().x()<<" "<<p1.get_quat().y()<<" "
        <<p1.get_quat().z()<<" "<<p1.get_quat().w()<<" "
        <<endl;

    p1.show_matpose();

    return 0;
}
