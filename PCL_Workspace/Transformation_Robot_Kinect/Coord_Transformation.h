#ifndef COORD_TRANSFORMATION_H
#define COORD_TRANSFORMATION_H


#include <iostream>
#include <pcl/common/transforms.h>
#include <Eigen/SVD>


class Coord_Transformation{
    public:
        Eigen::Vector3f translate_l;
        Eigen::Vector3f rotate_l;
        Eigen::Affine3f Transform_T;

        Coord_Transformation(Eigen::Matrix3f a_old, Eigen::Matrix3f b);

};

Coord_Transformation::Coord_Transformation (Eigen::Matrix3f a_old, Eigen::Matrix3f b) : translate_l (Eigen::Vector3f::Zero()) , rotate_l(Eigen::Vector3f::Zero()), Transform_T(Eigen::Affine3f::Identity())
{


Eigen::Matrix3f a = a_old;

for (int i=0;i<50;++i)
	{
         
	Eigen::Vector3f CenterA = a.colwise().sum()/3;
	Eigen::Vector3f CenterB = b.colwise().sum()/3; 

	//std::cout << "CenterA= " <<  CenterA.transpose() << std::endl  << "CenterB= "  << CenterB.transpose() << std::endl << std::endl;

	Eigen::Matrix3f H = Eigen::Matrix3f::Zero();

	for (int j=0;j<3;++j)
	{
	H += (a.row(j).transpose()-CenterA)*(b.row(j)-CenterB.transpose());
	}

	//std::cout << "H= " << std::endl << H << std::endl << std::endl;

	Eigen::JacobiSVD<Eigen::Matrix3f> svd(H, Eigen::ComputeFullU | Eigen::ComputeFullV);

	Eigen::Matrix3f R = svd.matrixV()*svd.matrixU().transpose();

	//std::cout << "U= " << std::endl << svd.matrixU() << std::endl << std::endl << "V= " << std::endl << svd.matrixV() << std::endl << std::endl << "R= " << std::endl << R << std::endl << std::endl;

	Eigen::Vector3f t = -R * CenterA + CenterB;

	//std::cout << "t= " << t.transpose() << std::endl;

	Eigen::Affine3f Transform;
	Transform.linear() = R;
	Transform.translation() = t;

	Transform_T = Transform * Transform_T;

	//std::cout << "Transform_T= " << std::endl << Transform_T.matrix() << std::endl << std::endl;

	for (int j=0;j<3;++j)
	{
	a.row(j).transpose() = R*a.row(j).transpose()+t;
	}

	//std::cout << "a= " << std::endl << a << std::endl;

	}

for (int j=0;j<3;++j)
      std::cout << "point " << j << "  =" << b.row(j).transpose()-(Transform_T.linear()*a_old.row(j).transpose() +Transform_T.translation()) << std::endl;

float yaw, pitch, roll;
   pcl::getEulerAngles (Transform_T, roll, pitch, yaw);

	//std::cout << "roll= " << roll << "  pitch= " << pitch <<  "  yaw= " << yaw << std::endl;


translate_l << Transform_T.translation();
rotate_l << roll, pitch, yaw;

}


#endif
