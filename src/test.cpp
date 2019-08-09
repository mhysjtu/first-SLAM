#include <iostream>
using namespace std;

//test Eigen lib
#include <Eigen/Core>
#include <Eigen/Dense>

//test Sophus lib
#include "sophus/so3.h"
#include "sophus/se3.h"

//test opencv lib
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

//test g2o lib
#include <g2o/core/block_solver.h>
#include <g2o/solvers/dense/linear_solver_dense.h>


int main(int argc, char **argv){
	//test for cmake	
	cout<<"hello"<<endl;

	//test for eigen
	Eigen::Matrix<float, 3, 3> mat_33;
	mat_33<< 1, 2, 3, 4, 5, 6, 7, 8, 9;
	cout<<mat_33<<endl;

	//test for sophus
	Eigen::Matrix3d R = Eigen::AngleAxisd(M_PI/2, Eigen::Vector3d(0, 0, 1)).toRotationMatrix();
	Sophus::SO3 SO3_R(R);
	Sophus::SO3 SO3_v(0, 0, M_PI/2);
	cout<<"so3 from matrix: "<< SO3_R <<endl;
	cout<<"so3 from vector: "<< SO3_v <<endl;

	//test for opencv
	cv::Mat image;
	image = cv::imread(argv[1]);
	if(image.data == nullptr){
		cerr<<"file"<<argv[1]<<"does not exist"<<endl;
		return 0;
	}

	cv::imshow("image", image);
	cv::waitKey(0);
	cv::destroyAllWindows();

	// test for g2o
	typedef g2o::BlockSolver<g2o::BlockSolverTraits<3, 1>> Block;
	Block::LinearSolverType* linearSolver = new g2o::LinearSolverDense<Block::PoseMatrixType>();
	cout<<"g2o"<<endl;

	return 0;
}

