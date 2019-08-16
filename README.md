# first-SLAM
My first SLAM program, learning from gaoxiang12's slambook.

## Learning Roadmap
+ **Main route:**
- gao's video -> read gao's code -> **write the code** -> read book carefully and solver the problem -> **do practice**
+ **Side route:**
- A. For complex parts(e.g. Feature Detection, Epipolar Geometry, etc), read RVC or MVG book to study further.
- B. For Core parts(e.g. Motion Estimation, Nonlinear Optimization, Loop Detection, etc), read RVCMVG book or ETH course's PPT to practice.

## 0809 Feature Detection Code
### ch7/feature_extraction_orb.cpp
- ORB features and descriptors
- match: brute force
- select matches based on distance

## 0810 2D-2D pose estimation Code
### ch7/pose_estimation_2d2d.cpp

1. epipolar geometry 
- fundamental, essential, homography matrix 
2. recover pose from essential matrix
- using cv::recoverPose

## 0811 2D-3D, 3D-3D pose estimation and triangulation Code
### ch7/pose_estimation_2d3d.cpp<br>
- using cv::solvePnP(EPnP method)
### ch7/pose_estimation_3d3d.cpp<br>
- using ICP + SVD(Eigen)
### ch7/triangulation.cpp
- using cv::triangulatePoints<br>

## 0812 nonlinear optimization for least square
- In order to solve 2d3d and 3d3d by BA method, basic nonlinear optimization and ceres/g2o lib should be learned first.

### ch6/ceres_curve_fitting.cpp<br>
- using ceres lib to solver curve fitting problem using LM method
### ch6/g2o_curve_fitting.cpp<br>
- using g2o lib to solver curve fitting problem using LM, GN, DogLeg method

## 0813 BA for 2d-3d(PnP) and 3d-3d(ICP)
### in ch7/pose_estimation_2d3d_BA.cpp
1. using g2o lib to solver PnP problem using LM method
- CSparse and Levenberg
- Vertex:<br>
  VertexSE3Expmap and VertexSBAPointXYZ 
- Edge:<br>
  EdgeProjectXYZ2UV
### in ch7/pose_estimation_3d3d_BA.cpp<br>
1. using g2o lib to solver ICP problem using LM method
- Eigen and Levenberg
- Vertex:<br>
  VertexSE3Expmap 
- Edge:<br>
  EdgeProjectXYZRGBDPoseOnly : g2o::BaseUnaryEdge<br>
  linearizeOplus

## 0814 have a lesson, which introduces factor graph for BA & frameworks for popular VIO, such as MSCKF, ROVIO, VINS, VI-ORB and ICE-BA & VINS's BA and marginalization. Provided Shenlanxueyuan

## 0815 start reading VINS-Mono source code

## 0816 back-end BA, using g2o
### in ch10, which is a new package
- result is shown in beforeBA and afterBA.png
- using ceres for auto differentiate
- main code for using g2o solver is in src folder



