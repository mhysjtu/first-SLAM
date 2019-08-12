# first-SLAM
My first SLAM program, learning from gaoxiang12's slambook.

## Learning Roadmap
+ **Main route:**
- gao's video -> read gao's code -> **write the code** -> read book carefully and solver the problem -> **do practice**
+ **Side route:**
- A. For complex parts(e.g. Feature Detection, Epipolar Geometry, etc), read RVC or MVG book to study further.
- B. For Core parts(e.g. Motion Estimation, Nonlinear Optimization, Loop Detection, etc), read RVCMVG book or ETH course's PPT to practice.

## 0809 Feature Detection Code
- in ch7/feature_extraction_orb.cpp
1. ORB features and descriptors
2. match: brute force
3. select matches based on distance

## 0810 2D-2D pose estimation Code
- in ch7/pose_estimation_2d2d.cpp<br>

1. epipolar geometry 
- fundamental, essential, homography matrix 
2. recover pose from essential matrix
- using cv::recoverPose

## 0811 2D-3D, 3D-3D pose estimation and triangulation Code
- in ch7/pose_estimation_2d3d.cpp<br>
  using cv::solvePnP(EPnP method)
- in ch7/pose_estimation_3d3d.cpp<br>
  using ICP + SVD(Eigen)
- in ch7/triangulation.cpp
  using cv::triangulatePoints<br>

## 0812 nonlinear optimization for least square
- In order to solve 2d3d and 3d3d by BA method, basic nonlinear optimization and ceres/g2o lib should be learned first.

- in ch6/ceres_curve_fitting.cpp<br>
  using ceres lib to solver curve fitting problem using LM method
- in ch6/g2o_curve_fitting.cpp<br>
  using g2o lib to solver curve fitting problem using LM, GN, DogLeg method
