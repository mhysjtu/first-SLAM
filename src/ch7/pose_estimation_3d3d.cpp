#include<iostream>

#include<opencv2/core/core.hpp>
#include<opencv2/features2d/features2d.hpp>
#include<opencv2/highgui/highgui.hpp>
#include<opencv2/calib3d/calib3d.hpp>//find F, E, H Matrix & recoverPose

#include<Eigen/Core>
#include<Eigen/Geometry>
#include<Eigen/SVD>

using namespace std;
using namespace cv;

void find_feature_matches (
	const Mat& img_1, const Mat& img_2,
	std::vector<KeyPoint>& keypoints_1,
	std::vector<KeyPoint>& keypoints_2,
	std::vector< DMatch >& matches );

void pose_estimation_2d2d(
	const vector<KeyPoint> keypoints_1, 
	const vector<KeyPoint> keypoints_2,
	const vector< DMatch > matches,
	Mat& R, Mat& t);

void pose_estimation_3d3d(
	const vector<Point3f>& pts1, 
	const vector<Point3f>& pts2,
	Mat& R, Mat& t);

Point2d pixel2cam(const Point2d& p, const Mat& K );

int main(int argc, char **argv){

    if(argc!=5){
        cout<<"usage: pose_estimation_3d3d img1 img2 depthimg1 depthimg2"<<endl;
        return 1;
    }

    Mat img1 = imread(argv[1], CV_LOAD_IMAGE_COLOR);
    Mat img2 = imread(argv[2], CV_LOAD_IMAGE_COLOR);

    vector<KeyPoint> keypoints_1, keypoints_2;
    std::vector<DMatch> matches;
    find_feature_matches(img1, img2, keypoints_1, keypoints_2, matches);
    cout<<"totally find "<<matches.size()<<" pairs correspondences"<<endl;

    Mat R1,t1;
    pose_estimation_2d2d(keypoints_1, keypoints_2, matches, R1, t1);


    //constuct 3D points
    Mat depth1 = imread( argv[3], CV_LOAD_IMAGE_UNCHANGED);
    Mat depth2 = imread( argv[4], CV_LOAD_IMAGE_UNCHANGED);
    Mat K = ( Mat_<double> ( 3,3 ) << 520.9, 0, 325.1, 0, 521.0, 249.7, 0, 0, 1 );
    vector<Point3f> pts1, pts2;
    for ( DMatch m:matches ){
    	//get point1's depth info from depth image1:
    	ushort d1 = depth1.ptr<unsigned short>( int(keypoints_1[m.queryIdx].pt.y))[int(keypoints_1[m.queryIdx].pt.x)];
    	ushort d2 = depth2.ptr<unsigned short>( int(keypoints_2[m.trainIdx].pt.y))[int(keypoints_2[m.trainIdx].pt.x)];
    	if (d1 == 0 || d2 == 0 )
    		continue;

    	float dd1 = float ( d1 )/1000.0;
    	float dd2 = float ( d2 )/1000.0;
    	Point2d p1 = pixel2cam(keypoints_1[m.queryIdx].pt, K);
    	Point2d p2 = pixel2cam(keypoints_2[m.trainIdx].pt, K);//normalize

    	pts1.push_back(Point3f(p1.x*dd1, p1.y*dd1, dd1));//normalized points times depth
    	pts2.push_back(Point3f(p2.x*dd2, p2.y*dd2, dd2));
    }

    cout<<"3d-3d pairs: "<<pts1.size()<<endl;
    Mat R, t;
    pose_estimation_3d3d(pts1, pts2, R, t);
    cout<<"ICP via SVD results: "<<endl;
    cout<<"R = "<<R<<endl;
    cout<<"t = "<<t<<endl;
    cout<<"R_inv = "<<R.t() <<endl;
    cout<<"t_inv = "<<-R.t() *t<<endl;
    //inv of block matrix: [A,B,0,C]-->[A^-1, -A^-1*B*C^-1, 0, C^-1]
    //[R,t,0,1]-->[R^-1, -R^-1*t*I, 0, 1]=[R^T, -R^T*t, 0, 1]
    //OR vector transform:
    //t-->-t, -t left times R^T


    // verify p1 = R*p2 + t
    for ( int i=0; i<5; i++ )
    {
        cout<<"p1 = "<<pts1[i]<<endl;
        cout<<"p2 = "<<pts2[i]<<endl;
        cout<<"(R*p2+t) = "<<
            R * (Mat_<double>(3,1)<<pts2[i].x, pts2[i].y, pts2[i].z) + t
            <<endl;
        cout<<endl;
    }

    return 0;

}

void find_feature_matches(
	const Mat& img_1, const Mat& img_2,
	std::vector<KeyPoint>& keypoints_1,
	std::vector<KeyPoint>& keypoints_2,
	std::vector< DMatch >& matches ){
	
	Mat descriptors_1, descriptors_2;

	Ptr<FeatureDetector> detector = ORB::create();
	Ptr<DescriptorExtractor> descriptor = ORB::create();

	Ptr<DescriptorMatcher> matcher = DescriptorMatcher::create("BruteForce-Hamming");

	detector->detect(img_1, keypoints_1);
	detector->detect(img_2, keypoints_2);

	descriptor->compute(img_1, keypoints_1, descriptors_1);
	descriptor->compute(img_2, keypoints_2, descriptors_2);

	std::vector<DMatch> match;
	matcher->match(descriptors_1, descriptors_2, match);

	double min_dist = 10000, max_dist = 0;

	for (int i = 0; i < descriptors_1.rows; ++i)
	{
		double dist = match[i].distance;
		if (dist < min_dist) min_dist = dist;
		if (dist > max_dist) max_dist = dist;
	}

	printf ( "-- Max dist : %f \n", max_dist );
    printf ( "-- Min dist : %f \n", min_dist );

    for (int i = 0; i < descriptors_1.rows; ++i)
    {
    	if (match[i].distance <= max(2*min_dist, 30.0))
    	{
    		matches.push_back(match[i]);
    	}
    }
}

void pose_estimation_2d2d(
	const vector<KeyPoint> keypoints_1, 
	const vector<KeyPoint> keypoints_2,
	const vector< DMatch > matches,
	Mat& R, Mat& t){

    Mat K = (Mat_ <double> (3, 3) << 520.9, 0, 325.1, 0, 521.0, 249.7, 0, 0, 1);
	
	vector<Point2f> points1;
	vector<Point2f> points2;
	
	for(int i = 0; i<matches.size(); i++){
		points1.push_back(keypoints_1[matches[i].queryIdx].pt);
		points2.push_back(keypoints_2[matches[i].trainIdx].pt);
	}

	Mat fundamental_matrix;
	fundamental_matrix = findFundamentalMat(points1, points2, CV_FM_8POINT);
	cout<<"fundamental_matrix is: "<<endl<<fundamental_matrix<<endl;
	
	Point2d principal_point(325.1, 249.7);
	int focal_length = 521;
	Mat essential_matrix;
	essential_matrix = findEssentialMat(points1, points2, focal_length, principal_point, RANSAC);
	cout<<"essential_matrix is: "<<endl<<essential_matrix<<endl;

	cout<<"F: K-T*E*K-1="<<endl<<((K.t()).inv())*essential_matrix*(K.inv())<<endl;//it seems not working

	Mat homography_matrix;
	homography_matrix = findHomography(points1, points2, RANSAC, 3, noArray(), 2000, 0.99);
	cout<<"homography_matrix is: "<<endl<<homography_matrix<<endl;

	recoverPose(essential_matrix, points1, points2, R, t, focal_length, principal_point);
	cout<<"R is: "<<endl<<R<<endl;
	cout<<"t is: "<<endl<<t<<endl;

}

Point2d pixel2cam(const Point2d& p, const Mat& K ){
	return Point2d
			(
				((p.x - K.at<double>(0,2))/ K.at<double>(0,0)),
				((p.y - K.at<double>(1,2))/ K.at<double>(1,1))
			);
			//pixel points to normalized homogeneous coordinates:
			//x = (u-u0)/fx
			//y = (v-v0)/fy
}

void pose_estimation_3d3d(
	const vector<Point3f>& pts1, 
	const vector<Point3f>& pts2,
	Mat& R, Mat& t){

	//remove center of mass
	Point3f p1, p2;
	int N = pts1.size();
	for (int i = 0; i < N; ++i)
	{
		p1 += pts1[i];
		p2 += pts2[i];
	}
	p1 /= N; 
	p2 /= N;

	std::vector<Point3f> q1(N), q2(N);
	for (int i = 0; i < N; ++i)
	{
		q1[i] = pts1[i] - p1;
		q2[i] = pts2[i] - p2;
	}


	// compute q1*q2^T
	Eigen::Matrix3d w = Eigen::Matrix3d::Zero();
	for (int i = 0; i < N; ++i)
	{
		w += Eigen::Vector3d(q1[i].x, q1[i].y, q1[i].z) * Eigen::Vector3d(q2[i].x, 
			q2[i].y, q2[i].z).transpose();
	}
	cout<<"W = " <<w<<endl;

	//svd on w
	Eigen::JacobiSVD<Eigen::Matrix3d> svd(w, Eigen::ComputeFullU | Eigen::ComputeFullV);
	Eigen::Matrix3d U = svd.matrixU();
	Eigen::Matrix3d V = svd.matrixV();
	cout<<"U = " <<U<<endl;
	cout<<"V = " <<V<<endl;

	//R = U*V^T
	//t = p1 - R*p2
	Eigen::Matrix3d R_ = U*(V.transpose());
	Eigen::Vector3d t_ = Eigen::Vector3d(p1.x, p1.y, p1.z)- R_ * Eigen::Vector3d(p2.x, p2.y, p2.z);

	R = (Mat_<double>(3,3)<<
		R_(0,0), R_(0,1), R_(0,2),
		R_(1,0), R_(1,1), R_(1,2),
		R_(2,0), R_(2,1), R_(2,2)
	);
	t = (Mat_<double>(3,1)<<t_(0,0), t_(1,0), t_(2,0));

}