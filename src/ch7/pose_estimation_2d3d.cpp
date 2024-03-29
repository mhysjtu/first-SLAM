#include<iostream>

#include<opencv2/core/core.hpp>
#include<opencv2/features2d/features2d.hpp>
#include<opencv2/highgui/highgui.hpp>
#include<opencv2/calib3d/calib3d.hpp>//find F, E, H Matrix & recoverPose

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

Point2d pixel2cam(const Point2d& p, const Mat& K );

int main(int argc, char **argv){

    if(argc!=4){
        cout<<"usage: pose_estimation_2d3d img1 img2 depthimg1"<<endl;
        return 1;
    }

    Mat img1 = imread(argv[1], CV_LOAD_IMAGE_COLOR);
    Mat img2 = imread(argv[2], CV_LOAD_IMAGE_COLOR);

    vector<KeyPoint> keypoints_1, keypoints_2;
    std::vector<DMatch> matches;
    find_feature_matches(img1, img2, keypoints_1, keypoints_2, matches);
    cout<<"totally find "<<matches.size()<<" pairs correspondences"<<endl;

    //2D2D for comparison
    Mat R1,t1;
    pose_estimation_2d2d(keypoints_1, keypoints_2, matches, R1, t1);


    //2d-3d PNP
    Mat d1 = imread(argv[3], CV_LOAD_IMAGE_UNCHANGED);
    
    Mat K = (Mat_ <double> (3, 3) << 520.9, 0, 325.1, 0, 521.0, 249.7, 0, 0, 1);
    std::vector<Point3f> pts_3d;
    std::vector<Point2f> pts_2d;
    for ( DMatch m:matches ){
    	//get point1's depth info from depth image1:
    	ushort d = d1.ptr<unsigned short>( int(keypoints_1[m.queryIdx].pt.y))[int(keypoints_1[m.queryIdx].pt.x)];
    	if (d == 0)
    		continue;

    	float dd = d/1000.0;
    	Point2d p1 = pixel2cam(keypoints_1[m.queryIdx].pt, K);//normalize

    	pts_3d.push_back(Point3f(p1.x*dd, p1.y*dd, dd));//normalized points times depth
    	pts_2d.push_back(keypoints_2[m.trainIdx].pt);
    }

    cout<<"2d-3d pairs: "<<pts_3d.size()<<endl;

    Mat r, t;
    solvePnP(pts_3d, pts_2d, K, Mat(), r, t, false, cv::SOLVEPNP_EPNP);

    Mat R;
    cv::Rodrigues(r, R);//vector r to Matrix R, using Rodrigues formula

    cout<<"R = "<<endl<<R<<endl;
    cout<<"t = "<<endl<<t<<endl;
    //2d-3d PNP

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