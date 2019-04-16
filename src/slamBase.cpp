#include "slamBase.h"

slamBase::slamBase(void)
{
	// C.cx = 325.141442;
	// C.cy = 249.701764;
	// C.fx = 520.908620;
	// C.fy = 521.007327;
	// C.scale = 5208;

	C.cx = 325.1;
	C.cy = 249.7;
	C.fx = 520.9;
	C.fy = 521.0;
	C.scale = 5000;

	C.depthL = 0.18;
	C.depthH = 5.0;
    // cout << "Object is being created" << endl;
}

slamBase::~slamBase(void)
{
    cout << "Object is being deleted" << endl;
}


void slamBase::setCamera(CAMERA_INTRINSIC_PARAMETERS inC){
	C = inC;
}

CAMERA_INTRINSIC_PARAMETERS slamBase::getCamera(){
	return C;
}

void slamBase::findMatches(Mat rgb1,Mat rgb2,Mat depth1,Mat depth2,
			vector<Point2f> &p_UVs1,vector<Point2f> &p_UVs2,vector<Point3f> &p_XYZs1,vector<Point3f> &p_XYZs2){

	vector<KeyPoint> keypoints_1, keypoints_2;
    Mat descriptors_1, descriptors_2;

    Ptr<FeatureDetector> detector = ORB::create();
    Ptr<DescriptorExtractor> descriptor = ORB::create();

    detector->detect ( rgb1,keypoints_1 );
    detector->detect ( rgb2,keypoints_2 );

    descriptor->compute ( rgb1, keypoints_1, descriptors_1 );
    descriptor->compute ( rgb2, keypoints_2, descriptors_2 );

    Ptr<DescriptorMatcher> matcher  = DescriptorMatcher::create ( "BruteForce-Hamming" );
    vector<DMatch> match;
    matcher->match ( descriptors_1, descriptors_2, match );

    double min_dist=10000, max_dist=0;

    for ( int i = 0; i < descriptors_1.rows; i++ )
    {
        double dist = match[i].distance;
        if ( dist < min_dist ) min_dist = dist;
        if ( dist > max_dist ) max_dist = dist;
    }

    printf ( "-- Max dist : %f \n", max_dist );
    printf ( "-- Min dist : %f \n", min_dist );

    vector< DMatch > matches;
    for ( int i = 0; i < descriptors_1.rows; i++ )
    {
        if ( match[i].distance <= max ( 2*min_dist, 30.0 ) )
        {
            matches.push_back ( match[i] );
        }
    }


    for ( DMatch m:matches )
    {

        cv::Point2f p1 = keypoints_1[m.queryIdx].pt;
        cv::Point2f p2 = keypoints_2[m.trainIdx].pt;
        double d1 = double(depth1.ptr<unsigned short> ( int ( keypoints_1[m.queryIdx].pt.y ) ) [ int ( keypoints_1[m.queryIdx].pt.x ) ])/5000;
        double d2 = double(depth2.ptr<unsigned short> ( int ( keypoints_2[m.trainIdx].pt.y ) ) [ int ( keypoints_2[m.trainIdx].pt.x ) ])/5000;
        if ( d1<C.depthL||d1>C.depthH || d2<C.depthL||d2>C.depthH)   // bad depth
            continue;

        p_UVs1.push_back( p1 );
		p_UVs2.push_back( p2 );

        cv::Point3f p_XYZ;
        p_XYZ = point2dTo3d(p1,d1,C);
        p_XYZs1.push_back( p_XYZ );
        p_XYZ = point2dTo3d(p2,d2,C);
		p_XYZs2.push_back( p_XYZ );

    }

    // cout<<"3d-3d pairs: "<<p_XYZs1.size() <<endl;
    // cout<<"3d-3d pairs: "<<p_XYZs1<<endl;
}


cv::Point3f slamBase::point2dTo3d( cv::Point2f& point, double& d, CAMERA_INTRINSIC_PARAMETERS& camera )
{
    cv::Point3f p;
    p.z = float( d) ;
    p.x = ( point.x - camera.cx) * p.z / camera.fx;
    p.y = ( point.y - camera.cy) * p.z / camera.fy;

    return p;
}

double slamBase::reprojectionError( vector<Point3f> & p_XYZs1, vector<Point3f> & p_XYZs2, Mat & mat_r, Mat & vec_t ){
	double rpE = 0;

    cv::Mat T = cv::Mat::eye(4,4,CV_64F);

    mat_r.copyTo(T(cv::Rect(0, 0, 3, 3)));
    vec_t.copyTo(T(cv::Rect(3, 0, 1, 3)));
	cout << "T "<<T<<endl;

	rpE = reprojectionError(p_XYZs1,p_XYZs2,T);

 //    for (int i=0;i<p_XYZs1.size();i++){

 //        rpE = rpE + norm(mat_r* (Mat_<double>(3,1)<<p_XYZs1[1].x, p_XYZs1[1].y, p_XYZs1[1].z) + vec_t - 
	// 					(Mat_<double>(3,1)<<p_XYZs2[1].x, p_XYZs2[1].y, p_XYZs2[1].z) ) ;
 //    }

 //    rpE = rpE/p_XYZs1.size();

    // for (int i=0;i<5;i++){
    //     cv::Point3f pd1 = p_XYZs1[i];
    //     cv::Point3f pd2 = p_XYZs2[i];
    //     cout << "pd2 "<<(p_XYZs2[i])<<endl;
    //     cv::Mat ptMat = (cv::Mat_<double>(4, 1) << pd1.x, pd1.y, pd1.z, 1);
    //     cv::Mat dstMat = T*ptMat;
    //     cv::Point3f projPd1(dstMat.at<double>(0,0), dstMat.at<double>(1,0),dstMat.at<double>(2,0));
    //     cout << "projPd1 "<<projPd1<<endl;
    //     cout << ( mat_r* (Mat_<double>(3,1)<<p_XYZs1[i].x, p_XYZs1[i].y, p_XYZs1[i].z) + vec_t ).t()<<endl;
    // }
	return rpE;
}

double slamBase::reprojectionError( vector<Point3f> & p_XYZs1, vector<Point3f> & p_XYZs2, Mat & T ){
	double rpE = 0;
        for (int i=0;i<p_XYZs1.size();i++){
            cv::Point3f pd1 = p_XYZs1[i];
            cv::Point3f pd2 = p_XYZs2[i];

            cv::Mat ptMat = (cv::Mat_<double>(4, 1) << pd1.x, pd1.y, pd1.z, 1);
            cv::Mat dstMat = T*ptMat;
            cv::Point3f projPd1(dstMat.at<double>(0,0), dstMat.at<double>(1,0),dstMat.at<double>(2,0));
            // cout << "projPd1 "<<projPd1<<endl;
            rpE = rpE + norm(projPd1-pd2);
        }
        rpE = rpE/p_XYZs1.size();
	return rpE;
}