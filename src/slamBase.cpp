#include "slamBase.h"


#include <opencv2/opencv.hpp>  
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/xfeatures2d.hpp> // SIFT

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
    // cout << "Object is being deleted" << endl;
}


void slamBase::setCamera(CAMERA_INTRINSIC_PARAMETERS inC){
	C = inC;
}

CAMERA_INTRINSIC_PARAMETERS slamBase::getCamera(){
	return C;
}


SR4kFRAME slamBase::readSRFrame( string inFileName){
    SR4kFRAME f;

    int width = 176;
    int height = 144;

    cv::Mat I_gray = cv::Mat::zeros(height,width,CV_64F);

    int size[3] = { width, height, 3 };
    cv::Mat I_depth(3, size, CV_64F, cv::Scalar(10));

    ifstream inFile(inFileName);
    string str;

    int lineIdx = 0;


    while (getline(inFile, str))
    {
        lineIdx ++;
        if (lineIdx > 0 && lineIdx<(height+1))
        {
            for (int i = 0; i < width; i++)
            {
                inFile >> I_depth.at<double>(int(lineIdx-1),i,2) ;
            }
        }

        if (lineIdx > (height+1)*1 && lineIdx<(height+1)*2)
        {
            for (int i = 0; i < width; i++)
            {
                inFile >> I_depth.at<double>(int(lineIdx-1 -(height+1)*1 ),i,0) ;
            }
        }
        if (lineIdx > (height+1)*2 && lineIdx<(height+1)*3)
        {
            for (int i = 0; i < width; i++)
            {
                inFile >> I_depth.at<double>(int(lineIdx-1 -(height+1)*2 ),i,1);
            }
        }

        if (lineIdx > (height+1)*3 && lineIdx<(height+1)*4)
        {
            for (int i = 0; i < width; i++)
            {
                inFile >> I_gray.at<double>(int(lineIdx-1 -(height+1)*3 ),i,0) ;
            }
        }

    }

    I_gray = I_gray/8; //12bit
    I_gray.convertTo(I_gray,CV_8U);

    // I_gray.convertTo(I_gray, CV_8U, 1.0 / 256, 0);

    // equalizeHist( I_gray, I_gray );

    f.rgb = I_gray.clone();
    f.depthXYZ = I_depth.clone();

    return f;
}


void slamBase::findMatches(Mat rgb1,Mat rgb2,Mat depth1,Mat depth2,
			vector<Point2f> &p_UVs1,vector<Point2f> &p_UVs2,vector<Point3f> &p_XYZs1,vector<Point3f> &p_XYZs2){

	vector<Point2f> tp_UVs1;
	vector<Point2f> tp_UVs2;
	vector<Point3f> tp_XYZs1;
	vector<Point3f> tp_XYZs2;

	vector<KeyPoint> keypoints_1, keypoints_2;
    Mat descriptors_1, descriptors_2;

    // Ptr<FeatureDetector> detector = ORB::create();
    // Ptr<DescriptorExtractor> descriptor = ORB::create();

    // detector->detect ( rgb1,keypoints_1 );
    // detector->detect ( rgb2,keypoints_2 );

    // descriptor->compute ( rgb1, keypoints_1, descriptors_1 );
    // descriptor->compute ( rgb2, keypoints_2, descriptors_2 );

    // Ptr<DescriptorMatcher> matcher  = DescriptorMatcher::create ( "BruteForce-Hamming" );
    // vector<DMatch> match;
    // matcher->match ( descriptors_1, descriptors_2, match );

    // double min_dist=10000, max_dist=0;

    // for ( int i = 0; i < descriptors_1.rows; i++ )
    // {
    //     double dist = match[i].distance;
    //     if ( dist < min_dist ) min_dist = dist;
    //     if ( dist > max_dist ) max_dist = dist;
    // }

    // // printf ( "-- Max dist : %f \n", max_dist );
    // // printf ( "-- Min dist : %f \n", min_dist );

    // vector< DMatch > matches;
    // for ( int i = 0; i < descriptors_1.rows; i++ )
    // {
    //     if ( match[i].distance <= max ( 2*min_dist, 30.0 ) )
    //     {
    //         matches.push_back ( match[i] );
    //     }
    // }

    cv::Ptr<Feature2D> f2d = xfeatures2d::SIFT::create(80,3,.08,20,.8);
    f2d->detect ( rgb1,keypoints_1 );
    f2d->detect ( rgb2,keypoints_2 );

    f2d->compute ( rgb1, keypoints_1, descriptors_1 );
    f2d->compute ( rgb2, keypoints_2, descriptors_2 );

    BFMatcher matcher(NORM_L2);
    vector< DMatch > matches;
    matcher.match(descriptors_1,descriptors_2, matches);

    nth_element(matches.begin(),matches.begin()+50,matches.end());
    matches.erase(matches.begin()+50,matches.end());


    cv::Mat imgMatches;
    // cout <<"Find total "<<matches.size()<<" matches."<<endl;
    // cv::drawMatches( rgb1, keypoints_1, rgb2, keypoints_2, matches, imgMatches );
    // cv::imshow( "matches", imgMatches );
    // cv::waitKey( 0 );

    vector< DMatch > valid3Dmatches;
    for ( DMatch m:matches )
    {

        cv::Point2f p1 = keypoints_1[m.queryIdx].pt;
        cv::Point2f p2 = keypoints_2[m.trainIdx].pt;

        double d1 = double(depth1.ptr<unsigned short> ( int ( p1.y ) ) [ int ( p1.x ) ])/C.scale;
        double d2 = double(depth2.ptr<unsigned short> ( int ( p2.y ) ) [ int ( p2.x ) ])/C.scale;

        // d1 = 2;
        // d2 = 2;
        if ( d1<C.depthL||d1>C.depthH || d2<C.depthL||d2>C.depthH)   // bad depth
            continue;


        tp_UVs1.push_back( p1 );
		tp_UVs2.push_back( p2 );

        cv::Point3f p_XYZ;
        p_XYZ = point2dTo3d(p1,d1,C);
        tp_XYZs1.push_back( p_XYZ );
        p_XYZ = point2dTo3d(p2,d2,C);
		tp_XYZs2.push_back( p_XYZ );

		valid3Dmatches.push_back(m);

    }
	double camera_matrix_data[3][3] = {
	    {C.fx, 0, C.cx},
	    {0, C.fy, C.cy},
	    {0, 0, 1}
	};
	cv::Mat cameraMatrix( 3, 3, CV_64F, camera_matrix_data );
    Mat rvec,tvec;
    Mat inliers;
    cv::solvePnPRansac( tp_XYZs1, tp_UVs2, cameraMatrix, cv::Mat(), rvec, tvec, false, 100, 10, 0.95, inliers );

    // cout <<"inliers: "<<inliers.rows<<endl;
    // cout <<"R="<<rvec<<endl;
    // cout <<"t="<<tvec<<endl;
    vector< DMatch > RANSACmatches;
    for ( int i=0;i<inliers.rows;i++)
    {

        p_UVs1.push_back( tp_UVs1[ inliers.ptr<int>(i)[0]  ] );
		p_UVs2.push_back( tp_UVs2[ inliers.ptr<int>(i)[0]  ] );

        p_XYZs1.push_back( tp_XYZs1[ inliers.ptr<int>(i)[0]  ] );
		p_XYZs2.push_back( tp_XYZs2[ inliers.ptr<int>(i)[0]  ] );

		RANSACmatches.push_back(  valid3Dmatches[ inliers.ptr<int>(i)[0] ]   );

    }

    cout <<"Find total "<<inliers.rows<<" RANSAC inlier matches."<<endl;
    cv::drawMatches( rgb1, keypoints_1, rgb2, keypoints_2, RANSACmatches, imgMatches );
    cv::imshow( "RANSAC inlier matches", imgMatches );
    cv::waitKey( 0 );

    // cout<<"3d-3d pairs: "<<p_XYZs1.size() <<endl;
    // cout<<"3d-3d pairs: "<<p_XYZs1<<endl;
}

void slamBase::find4kMatches(Mat rgb1,Mat rgb2,Mat depth1,Mat depth2,
			vector<Point2f> &p_UVs1,vector<Point2f> &p_UVs2,vector<Point3f> &p_XYZs1,vector<Point3f> &p_XYZs2){

	vector<KeyPoint> keypoints_1, keypoints_2;
    Mat descriptors_1, descriptors_2;

    vector<Point2f> tp_UVs1;
    vector<Point2f> tp_UVs2;
    vector<Point3f> tp_XYZs1;
    vector<Point3f> tp_XYZs2;

    // // using orb
    // Ptr<FeatureDetector> detector = ORB::create();
    // Ptr<DescriptorExtractor> descriptor = ORB::create();

    // detector->detect ( rgb1,keypoints_1 );
    // detector->detect ( rgb2,keypoints_2 );

    // descriptor->compute ( rgb1, keypoints_1, descriptors_1 );
    // descriptor->compute ( rgb2, keypoints_2, descriptors_2 );

    // Ptr<DescriptorMatcher> matcher  = DescriptorMatcher::create ( "BruteForce-Hamming" );
    // vector<DMatch> match;
    // matcher->match ( descriptors_1, descriptors_2, match );

    // double min_dist=10000, max_dist=0;

    // for ( int i = 0; i < descriptors_1.rows; i++ )
    // {
    //     double dist = match[i].distance;
    //     if ( dist < min_dist ) min_dist = dist;
    //     if ( dist > max_dist ) max_dist = dist;
    // }

    // printf ( "-- Max dist : %f \n", max_dist );
    // printf ( "-- Min dist : %f \n", min_dist );

    // vector< DMatch > matches;
    // for ( int i = 0; i < descriptors_1.rows; i++ )
    // {
    //     if ( match[i].distance <= max ( 2*min_dist, 30.0 ) )
    //     {
    //         matches.push_back ( match[i] );
    //     }
    // }

        // int     nfeatures = 0,
        // int     nOctaveLayers = 3,
        // double  contrastThreshold = 0.04,
        // double  edgeThreshold = 10,
        // double  sigma = 1.6   


// nfeatures   The number of best features to retain. The features are ranked by their scores (measured in SIFT algorithm as the local contrast)
// nOctaveLayers   The number of layers in each octave. 3 is the value used in D. Lowe paper. The number of octaves is computed automatically from the image resolution.
// contrastThreshold   The contrast threshold used to filter out weak features in semi-uniform (low-contrast) regions. The larger the threshold, the less features are produced by the detector.
// edgeThreshold   The threshold used to filter out edge-like features. Note that the its meaning is different from the contrastThreshold, i.e. the larger the edgeThreshold, the less features are filtered out (more features are retained).
// sigma   The sigma of the Gaussian applied to the input image at the octave #0. If your image is captured with a weak camera with soft lenses, you might want to reduce the number.

    cv::Ptr<Feature2D> f2d = xfeatures2d::SIFT::create(80,3,.08,20,.8);
    f2d->detect ( rgb1,keypoints_1 );
    f2d->detect ( rgb2,keypoints_2 );

    f2d->compute ( rgb1, keypoints_1, descriptors_1 );
    f2d->compute ( rgb2, keypoints_2, descriptors_2 );

    BFMatcher matcher(NORM_L2);
    vector< DMatch > matches;
    matcher.match(descriptors_1,descriptors_2, matches);

    nth_element(matches.begin(),matches.begin()+50,matches.end());
    matches.erase(matches.begin()+50,matches.end());

    cv::Mat imgMatches;

    vector< DMatch > valid3Dmatches;
    for ( DMatch m:matches )
    {

        cv::Point2f p1 = keypoints_1[m.queryIdx].pt;
        cv::Point2f p2 = keypoints_2[m.trainIdx].pt;
        double d1 = depth1.at<double>(int(p1.x),int(p1.y),0);
        double d2 = depth2.at<double>(int(p2.x),int(p2.y),0);
        if ( d1<C.depthL||d1>C.depthH || d2<C.depthL||d2>C.depthH)   // bad depth
            continue;

        tp_UVs1.push_back( p1 );
        tp_UVs2.push_back( p2 );

        cv::Point3f p_XYZ;
        p_XYZ.x = depth1.at<double>(int(p1.x),int(p1.y),0);
        p_XYZ.y = depth1.at<double>(int(p1.x),int(p1.y),1);
        p_XYZ.z = depth1.at<double>(int(p1.x),int(p1.y),2);
        tp_XYZs1.push_back( p_XYZ );
        
        p_XYZ.x = depth2.at<double>(int(p2.x),int(p2.y),0);
        p_XYZ.y = depth2.at<double>(int(p2.x),int(p2.y),1);
        p_XYZ.z = depth2.at<double>(int(p2.x),int(p2.y),2);
        tp_XYZs2.push_back( p_XYZ );

        valid3Dmatches.push_back(m);

    }

    // cout <<"Find total "<<valid3Dmatches.size()<<" matches."<<endl;
    // cv::drawMatches( rgb1, keypoints_1, rgb2, keypoints_2, matches, imgMatches );
    // cv::imshow( "matches", imgMatches );
    // cv::waitKey( 0 );

    int no_RANSAC = 1;
    if (no_RANSAC){
        // p_XYZs1 = tp_XYZs1;
        // p_XYZs2 = tp_XYZs2;
        // p_UVs1 = tp_UVs1;
        // p_UVs2 = tp_UVs2;
        p_XYZs1.clear();
        p_XYZs2.clear();
        vector< DMatch > distMatches;
        for ( int i=0;i<tp_XYZs1.size();i++)
        {
            // cout << norm(tp_UVs1[i]-tp_UVs2[i]) << "  " << norm(tp_XYZs1[i]-tp_XYZs2[i])<<endl;
            if ( norm(tp_UVs1[i]-tp_UVs2[i])> 40 && norm(tp_XYZs1[i]-tp_XYZs2[i])>.5)
                continue;

            p_UVs1.push_back( tp_UVs1[ i ] );
            p_UVs2.push_back( tp_UVs2[ i ] );


            cv::Point3f p_XYZ;
            p_XYZ.x = depth1.at<double>(int(tp_UVs1[ i ].x),int(tp_UVs1[ i ].y),0);
            p_XYZ.y = depth1.at<double>(int(tp_UVs1[ i ].x),int(tp_UVs1[ i ].y),1);
            p_XYZ.z = depth1.at<double>(int(tp_UVs1[ i ].x),int(tp_UVs1[ i ].y),2);

            p_XYZs1.push_back( tp_XYZs1[ i ] );


            p_XYZ.x = depth2.at<double>(int(tp_UVs2[ i ].x),int(tp_UVs2[ i ].y),0);
            p_XYZ.y = depth2.at<double>(int(tp_UVs2[ i ].x),int(tp_UVs2[ i ].y),1);
            p_XYZ.z = depth2.at<double>(int(tp_UVs2[ i ].x),int(tp_UVs2[ i ].y),2);
            p_XYZs2.push_back( tp_XYZs2[ i ] );

            distMatches.push_back(  valid3Dmatches[ i ]   );

        }



        cout <<"Find total "<<distMatches.size()<<" matches."<<endl;
        cv::drawMatches( rgb1, keypoints_1, rgb2, keypoints_2, distMatches, imgMatches );
        cv::imshow( "matches", imgMatches );
        cv::waitKey( 0 );
    }
    else{
        double camera_matrix_data[3][3] = {
            {C.fx, 0, C.cx},
            {0, C.fy, C.cy},
            {0, 0, 1}
        };
        cv::Mat cameraMatrix( 3, 3, CV_64F, camera_matrix_data );
        Mat rvec,tvec;
        Mat inliers;
        cv::solvePnPRansac( tp_XYZs1, tp_UVs2, cameraMatrix, cv::Mat(), rvec, tvec, false, 100, 10, 0.95, inliers );
        // cout <<"inliers: "<<inliers.rows<<endl;
        // cout <<"R="<<rvec<<endl;
        // cout <<"t="<<tvec<<endl;
        vector< DMatch > RANSACmatches;
        for ( int i=0;i<inliers.rows;i++)
        {

            p_UVs1.push_back( tp_UVs1[ inliers.ptr<int>(i)[0]  ] );
            p_UVs2.push_back( tp_UVs2[ inliers.ptr<int>(i)[0]  ] );

            p_XYZs1.push_back( tp_XYZs1[ inliers.ptr<int>(i)[0]  ] );
            p_XYZs2.push_back( tp_XYZs2[ inliers.ptr<int>(i)[0]  ] );

            RANSACmatches.push_back(  valid3Dmatches[ inliers.ptr<int>(i)[0] ]   );

        }

        cout <<"Find total "<<inliers.rows<<" RANSAC inlier matches."<<endl;
        cv::drawMatches( rgb1, keypoints_1, rgb2, keypoints_2, RANSACmatches, imgMatches );
        cv::imshow( "RANSAC matches", imgMatches );
        cv::waitKey( 0 );
    }
    cout << "p_XYZs1.size() "<<p_XYZs1.size()  << endl;
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
	// cout << "T ==================================== "<<endl<<T<<endl;
	double rpE = 0;
        for (int i=0;i<p_XYZs1.size();i++){
            cv::Point3f pd1 = p_XYZs1[i];
            cv::Point3f pd2 = p_XYZs2[i];

            cv::Mat ptMat = (cv::Mat_<double>(4, 1) << pd1.x, pd1.y, pd1.z, 1);
            cv::Mat dstMat = T*ptMat;
            cv::Point3f projPd1(dstMat.at<double>(0,0), dstMat.at<double>(1,0),dstMat.at<double>(2,0));
            // cout <<pd1<<" "<< pd2<< "  "<<projPd1<< "  "<< norm(projPd1-pd2)<< endl;

            rpE = rpE + norm(projPd1-pd2);
            // if (norm(projPd1-pd2)>0.3){
            //     cout <<"bad point warnning"<<endl;
            // }
        }
        rpE = rpE/p_XYZs1.size();
	return rpE;
}

void slamBase::rotMtoRPY(Mat &mat_r, float &roll,float &pitch,float &yaw){
    float sy= sqrt(mat_r.at<double>(0,0) * mat_r.at<double>(0,0) +  mat_r.at<double>(1,0) * mat_r.at<double>(1,0) );
    roll = 180/3.14159265*atan2(mat_r.at<double>(2,1) , mat_r.at<double>(2,2));
    pitch = 180/3.14159265*atan2(-mat_r.at<double>(2,0), sy);
    yaw = 180/3.14159265*atan2(mat_r.at<double>(1,0), mat_r.at<double>(0,0));
}