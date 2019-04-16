#include <algorithm>
#include <iostream>
#include <fstream>
#include <sstream>

#include "pose_estimation.h"
#include "slamBase.h"

using namespace std;




int main( int argc, char** argv )
{

	double depthL = 0.180;
	double depthH = 5.000;

	// CAMERA_INTRINSIC_PARAMETERS C;

	// C.cx = 325.141442;
	// C.cy = 249.701764;
	// C.fx = 520.908620;
	// C.fy = 521.007327;
	// C.scale = 5208;
	// C.cx = 325.1;
	// C.cy = 249.7;
	// C.fx = 520.9;
	// C.fy = 521.0;
	// C.scale = 5000;
	// double camera_matrix_data[3][3] = {
	//     {C.fx, 0, C.cx},
	//     {0, C.fy, C.cy},
	//     {0, 0, 1}
	// };
	// cv::Mat cameraMatrix( 3, 3, CV_64F, camera_matrix_data );

    // string firstF = "/Users/lingqiujin/Data/RV_Data/Pitch/d1_-40/d1_0044.dat";
    // string secondF = "/Users/lingqiujin/Data/RV_Data/Pitch/d2_-37/d2_0044.dat";

    Mat rgb1 = imread ( "/Users/lingqiujin/Data/VOdata/color1.png", CV_LOAD_IMAGE_COLOR );
    Mat rgb2 = imread ( "/Users/lingqiujin/Data/VOdata/color2.png", CV_LOAD_IMAGE_COLOR );

    Mat depth1 = imread ( "/Users/lingqiujin/Data/VOdata/depth1.png", CV_LOAD_IMAGE_UNCHANGED ); 
    Mat depth2 = imread ( "/Users/lingqiujin/Data/VOdata/depth2.png", CV_LOAD_IMAGE_UNCHANGED );

    vector<Point2f> p_UVs1, p_UVs2;
    vector<Point3f> p_XYZs1, p_XYZs2;



    Mat mat_r, vec_t;
    cv::Mat T = cv::Mat::eye(4,4,CV_64F);
    double rpE = 0;

    pose_estimation myVO; 
    slamBase myBase; 

    CAMERA_INTRINSIC_PARAMETERS C = myBase.getCamera();
	double camera_matrix_data[3][3] = {
	    {C.fx, 0, C.cx},
	    {0, C.fy, C.cy},
	    {0, 0, 1}
	};
	cv::Mat cameraMatrix( 3, 3, CV_64F, camera_matrix_data );




    myBase.findMatches(rgb1,rgb2,depth1,depth2,p_UVs1,p_UVs2,p_XYZs1,p_XYZs2);

    myVO.pose3d3d_SVD(p_XYZs1, p_XYZs2, mat_r, vec_t );
    cout << mat_r <<endl<<vec_t<<endl;
    rpE =  myBase.reprojectionError( p_XYZs1, p_XYZs2, mat_r, vec_t );
    cout << "reprojection error " << 1000*rpE<< " mm"<<endl<<endl;

  
    myVO.pose3d3d_BA( p_XYZs1, p_XYZs2, mat_r, vec_t, T );
    rpE =  myBase.reprojectionError( p_XYZs1, p_XYZs2, T);
    cout << "reprojection error " << 1000*rpE<< " mm"<<endl<<endl;


    myVO.pose3d2d_PnP( p_XYZs1, p_UVs2, mat_r, vec_t, cameraMatrix);
    cout << mat_r <<endl<<vec_t<<endl;
    rpE =  myBase.reprojectionError( p_XYZs1, p_XYZs2, mat_r, vec_t );
    cout << "reprojection error " << 1000*rpE<< " mm"<<endl<<endl;

    myVO.pose3d2d_BA( p_XYZs1, p_UVs2, mat_r, vec_t, T, cameraMatrix  );
    rpE =  myBase.reprojectionError( p_XYZs1, p_XYZs2, mat_r, vec_t );
    cout << "reprojection error " << 1000*rpE<< " mm"<<endl<<endl;
    

    myVO.pose2d2d_8pts( p_UVs1, p_UVs2, mat_r, vec_t, cameraMatrix );
    myVO.pose2d2d_triangulation( p_UVs1, p_UVs2, mat_r, vec_t, cameraMatrix );


    return 0;
}