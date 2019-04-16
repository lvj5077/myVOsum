// Eigen
#include <Eigen/Core>
#include <Eigen/Geometry>

// OpenCV
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/core/eigen.hpp>
#include <opencv2/features2d/features2d.hpp>


#include <iostream>

using namespace cv;
using namespace std;

struct CAMERA_INTRINSIC_PARAMETERS 
{ 
    double cx, cy, fx, fy, scale, depthL, depthH;
};


class slamBase{
	private:
		CAMERA_INTRINSIC_PARAMETERS C;
		// double depthL = 0.18;
		// double depthH = 5.0;

	public:
		slamBase();
		~slamBase();
		void setCamera(CAMERA_INTRINSIC_PARAMETERS inC);
		CAMERA_INTRINSIC_PARAMETERS getCamera();
		void findMatches(Mat rgb1,Mat rgb2,Mat depth1,Mat depth2,
			vector<Point2f> &p_UVs1,vector<Point2f> &p_UVs2,vector<Point3f> &p_XYZs1,vector<Point3f> &p_XYZs2);
	    cv::Point3f point2dTo3d( cv::Point2f& point, double& d, CAMERA_INTRINSIC_PARAMETERS& camera );
	    double reprojectionError( vector<Point3f> & p_XYZs1, vector<Point3f> & p_XYZs2, Mat & mat_r, Mat & vec_t );
	    double reprojectionError( vector<Point3f> & p_XYZs1, vector<Point3f> & p_XYZs2, Mat & T );
};