// Eigen
#include <Eigen/Core>
#include <Eigen/Geometry>

// OpenCV
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/core/eigen.hpp>

#include <iostream>


using namespace cv;
using namespace std;

class pose_estimation{
	public:
		void test();
	    void pose3d3d_SVD(vector<Point3f> & p_XYZs1,vector<Point3f> & p_XYZs2, Mat&  mat_r, Mat&  vec_t );
	    void pose3d3d_BA( vector<Point3f> & p_XYZs1, vector<Point3f> & p_XYZs2, Mat & mat_r, Mat & vec_t, Mat & T );
	    void pose2d2d_8pts( vector<Point2f> & p_UVs1, vector<Point2f> & p_UVs2, Mat & mat_r, Mat & vec_t,Mat &cameraMatrix );
	    void pose2d2d_triangulation( vector<Point2f> & p_UVs1, vector<Point2f> & p_UVs2, Mat & mat_r, Mat & vec_t,Mat &cameraMatrix );
	    void pose3d2d_PnP( vector<Point3f> & p_XYZs1, vector<Point2f> & p_UVs2, Mat & mat_r, Mat & vec_t, Mat &cameraMatrix);
	    void pose3d2d_BA( vector<Point3f> & p_XYZs1, vector<Point2f> & p_UVs2, Mat & mat_r, Mat & vec_t,Mat & T ,Mat &cameraMatrix );
};
