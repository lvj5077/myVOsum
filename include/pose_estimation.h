// Eigen
#include <Eigen/Core>
#include <Eigen/Geometry>

// OpenCV
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/core/eigen.hpp>

#include "opencv2/imgproc/imgproc.hpp"

#include <opencv2/features2d/features2d.hpp>

#include <iostream>


using namespace cv;
using namespace std;

class pose_estimation{
	public:
		void test();
		void pose3d3d_dirctSVD(vector<Point3f> & p_XYZs1,vector<Point3f> & p_XYZs2, Mat& T);

	    void pose3d3d_SVD(vector<Point3f> & p_XYZs1,vector<Point3f> & p_XYZs2, Mat&  mat_r, Mat&  vec_t, Mat* T = NULL  );
	    void RANSACpose3d3d_SVD(vector<Point3f> & p_XYZs1,vector<Point3f> & p_XYZs2, Mat&  mat_r, Mat&  vec_t, Mat* T = NULL  );

	    void pose3d3d_BA( vector<Point3f> & p_XYZs1, vector<Point3f> & p_XYZs2, Mat & mat_r, Mat & vec_t, Mat& T);
		void pose3d3d_BApose( vector<Point3f> & p_XYZs1, vector<Point3f> & p_XYZs2, Mat & mat_r, Mat & vec_t, Mat& T);

	    void pose2d2d_8pts( vector<Point2f> & p_UVs1, vector<Point2f> & p_UVs2, Mat & mat_r, Mat & vec_t,Mat &cameraMatrix, Mat* T = NULL );
	    void pose2d2d_triangulation( vector<Point2f> & p_UVs1, vector<Point2f> & p_UVs2, Mat & mat_r, Mat & vec_t,Mat &cameraMatrix, Mat* T = NULL );
	    void pose3d2d_PnP( vector<Point3f> & p_XYZs1, vector<Point2f> & p_UVs2, Mat & mat_r, Mat & vec_t, Mat &cameraMatrix, Mat* T = NULL);
	    void pose3d2d_BA( vector<Point3f> & p_XYZs1, vector<Point2f> & p_UVs2, Mat & mat_r, Mat & vec_t,Mat & T ,Mat &cameraMatrix );
};
