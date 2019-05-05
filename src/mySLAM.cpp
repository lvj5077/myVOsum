#include <algorithm>
#include <iostream>
#include <fstream>
#include <sstream>
#include <iomanip>


#include "pose_estimation.h"
#include "slamBase.h"

#include <stdlib.h>    

using namespace std;
using namespace pcl;

int main( int argc, char** argv )
{


	double depthL = 0.80;
	double depthH = 8.000;

    CAMERA_INTRINSIC_PARAMETERS C_sr4k;

    C_sr4k.cx = 88.5320;
    C_sr4k.cy = 72.5102;
    C_sr4k.fx = 222.6132;
    C_sr4k.fy = 225.6439;
    C_sr4k.scale = 1000;
    C_sr4k.depthL = 0.180;
    C_sr4k.depthH = 7.000;
    C_sr4k.height = 144;
    C_sr4k.width = 176;
    C_sr4k.exp = 50;


    string firstF = "/Users/lingqiujin/Data/RV_Data/Pitch/d1_-40/d1_0021.dat";
    string secondF = "/Users/lingqiujin/Data/RV_Data/Pitch/d2_-37/d2_0002.dat";

    vector<Point2f> p_UVs1, p_UVs2;
    vector<Point3f> p_XYZs1, p_XYZs2;


    Mat mat_r, vec_t;
    cv::Mat T = cv::Mat::eye(4,4,CV_64F);

    struct myPoseAtoB
    {
        int posA_id = 0;
        int posB_id = 0;
        Mat T_ab = cv::Mat::eye(4,4,CV_64F);
        
    };
    std::vector<myPoseAtoB> poseChain;


    pose_estimation myVO_SR4k; 
    slamBase myBase_SR4k; 

    myBase_SR4k.setCamera(C_sr4k);

    CAMERA_INTRINSIC_PARAMETERS C = myBase_SR4k.getCamera();
    double camera_matrix_data_4k[3][3] = {
        {C.fx, 0, C.cx},
        {0, C.fy, C.cy},
        {0, 0, 1}
    };
    cv::Mat cameraMatrix_4k( 3, 3, CV_64F, camera_matrix_data_4k );

    int startIdx=1;
    int endIdx=77;

    // PointCloud<pcl::PointXYZ> pc_all;
    pcl::PointCloud<pcl::PointXYZ>::Ptr pc_all (new pcl::PointCloud<pcl::PointXYZ>);

    for (int idx=startIdx;idx<endIdx;idx++){

        std::stringstream ss;
        int firstF_id = idx;
        int secondF_id = idx+1;

        ss << "/Users/lingqiujin/Data/RV_Data2/d1_"<<std::setw(4) << std::setfill('0') << firstF_id <<".dat";
        firstF = ss.str();

        ss= std::stringstream();
        ss << "/Users/lingqiujin/Data/RV_Data2/d1_"<<std::setw(4) << std::setfill('0') << secondF_id <<".dat";
        secondF = ss.str();

        cout << endl<<"load "<<firstF<<endl;
        cout << "load "<<secondF<<endl;
    



        SR4kFRAME f1_4k = myBase_SR4k.readSRFrame(firstF);
        SR4kFRAME f2_4k = myBase_SR4k.readSRFrame(secondF);
        p_UVs1.clear();
        p_UVs2.clear();
        p_XYZs1.clear();
        p_XYZs2.clear();
        
        myBase_SR4k.find4kMatches(f1_4k.rgb,f2_4k.rgb,f1_4k.depthXYZ,f2_4k.depthXYZ,p_UVs1,p_UVs2,p_XYZs1,p_XYZs2);

        std::vector<int> inliers;
        PointCloud<pcl::PointXYZ> pc1;
        PointCloud<pcl::PointXYZ> pc2;
        
        vector<Point3f> pts1;
        vector<Point3f> pts2;


        myVO_SR4k.RANSACpose3d3d_SVD(p_XYZs2, p_XYZs1, mat_r, vec_t, inliers, &T );

        pts1 = myBase_SR4k.imagToCVpt( f1_4k.depthXYZ, C_sr4k );
        pts2 = myBase_SR4k.imagToCVpt( f2_4k.depthXYZ, C_sr4k );

        pc1 = myBase_SR4k.cvPtsToPCL(pts1);
        pc2 = myBase_SR4k.cvPtsToPCL(pts2);

        Eigen::Isometry3d T_eigen = myBase_SR4k.cvTtoEigenT(T);

        pcl::transformPointCloud( *pc_all, *pc_all, T_eigen.matrix() );
        *pc_all = pc2+*pc_all;

        pcl::VoxelGrid<pcl::PointXYZ> sor;
        sor.setLeafSize (0.01f, 0.01f, 0.01f); //1cm
        sor.setInputCloud (pc_all);
        sor.filter (*pc_all);

        pcl::io::savePCDFile( "./pc_all.pcd", *pc_all );

        myPoseAtoB findPose;

        findPose.posA_id = firstF_id;
        findPose.posB_id = secondF_id;
        findPose.T_ab = T;

        poseChain.push_back(findPose);

        // pcl::visualization::CloudViewer viewer( "viewer" );
        // viewer.showCloud( pc_12 );
        // while( !viewer.wasStopped() )
        // {
            
        // }

    }

    return 0;
}