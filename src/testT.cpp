#include <algorithm>
#include <iostream>
#include <fstream>
#include <sstream>

#include "pose_estimation.h"
#include "slamBase.h"

#include <stdlib.h>    

using namespace std;
using namespace pcl;

int main( int argc, char** argv )
{


	double depthL = 0.8;
	double depthH = 8.000;

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
    C_sr4k.exp = 8;


    vector<Point2f> p_UVs1, p_UVs2;
    vector<Point3f> p_XYZs1, p_XYZs2;


    Mat mat_r, vec_t;
    cv::Mat T = cv::Mat::eye(4,4,CV_64F);
    double rpE = 0;
    float roll,pitch,yaw;
    float sy;
    CAMERA_INTRINSIC_PARAMETERS C;

    int dataSize = 50;
    int myFakedata = 0;
    int randomNoise = 0;
    int mismatch = 0;
    if(argc>=3) 
    { 
        myFakedata = (int)argv[2][0]-48;
        randomNoise = (int)argv[2][1]-48; 
        mismatch = (int)argv[2][2]-48;  
    } 
    cv::Mat mroll(cv::Size(dataSize,1),CV_64FC1, Scalar(0));
    cv::Mat mpitch(cv::Size(dataSize,1),CV_64FC1, Scalar(0));
    cv::Mat myaw(cv::Size(dataSize,1),CV_64FC1, Scalar(0));
    cv::Mat mx(cv::Size(dataSize,1),CV_64FC1, Scalar(0));
    cv::Mat my(cv::Size(dataSize,1),CV_64FC1, Scalar(0));
    cv::Mat mz(cv::Size(dataSize,1),CV_64FC1, Scalar(0));

    string firstF,secondF;

    cout << "start!"<<endl<<endl;
    for (int idx=1;idx<dataSize+1;idx++){

        // if (idx<10){
        //     firstF = "/Users/lingqiujin/Data/RV_Data1_Test/d7_-22/d7_000"+to_string(idx)+".dat";
        // }else{
        //     firstF = "/Users/lingqiujin/Data/RV_Data1_Test/d7_-22/d7_00"+to_string(idx)+".dat";
        // }
        
        // if (idx<10){
        //     secondF = "/Users/lingqiujin/Data/RV_Data1_Test/d5_-28/d5_000"+to_string(idx)+".dat";
        // }else{
        //     secondF = "/Users/lingqiujin/Data/RV_Data1_Test/d5_-28/d5_00"+to_string(idx)+".dat";
        // }


        firstF = "/Users/lingqiujin/Data/RV_Data1_Test/d7_-22/d7_0099.dat";
        secondF = "/Users/lingqiujin/Data/RV_Data1_Test/d7_-22/d7_0009.dat";
        if (idx<10){
            firstF = "/Users/lingqiujin/Data/RV_Data/Translation/Y1/frm_000"+to_string(idx)+".dat";
        }else{
            firstF = "/Users/lingqiujin/Data/RV_Data/Translation/Y1/frm_00"+to_string(idx)+".dat";
        }
        
        if (idx<10){
            secondF = "/Users/lingqiujin/Data/RV_Data/Translation/Y2/frm_000"+to_string(idx)+".dat";
        }else{
            secondF = "/Users/lingqiujin/Data/RV_Data/Translation/Y2/frm_00"+to_string(idx)+".dat";
        }

        // if (idx<10){
        //     firstF = "/Users/lingqiujin/Data/RV_Data/Yaw/d1_44/d1_000"+to_string(idx)+".dat";
        // }else{
        //     firstF = "/Users/lingqiujin/Data/RV_Data/Yaw/d1_44/d1_00"+to_string(idx)+".dat";
        // }
        
        // if (idx<10){
        //     secondF = "/Users/lingqiujin/Data/RV_Data/Yaw/d2_41/d2_000"+to_string(idx)+".dat";
        // }else{
        //     secondF = "/Users/lingqiujin/Data/RV_Data/Yaw/d2_41/d2_00"+to_string(idx)+".dat";
        // }

        cout << endl<<"load "<<firstF<<endl;
        cout << "load "<<secondF<<endl;

        pose_estimation myVO_SR4k; 
        slamBase myBase_SR4k; 

        myBase_SR4k.setCamera(C_sr4k);

        C = myBase_SR4k.getCamera();
        double camera_matrix_data_4k[3][3] = {
            {C.fx, 0, C.cx},
            {0, C.fy, C.cy},
            {0, 0, 1}
        };
        cv::Mat cameraMatrix_4k( 3, 3, CV_64F, camera_matrix_data_4k );

        SR4kFRAME f1_4k = myBase_SR4k.readSRFrame(firstF);
        SR4kFRAME f2_4k = myBase_SR4k.readSRFrame(secondF);
        p_UVs1.clear();
        p_UVs2.clear();
        p_XYZs1.clear();
        p_XYZs2.clear();
        
        myBase_SR4k.find4kMatches(f1_4k.rgb,f2_4k.rgb,f1_4k.depthXYZ,f2_4k.depthXYZ,p_UVs1,p_UVs2,p_XYZs1,p_XYZs2);

        float myRoll = 0*3.1415926/180;
        float myPitch = 0*3.1415926/180;
        float myYaw = 0*3.1415926/180;
        Mat myR = myBase_SR4k.eulerAnglesToRotationMatrix(myRoll, myPitch, myYaw);
        Mat Tgt = cv::Mat::eye(4,4,CV_64F);
        myR.copyTo(Tgt(cv::Rect(0, 0, 3, 3)));
        Tgt.at<double>(0,3) = 0.0;
        Tgt.at<double>(1,3) = 0.0;
        Tgt.at<double>(2,3) = 0.0;

        vector<Point3f> p_XYZs;
        vector<Point3f> p_XYZst;
        for (int i =0;i<p_XYZs1.size();i++){
            cv::Mat ptMat = (cv::Mat_<double>(4, 1) << p_XYZs1[ i ].x, p_XYZs1[ i ].y, p_XYZs1[ i ].z, 1);
            cv::Mat dstMat = Tgt*ptMat;
            cv::Point3f projPd1(dstMat.at<double>(0,0), dstMat.at<double>(1,0),dstMat.at<double>(2,0));
            // cout << "check data "<<p_XYZs1[ i ]<< "  "<< p_XYZs2[ i ]<< "  "<< projPd1<< " "
            //         << (p_XYZs2[ i ] - projPd1)<<"  "<< norm((p_XYZs2[ i ] - projPd1))<<endl;
            // cout << projPd1<<endl;
            
            if (randomNoise==1){
                projPd1.x = projPd1.x+ (rand() % 10 -4.5)/200 ;
                projPd1.y = projPd1.y+ (rand() % 10 -4.5)/200 ;
                projPd1.z = projPd1.z+ (rand() % 10 -4.5)/200 ;               
            }
            if (mismatch==1){
                if ((i %20) ==1){
                    projPd1.x = (rand() % 10 -4.5)/20 ;
                    projPd1.y = (rand() % 10 -4.5)/20 ;
                    projPd1.z = (rand() % 10 -4.5)/20 ;   
                }
        
            }
            // if (norm((p_XYZs2[ i ] - projPd1)) < 0.1){
            //     p_XYZs.push_back( p_XYZs2[ i ] );
            //     p_XYZst.push_back( p_XYZs1[ i ] );
            //     // cout << " norm(projPd1-pd2)  "<< norm(projPd1-pd2)<<endl;
            // }
            p_XYZs.push_back( projPd1 );
            p_XYZst.push_back( p_XYZs1[ i ] );
        }  

        if (myFakedata == 1){
            p_XYZs2.clear();
            p_XYZs2 = p_XYZs;
            p_XYZs1.clear();
            p_XYZs1 = p_XYZst;
        }

        cout << "p_XYZs1.size() "<< p_XYZs1.size()<<endl;
 
        cout << "======================================"<<endl;

        myBase_SR4k.rotMtoRPY(Tgt, roll, pitch, yaw);
        cout << "roll " << roll<<" pitch " << pitch<<" yaw " << yaw<<endl;
        double baseE =  myBase_SR4k.reprojectionError( p_XYZs1, p_XYZs2, Tgt);
        cout << "groundTruth error " << 1000*baseE<< " mm"<<endl<<endl;

        char method = '0';
        if(argc>=2) 
        { 
            method = argv[1][0]; 
        } 

        cv::Mat outM3by4G;// = cv::Mat::zeros(3,4,CV_64F);            
        cv::Mat inliers3dG;
        std::vector<int> inliers;
        PointCloud<pcl::PointXYZ> pc1;
        PointCloud<pcl::PointXYZ> pc2;
        PointCloud<pcl::PointXYZ> pc_12;
        PointCloud<pcl::PointXYZ> pc_21;
        vector<Point3f> pts1;
        vector<Point3f> pts2;
        switch(method) {
          case '1' :
            myVO_SR4k.pose3d3d_SVD(p_XYZs2, p_XYZs1, mat_r, vec_t, &T );
            myBase_SR4k.rotMtoRPY(mat_r, roll, pitch, yaw);
            cout << "roll " << roll<<" pitch " << pitch<<" yaw " << yaw<<endl;
            rpE =  myBase_SR4k.reprojectionError( p_XYZs1, p_XYZs2, mat_r, vec_t );
            cout << "pose3d3d_SVD error " <<endl<< 1000*rpE<< " mm"<<endl<<endl;
            break;
          case '2' :
            myVO_SR4k.pose3d3d_BApose( p_XYZs2, p_XYZs1, mat_r, vec_t, T );
            myBase_SR4k.rotMtoRPY(T, roll, pitch, yaw);
            cout << "roll " << roll<<" pitch " << pitch<<" yaw " << yaw<<endl;
            rpE =  myBase_SR4k.reprojectionError( p_XYZs1, p_XYZs2, T );
            cout << "pose3d3d_BApose error " <<endl<< 1000*rpE<< " mm"<<endl<<endl;
             break;
          case '3' :
            myVO_SR4k.pose3d3d_dirctSVD( p_XYZs2, p_XYZs1, T );
            myBase_SR4k.rotMtoRPY(T, roll, pitch, yaw);
            cout << "roll " << roll<<" pitch " << pitch<<" yaw " << yaw<<endl; 
            rpE =  myBase_SR4k.reprojectionError( p_XYZs1, p_XYZs2, T );
            cout << "pose3d3d_dirctSVD error " <<endl<< 1000*rpE<< " mm"<<endl<<endl;
            break;
          case '4' :
            myVO_SR4k.RANSACpose3d3d_SVD(p_XYZs2, p_XYZs1, mat_r, vec_t, inliers,&T );
            myBase_SR4k.rotMtoRPY(mat_r, roll, pitch, yaw);
            cout << "roll " << roll<<" pitch " << pitch<<" yaw " << yaw<<endl;
            rpE =  myBase_SR4k.reprojectionError( p_XYZs1, p_XYZs2, T);
            cout << "RANSACpose3d3d_SVD error " <<endl<< 1000*rpE<< " mm"<<endl<<endl;
            break;
          case '5' :
            // cv::estimateAffine3D(p_XYZs2,p_XYZs1, outM3by4G, inliers3dG, 0.5, 0.8); 
            myVO_SR4k.pose3d2d_BA(p_XYZs2, p_UVs1, mat_r, vec_t, T, cameraMatrix_4k );
            myBase_SR4k.rotMtoRPY(outM3by4G, roll, pitch, yaw);
            cout << "roll " << roll<<" pitch " << pitch<<" yaw " << yaw<<endl;
            break;
          // case '5' :
          //   cv::Mat outM3by4G;// = cv::Mat::zeros(3,4,CV_64F);            
          //   cv::Mat inliers3dG;
          //   cv::estimateAffine3D(p_XYZs2,p_XYZs1, outM3by4G, inliers3dG, 3, 0.999); 
          //   myBase_SR4k.rotMtoRPY(outM3by4G, roll, pitch, yaw);
          //   cout << "roll " << roll<<" pitch " << pitch<<" yaw " << yaw<<endl;
          //   break;
          default :
            myVO_SR4k.RANSACpose3d3d_SVD(p_XYZs2, p_XYZs1, mat_r, vec_t, inliers,&T );
            myBase_SR4k.rotMtoRPY(mat_r, roll, pitch, yaw);
            cout << "roll " << roll<<" pitch " << pitch<<" yaw " << yaw<<endl;
            rpE =  myBase_SR4k.reprojectionError( p_XYZs1, p_XYZs2, T);
            cout << "RANSACpose3d3d_SVD error " <<endl<< 1000*rpE<< " mm"<<endl<<endl;
        }

        mroll.at<double>(idx-1) = roll;
        mpitch.at<double>(idx-1) = pitch ;
        myaw.at<double>(idx-1) = yaw;

        double x_mm = 1000*T.at<double>(0,3);
        double y_mm = 1000*T.at<double>(1,3);
        double z_mm = 1000*T.at<double>(2,3);

        mx.at<double>(idx-1) = x_mm ;
        my.at<double>(idx-1) = y_mm ;
        mz.at<double>(idx-1) = z_mm ;

        // pts1 = myBase_SR4k.imagToCVpt( f1_4k.depthXYZ, C );
        // pts2 = myBase_SR4k.imagToCVpt( f2_4k.depthXYZ, C );

        // pc1 = myBase_SR4k.cvPtsToPCL(pts1);
        // pc2 = myBase_SR4k.cvPtsToPCL(pts2);
        // pcl::io::savePCDFile( "./pc1.pcd", pc1 );
        // pcl::io::savePCDFile( "./pc2.pcd", pc2 );

        // Eigen::Isometry3d T_eigen = myBase_SR4k.cvTtoEigenT(T);
        // pcl::transformPointCloud( pc1, pc_12, T_eigen.matrix() );
        // pc_12 = pc_12+pc2;
        // pcl::io::savePCDFile( "./pc_12.pcd", pc_12 );


        // pcl::visualization::CloudViewer viewer( "viewer" );
        // viewer.showCloud( pc_12 );
        // while( !viewer.wasStopped() )
        // {
            
        // }
    }

    cv::Scalar mean,stddev;

    cv::meanStdDev(mroll,mean,stddev);
    cout <<"roll  mean: "<<mean.val[0]<<"  std: "<<stddev.val[0]<<endl;
    cv::meanStdDev(mpitch,mean,stddev);
    cout <<"pitch mean: "<<mean.val[0]<<"  std: "<<stddev.val[0]<<endl;
    cv::meanStdDev(myaw,mean,stddev);
    cout <<"Yaw   mean: "<<mean.val[0]<<"  std: "<<stddev.val[0]<<endl;

    cv::meanStdDev(mx,mean,stddev);
    cout <<"mx  mean: "<<mean.val[0]<<"  std: "<<stddev.val[0]<<endl;
    cv::meanStdDev(my,mean,stddev);
    cout <<"my mean: "<<mean.val[0]<<"  std: "<<stddev.val[0]<<endl;
    cv::meanStdDev(mz,mean,stddev);
    cout <<"mz   mean: "<<mean.val[0]<<"  std: "<<stddev.val[0]<<endl;


    return 0;
}