#include <algorithm>
#include <iostream>
#include <fstream>
#include <sstream>

#include "pose_estimation.h"
#include "slamBase.h"

#include <stdlib.h>    

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
    CAMERA_INTRINSIC_PARAMETERS C_sr4k;

    C_sr4k.cx = 88.5320;
    C_sr4k.cy = 72.5102;
    C_sr4k.fx = 222.6132;
    C_sr4k.fy = 225.6439;
    C_sr4k.scale = 1000;
    C_sr4k.depthL = 0.180;
    C_sr4k.depthH = 7.000;

    // string firstF = "/Users/lingqiujin/Data/RV_Data/Translation/Y1/frm_0002.dat";
    // string secondF = "/Users/lingqiujin/Data/RV_Data/Translation/Y1/frm_0004.dat";

    string firstF = "/Users/lingqiujin/Data/RV_Data/Pitch/d1_-40/d1_0021.dat";
    string secondF = "/Users/lingqiujin/Data/RV_Data/Pitch/d2_-37/d2_0002.dat";

    // Mat rgb1 = imread ( "/Users/lingqiujin/Data/rgbd_dataset_freiburg1_xyz/rgb/1305031102.475318.png", CV_LOAD_IMAGE_COLOR );
    // Mat rgb2 = imread ( "/Users/lingqiujin/Data/rgbd_dataset_freiburg1_xyz/rgb/1305031102.911185.png", CV_LOAD_IMAGE_COLOR );

    // Mat depth1 = imread ( "/Users/lingqiujin/Data/rgbd_dataset_freiburg1_xyz/depth/1305031102.462395.png", CV_LOAD_IMAGE_UNCHANGED ); 
    // Mat depth2 = imread ( "/Users/lingqiujin/Data/rgbd_dataset_freiburg1_xyz/depth/1305031102.926851.png", CV_LOAD_IMAGE_UNCHANGED );


    Mat rgb1 = imread ( "/Users/lingqiujin/Data/RV_Data/Pitch/37/color/100.png", CV_LOAD_IMAGE_COLOR );
    Mat depth1 = imread ( "/Users/lingqiujin/Data/RV_Data/Pitch/37/depth/100.png", CV_LOAD_IMAGE_UNCHANGED ); 

    Mat rgb2 = imread ( "/Users/lingqiujin/Data/RV_Data/Pitch/37/color/100.png", CV_LOAD_IMAGE_COLOR );
    Mat depth2 = imread ( "/Users/lingqiujin/Data/RV_Data/Pitch/37/depth/100.png", CV_LOAD_IMAGE_UNCHANGED );
    // Mat rgb2 = imread ( "/Users/lingqiujin/Data/RV_Data/Pitch/40/color/100.png", CV_LOAD_IMAGE_COLOR );
    // Mat depth2 = imread ( "/Users/lingqiujin/Data/RV_Data/Pitch/40/depth/100.png", CV_LOAD_IMAGE_UNCHANGED );


    vector<Point2f> p_UVs1, p_UVs2;
    vector<Point3f> p_XYZs1, p_XYZs2;


    Mat mat_r, vec_t;
    cv::Mat T = cv::Mat::eye(4,4,CV_64F);
    double rpE = 0;
    float roll,pitch,yaw;
    float sy;
    CAMERA_INTRINSIC_PARAMETERS C;

 //    pose_estimation myVO; 
 //    slamBase myBase; 

 //    myBase.setCamera(C_sr4k);


	// SR4kFRAME f1 = myBase.readSRFrame(firstF);
	// SR4kFRAME f2 = myBase.readSRFrame(secondF);
	
	// // myBase.find4kMatches(f1.rgb,f2.rgb,f1.depthXYZ,f2.depthXYZ,p_UVs1,p_UVs2,p_XYZs1,p_XYZs2);

 //    myBase.findMatches(rgb1,rgb2,depth1,depth2,p_UVs1,p_UVs2,p_XYZs1,p_XYZs2);


 //    C = myBase.getCamera();
	// double camera_matrix_data[3][3] = {
	//     {C.fx, 0, C.cx},
	//     {0, C.fy, C.cy},
	//     {0, 0, 1}
	// };
	// cv::Mat cameraMatrix( 3, 3, CV_64F, camera_matrix_data );





    // myVO.pose3d3d_dirctSVD(p_XYZs1, p_XYZs2, T);
    // myBase.rotMtoRPY(T, roll, pitch, yaw);
    // cout << "roll " << roll<<" pitch " << pitch<<" yaw " << yaw<<endl;
    // rpE =  myBase.reprojectionError( p_XYZs1, p_XYZs2, T);
    // cout << "reprojection error pose3d3d_dirctSVD" <<endl<< 1000*rpE<< " mm"<<endl<<endl;

    // myVO.pose3d3d_SVD(p_XYZs1, p_XYZs2, mat_r, vec_t );
    // myBase.rotMtoRPY(mat_r, roll, pitch, yaw);
    // cout << "roll " << roll<<" pitch " << pitch<<" yaw " << yaw<<endl;
    // rpE =  myBase.reprojectionError( p_XYZs1, p_XYZs2, mat_r, vec_t );
    // cout << "reprojection error pose3d3d_SVD" <<endl<< 1000*rpE<< " mm"<<endl<<endl;

    // myVO.RANSACpose3d3d_SVD(p_XYZs1, p_XYZs2, mat_r, vec_t, &T);
    // myBase.rotMtoRPY(T, roll, pitch, yaw);
    // cout << "roll " << roll<<" pitch " << pitch<<" yaw " << yaw<<endl;
    // rpE =  myBase.reprojectionError( p_XYZs1, p_XYZs2, T );
    // cout << "reprojection error RANSACpose3d3d_SVD" <<endl<< 1000*rpE<< " mm"<<endl<<endl;

  
    // myVO.pose3d3d_BA( p_XYZs1, p_XYZs2, mat_r, vec_t, T );
    // myBase.rotMtoRPY(T, roll, pitch, yaw);
    // cout << "roll " << roll<<" pitch " << pitch<<" yaw " << yaw<<endl;
    // rpE =  myBase.reprojectionError( p_XYZs1, p_XYZs2, T);
    // cout << "reprojection error pose3d3d_BA" <<endl<< 1000*rpE<< " mm"<<endl<<endl;


    // myVO.pose3d2d_PnP( p_XYZs1, p_UVs2, mat_r, vec_t, cameraMatrix);
    // myBase.rotMtoRPY(mat_r, roll, pitch, yaw);
    // cout << "roll " << roll<<" pitch " << pitch<<" yaw " << yaw<<endl;
    // rpE =  myBase.reprojectionError( p_XYZs1, p_XYZs2, mat_r, vec_t );
    // cout << "reprojection error pose3d2d_PnP" <<endl<< 1000*rpE<< " mm"<<endl<<endl;

    // myVO.pose3d2d_BA( p_XYZs1, p_UVs2, mat_r, vec_t, T, cameraMatrix  );
    // myBase.rotMtoRPY(T, roll, pitch, yaw);
    // cout << "roll " << roll<<" pitch " << pitch<<" yaw " << yaw<<endl;
    // rpE =  myBase.reprojectionError( p_XYZs1, p_XYZs2, T);
    // cout << "reprojection error pose3d2d_BA" <<endl<< 1000*rpE<< " mm"<<endl<<endl;
    
	// double gt_data[4][4] = {
	//     {1.0000,         0,         0,   0.00},
	//     {0.0000,    1.0000,         0,   0.00},
	//     {0.0000,   0.0000,    1.0000,    0.00},
	//     {0,         0,         0,    1.0000}
	// };

 //    double gt_data[4][4] = {
 //        {0.9945,         0,         0.1045,   0.00},
 //        {0.0000,    1,        0,   0.00},
 //        {-0.1045,   0,    0.9945,    0.00},
 //        {0,         0,         0,    1.0000}
 //    };

	// cv::Mat Tgt( 4, 4, CV_64F, gt_data );

 //    rpE =  myBase.reprojectionError( p_XYZs1, p_XYZs2, Tgt);
 //    cout << "reprojection error groundTruth" <<endl<< 1000*rpE<< " mm"<<endl<<endl;


 //    myVO.pose2d2d_8pts( p_UVs1, p_UVs2, mat_r, vec_t, cameraMatrix );


 //    myBase.rotMtoRPY(mat_r, roll, pitch, yaw);
 //    cout << "roll " << roll<<" pitch " << pitch<<" yaw " << yaw<<endl;

    // rpE =  myBase.reprojectionError( p_XYZs1, p_XYZs2, mat_r, vec_t );
    // cout << "reprojection error pose3d2d_PnP" <<endl<< 1000*rpE<< " mm"<<endl<<endl;
    // myVO.pose2d2d_triangulation( p_UVs1, p_UVs2, mat_r, vec_t, cameraMatrix );

    int dataSize = 5;
    cv::Mat mroll(cv::Size(dataSize,1),CV_64FC1, Scalar(0));
    cv::Mat mpitch(cv::Size(dataSize,1),CV_64FC1, Scalar(0));
    cv::Mat myaw(cv::Size(dataSize,1),CV_64FC1, Scalar(0));
    cv::Mat mx(cv::Size(dataSize,1),CV_64FC1, Scalar(0));
    cv::Mat my(cv::Size(dataSize,1),CV_64FC1, Scalar(0));
    cv::Mat mz(cv::Size(dataSize,1),CV_64FC1, Scalar(0));

    for (int idx=1;idx<dataSize+1;idx++){

        if (idx<10){
            firstF = "/Users/lingqiujin/Data/RV_Data1_Test/d7_-22/d7_000"+to_string(idx)+".dat";
        }else{
            firstF = "/Users/lingqiujin/Data/RV_Data1_Test/d7_-22/d7_00"+to_string(idx)+".dat";
        }
        
        if (idx<10){
            secondF = "/Users/lingqiujin/Data/RV_Data1_Test/d5_-28/d5_000"+to_string(idx)+".dat";
        }else{
            secondF = "/Users/lingqiujin/Data/RV_Data1_Test/d5_-28/d5_00"+to_string(idx)+".dat";
        }
        // firstF = "/Users/lingqiujin/Data/RV_Data1_Test/d7_-22/d7_0099.dat";
        // secondF = "/Users/lingqiujin/Data/RV_Data1_Test/d7_-22/d7_0009.dat";
        // if (idx<10){
        //     firstF = "/Users/lingqiujin/Data/RV_Data/Translation/Y1/frm_000"+to_string(idx)+".dat";
        // }else{
        //     firstF = "/Users/lingqiujin/Data/RV_Data/Translation/Y1/frm_00"+to_string(idx)+".dat";
        // }
        
        // if (idx<10){
        //     secondF = "/Users/lingqiujin/Data/RV_Data/Translation/Y3/frm_000"+to_string(idx)+".dat";
        // }else{
        //     secondF = "/Users/lingqiujin/Data/RV_Data/Translation/Y3/frm_00"+to_string(idx)+".dat";
        // }

        // if (idx<10){
        //     firstF = "/Users/lingqiujin/Data/RV_Data/Pitch/d1_-40/d1_000"+to_string(idx)+".dat";
        // }else{
        //     firstF = "/Users/lingqiujin/Data/RV_Data/Pitch/d1_-40/d1_00"+to_string(idx)+".dat";
        // }
        
        // if (idx<10){
        //     secondF = "/Users/lingqiujin/Data/RV_Data/Pitch/d2_-37/d2_000"+to_string(idx)+".dat";
        // }else{
        //     secondF = "/Users/lingqiujin/Data/RV_Data/Pitch/d2_-37/d2_00"+to_string(idx)+".dat";
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

        cout << "3d-3d =========================================" <<endl;
        // for (int i =0;i<p_XYZs1.size();i++){
        //     cout << p_XYZs1[i]<<"  "<<p_XYZs2[i] <<"    "<< p_UVs1[i]<<"  "<<p_UVs2[i] <<endl;
        // }

        float myRoll = 0*3.1415926/180;
        float myPitch = 6*3.1415926/180;
        float myYaw = 0*3.1415926/180;
        Mat myR = myBase_SR4k.eulerAnglesToRotationMatrix(myRoll, myPitch, myYaw);
        Mat Tgt = cv::Mat::eye(4,4,CV_64F);
        myR.copyTo(Tgt(cv::Rect(0, 0, 3, 3)));

        int myFakedata = 1;
        int randomNoise = 1;
        if (myFakedata){
            p_XYZs2.clear();
            for (int i =0;i<p_XYZs1.size();i++){
                cv::Mat ptMat = (cv::Mat_<double>(4, 1) << p_XYZs1[ i ].x, p_XYZs1[ i ].y, p_XYZs1[ i ].z, 1);
                cv::Mat dstMat = Tgt*ptMat;
                cv::Point3f projPd1(dstMat.at<double>(0,0), dstMat.at<double>(1,0),dstMat.at<double>(2,0));
                // cout << p_XYZs1[ i ]<<endl;
                // cout << projPd1<<endl;
                // cout << "=============================="<<endl;
                if (randomNoise){
                    projPd1.x = projPd1.x+ (rand() % 10 -4.5)/200 ;
                    projPd1.y = projPd1.y+ (rand() % 10 -4.5)/200 ;
                    projPd1.z = projPd1.z+ (rand() % 10 -4.5)/200 ;               
                }
                p_XYZs2.push_back( projPd1 );
            }  
            // cout << "3d-3d" <<endl;
            // for (int i =0;i<p_XYZs1.size();i++){
            //     cout << p_XYZs1[i]<<"  "<<p_XYZs2[i] <<endl;
            // }
        }


 
        cout << "======================================"<<endl;

        myBase_SR4k.rotMtoRPY(Tgt, roll, pitch, yaw);
        cout << "roll " << roll<<" pitch " << pitch<<" yaw " << yaw<<endl;
        double baseE =  myBase_SR4k.reprojectionError( p_XYZs1, p_XYZs2, Tgt);
        cout << "groundTruth error " << 1000*baseE<< " mm"<<endl<<endl;

        int method = 3;
        switch(method) {
          case 1 :
            myVO_SR4k.pose3d3d_SVD(p_XYZs2, p_XYZs1, mat_r, vec_t, &T );
            myBase_SR4k.rotMtoRPY(mat_r, roll, pitch, yaw);
            cout << "roll " << roll<<" pitch " << pitch<<" yaw " << yaw<<endl;
            rpE =  myBase_SR4k.reprojectionError( p_XYZs1, p_XYZs2, mat_r, vec_t );
            cout << "pose3d3d_SVD error " <<endl<< 1000*rpE<< " mm"<<endl<<endl;
            break;
          case 2 :
            myVO_SR4k.pose3d3d_SVD(p_XYZs2, p_XYZs1, mat_r, vec_t, &T );
            myBase_SR4k.rotMtoRPY(mat_r, roll, pitch, yaw);
            cout << "roll " << roll<<" pitch " << pitch<<" yaw " << yaw<<endl;
            rpE =  myBase_SR4k.reprojectionError( p_XYZs1, p_XYZs2, mat_r, vec_t );
            cout << "pose3d3d_SVD error " <<endl<< 1000*rpE<< " mm"<<endl<<endl;
            break;
          case 3 :
            myVO_SR4k.pose3d3d_BApose( p_XYZs2, p_XYZs1, mat_r, vec_t, T );
            myBase_SR4k.rotMtoRPY(T, roll, pitch, yaw);
            cout << "roll " << roll<<" pitch " << pitch<<" yaw " << yaw<<endl;
            rpE =  myBase_SR4k.reprojectionError( p_XYZs1, p_XYZs2, T );
            cout << "pose3d3d_BApose error " <<endl<< 1000*rpE<< " mm"<<endl<<endl;
             break;
          case 4 :
            myVO_SR4k.pose3d3d_dirctSVD( p_XYZs2, p_XYZs1, T );
            myBase_SR4k.rotMtoRPY(T, roll, pitch, yaw);
            cout << "roll " << roll<<" pitch " << pitch<<" yaw " << yaw<<endl; 
            rpE =  myBase_SR4k.reprojectionError( p_XYZs1, p_XYZs2, T );
            cout << "pose3d3d_dirctSVD error " <<endl<< 1000*rpE<< " mm"<<endl<<endl;
            break;
          case 5 :
            myVO_SR4k.RANSACpose3d3d_SVD(p_XYZs1, p_XYZs2, mat_r, vec_t, &T );
            myBase_SR4k.rotMtoRPY(T, roll, pitch, yaw);
            cout << "roll " << roll<<" pitch " << pitch<<" yaw " << yaw<<endl;
            rpE =  myBase_SR4k.reprojectionError( p_XYZs1, p_XYZs2, T);
            cout << "RANSACpose3d3d_SVD error " <<endl<< 1000*rpE<< " mm"<<endl<<endl;
          default :
             cout << "Invalid grade" << endl;
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