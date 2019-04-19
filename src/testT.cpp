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


    int TestSize = 2;
    for (int idx=1;idx<TestSize;idx++){

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
        for (int i =0;i<p_XYZs1.size();i++){
            cout << p_XYZs1[i]<<"  "<<p_XYZs2[i] <<"    "<< p_UVs1[i]<<"  "<<p_UVs2[i] <<endl;
        }
        // double gt_data[4][4] = {
        //     {1.0000,         0,         0,   0.00},
        //     {0.0000,    0.9945,        -0.1045,   0.00},
        //     {0.0000,   0.1045,    0.9945,    0.00},
        //     {0,         0,         0,    1.0000}
        // };  //yaw  
        double gt_data[4][4] = {
            {0.9945,         0,         0.1045,   0.00},
            {0.0000,    1,        0,   0.00},
            {-0.1045,   0,    0.9945,    0.00},
            {0,         0,         0,    1.0000}
        };  // pitch

        // double gt_data[4][4] = {
        //     {0.9945,         -0.1045,         0.0,   0.00},
        //     {0.1045,    0.9945,        0,   0.00},
        //     {0,   0,    1,    0.00},
        //     {0,         0,         0,    1.0000}
        // }; //roll

        cv::Mat Tgt( 4, 4, CV_64F, gt_data );

        int myFakedata = 1;
        if (myFakedata){
            p_XYZs2.clear();
            for (int i =0;i<p_XYZs1.size();i++){
                cv::Mat ptMat = (cv::Mat_<double>(4, 1) << p_XYZs1[ i ].x, p_XYZs1[ i ].y, p_XYZs1[ i ].z, 1);
                cv::Mat dstMat = Tgt*ptMat;
                cv::Point3f projPd1(dstMat.at<double>(0,0), dstMat.at<double>(1,0),dstMat.at<double>(2,0));
                // cout << p_XYZs1[ i ]<<endl;
                // cout << projPd1<<endl;
                // cout << "=============================="<<endl;
                p_XYZs2.push_back( projPd1 );
            }  
            cout << "3d-3d" <<endl;
            for (int i =0;i<p_XYZs1.size();i++){
                cout << p_XYZs1[i]<<"  "<<p_XYZs2[i] <<endl;
            }
        }


 
        cout << "======================================"<<endl;

        // myVO_SR4k.pose3d3d_dirctSVD(p_XYZs1, p_XYZs2, T);
        // myVO_SR4k.pose3d3d_SVD(p_XYZs1, p_XYZs2, mat_r, vec_t, &T );
        // cout << "p_XYZs1.size() "<<p_XYZs1.size()  << endl;
        
        
        // myVO_SR4k.pose3d3d_BA( p_XYZs1, p_XYZs2, mat_r, vec_t, T );
        // myVO_SR4k.pose3d3d_BApose( p_XYZs1, p_XYZs2, mat_r, vec_t, T );
        // rpE =  myBase.reprojectionError( p_XYZs1, p_XYZs2, mat_r, vec_t );

        // myVO_SR4k.pose3d2d_PnP( p_XYZs1, p_UVs2, mat_r, vec_t, cameraMatrix_4k, &T);
        // myVO_SR4k.pose3d2d_BA( p_XYZs1, p_UVs2, mat_r, vec_t, T, cameraMatrix  );
 

        // double gt_data[4][4] = {
        //     {1.0000,         0,         0,   0.00},
        //     {0.0000,    0.9945,        -0.1045,   0.00},
        //     {0.0000,   0.1045,    0.9945,    0.00},
        //     {0,         0,         0,    1.0000}
        // };
        // double gt_data[4][4] = {
        //     {0.9945,         0,         0.1045,   0.00},
        //     {0.0000,    1,        0,   0.00},
        //     {-0.1045,   0,    0.9945,    0.00},
        //     {0,         0,         0,    1.0000}
        // };

        myBase_SR4k.rotMtoRPY(Tgt, roll, pitch, yaw);
        cout << "roll " << roll<<" pitch " << pitch<<" yaw " << yaw<<endl;
        double baseE =  myBase_SR4k.reprojectionError( p_XYZs1, p_XYZs2, Tgt);
        cout << "groundTruth error " << 1000*baseE<< " mm"<<endl<<endl;

        myVO_SR4k.pose3d3d_SVD(p_XYZs1, p_XYZs2, mat_r, vec_t, &T );
        myBase_SR4k.rotMtoRPY(mat_r, roll, pitch, yaw);
        cout << "roll " << roll<<" pitch " << pitch<<" yaw " << yaw<<endl;
        rpE =  myBase_SR4k.reprojectionError( p_XYZs1, p_XYZs2, mat_r, vec_t );
        cout << "pose3d3d_SVD error " <<endl<< 1000*rpE<< " mm"<<endl<<endl;

        // if (rpE>2*baseE){
        //     cout << "reprojection error " <<endl<< 1000*rpE<< " mm"<<endl<<endl;
        // }
        
        myVO_SR4k.pose3d3d_BA( p_XYZs1, p_XYZs2, mat_r, vec_t, T );
        myBase_SR4k.rotMtoRPY(mat_r, roll, pitch, yaw);
        cout << "roll " << roll<<" pitch " << pitch<<" yaw " << yaw<<endl;
        rpE =  myBase_SR4k.reprojectionError( p_XYZs1, p_XYZs2, mat_r, vec_t );
        cout << "pose3d3d_BA error " <<endl<< 1000*rpE<< " mm"<<endl<<endl;


        myVO_SR4k.pose3d3d_dirctSVD( p_XYZs1, p_XYZs2, T );
        myBase_SR4k.rotMtoRPY(T, roll, pitch, yaw);
        cout << "roll " << roll<<" pitch " << pitch<<" yaw " << yaw<<endl; 
        rpE =  myBase_SR4k.reprojectionError( p_XYZs1, p_XYZs2, T );
        cout << "pose3d3d_dirctSVD error " <<endl<< 1000*rpE<< " mm"<<endl<<endl;



        myVO_SR4k.RANSACpose3d3d_SVD(p_XYZs1, p_XYZs1, mat_r, vec_t, &T );
        myBase_SR4k.rotMtoRPY(mat_r, roll, pitch, yaw);
        cout << "roll " << roll<<" pitch " << pitch<<" yaw " << yaw<<endl;
        rpE =  myBase_SR4k.reprojectionError( p_XYZs1, p_XYZs2, T );
        cout << "RANSACpose3d3d_SVD error " <<endl<< 1000*rpE<< " mm"<<endl<<endl;


        // if (yaw <0){
        //     yaw = -yaw;
        //     pitch = -pitch;
        //     roll = -roll;
        // } 
        


    }


    // int dataSize = 5;
    // cv::Mat mroll(cv::Size(dataSize,dataSize),CV_64FC1, Scalar(0));
    // cv::Mat mpitch(cv::Size(dataSize,dataSize),CV_64FC1, Scalar(0));
    // cv::Mat myaw(cv::Size(dataSize,dataSize),CV_64FC1, Scalar(0));
    // cv::Mat mx(cv::Size(dataSize,dataSize),CV_64FC1, Scalar(0));
    // cv::Mat my(cv::Size(dataSize,dataSize),CV_64FC1, Scalar(0));
    // cv::Mat mz(cv::Size(dataSize,dataSize),CV_64FC1, Scalar(0));


    // ofstream myfile;
    // myfile.open ("/Users/lingqiujin/Desktop/data.txt");
    
    // string firstFrgb;
    // string firstFdepth;
    // string secondFrgb;
    // string secondFdepth;

    // int gN1 = 0;
    // int gN2 = 0;
    // for (int g1=0;g1<dataSize;g1++){
    //     for(int g2=0;g2<dataSize;g2++){
    //         gN1 = g1+1;
    //         gN2 = g2+1;
    //         // gN2  = 100;
    //         // firstFrgb = "/Users/lingqiujin/Data/RV_Data/Pitch/40/color/"+to_string(gN1)+".png";
    //         // firstFdepth = "/Users/lingqiujin/Data/RV_Data/Pitch/40/depth/"+to_string(gN1)+".png";

    //         // secondFrgb = "/Users/lingqiujin/Data/RV_Data/Pitch/37/color/"+to_string(gN2)+".png";
    //         // secondFdepth = "/Users/lingqiujin/Data/RV_Data/Pitch/37/depth/"+to_string(gN2)+".png";   

    //         firstFrgb = "/Users/lingqiujin/Data/RV_Data/Translation/Y1_pic/color/"+to_string(gN1)+".png";
    //         firstFdepth = "/Users/lingqiujin/Data/RV_Data/Translation/Y1_pic/depth/"+to_string(gN1)+".png";

    //         secondFrgb = "/Users/lingqiujin/Data/RV_Data/Translation/Y2_pic/color/"+to_string(gN2)+".png";
    //         secondFdepth = "/Users/lingqiujin/Data/RV_Data/Translation/Y2_pic/depth/"+to_string(gN2)+".png";   
    //         // if (gN1<10){
    //         //     firstF = "/Users/lingqiujin/Data/RV_Data/Pitch/d1_-40/d1_000"+to_string(gN1)+".dat";
    //         // }else{
    //         //     firstF = "/Users/lingqiujin/Data/RV_Data/Pitch/d1_-40/d1_00"+to_string(gN1)+".dat";
    //         // }
            
    //         // if (gN2<10){
    //         //     secondF = "/Users/lingqiujin/Data/RV_Data/Pitch/d2_-37/d2_000"+to_string(gN2)+".dat";
    //         // }else{
    //         //     secondF = "/Users/lingqiujin/Data/RV_Data/Pitch/d2_-37/d2_00"+to_string(gN2)+".dat";
    //         // }
    //         cout << endl<<"load "<<firstF<<endl;
    //         cout << "load "<<secondF<<endl;
    //         cout << endl<<"load "<<firstFrgb<<endl;
    //         cout << "load "<<secondFrgb<<endl;
    //         pose_estimation myVO_SR4k; 
    //         slamBase myBase_SR4k; 

    //         myBase_SR4k.setCamera(C_sr4k);

    //         C = myBase_SR4k.getCamera();
    //         double camera_matrix_data_4k[3][3] = {
    //             {C.fx, 0, C.cx},
    //             {0, C.fy, C.cy},
    //             {0, 0, 1}
    //         };
    //         cv::Mat cameraMatrix_4k( 3, 3, CV_64F, camera_matrix_data_4k );

    //         // SR4kFRAME f1_4k = myBase.readSRFrame(firstF);
    //         // SR4kFRAME f2_4k = myBase.readSRFrame(secondF);
            
    //         // myBase_SR4k.find4kMatches(f1_4k.rgb,f2_4k.rgb,f1_4k.depthXYZ,f2_4k.depthXYZ,p_UVs1,p_UVs2,p_XYZs1,p_XYZs2);
    //         Mat rgb1 = imread ( firstFrgb , CV_LOAD_IMAGE_COLOR );
    //         Mat rgb2 = imread ( secondFrgb , CV_LOAD_IMAGE_COLOR );
    //         Mat depth1 = imread ( firstFdepth, CV_LOAD_IMAGE_UNCHANGED ); 
    //         Mat depth2 = imread ( secondFdepth, CV_LOAD_IMAGE_UNCHANGED ); 

    //         myBase_SR4k.findMatches(rgb1,rgb2,depth1,depth2,p_UVs1,p_UVs2,p_XYZs1,p_XYZs2);
    //         // myVO_SR4k.pose3d3d_dirctSVD(p_XYZs1, p_XYZs2, T);
    //         // myVO_SR4k.pose3d3d_SVD(p_XYZs1, p_XYZs2, mat_r, vec_t, &T );
    //         // myVO_SR4k.pose3d3d_BA( p_XYZs1, p_XYZs2, mat_r, vec_t, T );
    //         // myVO_SR4k.pose3d3d_BApose( p_XYZs1, p_XYZs2, mat_r, vec_t, T );
    //         // rpE =  myBase.reprojectionError( p_XYZs1, p_XYZs2, mat_r, vec_t );

    //         myVO_SR4k.pose3d2d_PnP( p_XYZs1, p_UVs2, mat_r, vec_t, cameraMatrix_4k, &T);
    //         // myVO_SR4k.pose3d2d_BA( p_XYZs1, p_UVs2, mat_r, vec_t, T, cameraMatrix  );
    //         double gt_data[4][4] = {
    //             {1.0000,         0,         0,   0.00},
    //             {0.0000,    1.0000,         0,   0.20},
    //             {0.0000,   0.0000,    1.0000,    0.00},
    //             {0,         0,         0,    1.0000}
    //         };
    //         cv::Mat Tgt( 4, 4, CV_64F, gt_data );
    //         double baseE =  myBase_SR4k.reprojectionError( p_XYZs1, p_XYZs2, Tgt);
    //         rpE =  myBase_SR4k.reprojectionError( p_XYZs1, p_XYZs2, T);
    //         // if (rpE>2*baseE){
    //         //     cout << "reprojection error " <<endl<< 1000*rpE<< " mm"<<endl<<endl;
    //         // }
            
    //         myBase_SR4k.rotMtoRPY(T, roll, pitch, yaw);

    //         if (yaw <0){
    //             yaw = -yaw;
    //             pitch = -pitch;
    //             roll = -roll;
    //         } 
    //         cout << "roll " << roll<<" pitch " << pitch<<" yaw " << yaw<<endl;



    //         // if (yaw <2.5)
    //         //     continue;
    //         mroll.at<double>(g1,g2) = roll;
    //         mpitch.at<double>(g1,g2) = pitch ;
    //         myaw.at<double>(g1,g2) = yaw;

    //         double x_mm = 1000*T.at<double>(0,3);
    //         double y_mm = 1000*T.at<double>(1,3);
    //         double z_mm = 1000*T.at<double>(2,3);

    //         mx.at<double>(g1,g2) = x_mm ;
    //         my.at<double>(g1,g2) = y_mm ;
    //         mz.at<double>(g1,g2) = z_mm ;
            
    //         stringstream ss;
    //         ss << roll<<","<<pitch<<","<<yaw<<","<< x_mm << ","<< y_mm<< "," << z_mm;

    //         string rpy = ss.str();
    //         myfile << rpy <<"\n";
    //     }
    // }

    // myfile.close();


    // cv::Scalar mean,stddev;

    // cv::meanStdDev(mroll,mean,stddev);
    // cout <<"roll  mean: "<<mean.val[0]<<"  std: "<<stddev.val[0]<<endl;
    // cv::meanStdDev(mpitch,mean,stddev);
    // cout <<"pitch mean: "<<mean.val[0]<<"  std: "<<stddev.val[0]<<endl;
    // cv::meanStdDev(myaw,mean,stddev);
    // cout <<"Yaw   mean: "<<mean.val[0]<<"  std: "<<stddev.val[0]<<endl;

    // cv::meanStdDev(mx,mean,stddev);
    // cout <<"mx  mean: "<<mean.val[0]<<"  std: "<<stddev.val[0]<<endl;
    // cv::meanStdDev(my,mean,stddev);
    // cout <<"my mean: "<<mean.val[0]<<"  std: "<<stddev.val[0]<<endl;
    // cv::meanStdDev(mz,mean,stddev);
    // cout <<"mz   mean: "<<mean.val[0]<<"  std: "<<stddev.val[0]<<endl;

    return 0;
}