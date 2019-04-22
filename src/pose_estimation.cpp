#include "pose_estimation.h" 
#include "myG2Oedge.h"

#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/SVD>

#include <iostream>     // std::cout
#include <algorithm>    // std::random_shuffle
#include <vector>       // std::vector
#include <ctime>        // std::time
#include <cstdlib>      // std::rand, std::srand
#include <cmath>

#include <chrono>

void pose_estimation::pose3d3d_dirctSVD(vector<Point3f> & p_XYZs1,vector<Point3f> & p_XYZs2, Mat& T){

    int N = p_XYZs1.size();
    cv::Mat firstM = cv::Mat::zeros(N,4,CV_64F);
    cv::Mat secondM = cv::Mat::zeros(N,4,CV_64F);
    for (int i=0;i<N;i++){
        firstM.at<double>(i,3) = 1.0;
        secondM.at<double>(i,3) = 1.0;

        firstM.at<double>(i,0) = (p_XYZs1[i]).x;
        secondM.at<double>(i,0) = (p_XYZs2[i]).x;

        firstM.at<double>(i,1) = (p_XYZs1[i]).y;
        secondM.at<double>(i,1) = (p_XYZs2[i]).y;

        firstM.at<double>(i,2) = (p_XYZs1[i]).z;
        secondM.at<double>(i,2) = (p_XYZs2[i]).z;
    }
    cv::Mat Tm;
    cv::solve(secondM,firstM,Tm,DECOMP_SVD);
    cv::transpose(Tm,Tm);
    // cout << "Tm"<<endl<<Tm<<endl;
    T = Tm;
    // R= Tm(cv::Rect(0,0,3,3));
    // tvecN = Tm(cv::Rect(3,0,1,3));

}

void pose_estimation::RANSACpose3d3d_SVD(vector<Point3f>&  p_XYZs1,vector<Point3f>&  p_XYZs2, Mat & mat_r, Mat & vec_t, std::vector<int> & inliers ,Mat* T ){

    // cv::Mat outM3by4G;// = cv::Mat::zeros(3,4,CV_64F);
    // cv::Mat inliers3dG;
    // cv::estimateAffine3D(p_XYZs2,p_XYZs1, outM3by4G, inliers3dG, 3, 0.999); 

    // cout << outM3by4G<<endl;
    // Mat Tm = cv::Mat::eye(4,4,CV_64F);
    // mat_r = outM3by4G(cv::Rect(0,0,3,3));
    // vec_t = outM3by4G(cv::Rect(3,0,1,3));
    // mat_r.copyTo(Tm(cv::Rect(0, 0, 3, 3)));
    // vec_t.copyTo(Tm(cv::Rect(3, 0, 1, 3)));  
    // *T = Tm;

    int iterations = 0;
    int N = p_XYZs1.size();

    // cout << "p_XYZs1.size() "<<p_XYZs1.size()  << endl;
    // std::cout << "myvector contains:";
    // for (int i=0; i<10; i++){
    //     std::cout << ' ' << myvector[i];
    // }
    
    
    int foundT = 0;
    int foundN = 0;
    int maxN = 0;
    int inlierR = 20;
    double errorThreshold = .1;

    double inlierError = 0;
    double minError = 10000;
    int testPtsN = 3;

    Mat Tfound;
    std::vector<int> finliers;

    while (iterations < 3000 ) { //&& foundT==0
        iterations++;
        // cout << "iterations " << iterations<<endl;
        foundN = 0; 
        finliers.clear();
        std::srand ( unsigned ( std::time(0) ) );
        std::vector<int> myvector;

        for (int i=0; i<N; i++) myvector.push_back(i); 

        std::random_shuffle ( myvector.begin(), myvector.end() );
        vector<Point3f> test1,test2,check1,check2;
        // cout << "==================================="<<endl;
        for (int i =0;i<testPtsN;i++){
            // cout << " random pts  "<< myvector[i]<<endl;
            test1.push_back( p_XYZs1[myvector[i]]);
            test2.push_back( p_XYZs2[myvector[i]]);
            finliers.push_back( myvector[i] );
        }
        // for (int i =3;i<N;i++){
        //     check1.push_back( p_XYZs1[myvector[i]]);
        //     check2.push_back( p_XYZs2[myvector[i]]);
        // }
        Mat fmat_r,fvec_t;
        cv::Mat Tfound = cv::Mat::eye(4,4,CV_64F);
        pose3d3d_SVD(test1,test2,fmat_r,fvec_t,&Tfound );
        // T = cv::Mat::eye(4,4,CV_64F);    only translation
        // T.at<double>(0,3) = p_XYZs2[myvector[1]].x -p_XYZs1[myvector[1]].x;
        // T.at<double>(1,3) = p_XYZs2[myvector[1]].y -p_XYZs1[myvector[1]].y;
        // T.at<double>(2,3) = p_XYZs2[myvector[1]].z -p_XYZs1[myvector[1]].z;
        float sy= sqrt(Tfound.at<double>(0,0) * Tfound.at<double>(0,0) +  Tfound.at<double>(1,0) * Tfound.at<double>(1,0) );
        float roll = 180/3.14159265*atan2(Tfound.at<double>(2,1) , Tfound.at<double>(2,2));
        float pitch = 180/3.14159265*atan2(-Tfound.at<double>(2,0), sy);
        float yaw = 180/3.14159265*atan2(Tfound.at<double>(1,0), Tfound.at<double>(0,0));


        inlierError = 0;
        for (int i =testPtsN;i<N;i++){
            cv::Point3f pd2 = p_XYZs1[myvector[i]];
            cv::Point3f pd1 = p_XYZs2[myvector[i]];

            cv::Mat ptMat = (cv::Mat_<double>(4, 1) << pd1.x, pd1.y, pd1.z, 1);
            cv::Mat dstMat = Tfound*ptMat;
            cv::Point3f projPd1(dstMat.at<double>(0,0), dstMat.at<double>(1,0),dstMat.at<double>(2,0));
            // cout << "projPd1 "<<projPd1<<endl;


            if (norm(projPd1-pd2) < errorThreshold ){ //
                foundN++;
                inlierError = norm(projPd1-pd2) +inlierError;
                finliers.push_back( myvector[i] );
                // cout << " norm(projPd1-pd2)  "<< norm(projPd1-pd2)<<endl;
            }
        }


        // if( abs(roll-0)<1 && abs(pitch-6)<1 && abs(yaw-0)<1){
        //     cout << "roll " << roll<<" pitch " << pitch<<" yaw " << yaw<<endl;
        //     cout <<"foundN "<<foundN<< " ======================== seems right "<< inlierError << endl;
        //     // cout << "iterations " << iterations<<endl;
        // }

        if ( foundN > 0){
            foundT =1;
            inlierError = inlierError/foundN;
            // cout << "inlier " << foundN << " in "<< N <<endl;
        }  
        if (foundN>=maxN ) { 
            if (foundN==maxN && inlierError>minError){
                // cout<<"no update"<<endl;
            }
            else
            {
                minError =  inlierError; 
                maxN = max(maxN,foundN);
                minError = min(inlierError,minError);
                *T = Tfound;
                mat_r = Tfound(cv::Rect(0,0,3,3));
                vec_t = Tfound(cv::Rect(3,0,1,3));
                inliers = finliers;
                // cout <<"current maxN =====" << inlierError <<endl;
                // sy= sqrt(fmat_r.at<double>(0,0) * fmat_r.at<double>(0,0) +  fmat_r.at<double>(1,0) * fmat_r.at<double>(1,0) );
                // roll = 180/3.14159265*atan2(fmat_r.at<double>(2,1) , fmat_r.at<double>(2,2));
                // pitch = 180/3.14159265*atan2(-fmat_r.at<double>(2,0), sy);
                // yaw = 180/3.14159265*atan2(fmat_r.at<double>(1,0), fmat_r.at<double>(0,0));
                // cout << "roll " << roll<<" pitch " << pitch<<" yaw " << yaw<<endl<<endl;
            }
            // cout <<"current maxN =============================" <<endl;
            // sy= sqrt(fmat_r.at<double>(0,0) * fmat_r.at<double>(0,0) +  fmat_r.at<double>(1,0) * fmat_r.at<double>(1,0) );
            // roll = 180/3.14159265*atan2(fmat_r.at<double>(2,1) , fmat_r.at<double>(2,2));
            // pitch = 180/3.14159265*atan2(-fmat_r.at<double>(2,0), sy);
            // yaw = 180/3.14159265*atan2(fmat_r.at<double>(1,0), fmat_r.at<double>(0,0));
            // cout << "roll " << roll<<" pitch " << pitch<<" yaw " << yaw<<endl<<endl;
        }
        // cout << "I am here XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX"<<endl;
    }


    if (foundT == 0){
        cout <<"no found========================================="<<endl;
        Mat Tm = cv::Mat::eye(4,4,CV_64F);
        mat_r = Tm(cv::Rect(0,0,3,3));
        vec_t = Tm(cv::Rect(3,0,1,3));
        *T = Tm;
    }
    // cout << "inlier " << maxN + testPtsN<< " in "<< N <<endl;
    cout << "inlier size " << inliers.size() << " in "<< N <<endl;
    cout << "inlier error " << minError <<endl;
    cout <<"======================================================================================================================="<<endl;

}

void pose_estimation::pose3d3d_SVD(vector<Point3f>&  p_XYZs1,vector<Point3f>&  p_XYZs2, Mat & mat_r, Mat & vec_t, Mat* T ){
    Point3f p1, p2;     // center of mass
    int N = p_XYZs1.size();
    for ( int i=0; i<N; i++ )
    {
        p1 += p_XYZs1[i];
        p2 += p_XYZs2[i];
    }
    p1 = Point3f( Vec3f(p1) /  N);
    p2 = Point3f( Vec3f(p2) / N);
    vector<Point3f>     q1 ( N ), q2 ( N ); // remove the center
    for ( int i=0; i<N; i++ )
    {
        q1[i] = p_XYZs1[i] - p1;
        q2[i] = p_XYZs2[i] - p2;
    }

    // compute q1*q2^T
    Eigen::Matrix3d W = Eigen::Matrix3d::Zero();
    for ( int i=0; i<N; i++ )
    {
        W += Eigen::Vector3d ( q1[i].x, q1[i].y, q1[i].z ) * Eigen::Vector3d ( q2[i].x, q2[i].y, q2[i].z ).transpose();
    }
    // cout<<"W="<<W<<endl;

    // SVD on W
    Eigen::JacobiSVD<Eigen::Matrix3d> svd ( W, Eigen::ComputeFullU|Eigen::ComputeFullV );
    Eigen::Matrix3d U = svd.matrixU();
    Eigen::Matrix3d V = svd.matrixV();
    // cout<<"U="<<U<<endl;
    // cout<<"V="<<V<<endl;

    Eigen::Matrix3d R_ = U* ( V.transpose() );

    double detX = R_.determinant();
    
    if (detX < -0.5){
        // cout <<U<<endl;
        U(2,0) = -U(2,0);
        U(2,1) = -U(2,1);
        U(2,2) = -U(2,2);
        // cout <<U<<endl;
        R_ = U* ( V.transpose() );
    }

    Eigen::Vector3d t_ = Eigen::Vector3d ( p1.x, p1.y, p1.z ) - R_ * Eigen::Vector3d ( p2.x, p2.y, p2.z );

    // convert to cv::Mat
    mat_r = ( Mat_<double> ( 3,3 ) <<
          R_ ( 0,0 ), R_ ( 0,1 ), R_ ( 0,2 ),
          R_ ( 1,0 ), R_ ( 1,1 ), R_ ( 1,2 ),
          R_ ( 2,0 ), R_ ( 2,1 ), R_ ( 2,2 )
        );
    vec_t = ( Mat_<double> ( 3,1 ) << t_ ( 0,0 ), t_ ( 1,0 ), t_ ( 2,0 ) );

    if(T != NULL) {
        Mat T_m = cv::Mat::eye(4,4,CV_64F);
        mat_r.copyTo(T_m(cv::Rect(0, 0, 3, 3)));
        vec_t.copyTo(T_m(cv::Rect(3, 0, 1, 3))); 
        *T = T_m;
    }
}

void pose_estimation::pose3d3d_BApose( vector<Point3f> & p_XYZs1, vector<Point3f> & p_XYZs2, Mat & mat_r, Mat & vec_t, Mat & T ){
    g2o::SparseOptimizer optimizer;
    typedef g2o::BlockSolver_6_3 SlamBlockSolver; 
    typedef g2o::LinearSolverEigen< SlamBlockSolver::PoseMatrixType > SlamLinearSolver; 
    std::unique_ptr<SlamLinearSolver> linearSolver ( new SlamLinearSolver());
    linearSolver->setBlockOrdering( false );
    std::unique_ptr<SlamBlockSolver> blockSolver ( new SlamBlockSolver ( std::move(linearSolver)));
    g2o::OptimizationAlgorithmLevenberg* solver = new g2o::OptimizationAlgorithmLevenberg( std::move(blockSolver) );
    optimizer.setAlgorithm( solver );

    // vertex
    g2o::VertexSE3Expmap* pose = new g2o::VertexSE3Expmap(); // camera pose
    Eigen::Matrix3d R_mat;
    R_mat <<
          mat_r.at<double> ( 0,0 ), mat_r.at<double> ( 0,1 ), mat_r.at<double> ( 0,2 ),
               mat_r.at<double> ( 1,0 ), mat_r.at<double> ( 1,1 ), mat_r.at<double> ( 1,2 ),
               mat_r.at<double> ( 2,0 ), mat_r.at<double> ( 2,1 ), mat_r.at<double> ( 2,2 );
    pose->setId ( 0 );

    pose->setEstimate( g2o::SE3Quat(
        Eigen::Matrix3d::Identity(),
        Eigen::Vector3d( 0,0,0 )
    ) );
    // pose->setEstimate ( g2o::SE3Quat (
    //                         R_mat,
    //                         Eigen::Vector3d ( vec_t.at<double> ( 0,0 ), vec_t.at<double> ( 1,0 ), vec_t.at<double> ( 2,0 ) )
    //                     ) );
    optimizer.addVertex ( pose );

    // edges
    int index = 1;

    for ( size_t i=0; i<p_XYZs1.size(); i++ )
    {
        EdgeProjectXYZRGBDPoseOnly* edge = new EdgeProjectXYZRGBDPoseOnly(
            Eigen::Vector3d(p_XYZs2[i].x, p_XYZs2[i].y, p_XYZs2[i].z) );
        pose->setId( index );
        edge->setVertex( 0, dynamic_cast<g2o::VertexSE3Expmap*> (pose) );
        edge->setMeasurement( Eigen::Vector3d(
            p_XYZs1[i].x, p_XYZs1[i].y, p_XYZs1[i].z) );

        Eigen::Matrix3d information = Eigen::Matrix< double, 3,3 >::Identity();

        information(0,0) = 1;
        information(1,1) = 1;
        information(2,2) = 1;

        edge->setInformation( information );

        optimizer.addEdge(edge);
        index++;
    }


    // Eigen::Isometry3d T_prior =  Eigen::Isometry3d::Identity();
    // Mat cvR = cv::Mat::eye(3,3,CV_64F);
    // Eigen::Matrix3d r_eigen;
    // for ( int i=0; i<3; i++ )
    //     for ( int j=0; j<3; j++ ) 
    //         r_eigen(i,j) = cvR.at<double>(i,j);

    // Eigen::AngleAxisd angle(r_eigen);
    // T_prior = angle;
    // T_prior(0,3) = 0.0; 
    // T_prior(1,3) = 200000000; 
    // T_prior(2,3) = 0.0;

    // // cout << T_prior.matrix()<<endl;

    // g2o::EdgeSE3* edge_T = new g2o::EdgeSE3();
    // edge_T->setVertex( 0, dynamic_cast<g2o::VertexSE3Expmap*> (pose) );
    // edge_T->setMeasurement( T_prior );

    // edge_T->vertices() [0] = optimizer.vertex( 1 );
    // edge_T->vertices() [1] = optimizer.vertex( 2 );

    // Eigen::Matrix<double, 6, 6> information_T = Eigen::Matrix< double, 6,6 >::Identity();

    // information_T(0,0) = 100000000;
    // information_T(1,1) = 100000000;
    // information_T(2,2) = 100000000;

    // information_T(3,3) = 100000000;
    // information_T(4,4) = 100000000;
    // information_T(5,5) = 100000000;

    // edge_T->setInformation( information_T );

    // optimizer.addEdge(edge_T);


    chrono::steady_clock::time_point t1 = chrono::steady_clock::now();
    optimizer.setVerbose( false );
    optimizer.initializeOptimization();
    optimizer.save("result_before.g2o");
    optimizer.optimize(10);
    chrono::steady_clock::time_point t2 = chrono::steady_clock::now();
    chrono::duration<double> time_used = chrono::duration_cast<chrono::duration<double>>(t2-t1);
    // cout<<"optimization costs time: "<<time_used.count()<<" seconds."<<endl;

    // cout<<endl<<"after optimization:"<<endl;
    // cout<<"T="<<endl<<Eigen::Isometry3d( pose->estimate() ).matrix()<<endl;

    eigen2cv(Eigen::Isometry3d( pose->estimate() ).matrix(), T);

}

void pose_estimation::pose3d3d_BA( vector<Point3f> & p_XYZs1, vector<Point3f> & p_XYZs2, Mat & mat_r, Mat & vec_t, Mat & T ){
    g2o::SparseOptimizer optimizer;
    typedef g2o::BlockSolver_6_3 SlamBlockSolver; 
    typedef g2o::LinearSolverEigen< SlamBlockSolver::PoseMatrixType > SlamLinearSolver; 
    std::unique_ptr<SlamLinearSolver> linearSolver ( new SlamLinearSolver());
    linearSolver->setBlockOrdering( false );
    std::unique_ptr<SlamBlockSolver> blockSolver ( new SlamBlockSolver ( std::move(linearSolver)));
    g2o::OptimizationAlgorithmLevenberg* solver = new g2o::OptimizationAlgorithmLevenberg( std::move(blockSolver) );
    optimizer.setAlgorithm( solver );

    // vertex
    g2o::VertexSE3Expmap* pose = new g2o::VertexSE3Expmap(); // camera pose
    Eigen::Matrix3d R_mat;
    R_mat <<
          mat_r.at<double> ( 0,0 ), mat_r.at<double> ( 0,1 ), mat_r.at<double> ( 0,2 ),
          mat_r.at<double> ( 1,0 ), mat_r.at<double> ( 1,1 ), mat_r.at<double> ( 1,2 ),
          mat_r.at<double> ( 2,0 ), mat_r.at<double> ( 2,1 ), mat_r.at<double> ( 2,2 );
    Eigen::Vector3d t_vect;
    t_vect << vec_t.at<double> ( 0,0 ), vec_t.at<double> ( 1,0 ), vec_t.at<double> ( 2,0 );

    t_vect << 0, 0.0, 0;
    R_mat = Eigen::Matrix3d::Identity();
    // pose->setEstimate( g2o::SE3Quat(
    //     Eigen::Matrix3d::Identity(),
    //     Eigen::Vector3d( 0,0,0 )
    // ) );
    pose->setId ( 0 );
    pose->setEstimate ( g2o::SE3Quat (
                            R_mat,
                            t_vect
                        ) );
    optimizer.addVertex ( pose );

    // edges
    int pointIndex = 1;                                     
    for( auto &p: p_XYZs2 ){
        auto point = new g2o::VertexSBAPointXYZ();
        point->setId( pointIndex++ );
        point->setMarginalized( true );                    
        point->setEstimate( Eigen::Vector3d( p.x, p.y, p.z ) );
        optimizer.addVertex( point );
    }

    // edges
    int index = 0;
    vector<EdgeProjectXYZRGBDPoseAndPts*> edges;
    for ( size_t i=0; i<p_XYZs1.size(); i++ )
    {
        auto edge = new EdgeProjectXYZRGBDPoseAndPts( );      

        edge->setId( index );
        edge->setVertex( 0 , dynamic_cast< g2o::VertexSBAPointXYZ *> ( optimizer.vertex(index)) );
        edge->setVertex( 1 , dynamic_cast< g2o::OptimizableGraph::Vertex *> ( optimizer.vertex(0) ) );

        edge->setMeasurement( Eigen::Vector3d (
                p_XYZs1[i].x, p_XYZs1[i].y, p_XYZs1[i].z)
                            );
        edge->setInformation( Eigen::Matrix3d::Identity()*1e4 );
        optimizer.addEdge(edge);                           
        index++;
        edges.push_back(edge);                            
    }

    chrono::steady_clock::time_point t1 = chrono::steady_clock::now();
    optimizer.setVerbose( false );
    optimizer.initializeOptimization();
    optimizer.optimize(100);
    chrono::steady_clock::time_point t2 = chrono::steady_clock::now();
    chrono::duration<double> time_used = chrono::duration_cast<chrono::duration<double>>(t2-t1);

    // cout<<"optimization costs time: "<<time_used.count()<<" seconds."<<endl;

    // cout<<endl<<"after optimization:"<<endl;
    // cout<<"T="<<endl<<Eigen::Isometry3d( pose->estimate() ).matrix()<<endl;

    eigen2cv(Eigen::Isometry3d( pose->estimate() ).matrix(), T);

}

void pose_estimation::pose3d2d_PnP( vector<Point3f> & p_XYZs1, vector<Point2f> & p_UVs2, Mat & mat_r, Mat & vec_t,Mat &cameraMatrix, Mat* T ){

    Mat vec_r;
    solvePnP ( p_XYZs1, p_UVs2, cameraMatrix, Mat(), vec_r, vec_t, false ); 
    cv::Rodrigues ( vec_r, mat_r ); 
    if(T != NULL) {
        Mat T_m = cv::Mat::eye(4,4,CV_64F);
        mat_r.copyTo(T_m(cv::Rect(0, 0, 3, 3)));
        vec_t.copyTo(T_m(cv::Rect(3, 0, 1, 3))); 
        *T = T_m;
    }
}


void pose_estimation::pose3d2d_BA( vector<Point3f> & p_XYZs1, vector<Point2f> & p_UVs2, Mat & mat_r, Mat & vec_t,Mat & T ,Mat &cameraMatrix ){

    g2o::SparseOptimizer optimizer;
    typedef g2o::BlockSolver_6_3 SlamBlockSolver; 
    typedef g2o::LinearSolverEigen< SlamBlockSolver::PoseMatrixType > SlamLinearSolver; 
    std::unique_ptr<SlamLinearSolver> linearSolver ( new SlamLinearSolver());
    linearSolver->setBlockOrdering( false );
    std::unique_ptr<SlamBlockSolver> blockSolver ( new SlamBlockSolver ( std::move(linearSolver)));
    g2o::OptimizationAlgorithmLevenberg* solver = new g2o::OptimizationAlgorithmLevenberg( std::move(blockSolver) );
    optimizer.setAlgorithm( solver );

    // vertex
    g2o::VertexSE3Expmap* pose = new g2o::VertexSE3Expmap(); // camera pose
    Eigen::Matrix3d R_mat;
    R_mat <<
          mat_r.at<double> ( 0,0 ), mat_r.at<double> ( 0,1 ), mat_r.at<double> ( 0,2 ),
               mat_r.at<double> ( 1,0 ), mat_r.at<double> ( 1,1 ), mat_r.at<double> ( 1,2 ),
               mat_r.at<double> ( 2,0 ), mat_r.at<double> ( 2,1 ), mat_r.at<double> ( 2,2 );
    pose->setId ( 0 );
    // pose->setEstimate( g2o::SE3Quat(
    //     Eigen::Matrix3d::Identity(),
    //     Eigen::Vector3d( 0,0,0 )
    // ) );

    pose->setEstimate ( g2o::SE3Quat (
                            R_mat,
                            Eigen::Vector3d ( vec_t.at<double> ( 0,0 ), vec_t.at<double> ( 1,0 ), vec_t.at<double> ( 2,0 ) )
                        ) );
    
    optimizer.addVertex ( pose );

    int index = 1;
    for ( const Point3f p:p_XYZs1 )   // landmarks
    {
        g2o::VertexSBAPointXYZ* point = new g2o::VertexSBAPointXYZ();
        point->setId ( index++ );
        point->setEstimate ( Eigen::Vector3d ( p.x, p.y, p.z ) );
        point->setMarginalized ( true ); // g2o 中必须设置 marg 参见第十讲内容
        optimizer.addVertex ( point );
    }

    // parameter: camera intrinsics
    g2o::CameraParameters* camera = new g2o::CameraParameters (
        cameraMatrix.at<double> ( 0,0 ), Eigen::Vector2d ( cameraMatrix.at<double> ( 0,2 ), cameraMatrix.at<double> ( 1,2 ) ), 0
    );
    camera->setId ( 0 );
    optimizer.addParameter ( camera );

    // edges
    index = 1;
    for ( const Point2f p:p_UVs2 )
    {
        g2o::EdgeProjectXYZ2UV* edge = new g2o::EdgeProjectXYZ2UV();
        edge->setId ( index );
        edge->setVertex ( 0, dynamic_cast<g2o::VertexSBAPointXYZ*> ( optimizer.vertex ( index ) ) );
        edge->setVertex ( 1, pose );
        edge->setMeasurement ( Eigen::Vector2d ( p.x, p.y ) );
        edge->setParameterId ( 0,0 );
        edge->setInformation ( Eigen::Matrix2d::Identity() );
        optimizer.addEdge ( edge );
        index++;
    }

    chrono::steady_clock::time_point t1 = chrono::steady_clock::now();
    optimizer.setVerbose ( false );
    optimizer.initializeOptimization();
    optimizer.optimize ( 100 );
    chrono::steady_clock::time_point t2 = chrono::steady_clock::now();
    chrono::duration<double> time_used = chrono::duration_cast<chrono::duration<double>> ( t2-t1 );
    // cout<<"optimization costs time: "<<time_used.count() <<" seconds."<<endl;

    // cout<<endl<<"after optimization:"<<endl;
    // cout<<"T="<<endl<<Eigen::Isometry3d ( pose->estimate() ).matrix() <<endl;

    eigen2cv(Eigen::Isometry3d ( pose->estimate() ).matrix(), T);
    // cout<<"T="<<endl<<T <<endl;
}


void pose_estimation::pose2d2d_8pts( vector<Point2f> & p_UVs1, vector<Point2f> & p_UVs2, Mat & mat_r, Mat & vec_t,Mat &cameraMatrix, Mat* T  ){

    // Mat homography_matrix;
    // homography_matrix = findHomography ( p_UVs1, p_UVs2, RANSAC, 3 );

    // Mat fundamental_matrix;
    // fundamental_matrix = findFundamentalMat ( p_UVs1, p_UVs2, CV_FM_8POINT );
    // cout<<"fundamental_matrix is "<<endl<< fundamental_matrix<<endl;
    // F =K^−T E K^-1

    Point2d principal_point ( cameraMatrix.at<double> ( 0,2 ), cameraMatrix.at<double> ( 1,2 )  );  
    double focal_length = ( cameraMatrix.at<double> ( 0,0 ) + cameraMatrix.at<double> ( 1,1 ) ) /2;


    Mat essential_matrix;
    essential_matrix = findEssentialMat ( p_UVs1, p_UVs2, focal_length, principal_point );
    cout<<"essential_matrix is "<<endl<< essential_matrix<<endl;

    recoverPose ( essential_matrix, p_UVs1, p_UVs2, mat_r, vec_t, focal_length, principal_point );

    cout<<"R is "<<endl<<mat_r<<endl;
    cout<<"t is "<<endl<<vec_t<<endl<<endl;
    if(T != NULL) {
        Mat T_m = cv::Mat::eye(4,4,CV_64F);
        mat_r.copyTo(T_m(cv::Rect(0, 0, 3, 3)));
        vec_t.copyTo(T_m(cv::Rect(3, 0, 1, 3))); 
        *T = T_m;
    }
}
void pose_estimation::pose2d2d_triangulation( vector<Point2f> & p_UVs1, vector<Point2f> & p_UVs2, Mat & mat_r, Mat & vec_t,Mat &cameraMatrix , Mat* T ){}
