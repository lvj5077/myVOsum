#include "pose_estimation.h" 

#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/SVD>
#include <g2o/core/base_vertex.h>
#include <g2o/core/base_unary_edge.h>
#include <g2o/core/block_solver.h>
#include <g2o/core/optimization_algorithm_gauss_newton.h>
#include <g2o/solvers/eigen/linear_solver_eigen.h>
#include <g2o/types/sba/types_six_dof_expmap.h>

#include <g2o/core/sparse_optimizer.h>
#include <g2o/core/block_solver.h>
#include <g2o/core/robust_kernel.h>
#include <g2o/core/robust_kernel_impl.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/solvers/cholmod/linear_solver_cholmod.h>
#include <g2o/types/slam3d/se3quat.h>
#include <g2o/types/sba/types_six_dof_expmap.h>

#include <g2o/types/slam3d/types_slam3d.h>
#include <g2o/core/factory.h>
#include <g2o/core/optimization_algorithm_factory.h>
#include <g2o/core/optimization_algorithm_gauss_newton.h>

#include <g2o/core/robust_kernel_factory.h>
#include <g2o/solvers/eigen/linear_solver_eigen.h>


#include <chrono>


void pose_estimation::pose3d3d_SVD(vector<Point3f>&  p_XYZs1,vector<Point3f>&  p_XYZs2, Mat & mat_r, Mat & vec_t ){
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
    cout<<"W="<<W<<endl;

    // SVD on W
    Eigen::JacobiSVD<Eigen::Matrix3d> svd ( W, Eigen::ComputeFullU|Eigen::ComputeFullV );
    Eigen::Matrix3d U = svd.matrixU();
    Eigen::Matrix3d V = svd.matrixV();
    cout<<"U="<<U<<endl;
    cout<<"V="<<V<<endl;

    Eigen::Matrix3d R_ = U* ( V.transpose() );
    Eigen::Vector3d t_ = Eigen::Vector3d ( p1.x, p1.y, p1.z ) - R_ * Eigen::Vector3d ( p2.x, p2.y, p2.z );

    // convert to cv::Mat
    mat_r = ( Mat_<double> ( 3,3 ) <<
          R_ ( 0,0 ), R_ ( 0,1 ), R_ ( 0,2 ),
          R_ ( 1,0 ), R_ ( 1,1 ), R_ ( 1,2 ),
          R_ ( 2,0 ), R_ ( 2,1 ), R_ ( 2,2 )
        );
    vec_t = ( Mat_<double> ( 3,1 ) << t_ ( 0,0 ), t_ ( 1,0 ), t_ ( 2,0 ) );
}

// g2o edge
class EdgeProjectXYZRGBDPoseOnly : public g2o::BaseUnaryEdge<3, Eigen::Vector3d, g2o::VertexSE3Expmap>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    EdgeProjectXYZRGBDPoseOnly( const Eigen::Vector3d& point ) : _point(point) {}

    virtual void computeError()
    {
        const g2o::VertexSE3Expmap* pose = static_cast<const g2o::VertexSE3Expmap*> ( _vertices[0] );
        // measurement is p, point is p'
        _error = _measurement - pose->estimate().map( _point );
    }

    virtual void linearizeOplus()
    {
        g2o::VertexSE3Expmap* pose = static_cast<g2o::VertexSE3Expmap *>(_vertices[0]);
        g2o::SE3Quat T(pose->estimate());
        Eigen::Vector3d xyz_trans = T.map(_point);
        double x = xyz_trans[0];
        double y = xyz_trans[1];
        double z = xyz_trans[2];

        _jacobianOplusXi(0,0) = 0;
        _jacobianOplusXi(0,1) = -z;
        _jacobianOplusXi(0,2) = y;
        _jacobianOplusXi(0,3) = -1;
        _jacobianOplusXi(0,4) = 0;
        _jacobianOplusXi(0,5) = 0;

        _jacobianOplusXi(1,0) = z;
        _jacobianOplusXi(1,1) = 0;
        _jacobianOplusXi(1,2) = -x;
        _jacobianOplusXi(1,3) = 0;
        _jacobianOplusXi(1,4) = -1;
        _jacobianOplusXi(1,5) = 0;

        _jacobianOplusXi(2,0) = -y;
        _jacobianOplusXi(2,1) = x;
        _jacobianOplusXi(2,2) = 0;
        _jacobianOplusXi(2,3) = 0;
        _jacobianOplusXi(2,4) = 0;
        _jacobianOplusXi(2,5) = -1;
    }

    bool read ( istream& in ) {}
    bool write ( ostream& out ) const {}
protected:
    Eigen::Vector3d _point;
};

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
    // g2o::VertexSE3Expmap* pose = new g2o::VertexSE3Expmap(); // camera pose
    // pose->setId(0);
    // pose->setEstimate( g2o::SE3Quat(
    //     Eigen::Matrix3d::Identity(),
    //     Eigen::Vector3d( 0,0,0 )
    // ) );
    // optimizer.addVertex( pose );
    g2o::VertexSE3Expmap* pose = new g2o::VertexSE3Expmap(); // camera pose
    Eigen::Matrix3d R_mat;
    R_mat <<
          mat_r.at<double> ( 0,0 ), mat_r.at<double> ( 0,1 ), mat_r.at<double> ( 0,2 ),
               mat_r.at<double> ( 1,0 ), mat_r.at<double> ( 1,1 ), mat_r.at<double> ( 1,2 ),
               mat_r.at<double> ( 2,0 ), mat_r.at<double> ( 2,1 ), mat_r.at<double> ( 2,2 );
    pose->setId ( 0 );
    pose->setEstimate ( g2o::SE3Quat (
                            R_mat,
                            Eigen::Vector3d ( vec_t.at<double> ( 0,0 ), vec_t.at<double> ( 1,0 ), vec_t.at<double> ( 2,0 ) )
                        ) );
    optimizer.addVertex ( pose );

    // edges
    int index = 1;
    vector<EdgeProjectXYZRGBDPoseOnly*> edges;
    for ( size_t i=0; i<p_XYZs1.size(); i++ )
    {
        EdgeProjectXYZRGBDPoseOnly* edge = new EdgeProjectXYZRGBDPoseOnly(
            Eigen::Vector3d(p_XYZs2[i].x, p_XYZs2[i].y, p_XYZs2[i].z) );
        edge->setId( index );
        edge->setVertex( 0, dynamic_cast<g2o::VertexSE3Expmap*> (pose) );
        edge->setMeasurement( Eigen::Vector3d(
            p_XYZs1[i].x, p_XYZs1[i].y, p_XYZs1[i].z) );
        edge->setInformation( Eigen::Matrix3d::Identity()*1e4 );
        optimizer.addEdge(edge);
        index++;
        edges.push_back(edge);
    }

    chrono::steady_clock::time_point t1 = chrono::steady_clock::now();
    optimizer.setVerbose( false );
    optimizer.initializeOptimization();
    optimizer.optimize(10);
    chrono::steady_clock::time_point t2 = chrono::steady_clock::now();
    chrono::duration<double> time_used = chrono::duration_cast<chrono::duration<double>>(t2-t1);
    cout<<"optimization costs time: "<<time_used.count()<<" seconds."<<endl;

    cout<<endl<<"after optimization:"<<endl;
    cout<<"T="<<endl<<Eigen::Isometry3d( pose->estimate() ).matrix()<<endl;

    eigen2cv(Eigen::Isometry3d( pose->estimate() ).matrix(), T);

}


void pose_estimation::pose2d2d_8pts( vector<Point2f> & p_UVs1, vector<Point2f> & p_UVs2, Mat & mat_r, Mat & vec_t,Mat &cameraMatrix ){

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

}
void pose_estimation::pose2d2d_triangulation( vector<Point2f> & p_UVs1, vector<Point2f> & p_UVs2, Mat & mat_r, Mat & vec_t,Mat &cameraMatrix ){}

void pose_estimation::pose3d2d_PnP( vector<Point3f> & p_XYZs1, vector<Point2f> & p_UVs2, Mat & mat_r, Mat & vec_t,Mat &cameraMatrix){

	Mat vec_r;
    solvePnP ( p_XYZs1, p_UVs2, cameraMatrix, Mat(), vec_r, vec_t, false ); 
    cv::Rodrigues ( vec_r, mat_r ); 

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
    double focal_length = ( cameraMatrix.at<double> ( 0,0 ) + cameraMatrix.at<double> ( 1,1 ) ) /2;
    g2o::CameraParameters* camera = new g2o::CameraParameters (
        focal_length, Eigen::Vector2d ( cameraMatrix.at<double> ( 0,2 ), cameraMatrix.at<double> ( 1,2 ) ), 0
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
    cout<<"optimization costs time: "<<time_used.count() <<" seconds."<<endl;

    cout<<endl<<"after optimization:"<<endl;
    cout<<"T="<<endl<<Eigen::Isometry3d ( pose->estimate() ).matrix() <<endl;
    eigen2cv(Eigen::Isometry3d( pose->estimate() ).matrix(), T);
}