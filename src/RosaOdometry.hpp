#include "RosaOdometry.hpp"

using namespace odometry;

RosaOdometry::RosaOdometry(const Configuration& config)
: config(config),
  sampling(config)
{
}

RosaOdometry::~RosaOdometry()
{
}

Eigen::Matrix3d RosaOdometry::getPositionError()
{
    return Eigen::Matrix3d(sampling.poseCov.bottomRightCorner<3,3>());
}

Eigen::Matrix3d RosaOdometry::getOrientationError()
{
    return Eigen::Matrix3d(sampling.poseCov.topLeftCorner<3,3>());
}

Matrix6d RosaOdometry::getPoseError()
{
    return sampling.poseCov;
}

base::Pose RosaOdometry::getPoseSample()
{
    return base::Pose( sampling.sample() );
}

void RosaOdometry::update(const base::samples::RigidBodyState& depth, const Eigen::Quaterniond& orientation)
{
    /* update state with the measuremment of the depth sensor 
    * and with the roll measuremment of the inclinometer
    * */
    base::Pose pose = base::Pose(depth.position, orientation);
    
    // TODO calculate error matrix
    Eigen::Vector4d vec = config.constError.toVector4d();
    Vector6d var;
    var << 0, 0, vec.w(), vec.head<3>();

    sampling.update( pose.toVector6d(), var.asDiagonal() );
}
