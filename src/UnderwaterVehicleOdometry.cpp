#include "UnderwaterVehicleOdometry.hpp"

const double BAR2METER = 10.06;

using namespace odometry;

UnderwaterVehicleOdometry::UnderwaterVehicleOdometry(const Configuration& config)
: config(config),
  sampling(config)
{
}

UnderwaterVehicleOdometry::~UnderwaterVehicleOdometry()
{
}

Eigen::Matrix3d UnderwaterVehicleOdometry::getPositionError()
{
    return Eigen::Matrix3d(sampling.poseCov.bottomRightCorner<3,3>());
}

Eigen::Matrix3d UnderwaterVehicleOdometry::getOrientationError()
{
    return Eigen::Matrix3d(sampling.poseCov.topLeftCorner<3,3>());
}

Matrix6d UnderwaterVehicleOdometry::getPoseError()
{
    return sampling.poseCov;
}

base::Pose UnderwaterVehicleOdometry::getPoseDeltaSample()
{
    return base::Pose( sampling.sample() );
}

base::Pose2D UnderwaterVehicleOdometry::getPoseDeltaSample2D()
{
    return projectPoseDelta( orientation, getPoseDeltaSample() );
}

base::Pose UnderwaterVehicleOdometry::getPoseDelta()
{
    return base::Pose( sampling.poseMean );
}

void UnderwaterVehicleOdometry::update(const odometry::UnderwaterVehicleState& bs, const Eigen::Quaterniond& orientation)
{
    // update state
    state.update( bs );
    this->orientation = orientation;

    if( !state.isValid() )
    {
	state.update( bs );
	prevOrientation = orientation;
    }

    // get relative rotation between updates
    // this is assumed to be the correct rotation (with error of course)
    Eigen::Quaterniond delta_rotq( prevOrientation.inverse() * orientation );
  
    double deltaPressure = state.getPrevious().pressure - state.getCurrent().pressure;
    double deltaZ = BAR2METER*deltaPressure;
    
    //TODO calculate the translation in x,y
    base::Pose deltaPose = base::Pose(Eigen::Vector3d::UnitZ()*deltaZ, delta_rotq);

    // calculate error matrix
    // TODO this is based on the wheel odometry error model I think it could be
    // more accurate by looking the the covariance of the contact position
    // differences 
    
    double tilt = acos(Eigen::Vector3d::UnitZ().dot(orientation*Eigen::Vector3d::UnitZ()));
    double d = deltaPose.position.norm();
;
    double dtheta = Eigen::AngleAxisd( delta_rotq ).angle();

    Eigen::Vector4d vec =
	config.constError.toVector4d() +
	d * config.distError.toVector4d() +
	tilt * config.tiltError.toVector4d() +
	dtheta * config.dthetaError.toVector4d();
    
    //TODO check useZeroVelocity
    double zeroCheck = 0;	
    const double zeroVelocityThreshold = 1e-9;
    if( config.useZeroVelocity && zeroCheck < zeroVelocityThreshold )
    {
	deltaPose = base::Pose();
	vec.setZero();
    }

    Vector6d var;
    var << 0, 0, vec.w(), vec.head<3>();

    sampling.update( deltaPose.toVector6d(), var.asDiagonal() );

    prevOrientation = orientation;
}