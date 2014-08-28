#ifndef __ODOMETRY_UNDERWATERVEHICLE_HPP__
#define __ODOMETRY_UNDERWATERVEHICLE_HPP__

#include "UnderwaterVehicleState.hpp"

#include <base/time.h>

#include <odometry/Configuration.hpp>
#include <odometry/State.hpp>
#include <odometry/Gaussian.hpp>
#include <odometry/Gaussian3D.hpp>
#include <odometry/Sampling3D.hpp>
#include <odometry/Sampling2D.hpp>

#include <sonaroctomap/SonarOcTree.hpp>

namespace odometry 
{
typedef Eigen::Matrix<double,6,6> Matrix6d;
typedef Eigen::Matrix<double,6,1> Vector6d;
  
class UnderwaterVehicleOdometry : 
    public Gaussian3D,
    public Sampling3D,
    public Sampling2D
{
public: 
    //EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    UnderwaterVehicleOdometry(const Configuration& config);
    virtual ~UnderwaterVehicleOdometry();
    void update(const struct UnderwaterVehicleState& state, const Eigen::Quaterniond& orientation);

    base::Pose getPoseDelta();
    Eigen::Matrix3d getPositionError();
    Eigen::Matrix3d getOrientationError();
    Matrix6d getPoseError();

public:
    base::Pose getPoseDeltaSample();
    base::Pose2D getPoseDeltaSample2D();

    Eigen::Quaterniond orientation, prevOrientation;

    State<UnderwaterVehicleState> state;
    
    //State<UnderwaterVehicleState> state;

private:
    /** Odometry configuration */
    Configuration config;

    GaussianSamplingPose3D sampling;    

};
}

#endif