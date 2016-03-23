#ifndef __IMAGING_SONAR_POSEPARTICLE_HPP__
#define __IMAGINS_SONAR_POSEPARTICLE_HPP__

#include <limits>
#include <Eigen/Core>
#include <Eigen/Geometry>

#include <base/Eigen.hpp>
#include <base/Pose.hpp>
#include <base/Time.hpp>

#include <vector>

#include <envire/tools/GaussianMixture.hpp>

namespace imaging_sonar_localization
{
struct PoseParticle
{
    PoseParticle() {};
    PoseParticle( const base::Vector3d& position, Eigen::Matrix3f orientation)
	: initial_position(position),
        orientation(orientation),
        weight(0) {};

    base::Affine3d getPose()
    {
	Eigen::Affine3d pose;
        //TODO
    }
    
    //3D position
    base::Vector3d initial_position;
    
    base::Vector3d odometry_position;
    
    //Orientation matrix
    Eigen::Matrix3f orientation;

    //Euler Angles
    double roll;
    double pitch;
    double yaw;

    double mprob;

    // particle weight
    double weight;
};

struct PoseDistribution
{
    // we need to force the GMM model to use the base types
    // here instead of the generic eigen types
    struct BaseAdapter
    {
	enum { Dimension = 2 };
	typedef double Scalar;
	typedef base::Vector2d Vector;
	typedef base::Matrix2d Matrix;
    };

    typedef envire::GaussianMixture<double, 2, BaseAdapter> GMM;
    // Force instanciation of some of the templated code. This is needed for
    // gccxml (and therefore orogen)
    //
    // It is harmless outside these contexts
    struct gccxml_workaround {
	GMM::Parameter field;
    };

    base::Time time;
    std::vector<PoseParticle> particles;
    GMM gmm;

};
}

#endif