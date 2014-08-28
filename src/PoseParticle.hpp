#ifndef __ESLAM_POSEPARTICLE_HPP__
#define __ESLAM_POSEPARTICLE_HPP__

#include <limits>
#include <Eigen/Core>
#include <Eigen/Geometry>

#include <base/Eigen.hpp>
#include <base/Pose.hpp>
#include <base/Time.hpp>

#include <vector>

#include <envire/tools/GaussianMixture.hpp>

#include "UnderwaterVehicleState.hpp"

namespace sonar_localization
{
struct PoseParticle
{
    PoseParticle() {};
    PoseParticle( const base::Vector2d& position, double orientation, double zpos = 0, double zsigma = 0)
	: position(position), 
	orientation(orientation), 
	zPos(zpos), 
	zSigma(zsigma), 
	weight(0) {};

    base::Affine3d getPose( const base::Quaterniond& _orientation )
    {
	base::Vector3d pos( position.x(), position.y(), position.z());
	base::Affine3d t = 
	    Eigen::Translation3d( pos ) 
	    * Eigen::AngleAxisd( orientation, Eigen::Vector3d::UnitZ() )
	    * base::removeYaw( _orientation );
	
	return t;
    }

    base::Vector2d position;
    double orientation;

    double zPos;
    double zSigma;

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
    base::Quaterniond orientation;
    odometry::UnderwaterVehicleState state;
};
}

#endif
