#ifndef __IMAGING_SONAR_POSEPARTICLE_HPP__
#define __IMAGING_SONAR_POSEPARTICLE_HPP__

#include <limits>
#include <Eigen/Core>
#include <Eigen/Geometry>

#include <base/Eigen.hpp>
#include <base/Pose.hpp>
#include <base/Time.hpp>

#include <vector>

//#include <envire/tools/GaussianMixture.hpp>

namespace imaging_sonar_localization
{
struct PoseParticle
{
    PoseParticle() {};
    PoseParticle( const base::Vector2d& position, double yaw, double water_column)
	: xy_position(position),
        water_column(water_column),
        yaw(yaw),
        weight(0) {};

    //3D position
    base::Vector2d xy_position;

    double  water_column;
    
    //Yaw is sampled by now
    //TODO change to sample the yaw drift
    double yaw;

    double mprob;

    // particle weight
    double weight;
};

struct PoseDistribution
{
    // we need to force the GMM model to use the base types
    // here instead of the generic eigen types
   // struct BaseAdapter
   // {
//	enum { Dimension = 2 };
//	typedef double Scalar;
//	typedef base::Vector2d Vector;
//	typedef base::Matrix2d Matrix;
  //  };

    //typedef envire::GaussianMixture<double, 2, BaseAdapter> GMM;
    // Force instanciation of some of the templated code. This is needed for
    // gccxml (and therefore orogen)
    //
    // It is harmless outside these contexts
   // struct gccxml_workaround {
//	GMM::Parameter field;
  //  };

    base::Time time;
    std::vector<PoseParticle> particles;
    //GMM gmm;

};
}

#endif
