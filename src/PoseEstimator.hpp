#ifndef __POSE_ESTIMATOR_HPP__
#define __POSE_ESTIMATOR_HPP__

#include "PoseParticle.hpp"
#include "Configuration.hpp"
#include "UnderwaterVehicleOdometry.hpp"

#include "ParticleFilter.hpp"
#include <boost/random/normal_distribution.hpp>
#include <boost/intrusive_ptr.hpp>

#include <sonaroctomap/SonarOcTree.hpp>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <base/Pose.hpp>

#include<odometry/Sampling3D.hpp>

#include <limits>

namespace sonar_localization
{

class PoseEstimator :
    public ParticleFilter<PoseParticle>
{
public:
    PoseEstimator(odometry::UnderwaterVehicleOdometry& odometry, octomap::SonarOcTree& map, const sonar_localization::Configuration& config);
    ~PoseEstimator();

    void init(int numParticles, const base::Pose2D& mu, const base::Pose2D& sigma, double zpos = 0, double zsigma = 0);
    void project(const base::Quaterniond& orientation);
    void update(const octomap::SonarOcTree& sonarData, const base::Quaterniond& orientation);
    base::Pose getCentroid();

private:
    void updateWeights(const octomap::SonarOcTree& sonarData, const base::Quaterniond& orientation);

    boost::variate_generator<boost::minstd_rand&, boost::normal_distribution<> > rand_norm;
    boost::variate_generator<boost::minstd_rand&, boost::uniform_real<> > rand_uni;
    base::Pose2D samplePose2D( const base::Pose2D& mu, const base::Pose2D& sigma );

    sonar_localization::Configuration config;
    odometry::UnderwaterVehicleOdometry &odometry;
    
    octomap::SonarOcTree map;
   
    bool useShared;
    base::Quaterniond zCompensatedOrientation;
    double max_weight;
};

}
#endif
