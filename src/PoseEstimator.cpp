#include "PoseEstimator.hpp"
#include <algorithm>
#include <stdexcept>
#include <set>
#include <omp.h>
#include <boost/bind.hpp>

using namespace sonar_localization;

PoseEstimator::PoseEstimator(odometry::UnderwaterVehicleOdometry& odometry, octomap::SonarOcTree& map, const sonar_localization::Configuration& config )
    : ParticleFilter<Particle>(config.seed), 
    rand_norm(rand_gen, boost::normal_distribution<>(0,1.0) ),
    rand_uni(rand_gen, boost::uniform_real<>(0,1.0) ),
    config(config),
    odometry(odometry),
    map(map),
    max_weight(0)
{
  
}

PoseEstimator::~PoseEstimator()
{
}

base::Pose2D PoseEstimator::samplePose2D( const base::Pose2D& mu, const base::Pose2D& sigma )
{
    double x = rand_norm(), y = rand_norm(), theta = rand_norm();

    return base::Pose2D( 
	    base::Vector2d( 
		x * sigma.position.x() + mu.position.x(),
		y * sigma.position.y() + mu.position.y() ),
	    theta * sigma.orientation + mu.orientation );
}

void PoseEstimator::init(int numParticles, const base::Pose2D& mu, const base::Pose2D& sigma, double zpos, double zsigma) 
{
    for(int i=0;i<numParticles;i++)
    {
	base::Pose2D sample = samplePose2D( mu, sigma );

	xi_k.push_back( 
		Particle( 
		    sample.position, 
		    sample.orientation,
		    zpos,
		    zsigma
		    ));
    }
}
/**
 * this is a piecewise linear function, with 
 * @param x as the function input
 * @param alpha minimum threshold any value of x below this value will be 1.0
 * @param beta any x above beta will be set to gamma
 * @param gamma result if x is above beta
 *
 * the interval between alpha and beta is linear, so that the function is
 * continous.
 */
double weightingFunction( double x, double alpha = 0.1, double beta = 0.9, double gamma = 0.05 )
{
    if( x < alpha )
	return 1.0;
    if( x < beta )
    {
	double a = (1.0-gamma)/(alpha-beta);
	double b = 1.0-alpha*a;
	return a*x + b; 
    }
    if( x >= beta )
	return gamma;

    return 0.0;
}


/**
 * Update every particle using the odometry data
 */
void PoseEstimator::project(const base::Quaterniond& orientation)
{   //Why not use the orientation from the odometry??
    double yaw = base::getYaw( orientation );
    zCompensatedOrientation = base::removeYaw( orientation );
    Eigen::Affine3d dTrans = orientation * odometry.getPoseDelta().toTransform();
    const double zDelta = dTrans.translation().z();
    
    //const double z_var = 1e-3;
    const double zVar = odometry.getPositionError()(2,2) * 2.0;

    double spread = weightingFunction( max_weight, 0.0, config.spreadThreshold, 0.0 );

    for(size_t i=0;i<xi_k.size();i++)
    {
	base::Pose2D delta = odometry.getPoseDeltaSample2D();

	Particle &p( xi_k[i] );
	p.position += Eigen::Rotation2D<double>(p.orientation) * delta.position;
	p.orientation += delta.orientation;

	if( config.maxYawDeviation > 0.0 )
	{
	    if( fabs(p.orientation - yaw) > config.maxYawDeviation )
	    {
		// TODO check how much we want to penalize particles going out
		// of the yaw bounds
		p.weight *= 0.7;
	    }
	}

	p.zPos += zDelta;
	p.zSigma = sqrt( p.zSigma*p.zSigma + zVar );

	// the particle with the lowest weight
	// is below a threshold. depending on what sort
	// of mode we are in, do different things.
	if(spread > 0) 
	{
	    // otherwise just spread out the particles and see if we can 
	    // recover this way.
	    const double trans_fac = config.spreadTranslationFactor * spread;
	    const double rot_fac = config.spreadRotationFactor * spread;
	    base::Pose2D sample = samplePose2D( 
		    base::Pose2D(), 
		    base::Pose2D( base::Vector2d( trans_fac, trans_fac ), rot_fac ) );

	    p.position += sample.position;
	    p.orientation += sample.orientation;
	}
    }
}

void PoseEstimator::update(const octomap::SonarOcTree& sonarData, const base::Quaterniond& orientation)
{
    updateWeights(sonarData, orientation);
    double eff = normalizeWeights();
    if( eff < config.minEffective )
    {
	resample();
    }
}

void PoseEstimator::updateWeights(const octomap::SonarOcTree& sonarData,const base::Quaterniond& orientation)
{
  
    double last_max_weight = max_weight;
    max_weight = 0;

    for(size_t i=0;i<xi_k.size();i++)
    {
	Particle &pose(xi_k[i]);
	base::Vector3d particlePosition( pose.position.x(), pose.position.y(), pose.zPos );
	
	Eigen::AngleAxisd yaw = Eigen::AngleAxisd( pose.orientation, Eigen::MatrixBase<base::Vector3d>::UnitZ() );
	
	base::Quaterniond particleOrientation =  zCompensatedOrientation*yaw;
        
	map.compareTrees(sonarData,particlePosition,particleOrientation);   
	 
	// use some measurement of the variance as the weight 
	const double weight=0;//  = map. compareTrees(sonarData,pos);
        xi_k[i].weight *= weight;
	xi_k[i].mprob = weight;

	// store the current maximum weight
	max_weight = std::max( max_weight, weight );

    }

}

base::Pose PoseEstimator::getCentroid()
{
    normalizeWeights();

    // calculate the weighted mean for now
    base::Pose2D mean;
    double zMean = 0.0;
    double sumWeights = 0.0;
    for(size_t i=0;i<xi_k.size();i++)
    {
	const Particle &particle(xi_k[i]);
	//if( !particle.x.floating )
	{
	    mean.position += particle.position * particle.weight;
	    mean.orientation += particle.orientation * particle.weight;
	    zMean += particle.zPos * particle.weight;
	    sumWeights += particle.weight;
	}
    }
    mean.position /= sumWeights;
    mean.orientation /= sumWeights;
    zMean /= sumWeights;

    // and convert into a 3d position
    base::Pose result( 
	    base::Vector3d( mean.position.x(), mean.position.y(), zMean ), 
	    Eigen::AngleAxisd( mean.orientation, Eigen::MatrixBase<base::Vector3d>::UnitZ() ) * zCompensatedOrientation );

    return result;
}
