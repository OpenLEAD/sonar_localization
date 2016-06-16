#ifndef __IMAGING_SONAR_CONFIGURATION_HPP__
#define __IMAGING_SONAR_CONFIGURATION_HPP__

#include <cmath>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <base/Eigen.hpp>

namespace imaging_sonar_localization 
{

struct UpdateThreshold
{
    UpdateThreshold() {};
    UpdateThreshold( double distance, double angle )
	: distance( distance ), angle( angle ) {};

    bool test( double distance, double angle )
    {
	return distance > this->distance || angle > this->angle;
    }

    bool test( const Eigen::Affine3d& pdelta )
    {
	return test( Eigen::AngleAxisd( pdelta.linear() ).angle(), pdelta.translation().norm() );
    }

    double distance;
    double angle;
};
struct Configuration
{
    Configuration() :
	seed( 42u ),
	particleCount( 250 ), 
	minEffective( 50 ), 
	initialRotationError( base::Vector3d(0, 0, 0.1) ),
	initialTranslationError( base::Vector3d(0.1, 0.1, 1.0) ),
	measurementError( 0.1 ),
	spreadThreshold( 0.9 ),
	spreadTranslationFactor( 0.1 ),
	spreadRotationFactor( 0.05 ),
	maxYawDeviation( 15*M_PI/180.0 ),
	measurementThreshold( 0.1, 10*M_PI/180.0 ),
	maxSensorRange( 3.0 ),
	logDebug( false ),
	logParticlePeriod( 100 ),
        n_u_sigma(0.01),
        n_v_sigma(0.01),
        n_r_sigma(0.01)
    {};

    /** seed for all random processes in the filter */
    unsigned long seed;
    /** number of particles to use in the filter */
    size_t particleCount;
    /** if the effective number of particles goes below this count, 
     * the particles are resampled
     */
    size_t minEffective;
    /** initial sampling spread of the particles. 
     * This vector is split into the rotational (3) and translational (3) parts
     * of the error, so that [r t] is a 6 vector. That vector is effectively the
     * diagonal of the full covariance matrix of the initial error distribution.
     */
    base::Vector3d initialRotationError;
    /** initial sampling spread of the particles. 
     * This vector is split into the rotational (3) and translational (3) parts
     * of the error, so that [r t] is a 6 vector. That vector is effectively the
     * diagonal of the full covariance matrix of the initial error distribution.
     */
    base::Vector3d initialTranslationError;
    /** standard deviation of contact point measurement error in m.
     * This includes errors due to modelling errors, but not map errors.
     */
    double measurementError;
    /** threshold value for particle spreading. Particle spreading is
     * activated when the maximum particle weight is below this value.
     * Set to 0.0 to disable spreading.
     */ 
    double spreadThreshold;
    /** spread factor for translational component
     */
    double spreadTranslationFactor;
    /** spread factor for rotational component
     */
    double spreadRotationFactor;
    /** when set to a positive value, the filter will try to keep the deviation
     * of the estimated yaw value compared to the yaw value from the odometry
     * to this value
     */
    double maxYawDeviation;
    /** distance and rotation threshold for measurement updates. This provides the
     * minimum distance/rotation after which a new measurement is considered.
     */
    UpdateThreshold measurementThreshold;
    double maxSensorRange;
    /** flag if visual update method should be used.
     */
    bool logDebug;
    /** controls how often the particle distribution is logged
     * a value of 1 will log at every step (large log file)
     * a value of 0 will skip logging the particles
     * values greater than 1 will log every nth distribution.
     */
    unsigned int logParticlePeriod;

    /** variances of the motion model */
    double n_u_sigma, n_v_sigma, n_r_sigma;
};

}
#endif
