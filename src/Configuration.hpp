#ifndef __SONARLOCALISATION_CONFIGURATION_HPP__
#define __SONARLOCALISATION_CONFIGURATION_HPP__

#include <cmath>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <base/Eigen.hpp>

namespace sonar_localization 
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
	discountFactor( 0.9 ),
	spreadThreshold( 0.9 ),
	spreadTranslationFactor( 0.1 ),
	spreadRotationFactor( 0.05 ),
	slipFactor( 0.05 ),
	maxYawDeviation( 15*M_PI/180.0 ),
	measurementThreshold( 0.1, 10*M_PI/180.0 ),
	mappingThreshold( 0.02, 5*M_PI/180.0 ),
	mappingCameraThreshold( 1.0, 30*M_PI/180.0 ),
	gridSize( 20.0 ),
	gridResolution( 0.05 ),
	gridThreshold( 0.5 ),
	gridPatchThickness( 0.1 ),
	gridGapSize( 1.5 ),
	gridUseNegativeInformation( false ),
	maxSensorRange( 3.0 ),
	useVisualUpdate( false ),
	logDebug( false ),
	logParticlePeriod( 100 )
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
    /** particles which don't have contact with the environment
     * will be discounted over time with this factor. Set to 1.0 to
     * disable discounting.
     */ 
    double discountFactor;
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
    /** value between 0.0 and 1.0, gives the factor at which slip occurs.  So a
     * value of 0.1 will produce slip at 10% of the propagation steps.  Slip
     * will cause the translation in y direction to be randomly multiplied with
     * a value between 0 and 1.0
     */
    double slipFactor;
    /** when set to a positive value, the filter will try to keep the deviation
     * of the estimated yaw value compared to the yaw value from the odometry
     * to this value
     */
    double maxYawDeviation;
    /** distance and rotation threshold for measurement updates. This provides the
     * minimum distance/rotation after which a new measurement is considered.
     */
    UpdateThreshold measurementThreshold;
    /** minimum distance/rotation after which a new mapping input is considered.
     */
    UpdateThreshold mappingThreshold;
    /** minimum distance/rotation after which a camera input is considered.
     */
    UpdateThreshold mappingCameraThreshold;
    /** Size of newly created square grid patches in m
     */
    double gridSize;
    /** resolution of the grid in m per grid cell
     */
    double gridResolution;
    /** if the robots position has a distance of more than gridThreshold *
     * gridSize, a new grid is created.
     */
    double gridThreshold;
    /** patchThickness parameter for the MLS
     */
    double gridPatchThickness;
    /** gapSize parameter for the MLS
     */
    double gridGapSize;
    /** if set to true, negative information (like the absence of things) will
     * also be used in the processing chain.
     */
    bool gridUseNegativeInformation;
    /** maximum range value for camera sensor data
     */
    double maxSensorRange;
    /** flag if visual update method should be used.
     */
    bool useVisualUpdate;
    /** configuration options for the contact model
     */
    /** if set to true, the filter will generate debug information in the particles.
     * this will result in very large log files.
     */
    bool logDebug;
    /** controls how often the particle distribution is logged
     * a value of 1 will log at every step (large log file)
     * a value of 0 will skip logging the particles
     * values greater than 1 will log every nth distribution.
     */
    unsigned int logParticlePeriod;
};

}
#endif
