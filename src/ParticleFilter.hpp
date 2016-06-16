#ifndef __PARTICLE_FILTER_HPP__
#define __PARTICLE_FILTER_HPP__

#include <boost/random/linear_congruential.hpp>
#include <boost/random/uniform_real.hpp>
#include <boost/random/variate_generator.hpp>
#include <limits>

namespace imaging_sonar_localization
{

/** 
 * Generic Particle Filter implementation 
 *
 * The class is templated for the particle class, which needs to have a weight
 * field of scalar type. The particles need to be copyable. 
 */
template <class _Particle>
class ParticleFilter
{
public:
    typedef _Particle Particle;

    ParticleFilter( unsigned long seed ) :
	rand_gen( seed )
    {
    };

    ParticleFilter() :
	rand_gen( 42u )
    {
    };

    double getWeightsSum() const
    {
	double sumWeights = 0;
	for(size_t n=0;n<xi_k.size();sumWeights+=xi_k[n++].weight);
	return sumWeights;
    }

    double getWeightAvg() const
    {
	return getWeightsSum() / xi_k.size();
    }

    double normalizeWeights()
    {
	double sumWeights = getWeightsSum(); 

	double effective = 0;
	if( sumWeights <= 0.0 )
	{
	    for(size_t n=0;n<xi_k.size();n++)
	    {
		double &w(xi_k[n].weight);
		w = 1.0/xi_k.size();
		effective += w*w;
	    }
	}
	else{
	    for(size_t n=0;n<xi_k.size();n++)
	    {
		double &w(xi_k[n].weight);
		w /= sumWeights;
		effective += w*w;
	    }
	}

	return 1.0/effective;
    };

    void resample()
    {
	resample_stratified( xi_k.size() );
    }

    /** @brief implementation of a stratified resampling scheme
     *
     * The stratified resampling is more stable in variance to the multinomial
     * resampling scheme, and also more efficient.
     *
     * @param samples - number of particles to sample from the weighted
     *			proposal distribution 
     */
    void resample_stratified( size_t samples )
    {
	boost::variate_generator<boost::minstd_rand&, boost::uniform_real<> > 
	    rand(rand_gen, boost::uniform_real<>(0,1.0) );

	// need to have at least one particle in the original set of particles
	assert( xi_k.size() );

	std::vector<Particle> xi_kp;
	size_t idx = 0;
	double sum_w = xi_k[idx].weight;
	for( size_t k=0; k<samples; ++k )
	{
	    double sum_r = (k + rand())/samples;
	    while( sum_w < sum_r )
	    {
		++idx;
		sum_w += xi_k[idx].weight;
	    }
	    xi_kp.push_back( xi_k[idx] );
	}

	xi_k.swap( xi_kp );
    }

    /** @brief implementation of a multinomial resampling scheme
     *
     * multinomial resampling: imagine a strip of paper where each particle has
     * a section, where the length is proportional to its weight. randomly pick
     * a location on the strip n times, and pick the particle associated with
     * the section.
     *
     * @param samples - number of particles to sample from the weighted
     *			proposal distribution 
     */
    void resample_multinomial( size_t samples )
    {
	boost::variate_generator<boost::minstd_rand&, boost::uniform_real<> > 
	    rand(rand_gen, boost::uniform_real<>(0,1.0) );

	// need to have at least one particle in the original set of particles
	assert( xi_k.size() );

	std::vector<Particle> xi_kp;
	for(size_t n=0;n<samples;n++)
	{
	    double sum=0;
	    double r_n = rand();

	    for(size_t i=0;i<xi_k.size();i++)
	    {
		sum += xi_k[i].weight;
		if( r_n <= sum )
		{
		    Particle p( xi_k[i] );
		    p.weight = 1.0 / xi_k.size();
		    xi_kp.push_back(p);
		    break;
		}
	    }
	}

	xi_k.swap( xi_kp );
    };

    std::vector<Particle>& getParticles()
    {
	return xi_k;
    };

    const std::vector<Particle>& getParticles() const
    {
	return xi_k;
    };

    size_t getBestParticleIndex() const 
    {
	size_t index = 0;
	double weight = -std::numeric_limits<double>::infinity();
	for(size_t i=0;i<xi_k.size();i++)
	{
	    if( xi_k[i].weight > weight )
	    {
		index = i;
		weight = xi_k[i].weight;
	    }
	}
	return index;
    }

protected:
    std::vector<Particle> xi_k;
    boost::minstd_rand rand_gen;
};

}
#endif
