#ifndef ROBOCOMPPARTICLEFILTER_H
#define ROBOCOMPPARTICLEFILTER_H

#include <QMutex>
#include <QVector>

#include <biasedRandomSelector.h>

#include <omp.h>

/**
========================================
==  How to create your own particle:  ==
========================================
class ExampleParticle : public RCParticleFilter_Particle < image, force >
{
public:
	void initialize(const image &data, const force &control)
	{
		initialization using the data and, probably, the control
	}
	void adapt(const force &controlBack, const force &controlNew, const bool noValidCandidates)
	{
		velocity += force * incTime + noise
	}
	void computeWeight(const image &data)
	{
		weight = likelyhood_of_data_given_state(data)
	}
	
	void toBeNormalized()
	{
		return true;
	}
	
private:
	int velocity;
};


================================================
==  How to use the particle filter template:  ==
================================================
RCParticleFilter_Config c;
c.particles = 1000;
RCParticleFilter < char, int, ExampleParticle > pf('A', 1);

while (XXX)
{
	read data;
	read control;
	
	pf.step(data, control);
	
	std::cout << pf.getBest();
	
}
**/	


/**
 *    R C P F P a r t i c l e
 *
 */
template < typename RCPFInputData, typename RCPFControl, typename RCPFConfig >
class RCParticleFilter_Particle
{
public:
	virtual void initialize(const RCPFInputData &data, const RCPFControl &control, const RCPFConfig *cfg) = 0;
	virtual void adapt(const RCPFControl &controlBack, const RCPFControl &controlNew, const bool noValidCandidates) = 0;
	virtual void computeWeight(const RCPFInputData &data) = 0;

	double getWeight() const
	{
		return weight;
	}

protected:
	double weight;
};

/**
 *    R C P F C o n f i g
 *
 */
class RCParticleFilter_Config
{
public:
	uint32_t particles;
};


/**
 *    R C P a r t i c l e F i l t e r
 *
 *
 */
template < typename RCPFInputData, typename RCPFControl, typename RCPFParticle, typename RCParticleFilterConfig = RCParticleFilter_Config >
class RCParticleFilter
{
public:
	RCParticleFilter(RCParticleFilterConfig *conf, const RCPFInputData &data, const RCPFControl &control)
	{
		config = conf;
		lastControl = control;
		selector = new BiasedSelector(config->particles);
		for (int32_t i=0; i<(int32_t)config->particles; ++i)
		{
			RCPFParticle p;
			p.initialize(data, control, config);
			resampledParticles.push_back(p);
			weightedParticles.push_back(p);
		}
	}

	void step(const RCPFInputData &data, const RCPFControl &control, bool includeBest=false, uint32_t maxThreads=-1)
	{
		/// Particle Filter Step #1
		adaptParticles(lastControl, control);
		/// Particle Filter Step #2
		calculateWeights(data, maxThreads);
		/// Particle Filter Step #3
		clone(control);

		if (includeBest)
			forceIncludeParticle(best, 0);

		/// Store last control
		lastControl = control;
	}

	RCPFParticle getBest() const { return best; }


	void forceIncludeParticle(RCPFParticle p, uint32_t index)
	{
		weightedParticles[index] = p;
	}

	const QVector<RCPFParticle> &particles() const { return weightedParticles; }

	RCPFParticle getOrderedParticle(uint32_t p) const
	{
		return weightedParticles[selector->getNumber(p)];
	}

	RCPFParticle getResampledParticle(uint32_t p) const
	{
		return resampledParticles[p];
	}
protected:
	QMutex mutex;
	RCParticleFilterConfig *config;
	RCPFParticle best;
	bool noCandidates;
	BiasedSelector *selector;
	RCPFControl lastControl;
	QVector < RCPFParticle > resampledParticles, weightedParticles;

	void lock()
	{
		mutex.lock();
	}
	void unlock()
	{
		mutex.unlock();
	}	

	void initialize(const RCPFInputData &data, const RCPFControl &control, const RCParticleFilterConfig *cfg)
	{
		lastControl = control;
		for (int32_t i=0; i<(int32_t)config->particles; ++i)
		{
			RCPFParticle *p = &resampledParticles.operator[](i);
			p->initialize(data, control, cfg);
			weightedParticles[i] = *p;
		}
	}


	void adaptParticles(const RCPFControl &controlBack, const RCPFControl &controlNew)
	{
		RCPFParticle *p = &resampledParticles[0];
		for (int32_t i=0; i<(int32_t)config->particles; ++i)
		{
			p[i].adapt(controlBack, controlNew, false);
			weightedParticles[i] = p[i];
		}
	}

	void calculateWeights(const RCPFInputData &data, int32_t maxThreads)
	{
		RCPFParticle *p = &weightedParticles[0];

		if (maxThreads==0)
		{
			for (uint i=0; i<config->particles; ++i)
			{
				p[i].computeWeight(data);
			}
		}
		else
		{
			omp_set_num_threads(maxThreads<0?omp_get_max_threads():maxThreads);
			#pragma omp parallel for
			for (uint i=0; i<config->particles; ++i)
			{
				p[i].computeWeight(data);
			}
		}

		for (uint i=0; i<config->particles; ++i)
		{
			selector->setWeight(i, (1000.0*p[i].getWeight())*1000.0, true);
		}

		try
		{
			selector->sort();
		}
		catch (std::string s)
		{
			printf("catch %s\n", s.c_str());
		}

	}
	
	void clone(const RCPFControl &control)
	{
		// Check if there is any considerably probable particle...
		int selected;
		try
		{
			noCandidates = false;
			selected = selector->get();
		}
		catch (...)
		{
			noCandidates = true;
		}
		// Clone
		try
		{
			best = weightedParticles.operator[](selector->getFirst());
			// If there was, perform a regular clone process
			for (uint i=0; i<config->particles; ++i)
			{
				selected = noCandidates?i:selector->get();
				RCPFParticle p = weightedParticles.operator[](selected);
// 				p.adapt(lastControl, control, noCandidates);
				resampledParticles.operator[](i) = p;
			}
		}
		catch (...)
		{
			qFatal("THIS SHOULD NEVER HAPPEN! selector->get\n");
		}
	}
};

#endif
