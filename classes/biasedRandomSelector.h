#ifndef BIASEDSELECTORALGORITHM_H
#define BIASEDSELECTORALGORITHM_H

#include <stdint.h>
#include <stdlib.h>
#include <stdio.h>

#include <math.h>

#include <algorithm>
#include <vector>
#include <string>

#include <QObject>

#define BIASED_MULT 1000000.

struct BiasedCandidate
{
public:
	uint32_t id;
	double weight;
	double accum;
	bool operator()(BiasedCandidate const& a, BiasedCandidate const& b) const
	{
		return a.weight > b.weight;
	}
};

class BiasedSelector
{
public:
	// Constructor. The parameter 'size' stands for the maximum number of candidates.
	BiasedSelector(uint32_t size=1)
	{
		resize(size, true);
		totalWeight = 0.;
	}
	// Constructor. The parameter 'size' stands for the maximum number of candidates.
	BiasedSelector(const BiasedSelector &other)
	{
		copyFrom(other);
	}
	BiasedSelector &operator=(const BiasedSelector &other)
	{
		copyFrom(other);
		return *this;
	}

	void copyFrom(const BiasedSelector &other)
	{
		candidates = other.candidates;
		totalWeight = other.totalWeight;
	}

	double getTotalWeight() { return totalWeight; }

	void resize(size_t size, bool initialize=false)
	{
		size_t prevSize = candidates.size();
		candidates.resize(size);
		if (prevSize<size or initialize==true)
		{
			for (size_t i=initialize?0:prevSize; i<candidates.size(); ++i)
			{
				candidates[i].id = i;
				candidates[i].weight = 0.;
			}
		}
	}
	uint32_t getFirst()
	{
		return candidates[0].id;
	}
	uint32_t getNumber(uint32_t n)
	{
		return candidates[n].id;
	}
	uint32_t get()
	{
		if (fabs(totalWeight) <= 0.000000000000000000000000001)
		{
			throw std::string("BiasedSelector::Error: No possible choice.");
		}
		const long double result = fmodl(qrand()/RAND_MAX, totalWeight);
		for (size_t i=0; i<candidates.size(); i++)
		{
			if (candidates[i].accum >= result)
				return candidates[i].id;
		}
		return candidates[0].id;
	}
	double getWeight(uint32_t id)
	{
		for (size_t i=0; i<candidates.size(); ++i)
		{
			if (candidates[i].id == id)
			{
				return candidates[i].weight / BIASED_MULT;
			}
		}
		throw std::string("BiasedSelector::WTF!? 2");
	}
	bool setWeight(uint32_t id, double p, bool perform_sort=true)
	{
		if (p<0.)
			throw std::string("Me does not allows negatif veilius");
		for (size_t i=0; i<candidates.size(); ++i)
		{
			if (candidates[i].id == id)
			{
				totalWeight -= candidates[i].weight;
				candidates[i].weight = BIASED_MULT * p;
				totalWeight += candidates[i].weight;
				if (perform_sort)
					sort();
			}
		}
		return false;
	}
	void sort()
	{
		std::sort(candidates.begin(), candidates.end(), BiasedCandidate());
		double accum = 0;
		for (size_t i=0; i<candidates.size(); ++i)
		{
			accum += candidates[i].weight;
			candidates[i].accum = accum;
		}
		totalWeight = candidates[candidates.size()-1].accum;
	}
	void print()
	{
		printf("Biased selector weights (%g)\n", totalWeight);
		for (size_t i=0; i<candidates.size(); i++)
		{
			printf("ID: %d\tweight:%10.5g\taccum:%10.5g\n", candidates[i].id, candidates[i].weight, candidates[i].accum);
		}
	}

	std::vector<BiasedCandidate> candidates;
	double totalWeight;
};

#endif
