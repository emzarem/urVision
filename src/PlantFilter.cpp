#include "..\include\PlantFilter.h"
#include <numeric>

PlantFilter::PlantFilter(float minWeedSize, float maxWeedSize):
	m_minWeedSize(minWeedSize), m_maxWeedSize(maxWeedSize)
{
}

PlantFilter::~PlantFilter()
{
}

vector<KeyPoint> PlantFilter::filterWeeds(vector<KeyPoint> currentPlants)
{
	vector<KeyPoint> outputWeeds;
	outputWeeds.clear();

	// This is an initial condition
	for (vector<KeyPoint>::iterator it = currentPlants.begin(); it != currentPlants.end(); ++it)
	{
		// Verify bigger than minimum size, otherwise ignore
		if (it->size >= m_minWeedSize)
		{
			// Add to overall size accumulator
			addToAccumulator(m_allSizeAccumulator, it->size, DEFAULT_ACCUMULATOR_LENGTH);

			// If greater than max weed size, this is a crop
			if (it->size > m_maxWeedSize)
			{
				addToAccumulator(m_cropSizeAccumulator, it->size, DEFAULT_ACCUMULATOR_LENGTH);
				addToAccumulator(m_cropXAccumulator, it->pt.x, DEFAULT_ACCUMULATOR_LENGTH);
			}
			// Otherwise we are still trying to figure out if this is a weed
			else
			{
				// If we have enough information about identified crops
				if (DEFAULT_ACCUMULATOR_LENGTH == m_cropSizeAccumulator.size())
				{
					float cropSizeStdDev = getStdDev(m_cropSizeAccumulator);
					float cropSizeMean = getMean(m_cropSizeAccumulator);

					// If this size is less than 2 stddevs away from the average crop size
					if (abs(cropSizeMean - it->size) < 2 * cropSizeStdDev)
					{
						// This is just another crop!
						addToAccumulator(m_cropSizeAccumulator, it->size, DEFAULT_ACCUMULATOR_LENGTH);
						addToAccumulator(m_cropXAccumulator, it->pt.x, DEFAULT_ACCUMULATOR_LENGTH);
					}
					else
					{
						// Identify as a weed!
						addToAccumulator(m_weedSizeAccumulator, it->size, DEFAULT_ACCUMULATOR_LENGTH);
						addToAccumulator(m_weedXAccumulator, it->pt.x, DEFAULT_ACCUMULATOR_LENGTH);

						// Add to output list of filtered weeds
						outputWeeds.push_back(*it);
					}
				}
				else if (m_cropSizeAccumulator.size() < DEFAULT_ACCUMULATOR_LENGTH)
				{
					// Do we have enough information to classify?
				}
				else if (m_cropSizeAccumulator.size() > DEFAULT_ACCUMULATOR_LENGTH)
				{
					printf("Error -- accumulator growing too big\n");
				}
			}

		}
	}

	return outputWeeds;
}

template<typename T>
void PlantFilter::pop_front(std::vector<T> &v)
{
	if (v.size() > 0) {
		v.erase(v.begin());
	}
}

template<typename T>
void PlantFilter::addToAccumulator(std::vector<T> &v, T val, int maxLength)
{
	v.push_back(val);
	while (v.size() > maxLength) {
		pop_front(v);
	}
}

template<typename T>
T PlantFilter::getMean(std::vector<T> &v)
{
	T sum = (T)std::accumulate(v.begin(), v.end(), 0.0);
	T mean = sum / v.size();

	return mean;
}

template<typename T>
T PlantFilter::getStdDev(std::vector<T> &v)
{
	T sum = (T)std::accumulate(v.begin(), v.end(), 0.0);
	T mean = sum / v.size();

	T sq_sum = (T)std::inner_product(v.begin(), v.end(), v.begin(), 0.0);
	T stdev = std::sqrt(sq_sum / v.size() - mean * mean);

	return stdev;
}