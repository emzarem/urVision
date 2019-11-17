#include "..\include\PlantFilter.h"
#include <numeric>

#define USE_OTSU_FILTERING 1

PlantFilter::PlantFilter(Size frameSize, float minWeedSize, float maxWeedSize):
	m_minWeedSize(minWeedSize), m_maxWeedSize(maxWeedSize), m_frameSize(frameSize),
	m_otsuThreshold(-1)
{
}

PlantFilter::~PlantFilter()
{
}

vector<KeyPoint> PlantFilter::filterWeeds(vector<KeyPoint> currentPlants)
{
	vector<KeyPoint> outputWeeds;
	outputWeeds.clear();

	float maxSize = (float)m_frameSize.height;

	// This is an initial condition
	for (vector<KeyPoint>::iterator it = currentPlants.begin(); it != currentPlants.end(); ++it)
	{
		// Verify bigger than minimum size, otherwise ignore
		if (it->size >= m_minWeedSize)
		{
			uint8_t normalizedSize = (uint8_t)(255 * (it->size / (maxSize)));
			// Add normalized size to overall otsu size accumulator
			addToAccumulator(m_otsuAccumulator, normalizedSize, DEFAULT_HISTOGRAM_SIZE);

			// If greater than max weed size, this is a crop
			if (it->size > m_maxWeedSize)
			{
				addToAccumulator(m_cropSizeAccumulator, it->size, DEFAULT_ACCUMULATOR_LENGTH);
				addToAccumulator(m_cropXAccumulator, it->pt.x, DEFAULT_ACCUMULATOR_LENGTH);

				// Recalculate mean and stddev
				m_cropSizeStdDev = getStdDev(m_cropSizeAccumulator);
				m_cropSizeMean = getMean(m_cropSizeAccumulator);
			}
			// Otherwise we are still trying to figure out if this is a weed
			else
			{
				// If we have enough information about identified crops
				if (DEFAULT_ACCUMULATOR_LENGTH == m_cropSizeAccumulator.size())
				{
					// If this size is less than 2 stddevs away from the average crop size
					if (abs(m_cropSizeMean - it->size) <= 2 * m_cropSizeStdDev)
					{
						// This is just another crop!
						addToAccumulator(m_cropSizeAccumulator, it->size, DEFAULT_ACCUMULATOR_LENGTH);
						addToAccumulator(m_cropXAccumulator, it->pt.x, DEFAULT_ACCUMULATOR_LENGTH);

						// Recalculate mean and stddev
						m_cropSizeStdDev = getStdDev(m_cropSizeAccumulator);
						m_cropSizeMean = getMean(m_cropSizeAccumulator);
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
#if USE_OTSU_FILTERING
					// We can use our otsu size threshold (classifying objects)
					if (m_otsuThreshold > 0)
					{
						// If we can safely say this is a crop (based on otsu threshold)
						if ((float)normalizedSize > m_otsuThreshold)
						{
							// This is just another crop!
							addToAccumulator(m_cropSizeAccumulator, it->size, DEFAULT_ACCUMULATOR_LENGTH);
							addToAccumulator(m_cropXAccumulator, it->pt.x, DEFAULT_ACCUMULATOR_LENGTH);

							// Recalculate mean and stddev
							m_cropSizeStdDev = getStdDev(m_cropSizeAccumulator);
							m_cropSizeMean = getMean(m_cropSizeAccumulator);
						}
					}
#endif
				}
				else if (m_cropSizeAccumulator.size() > DEFAULT_ACCUMULATOR_LENGTH)
				{
					printf("Error -- accumulator growing too big\n");
				}
			}

		}
	}

	// If we have enough general data to classify an otsu threshold
	if (m_otsuAccumulator.size() >= DEFAULT_HISTOGRAM_SIZE)
	{
		// Classify using Otsu's (histogram) method
		Mat otsuHistogram, otsuOut;
		// Populate "histogram matrix"
		otsuHistogram = Mat(m_otsuAccumulator);
		m_otsuThreshold = (float)threshold(otsuHistogram, otsuOut, 0, UINT8_MAX, CV_THRESH_BINARY | CV_THRESH_OTSU);
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
	if (v.size() == 0)
	{
		return 0;
	}
	T sum = (T)std::accumulate(v.begin(), v.end(), 0.0);
	T mean = sum / v.size();

	return mean;
}

template<typename T>
T PlantFilter::getStdDev(std::vector<T> &v)
{
	if (v.size() == 0)
	{
		return 0;
	}
	T sum = (T)std::accumulate(v.begin(), v.end(), 0.0);
	T mean = sum / v.size();

	T sq_sum = (T)std::inner_product(v.begin(), v.end(), v.begin(), 0.0);
	T stdev = std::sqrt(sq_sum / v.size() - mean * mean);

	return stdev;
}