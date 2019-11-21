#include "../include/PlantFilter.h"
#include <numeric>

PlantFilter::PlantFilter(VisionParams visionParams) :
	m_visionParams(visionParams)
{

	m_maxSize = m_visionParams.frameSize.height;

	if (visionParams.defaultWeedThreshold > 0)
	{
		m_otsuThreshold = visionParams.defaultWeedThreshold;
		m_otsuThresholdNorm = (uint8_t)(255 * (m_otsuThreshold / ((float)m_maxSize)));
	}
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
		// Verify bigger than minimum size, otherwise completely ignore
		if (it->size >= m_visionParams.minWeedSize)
		{
			uint8_t normalizedSize = (uint8_t)(255 * (it->size / ((float)m_maxSize)));
			// Add normalized size to overall otsu size accumulator
			addToAccumulator(m_normalizedOtsuAccumulator, normalizedSize, m_visionParams.minAccumulatorSize);

			// If greater than max weed size, this is a crop
			if (it->size > m_visionParams.maxWeedSize)
			{
				addToAccumulator(m_cropSizeAccumulator, it->size, m_visionParams.maxAccumulatorSize);
				addToAccumulator(m_cropXAccumulator, it->pt.x, m_visionParams.maxAccumulatorSize);

				// Recalculate mean and stddev
				m_cropSizeStdDev = getStdDev(m_cropSizeAccumulator);
				m_cropSizeMean = getMean(m_cropSizeAccumulator);
			}
			// We can use our otsu size threshold (classifying objects)
			else if (m_otsuThresholdNorm > 0)
			{
				// If we can safely say this is a crop (based on otsu threshold)
				if (normalizedSize > m_otsuThresholdNorm)
				{
					// This is just another crop!
					addToAccumulator(m_cropSizeAccumulator, it->size, m_visionParams.maxAccumulatorSize);
					addToAccumulator(m_cropXAccumulator, it->pt.x, m_visionParams.maxAccumulatorSize);

					// Recalculate mean and stddev
					m_cropSizeStdDev = getStdDev(m_cropSizeAccumulator);
					m_cropSizeMean = getMean(m_cropSizeAccumulator);
				}
				else
				{
					// If we have enough information about identified crops
					if (m_cropSizeAccumulator.size() > m_visionParams.minAccumulatorSize)
					{
						// If this size is greater than the current mean
						// size is less than 2 stddevs away from the average crop size
						if (it->size > m_cropSizeMean ||
							abs(m_cropSizeMean - it->size) <= 2 * m_cropSizeStdDev)
						{
							// This is just another crop!
							addToAccumulator(m_cropSizeAccumulator, it->size, m_visionParams.maxAccumulatorSize);
							addToAccumulator(m_cropXAccumulator, it->pt.x, m_visionParams.maxAccumulatorSize);

							// Recalculate mean and stddev
							m_cropSizeStdDev = getStdDev(m_cropSizeAccumulator);
							m_cropSizeMean = getMean(m_cropSizeAccumulator);
						}
						else
						{

							// Identify as a weed!
							addToAccumulator(m_weedSizeAccumulator, it->size, m_visionParams.maxAccumulatorSize);
							addToAccumulator(m_weedXAccumulator, it->pt.x, m_visionParams.maxAccumulatorSize);

							// Add to output list of filtered weeds
							outputWeeds.push_back(*it);
						}

						// Check for error in accumulator
						if (m_cropSizeAccumulator.size() > m_visionParams.maxAccumulatorSize)
						{
							printf("Error -- accumulator growing too big\n");
						}
					}
				}
			}
		}
	}

	// If we have enough general data to classify an otsu threshold
	if (m_normalizedOtsuAccumulator.size() >= m_visionParams.minAccumulatorSize)
	{
		// Classify using Otsu's (histogram) method
		Mat otsuHistogram;
		// Populate "histogram matrix"
		otsuHistogram = Mat(m_normalizedOtsuAccumulator);
		m_otsuThresholdNorm = (float)threshold(otsuHistogram, otsuHistogram, 0, UINT8_MAX, CV_THRESH_BINARY | CV_THRESH_OTSU);
		// For pubishing purposes
		m_otsuThreshold = m_otsuThresholdNorm*((float)m_maxSize)/255;
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