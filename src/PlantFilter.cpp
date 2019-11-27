#include "../include/PlantFilter.h"
#include <numeric>
#include <ros/ros.h>

PlantFilter::PlantFilter(VisionParams visionParams) :
	m_visionParams(visionParams)
{

	m_maxSize = m_visionParams.frameSize.height;
	m_otsuThreshold = -1;

	if (visionParams.defaultWeedThreshold <= 0 || 
			visionParams.defaultCropThreshold <= 0 ||
			visionParams.defaultCropThreshold < visionParams.defaultWeedThreshold )
	{
		ROS_ERROR("Default weed/crop thresholds have not been set properly!");
		ros::requestShutdown();
	}
}

PlantFilter::~PlantFilter()
{
}

/*
	all objects with size < defaultWeedThreshold are marked as weeds
	all objects with size > defaultCropThreshold are marked as crops
	if defaultWeedThreshold < size < defaultCropThreshold, attempt to use otsuThresholding
*/
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
			// If size is greater than our default crop size threshold
			if (it->size > m_visionParams.defaultCropThreshold)
			{
				// Mark as crop
				addToAccumulator(m_cropSizeAccumulator, it->size, m_visionParams.maxAccumulatorSize);
				addToAccumulator(m_cropXAccumulator, it->pt.x, m_visionParams.maxAccumulatorSize);
			}
			// If size is less than the weed threshold
			else if (it->size < m_visionParams.defaultWeedThreshold)
			{
				// Identify as a weed!
				addToAccumulator(m_weedSizeAccumulator, it->size, m_visionParams.maxAccumulatorSize);
				addToAccumulator(m_weedXAccumulator, it->pt.x, m_visionParams.maxAccumulatorSize);
				// Add to output weed list
				outputWeeds.push_back(*it);
			}
			// Else object size is in between
			else
			{
				// Add size to overall otsu size accumulator
				addToAccumulator(m_otsuAccumulator, it->size, m_visionParams.maxAccumulatorSize);

				// Can only make a decision IF otsu threshold is in between the two default sizes
				if (m_otsuAccumulator.size() >= m_visionParams.minAccumulatorSize &&
						m_otsuThreshold > m_visionParams.defaultWeedThreshold &&
						m_otsuThreshold < m_visionParams.defaultCropThreshold)
				{
					if (it->size > m_otsuThreshold)
					{
						// Mark as crop
						addToAccumulator(m_cropSizeAccumulator, it->size, m_visionParams.maxAccumulatorSize);
						addToAccumulator(m_cropXAccumulator, it->pt.x, m_visionParams.maxAccumulatorSize);
					}
					else
					{
						// Identify as a weed!
						addToAccumulator(m_weedSizeAccumulator, it->size, m_visionParams.maxAccumulatorSize);
						addToAccumulator(m_weedXAccumulator, it->pt.x, m_visionParams.maxAccumulatorSize);
						// Add to output list of filtered weeds
						outputWeeds.push_back(*it);
					}
				}
			}
		}
	}

	// If we have enough general data to classify an otsu threshold
	if (m_otsuAccumulator.size() >= m_visionParams.minAccumulatorSize)
	{
		/* Classify using Otsu's (histogram) method */
		// Normalize otsu accumulator between 0-255
		// Get current max value
		float maxValue = *max_element(m_otsuAccumulator.begin(), m_otsuAccumulator.end());
		float minValue = *min_element(m_otsuAccumulator.begin(), m_otsuAccumulator.end());
		vector<uint8_t> otsuAccumulatorNorm;
		otsuAccumulatorNorm.clear();
		// Normalize the current values in the accumulator
		for (vector<float>::iterator it = m_otsuAccumulator.begin(); it != m_otsuAccumulator.end(); ++it)
		{
			uint8_t normalizedValue = (uint8_t)(((*it - minValue)/(maxValue - minValue))*255);
			otsuAccumulatorNorm.push_back(normalizedValue);
		}
		// Create "histogram" matrix
		Mat otsuHistogramNorm = Mat(otsuAccumulatorNorm);
		float m_otsuThresholdNorm = (float)threshold(otsuHistogramNorm, otsuHistogramNorm, 0, 255, CV_THRESH_BINARY | CV_THRESH_OTSU);
		// un-normalize the threshold!
		m_otsuThreshold = (m_otsuThresholdNorm/255.0)*(maxValue - minValue) + minValue;
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