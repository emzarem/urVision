#include "../include/PlantFilter.h"
#include <numeric>
#include <ros/ros.h>

using namespace cv;

/* euclidean_distance
 *      @brief Calculates euclidean distance between two objects
 */
inline float euclideanDistance(const KeyPoint& a, const KeyPoint& b)
{
    float delt_x = (float)a.pt.x - (float)b.pt.x;
    float delt_y = (float)a.pt.y - (float)b.pt.y;
    return sqrt(delt_x*delt_x + delt_y*delt_y);
}

PlantFilter::PlantFilter(VisionParams visionParams) :
	m_visionParams(visionParams)
{
	m_maxSize = m_visionParams.frameSize.height;
	m_distanceTol = m_visionParams.filterDistanceTol;
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
void PlantFilter::filter(vector<KeyPoint>& currentObjects, vector<KeyPoint>& outputWeeds, vector<KeyPoint>& outputCrops)
{
	// Clear both output vectors
	outputWeeds.clear();
	outputCrops.clear();

	static float distance = 0;
	auto itr1 = currentObjects.begin();
	// First, group objects as necessary
	for (int count = 1; itr1 != currentObjects.end(); count++)
	{
		// Go through all objects
		for (auto itr2 = currentObjects.begin(); itr2 != currentObjects.end();  ) 
		{
			if (itr1 != itr2)
			{
				distance = euclideanDistance(*itr1, *itr2);
				// Check euclidean distance between the 2 points
				if (distance < m_distanceTol)
				{
					// Group these objects
					itr1->pt.x = (itr1->pt.x + itr2->pt.x) / 2;
					itr1->pt.y = (itr1->pt.y + itr2->pt.y) / 2;

					// itr1->size = ((itr1->size / 2) + (distance / 2) + (itr2->size / 2) ) * 2;
					// Don't need to incorprate the distance in between if it is small enough
					itr1->size = (itr1->size > itr2->size) ? itr1->size : itr2->size;

					// Erase this object because we grouped it
					itr2 = currentObjects.erase(itr2);

					// Continue checking for objects to match
					continue;
				}
			}
			
			// Increment second-level iterator
			itr2++;
		}

		if (count > currentObjects.size())
			break;

		itr1 = currentObjects.begin() + count;
	}
	
	// Go through all currentObjects
	for (auto it = currentObjects.begin(); it != currentObjects.end(); ++it)
	{
		// Verify bigger than minimum size, otherwise completely ignore
		if (it->size >= m_visionParams.minWeedSize)
		{
			// If size is greater than our default crop size threshold
			if (it->size > m_visionParams.defaultCropThreshold)
			{
				// Mark as crop
				addToAccumulator(m_cropSizeAccumulator, it->size, m_visionParams.maxAccumulatorSize);
				// Add to output crop list
				outputCrops.push_back(*it);
			}
			// If size is less than the weed threshold
			else if (it->size < m_visionParams.defaultWeedThreshold)
			{
				// Identify as a weed!
				addToAccumulator(m_weedSizeAccumulator, it->size, m_visionParams.maxAccumulatorSize);
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
						// Add to output crop list
						outputCrops.push_back(*it);	
					}
					else
					{
						// Identify as a weed!
						addToAccumulator(m_weedSizeAccumulator, it->size, m_visionParams.maxAccumulatorSize);
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
		for (auto it = m_otsuAccumulator.begin(); it != m_otsuAccumulator.end(); ++it)
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

	return;
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