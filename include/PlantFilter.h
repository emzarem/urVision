#ifndef PLANT_FILTER_H
#define PLANT_FILTER_H

#include <opencv2/core/utility.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/opencv.hpp>

using namespace std;
using namespace cv;

const int DEFAULT_ACCUMULATOR_LENGTH = 10;
const int DEFAULT_HISTOGRAM_SIZE = 30;

class PlantFilter
{
public:
	PlantFilter(Size frameSize, float minWeedSize, float maxWeedSize);

	~PlantFilter();

	vector<KeyPoint> filterWeeds(vector<KeyPoint> currentPlants);

	float m_maxWeedSize;
	float m_minWeedSize;

	vector<float> m_weedXAccumulator;
	vector<float> m_cropXAccumulator;
	
	vector<float> m_weedSizeAccumulator;
	vector<float> m_cropSizeAccumulator;

	float m_cropSizeMean;
	float m_cropSizeStdDev;

	Size m_frameSize;

	vector<uint8_t> m_otsuAccumulator;
	float m_otsuThreshold;

private:
	template<typename T>
	void pop_front(std::vector<T> &v);

	template<typename T>
	void addToAccumulator(std::vector<T> &v, T val, int maxLength);

	template<typename T>
	T getMean(std::vector<T> &v);

	template<typename T>
	T getStdDev(std::vector<T> &v);
};

#endif // PLANT_FILTER_H