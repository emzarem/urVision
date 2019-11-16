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

class PlantFilter
{
public:
	PlantFilter(float minWeedSize, float maxWeedSize);

	~PlantFilter();

	vector<KeyPoint> filterWeeds(vector<KeyPoint> currentPlants);

	float m_maxWeedSize;
	float m_minWeedSize;

	vector<float> m_weedXAccumulator;
	vector<float> m_cropXAccumulator;
	
	vector<float> m_weedSizeAccumulator;
	vector<float> m_cropSizeAccumulator;
	vector<float> m_allSizeAccumulator;

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