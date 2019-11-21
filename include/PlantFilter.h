#ifndef PLANT_FILTER_H
#define PLANT_FILTER_H

#include <opencv2/core/utility.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/opencv.hpp>

using namespace std;
using namespace cv;

class VisionParams
{
public:
	VisionParams() {
		frameSize = Size(0,0);
		defaultWeedThreshold = -1;
		minWeedSize = 0; 
		maxWeedSize = 0;
		minAccumulatorSize = 0;
		maxAccumulatorSize = 0;
	}
	~VisionParams(){

	}

	cv::Size frameSize;
	float defaultWeedThreshold;
	float minWeedSize;
	float maxWeedSize;

	int minAccumulatorSize;
	int maxAccumulatorSize;

	// TODO: implement this one
	int otsuHistogramsize;

	int lowH;
	int lowS;
	int lowV;
	int highH;
	int morphSize;
	int morphClosingIters;
};

class PlantFilter
{
public:
	PlantFilter(VisionParams visionParams);

	~PlantFilter();

	vector<KeyPoint> filterWeeds(vector<KeyPoint> currentPlants);

	VisionParams m_visionParams;
	float m_otsuThreshold;

	int m_maxSize;

	vector<float> m_weedXAccumulator;
	vector<float> m_cropXAccumulator;
	
	vector<float> m_weedSizeAccumulator;
	vector<float> m_cropSizeAccumulator;

	vector<float> m_otsuAccumulator;

	float m_cropSizeMean;
	float m_cropSizeStdDev;

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