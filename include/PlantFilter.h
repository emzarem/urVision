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
		defaultCropThreshold = -1;
		minWeedSize = 0; 
		maxWeedSize = 0;
		minAccumulatorSize = 0;
		maxAccumulatorSize = 0;
		blurSize = 0;
		filterDistanceTol = 0;
	}
	~VisionParams(){

	}

	cv::Size frameSize;
	float defaultWeedThreshold;
	float defaultCropThreshold;

	float filterDistanceTol;
	
	float minWeedSize;
	float maxWeedSize;

	int minAccumulatorSize;
	int maxAccumulatorSize;

	// TODO: implement this one
	int otsuHistogramsize;

	int blurSize;
	int lowH;
	int lowS;
	int lowV;
	int highH;
	int morphSize;
	int morphOpeningIters;
	int morphClosingIters;

	float minCircularity;
	float minConvexity;
	float minInertiaRatio;
};

class PlantFilter
{
public:
	PlantFilter(VisionParams visionParams);

	~PlantFilter();

	// Main filtering function
	void filter(vector<KeyPoint>& currentObjects, vector<KeyPoint>& outputWeeds, vector<KeyPoint>& outputCrops);

	VisionParams m_visionParams;
	float m_otsuThreshold;

	int m_maxSize;
	float m_distanceTol;
	
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