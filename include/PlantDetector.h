#ifndef PLANT_DETECTOR_H
#define PLANT_DETECTOR_H

#include "../include/PlantFilter.h"

#include <opencv2/core/utility.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/opencv.hpp>

using namespace cv;
using namespace std;

class PlantDetector
{
public:
	PlantDetector(int showWindows);
	~PlantDetector();

	int init(VisionParams visionParams);
	int processFrame(Mat frame);

	vector<KeyPoint> getWeedList();
	float getWeedThreshold();

	vector<KeyPoint> m_lastObjectsFound;
	vector<KeyPoint> m_weedList;
	vector<KeyPoint> m_cropList;

	int m_showWindows;
	int m_inited;

	PlantFilter* m_plantFilter;

	// Blob detector parameters (dynamic)
	SimpleBlobDetector::Params m_blobParams;

private:
	vector<KeyPoint> DetectBlobs(Mat srcFrame);
	Mat ColorThresholding(Mat srcFrame);

	// Various image holders
	Mat scaledFrame;
	Mat greenFrame;
	Mat blurFrame;
	Mat colorMask;
	Mat hsvFrame;
	Mat morphFrame;
};

#endif
