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
	int processFrame(Mat& frame);

	vector<KeyPoint> getWeedList();
	vector<KeyPoint> getCropList();

	float getWeedThreshold();

	// Various image holders
	Mat scaledFrame;
	Mat greenFrame;
	Mat blurFrame;
	Mat colorMask;
	Mat hsvFrame;
	Mat morphFrame;

private:
	vector<KeyPoint> DetectBlobs(Mat& srcFrame);

	vector<KeyPoint> m_weedList;
	vector<KeyPoint> m_cropList;

	int m_showWindows;
	int m_inited;

	PlantFilter* m_plantFilter;

	// Blob detector parameters (dynamic)
	SimpleBlobDetector::Params m_blobParams;
	Ptr<SimpleBlobDetector> m_blobDetector;

	VisionParams m_visionParams;
};

#endif
