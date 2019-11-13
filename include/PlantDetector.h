#ifndef PLANT_DETECTOR_H
#define PLANT_DETECTOR_H

#include "../include/PlantTracker.h"
#include <opencv2/core/utility.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/opencv.hpp>

using namespace cv;
using namespace std;

// Some constants
const int MAX_EDGE_THRESHOLD = 100;
const int max_value_H = 360 / 2;
const int max_value = 255;

const int erosion_type = MORPH_ELLIPSE;
const int dilation_type = MORPH_ELLIPSE;


const String window_capture_name = "Current Frame";
const String window_color_threshold_name = "Color Threshold";
const String window_edge_detection = "Edge Detection";
const String window_test = "Window Test";
const String window_erosions = "Erosions Test";
const String window_dilations = "Dilations Test";

class PlantDetector
{
public:
	PlantDetector(int showWindows);
	PlantDetector(int showWindows, PlantTracker* tracker);
	~PlantDetector();

	int init();
	int processFrame(Mat frame);

	vector<KeyPoint> getLastObjectsFound();

	vector<KeyPoint> m_lastObjectsFound;
	PlantTracker* m_plantTracker;
	int m_showWindows;

private:
	vector<KeyPoint> DetectBlobs(Mat srcFrame);
	Mat ColorThresholding(Mat srcFrame);
	Mat GetEdges(Mat srcFrame);
	Mat Erosion(Mat srcFrame);
	Mat Dilation(Mat srcFrame);

	// Various image holders
	Mat scaledFrame;
	Mat greenFrame;
	Mat blurFrame;
	Mat cannyEdges;
	Mat colorMask;
	Mat hsvFrame;
	Mat erosion_dst;
	Mat dilation_dst;

	const int max_kernel_size = 21;
};

#endif PLANT_DETECTOR_H
