#ifndef PLANT_TRACKER_H
#define PLANT_TRACKER_H

#include <opencv2/core/utility.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/opencv.hpp>

#include <ctime>

using namespace std;
using namespace cv;

class PlantTracker
{
public:
	PlantTracker(int logging);
	~PlantTracker();

	int addToTracker(vector<KeyPoint> keypoints);
	vector<KeyPoint> getNewPlants();


	// All encompassing lastPlant
	vector<KeyPoint> m_lastPlants;
	// Specific weed vs. crop containers
	vector<KeyPoint> m_lastWeeds; 
	vector<KeyPoint> m_lastCrops;

	vector<KeyPoint> m_newPlants;

	float m_x_uncertainty;
	float m_r_uncertainty;

	float m_max_delta_y;
	float m_min_delta_y;

	time_t m_lastTime;
	int m_doLogging;
};

#endif PLANT_TRACKER_H


