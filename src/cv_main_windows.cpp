#include "PlantDetector.h"

using namespace std;
using namespace cv;

// Parameters (to get from ROS)
// Weed filtering
const float DEFAULT_MAX_WEED_SIZE = 1000;
const float DEFAULT_MIN_WEED_SIZE = 0;

// Constants
const float X_SCALE = (float)0.15;
const float Y_SCALE = (float)0.15;

// forward velocity in m/s
const int SHOW_WINDOWS = true;

static void help()
{
	printf("\nThis sample demonstrates rowDetect\n"
		"Call:\n"
		"    /.rowDetect [image_name -- Default is fruits.jpg]\n\n");
}

const char* keys =
{
	"{help h||}{@image |fruits.jpg|input image name}"
};

int main(int argc, const char** argv)
{	
	Mat origFrame, currentFrame;

	help();
	CommandLineParser parser(argc, argv, keys);
	string filename = parser.get<string>(0);

	// Read in image as BGR
	origFrame = imread(filename, IMREAD_COLOR);
	if (origFrame.empty())
	{
		printf("Cannot read image file: %s\n", filename.c_str());
		help();
		return -1;
	}
	
	// Resize image
	resize(origFrame, currentFrame, Size(0, 0), X_SCALE, Y_SCALE, INTER_LINEAR);
	
	Size frameSize = currentFrame.size();

	// Create detector attached to tracker
	PlantDetector detector(SHOW_WINDOWS, frameSize);

	// Init windows for testing
	if (!detector.init(DEFAULT_MIN_WEED_SIZE, DEFAULT_MAX_WEED_SIZE))
	{
		printf("Error initializing Plant Detector\n");
		return -1;
	}
	
	for (int frameNum = 0; true ;frameNum++)
	{
		// Operate on origFrame for now
		currentFrame = origFrame;

		// Resize image
		resize(currentFrame, currentFrame, Size(0, 0), X_SCALE, Y_SCALE, INTER_LINEAR);

		// Do processing on this frame
		if (!detector.processFrame(currentFrame))
		{
			printf("Error processing frame: %d\n", frameNum);
			return -1;
		}

		vector<KeyPoint> weedList = detector.getWeedList();
		for (vector<KeyPoint>::iterator it = weedList.begin(); it != weedList.end(); ++it)
		{
			printf("Found weed at (x,y,size) : (%f, %f, %f)\n", it->pt.x, it->pt.y, it->size);
		}

		char key = (char)waitKey(30);
		if (key == 'q' || key == 27)
		{
			break;
		}
	}
	return 0;
}
