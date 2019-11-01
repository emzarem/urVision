#include "opencv2/core/utility.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/imgcodecs.hpp"
#include "opencv2/highgui.hpp"

#include <stdio.h>

using namespace cv;
using namespace std;

// Constants
const float X_SCALE = 0.15;
const float Y_SCALE = 0.15;

const int MAX_SCHARR_R_VALUE = 800;

// Global variables
const int max_value = 255;
int low_b = 0, low_g = 125, low_r = 100;
int high_b = 85, high_g = max_value, high_r = 200;

int edgeThreshScharr = 500;

// Different image holders
Mat frame, scaledFrame, greenFrame, blurFrame, cannyEdges, edgeFrame;

const String window_capture_name = "Current Frame";
const String window_color_threshold_name = "Color Threshold";
const String window_edge_detection = "Edge Detection";

static void on_edge_thresh_trackbar(int, void *)
{
	setTrackbarPos("r", window_edge_detection, edgeThreshScharr);
}

static void on_low_b_thresh_trackbar(int, void *)
{
	low_b = min(high_b - 1, low_b);
	setTrackbarPos("Low b", window_color_threshold_name, low_b);
}
static void on_high_b_thresh_trackbar(int, void *)
{
	high_b = max(high_b, low_b + 1);
	setTrackbarPos("High b", window_color_threshold_name, high_b);
}
static void on_low_g_thresh_trackbar(int, void *)
{
	low_g = min(high_g - 1, low_g);
	setTrackbarPos("Low g", window_color_threshold_name, low_g);
}
static void on_high_g_thresh_trackbar(int, void *)
{
	high_g = max(high_g, low_g + 1);
	setTrackbarPos("High g", window_color_threshold_name, high_g);
}
static void on_low_r_thresh_trackbar(int, void *)
{
	low_r = min(high_r - 1, low_r);
	setTrackbarPos("Low r", window_color_threshold_name, low_r);
}
static void on_high_r_thresh_trackbar(int, void *)
{
	high_r = max(high_r, low_r + 1);
	setTrackbarPos("High r", window_color_threshold_name, high_r);
}

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
	help();
	CommandLineParser parser(argc, argv, keys);
	string filename = parser.get<string>(0);

	// Read in image as BGR
	frame = imread(samples::findFile(filename), IMREAD_COLOR);
	if (frame.empty())
	{
		printf("Cannot read image file: %s\n", filename.c_str());
		help();
		return -1;
	}

	// Resize image
	resize(frame, scaledFrame, Size(0, 0), X_SCALE, Y_SCALE, INTER_LINEAR);

	// Set up output images
	greenFrame.create(scaledFrame.size(), scaledFrame.type());

	// Create the windows
	namedWindow(window_capture_name);
	namedWindow(window_color_threshold_name);
	namedWindow(window_edge_detection);

	// Trackbars to set thresholds for rgb values
	createTrackbar("Low B", window_color_threshold_name, &low_b, max_value, on_low_b_thresh_trackbar);
	createTrackbar("High B", window_color_threshold_name, &high_b, max_value, on_high_b_thresh_trackbar);
	createTrackbar("Low G", window_color_threshold_name, &low_g, max_value, on_low_g_thresh_trackbar);
	createTrackbar("High G", window_color_threshold_name, &high_g, max_value, on_high_g_thresh_trackbar);
	createTrackbar("Low R", window_color_threshold_name, &low_r, max_value, on_low_r_thresh_trackbar);
	createTrackbar("High R", window_color_threshold_name, &high_r, max_value, on_high_r_thresh_trackbar);
    createTrackbar("Canny threshold Scharr", window_edge_detection, &edgeThreshScharr, MAX_SCHARR_R_VALUE, on_edge_thresh_trackbar);

	// Show the frames
	imshow(window_capture_name, scaledFrame);
	
	// Used for edge detection
	Mat dx, dy;

	while (true)
	{
		// Color thresholding
		inRange(scaledFrame, Scalar(low_b, low_g, low_r), Scalar(high_b, high_g, high_r), greenFrame);
		//blur(greenFrame, greenFrame, Size(3, 3));
		imshow(window_color_threshold_name, greenFrame);

		//// Edge detection
		// Blur with kernel size 3x3
		blur(greenFrame, blurFrame, Size(3,3));
		// Canny detector with Scharr
    	Scharr(blurFrame,dx,CV_16S,1,0);
    	Scharr(blurFrame,dy,CV_16S,0,1);
   		Canny( dx,dy, cannyEdges, edgeThreshScharr, edgeThreshScharr*3 );
    	/// Using Canny's output as a mask, we display our result
    	edgeFrame = Scalar::all(0);
		greenFrame.copyTo(edgeFrame, cannyEdges);
    	imshow(window_edge_detection, edgeFrame);

		char key = (char)waitKey(30);
		if (key == 'q' || key == 27)
		{
			break;
		}
	}
}
