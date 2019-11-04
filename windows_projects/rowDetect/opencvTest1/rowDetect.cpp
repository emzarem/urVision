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

const int MAX_EDGE_THRESHOLD = 100;
const int max_value_H = 360 / 2;
const int max_value = 255;

// Global variables
int low_H = 32, low_S = 95, low_V = 81;
int high_H = 87, high_S = max_value, high_V = max_value;

// For inRange Thresholding
int lowerThreshold = 128;

// Erosion, dilation variables
Mat erosion_src, dilation_src, erosion_dst, dilation_dst;

int edgeThresh1 = 100;
int edgeThresh2 = 3;

int erosion_elem = 2;
int erosion_size = 2;
int dilation_elem = 2;
int dilation_size = 9;

int const max_elem = 2;
int const max_kernel_size = 21;

// Different image holders
Mat frame, scaledFrame, colorMask, greenFrame, blurFrame, cannyEdges, edgeFrame, thresholdFrame, circlesFrame;

const String window_capture_name = "Current Frame";
const String window_color_threshold_name = "Color Threshold";
const String window_edge_detection = "Edge Detection";
const String window_test = "Window Test";
const String window_erosions = "Erosions Test";
const String window_dilations = "Dilations Test";

/* Function Prototypes */
void Erosion(int, void*);
void Dilation(int, void*);

/* Function Definitions */
static void on_edge_thresh1_trackbar(int, void *)
{
	setTrackbarPos("thresh1", window_edge_detection, edgeThresh1);
}

static void on_low_H_thresh_trackbar(int, void *)
{
	low_H = min(high_H - 1, low_H);
	setTrackbarPos("Low H", window_color_threshold_name, low_H);
}
static void on_high_H_thresh_trackbar(int, void *)
{
	high_H = max(high_H, low_H + 1);
	setTrackbarPos("High H", window_color_threshold_name, high_H);
}
static void on_low_S_thresh_trackbar(int, void *)
{
	low_S = min(high_S - 1, low_S);
	setTrackbarPos("Low S", window_color_threshold_name, low_S);
}
static void on_high_S_thresh_trackbar(int, void *)
{
	high_S = max(high_S, low_S + 1);
	setTrackbarPos("High S", window_color_threshold_name, high_S);
}
static void on_low_V_thresh_trackbar(int, void *)
{
	low_V = min(high_V - 1, low_V);
	setTrackbarPos("Low V", window_color_threshold_name, low_V);
}
static void on_high_V_thresh_trackbar(int, void *)
{
	high_V = max(high_V, low_V + 1);
	setTrackbarPos("High V", window_color_threshold_name, high_V);
}

/* erosions  */
void Erosion(int, void*)
{
	int erosion_type;
	if (erosion_elem == 0) { erosion_type = MORPH_RECT; }
	else if (erosion_elem == 1) { erosion_type = MORPH_CROSS; }
	else if (erosion_elem == 2) { erosion_type = MORPH_ELLIPSE; }

	Mat element = getStructuringElement(erosion_type,
		Size(2 * erosion_size + 1, 2 * erosion_size + 1),
		Point(erosion_size, erosion_size));

	/// Apply the erosion operation
	erode(erosion_src, erosion_dst, element);
	imshow(window_erosions, erosion_dst);
}

/* dilations  */
void Dilation(int, void*)
{
	int dilation_type;
	if (dilation_elem == 0) { dilation_type = MORPH_RECT; }
	else if (dilation_elem == 1) { dilation_type = MORPH_CROSS; }
	else if (dilation_elem == 2) { dilation_type = MORPH_ELLIPSE; }

	Mat element = getStructuringElement(dilation_type,
		Size(2 * dilation_size + 1, 2 * dilation_size + 1),
		Point(dilation_size, dilation_size));
	/// Apply the dilation operation
	dilate(dilation_src, dilation_dst, element);
	imshow(window_dilations, dilation_dst);
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

	// Convert from BGR to HSV colorspace	
	cvtColor(frame, frame, COLOR_BGR2HSV);
	// Resize image
	resize(frame, scaledFrame, Size(0, 0), X_SCALE, Y_SCALE, INTER_LINEAR);

	// Create the windows
	namedWindow(window_capture_name);
	namedWindow(window_color_threshold_name);
	namedWindow(window_edge_detection);

	/// Create windows
	namedWindow(window_erosions);
	namedWindow(window_dilations);

	namedWindow(window_test);

	// Trackbars to set thresholds for rgb values
	createTrackbar("Low H", window_color_threshold_name, &low_H, max_value, on_low_H_thresh_trackbar);
	createTrackbar("High H", window_color_threshold_name, &high_H, max_value, on_high_H_thresh_trackbar);
	createTrackbar("Low S", window_color_threshold_name, &low_S, max_value, on_low_S_thresh_trackbar);
	createTrackbar("High S", window_color_threshold_name, &high_S, max_value, on_high_S_thresh_trackbar);
	createTrackbar("Low V", window_color_threshold_name, &low_V, max_value, on_low_V_thresh_trackbar);
	createTrackbar("High V", window_color_threshold_name, &high_V, max_value, on_high_V_thresh_trackbar);
    createTrackbar("Canny threshold1 Scharr", window_edge_detection, &edgeThresh1, MAX_EDGE_THRESHOLD, on_edge_thresh1_trackbar);

	/// Create Erosion Trackbar
	createTrackbar("Element:\n 0: Rect \n 1: Cross \n 2: Ellipse", window_erosions,
		&erosion_elem, max_elem,
		Erosion);
	createTrackbar("Kernel size:\n 2n +1", window_erosions,
		&erosion_size, max_kernel_size,
		Erosion);

	/// Create Dilation Trackbar
	createTrackbar("Element:\n 0: Rect \n 1: Cross \n 2: Ellipse", window_dilations,
		&dilation_elem, max_elem,
		Dilation);
	createTrackbar("Kernel size:\n 2n +1", window_dilations,
		&dilation_size, max_kernel_size,
		Dilation);

	// Show the original, scaled frame
	imshow(window_capture_name, scaledFrame);

	// Used for edge detection
	Mat dx, dy;

	while (true)
	{
		// Color thresholding
		inRange(scaledFrame, Scalar(low_H, low_S, low_V), Scalar(high_H, high_S, high_V), colorMask);
		greenFrame = Scalar::all(0);
		// Copy original frame to greenFrame with the colorMask
		scaledFrame.copyTo(greenFrame, colorMask);
		//blur(greenFrame, greenFrame, Size(3, 3));
		imshow(window_color_threshold_name, greenFrame);

		// Erode/Dilate (use the green thresholded image)
		erosion_src = colorMask;
		Erosion(0, 0);
		dilation_src = erosion_dst;
		Dilation(0, 0);

		//// Edge detection
		// Blur input to edge detection
		cv::GaussianBlur(dilation_dst, blurFrame, cv::Size(7, 7), 2, 2);
		// Canny detector
		Canny(blurFrame, cannyEdges, edgeThresh1, edgeThresh1*3, 3);
		// We now have a binary mask of edges
		imshow(window_edge_detection, cannyEdges);

		// find contours
		vector<vector<Point> > contours;
		vector<Vec4i> hierarchy;
		findContours(cannyEdges, contours, hierarchy, RETR_TREE, CHAIN_APPROX_SIMPLE, Point(0, 0));

		// get the moments
		vector<Moments> mu(contours.size());
		for (int i = 0; i<contours.size(); i++)
		{
			mu[i] = moments(contours[i], false);
		}

		// get the centroid of figures.
		vector<Point2f> mc(contours.size());
		for (int i = 0; i<contours.size(); i++)
		{
			mc[i] = Point2f(mu[i].m10 / mu[i].m00, mu[i].m01 / mu[i].m00);
		}


		// draw contours
		Mat drawing(cannyEdges.size(), CV_8UC3, Scalar(255, 255, 255));
		for (int i = 0; i<contours.size(); i++)
		{
			Scalar color = Scalar(167, 151, 0); // B G R values
			circle(drawing, mc[i], 4, color, -1, 8, 0);
		}

		imshow(window_test, drawing);

		char key = (char)waitKey(30);
		if (key == 'q' || key == 27)
		{
			break;
		}
	}
}