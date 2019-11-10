#include <PlantDetect.h>

using namespace cv;
using namespace std;

// Constants
const float X_SCALE = 0.15;
const float Y_SCALE = 0.15;

const int MAX_EDGE_THRESHOLD = 100;
const int max_value_H = 360 / 2;
const int max_value = 255;

const int erosion_type = MORPH_ELLIPSE;
const int dilation_type = MORPH_ELLIPSE;

// Globals

// HSV thresholding
int low_H = 32, low_S = 95, low_V = 81;
int high_H = 87, high_S = max_value, high_V = max_value;

// Canny edge detection 
int edgeThresh1 = 100;

int erosion_size = 2;
int dilation_size = 10;

int const max_elem = 2;
int const max_kernel_size = 21;

const String window_capture_name = "Current Frame";
const String window_color_threshold_name = "Color Threshold";
const String window_edge_detection = "Edge Detection";
const String window_test = "Window Test";
const String window_erosions = "Erosions Test";
const String window_dilations = "Dilations Test";

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

/* HSV thresholding 
*	Returns a binary colorMask
*/
Mat ColorThresholding(Mat srcFrame)
{
	Mat outputMask;
	// Color thresholding
	inRange(srcFrame, Scalar(low_H, low_S, low_V), Scalar(high_H, high_S, high_V), outputMask);
	return outputMask;
}

/* Edge Detection  */
Mat GetEdges(Mat srcFrame)
{
	static Mat outputFrame;
	// Blur input to edge detection
	cv::GaussianBlur(srcFrame, outputFrame, cv::Size(7, 7), 2, 2);
	// Canny detector
	Canny(outputFrame, outputFrame, edgeThresh1, edgeThresh1 * 3, 3);
	return outputFrame;
}

/* erosions  */
Mat Erosion(Mat srcFrame)
{
	static Mat outputFrame;
	Mat element = getStructuringElement(erosion_type,
		Size(2 * erosion_size + 1, 2 * erosion_size + 1),
		Point(erosion_size, erosion_size));

	/// Apply the erosion operation
	erode(srcFrame, outputFrame, element);
	return outputFrame;
}

/* dilations  */
Mat Dilation(Mat srcFrame)
{
	static Mat outputFrame;
	Mat element = getStructuringElement(dilation_type,
		Size(2 * dilation_size + 1, 2 * dilation_size + 1),
		Point(dilation_size, dilation_size));
	/// Apply the dilation operation
	dilate(srcFrame, outputFrame, element);
	return outputFrame;
}

int InitTestWindows(void)
{
	// Create the windows
	namedWindow(window_capture_name);
	namedWindow(window_color_threshold_name);
	namedWindow(window_edge_detection);
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
	createTrackbar("Kernel size:\n 2n +1", window_erosions,
		&erosion_size, max_kernel_size,
		0);
	createTrackbar("Kernel size:\n 2n +1", window_dilations,
		&dilation_size, max_kernel_size,
		0);

	return true;
}

/* Main Processing Pipeline */
int ProcessFrame(Mat frame)
{
	// Different image holders
	Mat scaledFrame, greenFrame, blurFrame, cannyEdges, colorMask, hsvFrame,
			erosion_src, dilation_src, erosion_dst, dilation_dst;

	// Convert from BGR to HSV colorspace
	cvtColor(frame, hsvFrame, COLOR_BGR2HSV);
	
	// Resize image
	resize(hsvFrame, scaledFrame, Size(0, 0), X_SCALE, Y_SCALE, INTER_LINEAR);

	// Show the original, scaled frame
	imshow(window_capture_name, scaledFrame);

	colorMask = ColorThresholding(scaledFrame);
	// Copy original frame to greenFrame with the colorMask
	greenFrame = Scalar::all(0);
	scaledFrame.copyTo(greenFrame, colorMask);
	imshow(window_color_threshold_name, greenFrame);

	// Erode/Dilate (use the green thresholded image)
	erosion_dst = Erosion(colorMask);
	imshow(window_erosions, erosion_dst);
	dilation_dst = Dilation(erosion_dst);
	imshow(window_dilations, dilation_dst);

	//// Edge detection
	cannyEdges = GetEdges(dilation_dst);
	// We now have a binary mask of edges
	imshow(window_edge_detection, cannyEdges);

	// Shape Processing

	// Setup SimpleBlobDetector parameters.
	SimpleBlobDetector::Params params;

	// Change thresholds
	params.minThreshold = 10;
	params.maxThreshold = 255;

	params.filterByColor = false;

	// Filter by Area.
	params.filterByArea = false;
	params.minArea = 1000;

	// Filter by Circularity
	params.filterByCircularity = false;

	// Filter by Convexity
	params.filterByConvexity = false;
	params.minConvexity = 0.87;

	// Filter by Inertia
	params.filterByInertia = true;
	params.minInertiaRatio = 0.01;

	// Storage for blobs
	vector<KeyPoint> keypoints;

	// Set up detector with params
	Ptr<SimpleBlobDetector> detector = SimpleBlobDetector::create(params);

	// Detect blobs
	detector->detect(cannyEdges, keypoints);

	// Draw detected blobs as red circles.
	// DrawMatchesFlags::DRAW_RICH_KEYPOINTS flag ensures
	// the size of the circle corresponds to the size of blob
	Mat im_with_keypoints;
	drawKeypoints(cannyEdges, keypoints, im_with_keypoints, Scalar(0, 0, 255), DrawMatchesFlags::DRAW_RICH_KEYPOINTS);

	// Show blobs
	imshow(window_test, im_with_keypoints);

#if 0
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

#endif

	return true;
}