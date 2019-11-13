#include "..\include\PlantDetector.h"


// Canny edge detection 
int edgeThresh1 = 100;

// Erosions and dilations =
int erosion_size = 2;
int dilation_size = 10;

// HSV thresholding
int low_H = 32, low_S = 95, low_V = 81;
int high_H = 87, high_S = max_value, high_V = max_value;

static void on_edge_thresh1_trackbar(int, void *);
static void on_low_H_thresh_trackbar(int, void *);
static void on_high_H_thresh_trackbar(int, void *);
static void on_low_S_thresh_trackbar(int, void *);
static void on_high_S_thresh_trackbar(int, void *);
static void on_low_V_thresh_trackbar(int, void *);
static void on_high_V_thresh_trackbar(int, void *);


PlantDetector::PlantDetector(int showWindows) : m_showWindows(showWindows), m_plantTracker(NULL)
{
}

PlantDetector::PlantDetector(int showWindows, PlantTracker* tracker) : m_showWindows(showWindows), m_plantTracker(tracker)
{
}

PlantDetector::~PlantDetector()
{
}

int PlantDetector::init()
{
	if (m_showWindows)
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
	}
	else
	{
		printf("PlantDetector -- specified not to show windows, not initializing windows\n");
	}

	return true;
}

/* Main Processing Pipeline */
int PlantDetector::processFrame(Mat frame)
{
	// Convert from BGR to HSV colorspace
	cvtColor(frame, hsvFrame, COLOR_BGR2HSV);

	colorMask = ColorThresholding(hsvFrame);
	// Copy original frame to greenFrame with the colorMask
	greenFrame = Scalar::all(0);
	hsvFrame.copyTo(greenFrame, colorMask);

	// Erode/Dilate (use the green thresholded image)
	erosion_dst = Erosion(colorMask);
	dilation_dst = Dilation(erosion_dst);

	//// Edge detection
	cannyEdges = GetEdges(dilation_dst);
	// We now have a binary mask of edges

	// Shape Processing (Blob detection)
	vector<KeyPoint> detectedBlobs = DetectBlobs(cannyEdges);

	// Save latest objects detected
	m_lastObjectsFound = detectedBlobs;


	// If we are attached to a plantTracker
	if (NULL != m_plantTracker)
	{
		m_plantTracker->addToTracker(detectedBlobs);
	}

	// If we are showing windows ...
	if (m_showWindows)
	{
		// Show the original, scaled frame
		imshow(window_capture_name, hsvFrame);
		imshow(window_color_threshold_name, greenFrame);
		imshow(window_erosions, erosion_dst);
		imshow(window_dilations, dilation_dst);
		imshow(window_edge_detection, cannyEdges);
		// Draw detected blobs as red circles.
		// DrawMatchesFlags::DRAW_RICH_KEYPOINTS flag ensures
		// the size of the circle corresponds to the size of blob
		Mat im_with_keypoints;
		drawKeypoints(cannyEdges, detectedBlobs, im_with_keypoints, Scalar(0, 0, 255), DrawMatchesFlags::DRAW_RICH_KEYPOINTS);
		// Show blobs
		imshow(window_test, im_with_keypoints);
	}

	return true;
}

vector<KeyPoint> PlantDetector::getLastObjectsFound()
{
	return m_lastObjectsFound;
}

/* HSV thresholding
*	Returns a binary colorMask
*/
Mat PlantDetector::ColorThresholding(Mat srcFrame)
{
	Mat outputMask;
	// Color thresholding
	inRange(srcFrame, Scalar(low_H, low_S, low_V), Scalar(high_H, high_S, high_V), outputMask);
	return outputMask;
}

/* Edge Detection  */
Mat PlantDetector::GetEdges(Mat srcFrame)
{
	static Mat outputFrame;
	// Blur input to edge detection
	cv::GaussianBlur(srcFrame, outputFrame, cv::Size(7, 7), 2, 2);
	// Canny detector
	Canny(outputFrame, outputFrame, edgeThresh1, edgeThresh1 * 3, 3);
	return outputFrame;
}

/* erosions  */
Mat PlantDetector::Erosion(Mat srcFrame)
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
Mat PlantDetector::Dilation(Mat srcFrame)
{
	static Mat outputFrame;
	Mat element = getStructuringElement(dilation_type,
		Size(2 * dilation_size + 1, 2 * dilation_size + 1),
		Point(dilation_size, dilation_size));
	/// Apply the dilation operation
	dilate(srcFrame, outputFrame, element);
	return outputFrame;
}

vector<KeyPoint> PlantDetector::DetectBlobs(Mat srcFrame)
{
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

	return keypoints;
}

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