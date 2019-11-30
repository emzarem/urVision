#include "../include/PlantDetector.h"

const String window_capture_name = "Current Frame";
const String window_color_threshold_name = "Color Threshold";
const String window_blob_detection = "Blob Detection";
const String window_test = "Window Test";
const String window_morphs = "Morphological Output";

// Some constants
const int max_value_H = 360 / 2;
const int max_value = 255;

const int morph_type = MORPH_ELLIPSE;
const int max_kernel_size = 21;

// For erosions and dilations
int morph_size = 2;
int morph_opening_iterations = 1;
int morph_closing_iterations = 10;

// HSV thresholding
int low_H = 30, low_S = 95, low_V = 81;
int high_H = 90, high_S = max_value, high_V = max_value;

// Trackbar callback prototypes
static void on_edge_thresh1_trackbar(int, void *);
static void on_low_H_thresh_trackbar(int, void *);
static void on_high_H_thresh_trackbar(int, void *);
static void on_low_S_thresh_trackbar(int, void *);
static void on_high_S_thresh_trackbar(int, void *);
static void on_low_V_thresh_trackbar(int, void *);
static void on_high_V_thresh_trackbar(int, void *);

PlantDetector::PlantDetector(int showWindows) : 
	m_showWindows(showWindows), m_inited(false), m_plantFilter(NULL)
{
}

PlantDetector::~PlantDetector()
{
}

int PlantDetector::init(VisionParams visionParams)
{
	m_visionParams = visionParams;

	low_H = m_visionParams.lowH;	
	low_S = m_visionParams.lowS;	
	low_V = m_visionParams.lowV;	
	high_H = m_visionParams.highH;	
	morph_size = m_visionParams.morphSize;	
	morph_opening_iterations = m_visionParams.morphOpeningIters;	
	morph_closing_iterations = m_visionParams.morphClosingIters;	

	// If proper frame size not provided
	if (visionParams.frameSize == Size(0,0)) {
		return false;
	}
	// Initialize plant filter
	m_plantFilter = new PlantFilter(m_visionParams);

	if (NULL == m_plantFilter) {
		return false;
	}

	/* Initialize blob parameters */
	// Change thresholds
	m_blobParams.minThreshold = 0;
	m_blobParams.maxThreshold = 255;
	// Not filtering by color
	m_blobParams.filterByColor = false;

	// Filter by Area (TODO: EXPERIMENT WITH THIS)
	m_blobParams.filterByArea = false;
	m_blobParams.minArea = m_visionParams.minWeedSize;
	m_blobParams.maxArea = m_visionParams.frameSize.width;

	// Filter by circularity
	m_blobParams.filterByCircularity = true;
	m_blobParams.minCircularity = m_visionParams.minCircularity;
	m_blobParams.maxCircularity = 1;

	// Filter by Convexity
	m_blobParams.filterByConvexity = true;
	m_blobParams.minConvexity = m_visionParams.minConvexity;
	m_blobParams.maxConvexity = 1;

	// Filter by inertia
	m_blobParams.filterByInertia = true;
	m_blobParams.minInertiaRatio = m_visionParams.minInertiaRatio;
	m_blobParams.maxInertiaRatio  = 1;

	// Create the blob detector!
	m_blobDetector = SimpleBlobDetector::create(m_blobParams);

	if (m_showWindows)
	{
		// Create the windows
		namedWindow(window_color_threshold_name);
		namedWindow(window_blob_detection);
		namedWindow(window_morphs);
		namedWindow(window_test);

		// Trackbars to set thresholds for rgb values
		createTrackbar("Low H", window_color_threshold_name, &low_H, max_value, on_low_H_thresh_trackbar);
		createTrackbar("High H", window_color_threshold_name, &high_H, max_value, on_high_H_thresh_trackbar);
		createTrackbar("Low S", window_color_threshold_name, &low_S, max_value, on_low_S_thresh_trackbar);
		createTrackbar("High S", window_color_threshold_name, &high_S, max_value, on_high_S_thresh_trackbar);
		createTrackbar("Low V", window_color_threshold_name, &low_V, max_value, on_low_V_thresh_trackbar);
		createTrackbar("High V", window_color_threshold_name, &high_V, max_value, on_high_V_thresh_trackbar);
		createTrackbar("Kernel size (morphs):\n 2n +1", window_morphs,
			&morph_size, max_kernel_size,
			0);
	}

	m_inited = true;

	return true;
}

/* Main Processing Pipeline */
int PlantDetector::processFrame(Mat& frame)
{
	if (!m_inited) {
		return false;
	}

	// Blur the image
	cv::blur(frame, blurFrame, Size(m_visionParams.blurSize, m_visionParams.blurSize));

	// Convert from BGR to HSV colorspace
	cv::cvtColor(blurFrame, hsvFrame, COLOR_BGR2HSV);

	/* HSV thresholding
	*	Returns a binary colorMask
	*/
	inRange(hsvFrame, Scalar(low_H, low_S, low_V), Scalar(high_H, high_S, high_V), colorMask);

	// Performing morphological on the colorMask of frame
	morphFrame = colorMask;

		// Erode/Dilate (use the green thresholded image)
	Mat morphElement = getStructuringElement(morph_type,
		Size(2 * morph_size + 1, 2 * morph_size + 1),
		Point(morph_size, morph_size));

	// Do some number of morph openings
	erode(morphFrame, morphFrame, morphElement, Point(-1, -1), morph_opening_iterations);
	dilate(morphFrame, morphFrame, morphElement, Point(-1, -1), morph_opening_iterations);

	// Do some number of morph closings
	dilate(morphFrame, morphFrame, morphElement, Point(-1, -1), morph_closing_iterations);
	erode(morphFrame, morphFrame, morphElement, Point(-1, -1), morph_closing_iterations);

	// Copy original frame to greenFrame with the colorMask
	greenFrame = Scalar::all(0);
	blurFrame.copyTo(greenFrame, morphFrame);

	// Shape Processing (BLOB detection)
	vector<KeyPoint> detectedBlobs;
	m_blobDetector->detect(morphFrame, detectedBlobs);

	// Save latest objects detected
	m_lastObjectsFound = detectedBlobs;

	// Filter weeds from this object list
	m_weedList.clear();
	m_weedList = m_plantFilter->filterWeeds(m_lastObjectsFound);

	// If we are showing windows ...
	if (m_showWindows)
	{
		// Show color thresholded image
		imshow(window_color_threshold_name, greenFrame);
		// Show morphological output
		imshow(window_morphs, morphFrame);
		// Draw detected blobs as red circles.
		Mat im_with_keypoints;
		drawKeypoints(morphFrame, detectedBlobs, im_with_keypoints, Scalar(0, 0, 255), DrawMatchesFlags::DRAW_RICH_KEYPOINTS);
		imshow(window_blob_detection, im_with_keypoints);
		// Now draw only with filtered weeds
		drawKeypoints(morphFrame, m_weedList, im_with_keypoints, Scalar(0, 0, 255), DrawMatchesFlags::DRAW_RICH_KEYPOINTS);
		imshow(window_test, im_with_keypoints);

		cv::waitKey(3);
	}

	return true;
}

vector<KeyPoint> PlantDetector::getWeedList()
{
	return m_weedList;
}

float PlantDetector::getWeedThreshold()
{
	return m_plantFilter->m_otsuThreshold;
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