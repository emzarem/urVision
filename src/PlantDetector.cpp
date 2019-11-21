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
	// If proper frame size not provided
	if (visionParams.frameSize == Size(0,0)) {
		return false;
	}
	// Initialize plant filter
	m_plantFilter = new PlantFilter(visionParams);

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
	m_blobParams.minArea = visionParams.minWeedSize;
	m_blobParams.maxArea = visionParams.frameSize.width;

	// Filter by circularity ?
	m_blobParams.filterByCircularity = false;

	// Filter by Convexity
	m_blobParams.filterByConvexity = false;
	// m_blobParams.minConvexity = 0.87;

	// Filter by Inertia (TODO: ADJUST THIS)
	m_blobParams.filterByInertia = false;
	m_blobParams.minInertiaRatio = 0.01;

	if (m_showWindows)
	{
		// Create the windows
		namedWindow(window_capture_name);
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
int PlantDetector::processFrame(Mat frame)
{
	if (!m_inited) {
		return false;
	}

	// Convert from BGR to HSV colorspace
	cv::cvtColor(frame, hsvFrame, COLOR_BGR2HSV);

	// Blur the image
	cv::GaussianBlur(hsvFrame, blurFrame, cv::Size(1, 1), 2, 2);

	colorMask = ColorThresholding(hsvFrame);
	// Copy original frame to greenFrame with the colorMask
	greenFrame = Scalar::all(0);
	hsvFrame.copyTo(greenFrame, colorMask);

	// Erode/Dilate (use the green thresholded image)
	Mat morphElement = getStructuringElement(morph_type,
		Size(2 * morph_size + 1, 2 * morph_size + 1),
		Point(morph_size, morph_size));

	// Performing morphological on the colorMasked frame
	morphFrame = colorMask;

	// Do some number of morph openings
	erode(morphFrame, morphFrame, morphElement, Point(-1, -1), 1);
	dilate(morphFrame, morphFrame, morphElement, Point(-1, -1), 1);

	// Do some number of morph closings
	dilate(morphFrame, morphFrame, morphElement, Point(-1, -1), 10);
	erode(morphFrame, morphFrame, morphElement, Point(-1, -1), 10);

	// Shape Processing (Blob detection)
	vector<KeyPoint> detectedBlobs = DetectBlobs(morphFrame);

	// Save latest objects detected
	m_lastObjectsFound = detectedBlobs;

	// Filter weeds from this object list
	m_weedList.clear();
	m_weedList = m_plantFilter->filterWeeds(m_lastObjectsFound);

	// If we are showing windows ...
	if (m_showWindows)
	{
		// Show the original, scaled frame
		imshow(window_capture_name, blurFrame);
		imshow(window_color_threshold_name, greenFrame);
		imshow(window_morphs, morphFrame);
		// Draw detected blobs as red circles.
		// DrawMatchesFlags::DRAW_RICH_KEYPOINTS flag ensures
		// the size of the circle corresponds to the size of blob
		Mat im_with_keypoints;
		drawKeypoints(morphFrame, detectedBlobs, im_with_keypoints, Scalar(0, 0, 255), DrawMatchesFlags::DRAW_RICH_KEYPOINTS);
		// Show blobs
		imshow(window_blob_detection, im_with_keypoints);

		// Now draw only with filtered weeds
		drawKeypoints(morphFrame, m_weedList, im_with_keypoints, Scalar(0, 0, 255), DrawMatchesFlags::DRAW_RICH_KEYPOINTS);
		// Show blobs
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

vector<KeyPoint> PlantDetector::DetectBlobs(Mat srcFrame)
{
	// Storage for blobs
	vector<KeyPoint> keypoints;

	// Set up BlobDetector with params
	Ptr<SimpleBlobDetector> detector = SimpleBlobDetector::create(m_blobParams);

	// Detect blobs
	detector->detect(srcFrame, keypoints);

	return keypoints;
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