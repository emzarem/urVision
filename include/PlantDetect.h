#ifndef PLANT_DETECT_H
#define PLANT_DETECT_H

#include <opencv2/core/utility.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/opencv.hpp>

using namespace cv;

Mat Erosion(Mat srcFrame);
Mat Dilation(Mat srcFrame);

/* HSV thresholding 
*	Returns a binary colorMask
*/
Mat ColorThresholding(Mat srcFrame);
/* Edge Detection  */
Mat GetEdges(Mat srcFrame);

int ProcessFrame(Mat frame);

int InitTestWindows(void);

#endif // PLANT_DETECT_H