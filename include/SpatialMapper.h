#ifndef SPATIAL_MAPPER_H
#define SPATIAL_MAPPER_H

#include <ros/ros.h>
#include <opencv2/calib3d.hpp>
#include <urVision/weedData.h>

using namespace cv;

// SpatialMapper Class
// Used by the ImageConverter to provide spatial mapping between 
// Imaging and Robot Frame of Reference
class SpatialMapper
{
private:
    ros::NodeHandle& m_nodeHandle;

    int fovWidthCm, fovHeightCm;

    int m_frameWidth, m_frameHeight, fovCameraHeight;

    // OpenCV stuff
    Mat cameraMatrix, distCoeffs;
    Mat rvec, tvec, rotationMatrix;

    Mat cameraMatrixInv, rotationMatrixInv;
    Mat leftSideMat, rightSideMat, uvPoint;

public:
    float sizeScale;

public:
    SpatialMapper(ros::NodeHandle& nodeHandle, int frameWidth, int frameHeight);

    // Converts a coordinate from the processed frame into a coordinate in the reference frame
    //      The ret coordinate in the reference frame is in the physical reference frame of the delta arm.
    //      This function should do any of the necessary transformation matrices to enter the delta arm reference frame
    //          from the distorted image.
    bool keypointToReferenceFrame(const KeyPoint& keypoint, urVision::weedData& retData);

    // Meant to be used for utility
    //      Conversion should be exact opposite of function above
    bool referenceFrameToKeypoint(const urVision::weedData& weedData, KeyPoint& retData);

    // Spatial Mapping specific parameters
	bool readMappingParameters();

}; // SpatialMapper

#endif // SPATIAL_MAPPER_H