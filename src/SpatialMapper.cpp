#include <ros/ros.h>

#include <urVision/weedData.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d.hpp>

// SpatialMapper Class
// Used by the ImageConverter to provide spatial mapping between 
// Imaging and Robot Frame of Reference
class SpatialMapper
{
private:
    ros::NodeHandle& m_nodeHandle;

    int fovWidthCm, fovHeightCm;

    int m_frameWidth, m_frameHeight;

public:
    float scaleX, scaleY, scaleSize;

public:
    SpatialMapper(ros::NodeHandle& nodeHandle, int frameWidth, int frameHeight) :
        m_nodeHandle(nodeHandle), m_frameWidth(frameWidth), m_frameHeight(frameHeight)
    {
        if (!readMappingParameters()) {
			ROS_ERROR("Could not read general parameters required for SpatialMapper.");
			ros::requestShutdown();
		}

        // These are our scaling operations
        scaleX = ((float)fovWidthCm) / m_frameWidth;
        scaleY = ((float)fovHeightCm) / m_frameHeight;
        scaleSize = (scaleX + scaleY) / 2; // average

        ROS_INFO("Spatial Mapping:");
        ROS_INFO("\tImage Frame Size is %ix%i [pixels]", 
            m_frameWidth, m_frameHeight);
        ROS_INFO("\tField of View is %ix%i [cm]", fovWidthCm, fovHeightCm);
        ROS_INFO("\tScaleFactors: (xScale,yScale,sizeScale): (%.4f,%.4f,%.4f)", scaleX, scaleY, scaleSize);
    }

    // Converts a coordinate from the processed frame into a coordinate in the reference frame
    //      The ret coordinate in the reference frame is in the physical reference frame of the delta arm.
    //      This function should do any of the necessary transformation matrices to enter the delta arm reference frame
    //          from the distorted image.
    bool keypointToReferenceFrame(const KeyPoint& keypoint, urVision::weedData& retData)
    {
        // Need to change this to be centered around 0 for Delta arm operation
        // e.g. x_cm = (ptx - (framewidth / 2) * scaleFactorX;
        // e.g. y_cm = (pty - (frameheight / 2) * scaleFactorY;
        retData.x_cm = (float)((keypoint.pt.x - (m_frameWidth / 2))  * scaleX); 
        retData.y_cm = (float)((keypoint.pt.y - (m_frameHeight / 2)) * scaleY)*-1.0; 
        retData.z_cm = (float)0; // Just assume "height" == 0
        retData.size_cm = (float)(keypoint.size * scaleSize);

        return true;
    }

    // Meant to be used for utility
    //      Conversion should be exact opposite of function above
    bool referenceFrameToKeypoint(const urVision::weedData& weedData, KeyPoint& retData)
    {
        retData.pt.x = (float)((weedData.x_cm / scaleX) + (m_frameWidth / 2));
        retData.pt.y = (float)((-1.0*weedData.y_cm / scaleY) + (m_frameHeight / 2));
        retData.size = (float)((weedData.size_cm / scaleSize));

        return true;
    }

    // Spatial Mapping specific parameters
	bool readMappingParameters()
	{
		// Read vision parameters
		if (!m_nodeHandle.getParam("fov_width_cm", fovWidthCm)) return false;
		if (!m_nodeHandle.getParam("fov_height_cm", fovHeightCm)) return false;

		// if (!m_nodeHandle.getParam("fov_height_cm", fovHeightCm))
        // {
        //     return false
        // } 
        // else
        // {
            
        // }

        return true;
    }

}; // SpatialMapper