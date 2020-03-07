#include <SpatialMapper.h>

#include <ros/package.h>
#include <opencv2/imgproc/imgproc.hpp>

SpatialMapper::SpatialMapper(ros::NodeHandle& nodeHandle, int frameWidth, int frameHeight) :
    m_nodeHandle(nodeHandle), m_frameWidth(frameWidth), m_frameHeight(frameHeight)
{
    if (!readMappingParameters()) {
        ROS_ERROR("Could not read general parameters required for SpatialMapper.");
        ros::requestShutdown();
    }

    // These are our scaling operations
    float scaleX = ((float)fovWidthCm) / m_frameWidth;
    float scaleY = ((float)fovHeightCm) / m_frameHeight;
    // Set main size scale to be used
    sizeScale = scaleY;
    // sizeScale = (scaleX + scaleY) / 2; // average

    ROS_INFO("Spatial Mapping:");
    ROS_INFO("\tImage Frame Size is %ix%i [pixels]", 
        m_frameWidth, m_frameHeight);
    // ROS_INFO("\tField of View is %ix%i [cm]", fovWidthCm, fovHeightCm);
    ROS_INFO("\tScaleFactor (approx.): : %.1f", 1/sizeScale);
    
    // // TODO: Need some info here 
    // // We need to provide a worldPlane and an imagePlane (points need to coincide)
    // cv::solvePnP(worldPlane, imagePlane, cameraMatrix, distCoeffs, rvec, tvec);
    
    // Convert rotation vector to rotationMatrix
    rotationMatrix = Mat::ones(3, 3, cv::DataType<double>::type);
    cv::Rodrigues(rvec,rotationMatrix);

    // Stuff we need for our inverse projection calculations
    rotationMatrixInv = rotationMatrix.inv();
    cameraMatrixInv = cameraMatrix.inv();
    rightSideMat = rotationMatrixInv * tvec;
    uvPoint = Mat::ones(3,1,cv::DataType<double>::type);

    ROS_INFO_STREAM(std::endl << "Camera matrix: " << std::endl << cameraMatrix << std::endl
                    << "Distortion coef: " << std::endl << distCoeffs << std::endl
                    << "Translation vector: "<< std::endl << tvec << std::endl
                    << "Rotation vector: "<< std::endl << rvec << std::endl
    );
    
}

// Converts a coordinate from the processed frame into a coordinate in the reference frame
//      The ret coordinate in the reference frame is in the physical reference frame of the delta arm.
//      This function should do any of the necessary transformation matrices to enter the delta arm reference frame
//          from the distorted image.
bool SpatialMapper::keypointToReferenceFrame(const KeyPoint& keypoint, urVision::weedData& retData)
{
    // This is where the keypoint is in the image (u,v,1)
    uvPoint.at<double>(0,0) = keypoint.pt.x;
    uvPoint.at<double>(1,0) = keypoint.pt.y; 
    uvPoint.at<double>(2,0) = 1;
        
    // Do some fancy math ... (we already calculated rightSideMat)
    leftSideMat  = rotationMatrixInv * cameraMatrixInv * uvPoint;

    // Some more fancy math where fovCameraHeight is our know distance from Camera to ground projection ...
    double s = ((float)fovCameraHeight) + rightSideMat.at<double>(2,0);
    s /= leftSideMat.at<double>(2,0);

    // Get our REAL WORLD coordinates!
    Mat wcPoint = rotationMatrixInv * (s * cameraMatrixInv * uvPoint - tvec);

    // The coordinates to return
    // **NOTE**: x is y, y is x
    retData.point.x = wcPoint.at<double>(1, 0) / 10; // convert to cm
    retData.point.y = wcPoint.at<double>(0, 0) / 10; // convert to cm
    retData.point.z = (float)0;
    retData.size_cm = (float)(keypoint.size * sizeScale); 

    return true;
}

// Meant to be used for utility
//      Conversion should be exact opposite of function above
bool SpatialMapper::referenceFrameToKeypoint(const urVision::weedData& weedData, KeyPoint& retData)
{
    // Set up the location of the 
    // Convert points to mm (cm*10)
    // **NOTE**: x is y, y is x
    Point3f realPoint(weedData.point.y * 10.0, weedData.point.x * 10.0, fovCameraHeight); // point in world coordinates
    std::vector<Point3f> objectPoints;
    objectPoints.push_back(realPoint);
    std::vector<Point2f> projectedPoints;

    // Calculate projection from 2D to 3D
    projectPoints(objectPoints, rvec, tvec, cameraMatrix, distCoeffs, projectedPoints);

    // There should be only 1 projected point
    if (projectedPoints.size() == 1)
    {
        retData.pt.x = projectedPoints[0].x;
        retData.pt.y = projectedPoints[0].y;
        retData.size = (float)((weedData.size_cm / sizeScale));
    }
    else
    {
        return false;
    }

    return true;
}

// Spatial Mapping specific parameters
bool SpatialMapper::readMappingParameters()
{
    // Read vision parameters
    if (!m_nodeHandle.getParam("fov_width_cm", fovWidthCm)) return false;
    if (!m_nodeHandle.getParam("fov_height_cm", fovHeightCm)) return false;

    int fovCameraHeightCm = 0;
    if (!m_nodeHandle.getParam("fov_camera_distance_cm", fovCameraHeightCm)) return false;
    fovCameraHeight = fovCameraHeightCm*10;
    
    std::string cameraCalFile;
    if (!m_nodeHandle.getParam("camera_cal_file", cameraCalFile)) 
    {
        return false;
    }
    else
    {
        // Get path to camera config file
        std::string dir = ros::package::getPath("urVision");
        dir += "/config/";
        dir += cameraCalFile;

        ROS_DEBUG("OpenCV Camera Config File: %s", dir.c_str());

        // Open the camera properties (OpenCV config style)
        FileStorage configFile(dir, FileStorage::READ);

        if (!configFile.isOpened())
        {
            return false;
        }
        else
        {
            // Get distortion coefficients, camera matrix,
            // rotation vector, and translation vector
            configFile["distCoeffs"] >> distCoeffs;
            configFile["cameraMatrix"] >> cameraMatrix;
            configFile["rvec"] >> rvec;
            configFile["tvec"] >> tvec;

            configFile.release();
        }
    }

    return true;
}
