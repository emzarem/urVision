#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

// For spatial mapping utilities
#include "SpatialMapper.h"

// Msg types
#include <urVision/weedDataArray.h>
#include <urVision/weedData.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Header.h>

#include <urVision/QueryWeeds.h>

const std::string OPENCV_WINDOW = "urVision Window";

// Image Converter class
// This class is the main pipeline of the vision node
class ImageConverter
{
	ros::NodeHandle& m_nodeHandle;

	image_transport::ImageTransport m_imageTransport;

	// Subscribers
	image_transport::Subscriber m_imageSubscriber;

	// Publishers
	image_transport::Publisher m_imagePublisher;  
	image_transport::Publisher m_objIdPublisher;  
	ros::Publisher m_weedDataPublisher;
	ros::Publisher m_cropDataPublisher;

	ros::Publisher m_weedThresholdPublisher;
	ros::Publisher m_frameratePublisher;

	// Service clients
	ros::ServiceClient m_queryWeedsClient;

	// For topic names
	std::string m_cameraName;
	std::string m_imageTopic;

	std::string m_imagePublisherName;
	std::string m_objIdPublisherName;
	std::string m_weedDataPublisherName;
	std::string m_cropDataPublisherName;

	std::string m_weedThresholdPublisherName;
	std::string m_frameratePublisherName;

	std::string queryWeedServiceName;

	bool m_showWindow;
	bool m_haveFrame;
	uint64_t m_frameNum;

	int m_frameLogInterval;

	PlantDetector* m_detector;

	SpatialMapper* m_spatialMapper;

	VisionParams m_visionParams;

	float maxWeedSizeCm, minWeedSizeCm, defaultWeedSizeCm, defaultCropSizeCm, filterDistanceTolCm;

	// For performance logging
	ros::WallTime start_, end_;

public:
	ImageConverter(ros::NodeHandle& nodeHandle)
	: m_nodeHandle(nodeHandle), m_imageTransport(nodeHandle), m_haveFrame(false), m_frameNum(0), m_spatialMapper(NULL)
	{
		if (!readGeneralParameters()) {
			ROS_ERROR("Could not read general parameters required for ImageConverter.");
			ros::requestShutdown();
		}

		if (!readVisionParameters())
		{
			ROS_ERROR("Could not read vision parameters required for ImageConverter.");
			ros::requestShutdown();
		}

		// Create plant detector
		m_detector = new PlantDetector(m_showWindow);
	}

	~ImageConverter()
	{
	}

	int initialize()
	{

		// Subscribe to input video feed and publish output video feed
		m_imageSubscriber = m_imageTransport.subscribe(std::string(m_cameraName + m_imageTopic), 1, &ImageConverter::processImage, this);

		m_imagePublisher = m_imageTransport.advertise(m_imagePublisherName, 1);
		m_objIdPublisher = m_imageTransport.advertise(m_objIdPublisherName, 1);

	  	m_weedDataPublisher = m_nodeHandle.advertise<urVision::weedDataArray>(m_weedDataPublisherName, 1);
	  	m_cropDataPublisher = m_nodeHandle.advertise<urVision::weedDataArray>(m_cropDataPublisherName, 1);

	  	m_weedThresholdPublisher = m_nodeHandle.advertise<std_msgs::Float32>(m_weedThresholdPublisherName, 1);

	  	m_frameratePublisher = m_nodeHandle.advertise<std_msgs::Float32>(m_frameratePublisherName, 1);

		m_queryWeedsClient = m_nodeHandle.serviceClient<urVision::QueryWeeds>(queryWeedServiceName);

		ROS_INFO("ImageConverter Pipeline started successfully!");

		start_ = ros::WallTime::now();		

		return true;
	}
 
	void processImage(const sensor_msgs::ImageConstPtr& msg)
	{
		// Initialize static data
		static Mat currentFrame;
		static Mat im_with_keypoints;
		static Mat im_with_obj_ids;
		static std_msgs::Float32 otsuThreshold;

		// For calling back to tracker node show current valid weed.
    	urVision::QueryWeeds queryWeedSrv;
		queryWeedSrv.request.caller = 1; // we don't need this

		// Update framenumber
		m_frameNum++;
		// Get image from cv_bridge
		cv_bridge::CvImagePtr cv_ptr;
		try
		{
	  		cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
		}
		catch (cv_bridge::Exception& e)
		{
			ROS_ERROR("cv_bridge exception: %s", e.what());
			return;
		}

		// Get current frame
		currentFrame = cv_ptr->image;

		// If this is the first frame
		if (!m_haveFrame)
		{
			m_visionParams.frameSize = currentFrame.size();

			m_spatialMapper = new SpatialMapper(m_nodeHandle, m_visionParams.frameSize.width, m_visionParams.frameSize.height);

			m_visionParams.defaultWeedThreshold = ((float)defaultWeedSizeCm ) / m_spatialMapper->sizeScale;
			m_visionParams.defaultCropThreshold = ((float)defaultCropSizeCm ) / m_spatialMapper->sizeScale;
			m_visionParams.minWeedSize = ((float)minWeedSizeCm ) / m_spatialMapper->sizeScale;
			m_visionParams.maxWeedSize = ((float)maxWeedSizeCm ) / m_spatialMapper->sizeScale;

			m_visionParams.filterDistanceTol = ((float)filterDistanceTolCm ) / m_spatialMapper->sizeScale;

			// Now we have a frame (only had to to this initialization once)
			m_haveFrame = true;
			if (!m_detector->init(m_visionParams))
			{
				ROS_ERROR("Could not initialize PlantDetector instance!");
				ros::requestShutdown();
			}
		}

		// Do processing on this frame
		if (!m_detector->processFrame(currentFrame))
		{
			ROS_ERROR("Error processing frame: %i\n", (int)m_frameNum);
			ros::requestShutdown();
		}

		// Every 10 frames, log processing rate
		if ((m_frameNum % m_frameLogInterval) == 1 && m_frameNum > m_frameLogInterval)
		{
			end_ = ros::WallTime::now();

			double frame_rate_hz = ((double)m_frameLogInterval) / ((end_ - start_).toNSec() * 1e-9);
			ROS_INFO("Processed Frame: %i, running @ %f Hz", (int)m_frameNum, (float)frame_rate_hz);
			
			std_msgs::Float32 framerate;
			framerate.data = float(frame_rate_hz);
			m_frameratePublisher.publish(framerate);

			start_ = ros::WallTime::now();		
		}


		//// Publish the weedList for this frame
		// msg array to publish
		urVision::weedDataArray weed_msg;
		vector<KeyPoint> weedList = m_detector->getWeedList();
		for (auto it = weedList.begin(); it != weedList.end(); ++it)
		{
			// Populating weedData list to be published
			urVision::weedData weed_data;

			// Do spatial mapping conversion
			if (m_spatialMapper->keypointToReferenceFrame(*it, weed_data)) {
				weed_msg.weeds.push_back(weed_data);
			}
			else {
				ROS_ERROR("Spatial Mapping could not be performed on point (in image) (x,y)=(%f,%f)", it->pt.x, it->pt.y);
			}			
		}

		// Publish weeddaata
		if (weed_msg.weeds.size() > 0) {
			weed_msg.header.stamp = ros::Time::now();
			m_weedDataPublisher.publish(weed_msg);
		}

		//// Publish the cropList for this frame
		// msg array to publish
		urVision::weedDataArray crop_msg;
		vector<KeyPoint> cropList = m_detector->getCropList();
		for (auto it = cropList.begin(); it != cropList.end(); ++it)
		{
			// Populating weedData list to be published
			urVision::weedData crop_data;

			// Do spatial mapping conversion
			if (m_spatialMapper->keypointToReferenceFrame(*it, crop_data)) {
				crop_msg.weeds.push_back(crop_data);
			}
			else {
				ROS_ERROR("Spatial Mapping could not be performed on point (in image) (x,y)=(%f,%f)", it->pt.x, it->pt.y);
			}			
		}

		// Publish weeddaata
		if (crop_msg.weeds.size() > 0) {
			crop_msg.header.stamp = ros::Time::now();
			m_cropDataPublisher.publish(crop_msg);
		}

		im_with_keypoints = m_detector->greenFrame;
		// Draw weeds
		for (auto it = weedList.begin(); it != weedList.end(); it++)
		{
			cv::circle(im_with_keypoints, cv::Point(it->pt.x, it->pt.y), it->size / 2, Scalar(0, 0, 255), 5, 8);
		}
		// Draw crops
		for (auto it = cropList.begin(); it != cropList.end(); it++)
		{
			cv::circle(im_with_keypoints, cv::Point(it->pt.x, it->pt.y), it->size / 2, Scalar(0, 255, 0), 5, 8);
		}

		// Publish the output image with keypoints, bounding boxes, etc.
		cv_ptr->image = im_with_keypoints;
		m_imagePublisher.publish(cv_ptr->toImageMsg());

		//// Show the current valid weed (next to be harvested) if there is one
		im_with_obj_ids = im_with_keypoints;
		if (m_queryWeedsClient.call(queryWeedSrv))
		{
			auto pairList = queryWeedSrv.response.pairs;
			for (auto it = pairList.begin(); it != pairList.end(); it++)
			{
				KeyPoint imagePoint;
				
				// Map back to image coordinates
				if (m_spatialMapper->referenceFrameToKeypoint(it->weed, imagePoint))
				{
					// On the top one only
					if (it == pairList.begin())
					{
						// Draw red crosshair on next weed to target!
						cv::drawMarker(im_with_obj_ids, cv::Point(imagePoint.pt.x, imagePoint.pt.y),  
										cv::Scalar(0, 0, 255), MARKER_CROSS, imagePoint.size*2, 5);
					}

					// Draw object id on image1
					cv::putText(im_with_obj_ids, std::to_string(it->id), cv::Point(imagePoint.pt.x + imagePoint.size / 4, imagePoint.pt.y + imagePoint.size / 4), 
								cv::FONT_HERSHEY_DUPLEX, 3, Scalar(128,0,128), 3, 8, true);
				}
				else
				{
					ROS_ERROR("[REVERSE] Spatial Mapping could not be performed on weed (from tracker) (x,y)=(%f,%f)", it->weed.point.x, it->weed.point.y);
				}
			}
		}
		
		cv_ptr->image = im_with_obj_ids;
		m_objIdPublisher.publish(cv_ptr->toImageMsg());

		// Publish the current otsuThreshold
		// otsuThreshold.data = (float)(m_detector->getWeedThreshold() * m_spatialMapper->sizeScale);
		// m_weedThresholdPublisher.publish(otsuThreshold);

		// Done processing this frame
		return;
	}

	// General parameters for this node
	bool readGeneralParameters()
	{
		if (!m_nodeHandle.getParam("camera_name", m_cameraName)) return false;
		if (!m_nodeHandle.getParam("image_topic", m_imageTopic)) return false;
		
		if (!m_nodeHandle.getParam("image_publisher", m_imagePublisherName)) return false;
		if (!m_nodeHandle.getParam("object_id_publisher", m_objIdPublisherName)) return false;

		if (!m_nodeHandle.getParam("weed_data_publisher", m_weedDataPublisherName)) return false;
		if (!m_nodeHandle.getParam("crop_data_publisher", m_cropDataPublisherName)) return false;

		if (!m_nodeHandle.getParam("weed_threshold_publisher", m_weedThresholdPublisherName)) return false;
		if (!m_nodeHandle.getParam("framerate_publisher", m_frameratePublisherName)) return false;
		if (!m_nodeHandle.getParam("show_img_window", m_showWindow)) return false;

		if (!m_nodeHandle.getParam("frames_log_interval", m_frameLogInterval)) return false;

		if (!m_nodeHandle.getParam("query_weeds_service", queryWeedServiceName)) return false;

		return true;
	}

	// Image converter specific parameters
	bool readVisionParameters()
	{
		// Read vision parameters
		if (!m_nodeHandle.getParam("min_weed_size_cm", minWeedSizeCm)) return false;
		if (!m_nodeHandle.getParam("max_weed_size_cm", maxWeedSizeCm)) return false;
		if (!m_nodeHandle.getParam("default_weed_size_threshold_cm", defaultWeedSizeCm)) return false;
		if (!m_nodeHandle.getParam("default_crop_size_threshold_cm", defaultCropSizeCm)) return false;
		if (!m_nodeHandle.getParam("filter_distance_tolerance_cm", filterDistanceTolCm)) return false;

		if (!m_nodeHandle.getParam("min_accumulator_size", m_visionParams.minAccumulatorSize)) return false;
		if (!m_nodeHandle.getParam("max_accumulator_size", m_visionParams.maxAccumulatorSize)) return false;

		if (!m_nodeHandle.getParam("blur_kernel_size", m_visionParams.blurSize)) return false;
		if (!m_nodeHandle.getParam("low_hue", m_visionParams.lowH)) return false;
		if (!m_nodeHandle.getParam("low_sat", m_visionParams.lowS)) return false;
		if (!m_nodeHandle.getParam("low_value", m_visionParams.lowV)) return false;
		if (!m_nodeHandle.getParam("high_hue", m_visionParams.highH)) return false;
		if (!m_nodeHandle.getParam("morph_size", m_visionParams.morphSize)) return false;
		if (!m_nodeHandle.getParam("morph_closing_iterations", m_visionParams.morphClosingIters)) return false;
		if (!m_nodeHandle.getParam("morph_opening_iterations", m_visionParams.morphOpeningIters)) return false;
		
		if (!m_nodeHandle.getParam("min_circularity", m_visionParams.minCircularity)) return false;
		if (!m_nodeHandle.getParam("min_convexity", m_visionParams.minConvexity)) return false;
		if (!m_nodeHandle.getParam("min_inertia_ratio", m_visionParams.minInertiaRatio)) return false;

		return true;
	}

};
