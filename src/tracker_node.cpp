#include <ros/ros.h>

#include "urVision/ObjectTracker.h"

// Srv and msg types
#include <urGovernor/FetchWeed.h>
#include <urGovernor/MarkUprooted.h>
#include <urVision/weedDataArray.h>
#include <std_msgs/Float32.h>

// For tf2
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

// Transform message
#include <geometry_msgs/TransformStamped.h>
// Gives us position, orientiation
#include <geometry_msgs/PoseStamped.h>

#include <vector>
#include <mutex>

static inline Object weed_to_object(urVision::weedData& weed)
{
    /* all values are floats */
    return {(float)weed.x_cm, (float)weed.y_cm, (float)weed.z_cm, (float)weed.size_cm};
}

static void object_to_weed(Object& obj, urVision::weedData& weed)
{
    weed.x_cm = obj.x;
    weed.y_cm = obj.y;
    weed.z_cm = obj.z;
    weed.size_cm = obj.size;
}

class TrackerHolder
{
public:
    ObjectTracker* p_weedTracker;
    std::mutex weedTrackerLock;

    ObjectTracker* p_cropTracker;
    std::mutex cropTrackerLock;

    // Parameters to read from configs
    std::string weedPublisherName;
    std::string cropPublisherName;

    std::string frameratePublisherName;

    std::string fetchWeedServiceName;
    std::string queryWeedServiceName;
    std::string markUprootedServiceName;

    // tf2 parameters
    std::string worldFrameName;
    std::string cameraFrameName;

    Distance distanceTolerance;
    float maxTimeDisappeared;
    float minTimeValid;

    float targetFps;

    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener* tfListener;

    ros::NodeHandle m_nodeHandle;

public:
	TrackerHolder(ros::NodeHandle& nodeHandle)
	: m_nodeHandle(nodeHandle)
	{
        tfListener = new tf2_ros::TransformListener(tfBuffer);
    }

	~TrackerHolder()
	{
	}

public:
//// MSG Definitions
    // TODO: WeedData should just be a tf2::Point type
    void new_weeds_callback(const urVision::weedDataArray::ConstPtr& msg)
    {
        // This receives camera local coordinates
        ROS_DEBUG("Weed array received.");

        geometry_msgs::TransformStamped transformStamped;

        geometry_msgs::Point pointIn, pointOut;
        // Try finding the latest transform!
        try{    
            // We want to convert ALL coordinates cameraFrame --> worldFrame
            transformStamped = tfBuffer.lookupTransform(worldFrameName, cameraFrameName, ros::Time(0));

            std::vector<Object> new_objs;
            for (auto weed : msg->weeds)
            {
                // Convert weed to global coordinates
                pointIn.x = weed.x_cm;
                pointIn.y = weed.y_cm;
                pointIn.z = weed.z_cm;
                // TODO:: Figure this out
                // tf2::doTransform(pointIn, pointOut, transformStamped );
                weed.x_cm = pointOut.x;
                weed.y_cm = pointOut.y;

                ROS_DEBUG("\tNew weed: x- %f    y- %f      z- %f      size- %f", weed.x_cm, weed.y_cm, weed.z_cm, weed.size_cm);            
                new_objs.push_back(weed_to_object(weed));
            }

            weedTrackerLock.lock();
            p_weedTracker->update(new_objs);
            weedTrackerLock.unlock();

        } catch (tf2::TransformException &ex) {
            ROS_WARN("Could NOT transform cameraFrame to worldFrame: %s", ex.what());
        }
    }

    // This should receive camera-local coordinates using an array of 2 crops (lettuce that are found)
    void new_crops_callback(const urVision::weedDataArray::ConstPtr& msg)
    {
        ROS_DEBUG("Crop array received.");
        
        std::vector<Object> new_objs;

        for (auto weed : msg->weeds)
        {
            ROS_DEBUG("\tNew crop: x- %f    y- %f      z- %f      size- %f", weed.x_cm, weed.y_cm, weed.z_cm, weed.size_cm);
            new_objs.push_back(weed_to_object(weed));
        }

        cropTrackerLock.lock();
        p_cropTracker->update(new_objs);
        cropTrackerLock.unlock();
    }

    // msg callback
    void new_framerate_callback(const std_msgs::Float32::ConstPtr& msg)
    {
        weedTrackerLock.lock();
        p_weedTracker->updateFramerate(msg->data);
        weedTrackerLock.unlock();
    }

//// SRV definitions
    // fetch_weed_service (called by controller):
    // Response includes a valid weed to be uprooted.
    // Note that this indicates the object is IN_PROGRESS or 'being uprooted'
    // Response is in WORLD coordinates
    bool fetch_weed(urGovernor::FetchWeed::Request &req, urGovernor::FetchWeed::Response &res)
    {
        Object top_valid_obj;
        ObjectID obj_id = 0;
        bool retValue = false;
        
        if (req.request_id >= 0)
        {
            weedTrackerLock.lock();
            retValue = p_weedTracker->getObjectByID(top_valid_obj, req.request_id);
            weedTrackerLock.unlock();   
            obj_id = req.request_id;
        }
        else
        {
            weedTrackerLock.lock();
            retValue = p_weedTracker->topValidAndUproot(top_valid_obj, obj_id);
            weedTrackerLock.unlock();
        }

        if (retValue)
        {
            // Set response data to include weed fetched from the top
            object_to_weed(top_valid_obj, res.weed);
            res.tracking_id = obj_id;
        }
        else
        {
            ROS_DEBUG("fetch_weed_service: Issues with calls to tracker.");
        }

        return retValue;
    }

    // query_weed_service
    // Response is in WORLD coordinates
    bool query_weed(urGovernor::FetchWeed::Request &req, urGovernor::FetchWeed::Response &res)
    {
        Object top_valid_obj;
        bool retValue = false;

        weedTrackerLock.lock();
        retValue = p_weedTracker->topValid(top_valid_obj);
        weedTrackerLock.unlock();

        if (retValue)
        {
            // Set response data to include weed fetched from the top
            object_to_weed(top_valid_obj, res.weed);
        }

        return retValue;
    }

    // mark_uprooted_service
    bool mark_uprooted(urGovernor::MarkUprooted::Request &req, urGovernor::MarkUprooted::Response &res)
    {
        weedTrackerLock.lock();
        bool retValue = p_weedTracker->markUprooted(req.tracking_id, req.success);
        weedTrackerLock.unlock();

        if (!retValue)
        {
            ROS_DEBUG("mark_uprooted_service: Issues with calls to tracker.");
        }

        return retValue;
    }

    // General parameters for this node
    bool readGeneralParameters(ros::NodeHandle nodeHandle)
    {
        if (!nodeHandle.getParam("weed_data_publisher", weedPublisherName)) return false;
        if (!nodeHandle.getParam("crop_data_publisher", cropPublisherName)) return false;
        if (!nodeHandle.getParam("framerate_publisher", frameratePublisherName)) return false;
        
        if (!nodeHandle.getParam("fetch_weed_service", fetchWeedServiceName)) return false;
        if (!nodeHandle.getParam("query_weed_service", queryWeedServiceName)) return false;
        if (!nodeHandle.getParam("mark_uprooted_service", markUprootedServiceName)) return false;


        if (!nodeHandle.getParam("max_time_disappeared", maxTimeDisappeared)) return false;
        if (!nodeHandle.getParam("min_time_valid", minTimeValid)) return false;
        if (!nodeHandle.getParam("/target_fps", targetFps)) return false;

        if (!nodeHandle.getParam("distance_tolerance", distanceTolerance)) return false;

        // Transform parameters!
        if (!nodeHandle.getParam("tf_camera_frame", cameraFrameName)) return false;
        if (!nodeHandle.getParam("tf_world_frame", worldFrameName)) return false;

        return true;
    }

    bool initialize()
    {
        if (!readGeneralParameters(m_nodeHandle))
        {
            ROS_ERROR("Could not read general parameters for tracker.");
            ros::requestShutdown();
        }

        // Initialize weedTracker
        weedTrackerLock.lock();
        p_weedTracker = new ObjectTracker(distanceTolerance, targetFps, maxTimeDisappeared, minTimeValid, ObjectType::WEED);
        weedTrackerLock.unlock();

        // Initialize cropTracker
        cropTrackerLock.lock();
        p_cropTracker = new ObjectTracker(distanceTolerance, targetFps, maxTimeDisappeared, minTimeValid, ObjectType::CROP);
        cropTrackerLock.unlock();

        // Subscriber to the weed publisher (from urVision)
        ros::Subscriber weedSub = m_nodeHandle.subscribe(weedPublisherName, 1000, &TrackerHolder::new_weeds_callback, this);
        ros::Subscriber cropSub = m_nodeHandle.subscribe(cropPublisherName, 1000, &TrackerHolder::new_crops_callback, this);
        ros::Subscriber framerateSub = m_nodeHandle.subscribe(frameratePublisherName, 1, &TrackerHolder::new_framerate_callback, this);

        // Services to provide to controller (and vision node)
        ros::ServiceServer fetchWeedService = m_nodeHandle.advertiseService(fetchWeedServiceName, &TrackerHolder::fetch_weed, this);
        ros::ServiceServer queryWeedService = m_nodeHandle.advertiseService(queryWeedServiceName, &TrackerHolder::query_weed, this);
        ros::ServiceServer markUprootedService = m_nodeHandle.advertiseService(markUprootedServiceName, &TrackerHolder::mark_uprooted, this);

        ros::spin();

        weedTrackerLock.lock();
        delete p_weedTracker;
        weedTrackerLock.unlock();

        cropTrackerLock.lock();
        delete p_cropTracker;
        cropTrackerLock.unlock();
    }

};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "tracker_node");
    ros::NodeHandle nodeHandle("~");
	
	// Instantiate imageConverter class
    TrackerHolder trackerHolder(nodeHandle);
	// Initialize the imageConverter
	// This will start the main vision pipeline
	if (!trackerHolder.initialize())
	{
		ROS_ERROR("Error in call to initialize TrackerHolder.");
		ros::requestShutdown();
	}

	ros::spin();
	return 0;
}