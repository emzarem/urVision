#include <ros/ros.h>

#include "urVision/ObjectTracker.h"

// Srv and msg types
#include <urGovernor/FetchWeed.h>
#include <urGovernor/MarkUprooted.h>
#include <urVision/weedDataArray.h>
#include <std_msgs/Float32.h>

// For tf2
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>


#include <vector>
#include <mutex>

static inline Object weed_to_object(urVision::weedData& weed)
{
    /* all values are floats */
    return {(float)weed.point.x, (float)weed.point.y, (float)weed.point.z, (float)weed.size_cm};
}

static inline void object_to_weed(Object& obj, urVision::weedData& weed)
{
    weed.point.x = obj.x;
    weed.point.y = obj.y;
    weed.point.z = obj.z;
    weed.size_cm = obj.size;
}

/* This class exists because the tfBuffer and tfListener need to persist
 *  Having them as members in a class ensures that this happens, and they don't lose
 *  their caches. See the tf2 tutorials for more information.
 */
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
        delete tfListener;

        weedTrackerLock.lock();
        delete p_weedTracker;
        weedTrackerLock.unlock();

        cropTrackerLock.lock();
        delete p_cropTracker;
        cropTrackerLock.unlock();
	}

public:
//// MSG Definitions
    // Receives camera local coordinates of weeds
    void newWeedsCallback(const urVision::weedDataArray::ConstPtr& msg)
    {
        // This receives camera local coordinates
        ROS_DEBUG("Weed array received.");
        
        std::vector<Object> new_objs;
        try
        {
            // Try finding the camera to world transform
            geometry_msgs::TransformStamped transformStamped;

            geometry_msgs::Point pointIn, pointOut;

            // We want to convert ALL coordinates cameraFrame --> worldFrame
            transformStamped = tfBuffer.lookupTransform(worldFrameName, cameraFrameName, ros::Time(0));

            // IF the lookup worked!
            ROS_DEBUG("FOUND transform from cameraFrame to worldFrame.\n");
            for (auto weed : msg->weeds)
            {
                // Convert weed to global coordinates
                pointIn = weed.point;
                // Perform the transform on the input produced the output, based on the transform we just got
                tf2::doTransform(pointIn, pointOut, transformStamped );
                weed.point = pointOut;

                ROS_DEBUG("\tNew weed: x- %f    y- %f      z- %f      size- %f", weed.point.x, weed.point.y, weed.point.z, weed.size_cm);            
                new_objs.push_back(weed_to_object(weed));
            }
        } 
        catch (tf2::TransformException &ex) 
        {
            // Just use local coordinates
            ROS_DEBUG("Could NOT transform cameraFrame to worldFrame: %s", ex.what());

            for (auto weed : msg->weeds)
            {
                ROS_DEBUG("\tNew weed: x- %f    y- %f      z- %f      size- %f", weed.point.x, weed.point.y, weed.point.z, weed.size_cm);
                new_objs.push_back(weed_to_object(weed));
            }
        }

        // Update the tracker!
        weedTrackerLock.lock();
        p_weedTracker->update(new_objs);
        weedTrackerLock.unlock();
    }

    // Receives camera local coordinates of crops
    void newCropsCallback(const urVision::weedDataArray::ConstPtr& msg)
    {
        ROS_DEBUG("Crop array received.");
        
        std::vector<Object> new_objs;
        for (auto weed : msg->weeds)
        {
            ROS_DEBUG("\tNew crop: x- %f    y- %f      z- %f      size- %f", weed.point.x, weed.point.y, weed.point.z, weed.size_cm);
            new_objs.push_back(weed_to_object(weed));
        }

        std::vector<Object> ready_objects;
        
        cropTrackerLock.lock();
            // Add this crop to the tracker!
            p_cropTracker->update(new_objs);
            // Get current readyObjects
            ready_objects = p_cropTracker->getReadyObjects();
        cropTrackerLock.unlock();

        // If we have 2 or more objects
        if (ready_objects.size() >= 2)
        {
            float translationX = 0.0;
            float translationY = 0.0;
            float theta = 0;
            ROS_DEBUG("Updating camera->world transform.");
            // Calculate current translation and rotation

            // We are ready to establish our ground truth
            // Broadcast the known transform from current cameraFrame -> worldFrame
            static tf2_ros::TransformBroadcaster br;
            geometry_msgs::TransformStamped transformStamped;

            transformStamped.header.stamp = ros::Time::now();
            transformStamped.header.frame_id = worldFrameName;
            transformStamped.child_frame_id = cameraFrameName;
            transformStamped.transform.translation.x = translationX;
            transformStamped.transform.translation.y = translationY;
            transformStamped.transform.translation.z = 0.0;
            tf2::Quaternion q;
            q.setRPY(0, 0, theta);
            transformStamped.transform.rotation.x = q.x();
            transformStamped.transform.rotation.y = q.y();
            transformStamped.transform.rotation.z = q.z();
            transformStamped.transform.rotation.w = q.w();

            br.sendTransform(transformStamped);
        }
    }

    // msg callback
    void newFramerateCallback(const std_msgs::Float32::ConstPtr& msg)
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

    bool run()
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
        ros::Subscriber weedSub = m_nodeHandle.subscribe(weedPublisherName, 1000, &TrackerHolder::newWeedsCallback, this);
        ros::Subscriber cropSub = m_nodeHandle.subscribe(cropPublisherName, 1000, &TrackerHolder::newCropsCallback, this);
        ros::Subscriber framerateSub = m_nodeHandle.subscribe(frameratePublisherName, 1, &TrackerHolder::newFramerateCallback, this);

        // Services to provide to controller (and vision node)
        ros::ServiceServer fetchWeedService = m_nodeHandle.advertiseService(fetchWeedServiceName, &TrackerHolder::fetch_weed, this);
        ros::ServiceServer queryWeedService = m_nodeHandle.advertiseService(queryWeedServiceName, &TrackerHolder::query_weed, this);
        ros::ServiceServer markUprootedService = m_nodeHandle.advertiseService(markUprootedServiceName, &TrackerHolder::mark_uprooted, this);

	    ros::spin();

        return true;
    }

};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "tracker_node");
    ros::NodeHandle nodeHandle("~");
	
	// Instantiate TrackerHolder class
    TrackerHolder* trackerHolder = new TrackerHolder(nodeHandle);
	// Initialize the trackerHolder
    // This will advertise services and subcribe to messages
	if (!trackerHolder->run())
	{
		ROS_ERROR("Error in call to run TrackerHolder.");
		ros::requestShutdown();
	}

	ros::spin();

    delete trackerHolder;

	return 0;
}