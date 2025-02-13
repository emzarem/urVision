#include <ros/ros.h>

#include "urVision/ObjectTracker.h"

// Srv and msg types
#include <urVision/QueryWeeds.h>
#include <urVision/ClearTracker.h>
#include <urGovernor/FetchWeed.h>
#include <urGovernor/MarkUprooted.h>
#include <urGovernor/RemoveWeed.h>

#include <urVision/weedDataArray.h>
#include <std_msgs/Float32.h>
#include <geometry_msgs/Vector3.h>

#include <vector>
#include <mutex>

static ObjectTracker* p_weedTracker;
std::mutex weedTrackerLock;

static ObjectTracker* p_cropTracker;
std::mutex cropTrackerLock;

// System velocity Publisher
ros::Publisher velocityPublisher;

// Parameters to read from configs
std::string weedPublisherName;
std::string cropPublisherName;

std::string frameratePublisherName;
std::string velocityPublisherName;

std::string fetchWeedServiceName;
std::string queryWeedServiceName;
std::string markUprootedServiceName;
std::string removeWeedServiceName;

std::string resetTrackerServiceName;
std::string stopTrackerServiceName;

Distance distanceTolerance;
float maxTimeDisappeared;
float minTimeValid;

float targetFps;
float velLpfCutoff;

static inline Object weed_to_object(urVision::weedData& weed, ros::Time stamp)
{
    /* all values are floats */
    return {(float)weed.point.x, (float)weed.point.y, (float)weed.point.z, (float)weed.size_cm, (double)stamp.toSec()};
}

static inline void object_to_weed(Object& obj, urVision::weedData& weed)
{
    weed.point.x = obj.x;
    weed.point.y = obj.y;
    weed.point.z = obj.z;
    weed.size_cm = obj.size;
}

/* Service Definitions */

/* fetch_weed_service (called by controller):
 *      Response includes a valid weed to be uprooted.
 *      Note that this indicates the object is IN_PROGRESS or 'being uprooted'
 */
bool fetch_weed(urGovernor::FetchWeed::Request &req, urGovernor::FetchWeed::Response &res)
{
    Object top_valid_obj;
    ObjectID obj_id = 0;
    bool retValue = false;
    
    if (req.request_id >= 0)
    {
        weedTrackerLock.lock();
        if (!p_weedTracker)
        {
            weedTrackerLock.unlock();   
            return false;
        }
        retValue = p_weedTracker->getObjectByID(top_valid_obj, req.request_id);
        weedTrackerLock.unlock();   
        obj_id = req.request_id;
    }
    else
    {
        weedTrackerLock.lock();
        if (!p_weedTracker)
        {
            weedTrackerLock.unlock();   
            return false;
        }
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

// query_weeds_service
bool query_weeds(urVision::QueryWeeds::Request &req, urVision::QueryWeeds::Response &res)
{
    std::vector<std::pair<ObjectID, Object>> readyObjects;
    std::vector<std::pair<ObjectID, Object>> completedObjects;
    bool retValue = false;
    res.ready_list.clear();
    res.completed_list.clear();

    weedTrackerLock.lock();
    if (!p_weedTracker)
    {
        weedTrackerLock.unlock();   
        return false;
    }
    retValue = p_weedTracker->getReadyObjects(readyObjects);
    retValue = p_weedTracker->getCompletedObjects(completedObjects);
    weedTrackerLock.unlock();

    if (retValue)
    {
        for (auto it = readyObjects.begin(); it != readyObjects.end(); it++)
        {
            urVision::WeedPair weedPair;
            weedPair.id = it->first;
            object_to_weed(it->second, weedPair.weed);
            res.ready_list.push_back(weedPair);
        }
        for (auto it = completedObjects.begin(); it != completedObjects.end(); it++)
        {
            urVision::WeedPair weedPair;
            weedPair.id = it->first;
            object_to_weed(it->second, weedPair.weed);
            res.completed_list.push_back(weedPair);
        }
    }

    return retValue;
}

// mark_uprooted_service
bool mark_uprooted(urGovernor::MarkUprooted::Request &req, urGovernor::MarkUprooted::Response &res)
{
    weedTrackerLock.lock();
    if (!p_weedTracker)
    {
        weedTrackerLock.unlock();   
        return false;
    }
    bool retValue = p_weedTracker->markUprooted(req.tracking_id, req.success);
    weedTrackerLock.unlock();

    if (!retValue)
    {
        ROS_DEBUG("mark_uprooted_service: Issues with calls to tracker.");
    }

    return retValue;
}

// Deregister object
bool remove_object(urGovernor::RemoveWeed::Request &req, urGovernor::RemoveWeed::Response &res)
{
    weedTrackerLock.lock();
    if (!p_weedTracker)
    {
        weedTrackerLock.unlock();   
        return false;
    }
    bool retValue = p_weedTracker->remove_object(req.tracking_id);
    weedTrackerLock.unlock();

    return retValue;
}

// mark_uprooted_service
bool reset_tracker(urVision::ClearTracker::Request &req, urVision::ClearTracker::Response &res)
{
    // Delete and re-initialize
    weedTrackerLock.lock();
    if (p_weedTracker)
        delete p_weedTracker;
    p_weedTracker = new ObjectTracker(distanceTolerance, velLpfCutoff, targetFps, maxTimeDisappeared, minTimeValid);
    weedTrackerLock.unlock();

    // Delete and re-initialize
    cropTrackerLock.lock();
    if (p_cropTracker)
        delete p_cropTracker;
    p_cropTracker = new ObjectTracker(distanceTolerance, velLpfCutoff, targetFps, maxTimeDisappeared, minTimeValid, ObjectType::CROP);
    cropTrackerLock.unlock();

    return true;
}

bool stop_tracker(urVision::ClearTracker::Request &req, urVision::ClearTracker::Response &res)
{
    // Delete
    weedTrackerLock.lock();
    if (p_weedTracker)
    {
        delete p_weedTracker;
        p_weedTracker = NULL;
    }
    weedTrackerLock.unlock();

    // Delete
    cropTrackerLock.lock();
    if (p_cropTracker)
    {
        delete p_cropTracker;
        p_cropTracker = NULL;
    }
    cropTrackerLock.unlock();

    return true;
}

// msg callback
void new_weed_callback(const urVision::weedDataArray::ConstPtr& msg)
{
    ROS_DEBUG("Weed array received.");
    
    std::vector<Object> new_objs;

    for (auto weed : msg->weeds)
    {
        ROS_DEBUG("\tNew weed: x- %f    y- %f      z- %f      size- %f", weed.point.x, weed.point.y, weed.point.z, weed.size_cm);
        new_objs.push_back(weed_to_object(weed, msg->header.stamp));
    }

    weedTrackerLock.lock();
    if (!p_weedTracker)
    {
        weedTrackerLock.unlock();
        return;
    }
    p_weedTracker->update(new_objs);
    weedTrackerLock.unlock();
}

void new_crop_callback(const urVision::weedDataArray::ConstPtr& msg)
{
    ROS_DEBUG("Crop array received.");
    
    std::vector<Object> new_objs;

    for (auto weed : msg->weeds)
    {
        ROS_DEBUG("\tNew crop: x- %f    y- %f      z- %f      size- %f", weed.point.x, weed.point.y, weed.point.z, weed.size_cm);
        new_objs.push_back(weed_to_object(weed, msg->header.stamp));
    }

    // Publish the speed
    geometry_msgs::Vector3 currentVelocity;
    currentVelocity.z = 0;

    cropTrackerLock.lock();
    // Update the tracker with current objects
    if (!p_cropTracker)
    {
        cropTrackerLock.unlock();
        return;
    }
    p_cropTracker->update(new_objs);
    // Get the most recent velocity
    currentVelocity.x = p_cropTracker->getXVelocity();
    currentVelocity.y = p_cropTracker->getYVelocity();
    cropTrackerLock.unlock();

    velocityPublisher.publish(currentVelocity);
}

// msg callback
void new_framerate_callback(const std_msgs::Float32::ConstPtr& msg)
{
    weedTrackerLock.lock();
    if (!p_weedTracker)
    {
        weedTrackerLock.unlock();
        return;
    }
    p_weedTracker->updateFramerate(msg->data);
    weedTrackerLock.unlock();

    cropTrackerLock.lock();
    if (!p_cropTracker)
    {
        cropTrackerLock.unlock();
        return;
    }
    p_cropTracker->updateFramerate(msg->data);
    cropTrackerLock.unlock();
}

// msg callback
void new_velocity_callback(const geometry_msgs::Vector3::ConstPtr& msg)
{
    weedTrackerLock.lock();
    if (!p_weedTracker)
    {
        weedTrackerLock.unlock();
        return;
    }
    p_weedTracker->updateVelocity(msg->x, msg->y);
    weedTrackerLock.unlock();

    cropTrackerLock.lock();
    if (!p_cropTracker)
    {
        cropTrackerLock.unlock();
        return;
    }
    p_cropTracker->updateVelocity(msg->x, msg->y);
    cropTrackerLock.unlock();
}

// General parameters for this node
bool readGeneralParameters(ros::NodeHandle nodeHandle)
{
    if (!nodeHandle.getParam("weed_data_publisher", weedPublisherName)) return false;
    if (!nodeHandle.getParam("crop_data_publisher", cropPublisherName)) return false;
    if (!nodeHandle.getParam("framerate_publisher", frameratePublisherName)) return false;
    if (!nodeHandle.getParam("velocity_publisher", velocityPublisherName)) return false;

    if (!nodeHandle.getParam("fetch_weed_service", fetchWeedServiceName)) return false;
    if (!nodeHandle.getParam("query_weeds_service", queryWeedServiceName)) return false;
    if (!nodeHandle.getParam("mark_uprooted_service", markUprootedServiceName)) return false;
    if (!nodeHandle.getParam("remove_weed_service", removeWeedServiceName)) return false;

    if (!nodeHandle.getParam("reset_tracker_service", resetTrackerServiceName)) return false;
    if (!nodeHandle.getParam("stop_tracker_service", stopTrackerServiceName)) return false;


    if (!nodeHandle.getParam("max_time_disappeared", maxTimeDisappeared)) return false;
    if (!nodeHandle.getParam("min_time_valid", minTimeValid)) return false;
    if (!nodeHandle.getParam("/target_fps", targetFps)) return false;
    if (!nodeHandle.getParam("vel_lpf_cutoff", velLpfCutoff)) return false;

    if (!nodeHandle.getParam("distance_tolerance", distanceTolerance)) return false;
    
    return true;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "tracker_node");
    ros::NodeHandle nodeHandle("~");

    if (!readGeneralParameters(nodeHandle))
    {
        ROS_ERROR("Could not read general parameters for tracker.");
        ros::requestShutdown();
    }

    // Initialize weedTracker
    weedTrackerLock.lock();
    p_weedTracker = new ObjectTracker(distanceTolerance, velLpfCutoff, targetFps, maxTimeDisappeared, minTimeValid);
    weedTrackerLock.unlock();

    // Initialize cropTracker
    cropTrackerLock.lock();
    p_cropTracker = new ObjectTracker(distanceTolerance, velLpfCutoff, targetFps, maxTimeDisappeared, minTimeValid, ObjectType::CROP);
    cropTrackerLock.unlock();

    // Publishers
	velocityPublisher = nodeHandle.advertise<geometry_msgs::Vector3>(velocityPublisherName, 1);

    // Subscribers
    ros::Subscriber weedSub = nodeHandle.subscribe(weedPublisherName, 1000, new_weed_callback);
    ros::Subscriber cropSub = nodeHandle.subscribe(cropPublisherName, 1000, new_crop_callback);
    ros::Subscriber framerateSub = nodeHandle.subscribe(frameratePublisherName, 1, new_framerate_callback);
    ros::Subscriber velocitySub = nodeHandle.subscribe(velocityPublisherName, 1, new_velocity_callback);

    // Services
    ros::ServiceServer fetchWeedService = nodeHandle.advertiseService(fetchWeedServiceName, fetch_weed);
    ros::ServiceServer queryWeedService = nodeHandle.advertiseService(queryWeedServiceName, query_weeds);
    ros::ServiceServer markUprootedService = nodeHandle.advertiseService(markUprootedServiceName, mark_uprooted);
    ros::ServiceServer rmWeedService = nodeHandle.advertiseService(removeWeedServiceName, remove_object);

    ros::ServiceServer resetTracker = nodeHandle.advertiseService(resetTrackerServiceName, reset_tracker);
    ros::ServiceServer stopTracker = nodeHandle.advertiseService(stopTrackerServiceName, stop_tracker);

    ros::spin();
}

