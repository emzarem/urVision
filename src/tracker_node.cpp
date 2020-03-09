#include <ros/ros.h>

#include "urVision/ObjectTracker.h"

// Srv and msg types
#include <urVision/QueryWeeds.h>
#include <urGovernor/FetchWeed.h>
#include <urGovernor/MarkUprooted.h>
#include <urVision/weedDataArray.h>
#include <std_msgs/Float32.h>


#include <vector>
#include <mutex>

static ObjectTracker* p_weedTracker;
std::mutex weedTrackerLock;

static ObjectTracker* p_cropTracker;
std::mutex cropTrackerLock;

// Parameters to read from configs
std::string weedPublisherName;
std::string cropPublisherName;

std::string frameratePublisherName;

std::string fetchWeedServiceName;
std::string queryWeedServiceName;
std::string markUprootedServiceName;

Distance distanceTolerance;
float maxTimeDisappeared;
float minTimeValid;

float targetFps;

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

// query_weeds_service
bool query_weeds(urVision::QueryWeeds::Request &req, urVision::QueryWeeds::Response &res)
{
    std::vector<std::pair<ObjectID, Object>> objects;
    bool retValue = false;
    res.pairs.clear();

    weedTrackerLock.lock();
    retValue = p_weedTracker->getReadyObjects(objects);
    weedTrackerLock.unlock();

    if (retValue)
    {
        for (auto it = objects.begin(); it != objects.end(); it++)
        {
            urVision::WeedPair weedPair;
            weedPair.id = it->first;
            object_to_weed(it->second, weedPair.weed);
            res.pairs.push_back(weedPair);
        }
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

// msg callback
void new_weed_callback(const urVision::weedDataArray::ConstPtr& msg)
{
    ROS_DEBUG("Weed array received.");
    
    std::vector<Object> new_objs;

    for (auto weed : msg->weeds)
    {
        ROS_DEBUG("\tNew weed: x- %f    y- %f      z- %f      size- %f", weed.point.x, weed.point.y, weed.point.z, weed.size_cm);
        new_objs.push_back(weed_to_object(weed));
    }

    weedTrackerLock.lock();
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

// General parameters for this node
bool readGeneralParameters(ros::NodeHandle nodeHandle)
{
    if (!nodeHandle.getParam("weed_data_publisher", weedPublisherName)) return false;
    if (!nodeHandle.getParam("crop_data_publisher", cropPublisherName)) return false;
    if (!nodeHandle.getParam("framerate_publisher", frameratePublisherName)) return false;
    
    if (!nodeHandle.getParam("fetch_weed_service", fetchWeedServiceName)) return false;
    if (!nodeHandle.getParam("query_weeds_service", queryWeedServiceName)) return false;
    if (!nodeHandle.getParam("mark_uprooted_service", markUprootedServiceName)) return false;


    if (!nodeHandle.getParam("max_time_disappeared", maxTimeDisappeared)) return false;
    if (!nodeHandle.getParam("min_time_valid", minTimeValid)) return false;
    if (!nodeHandle.getParam("/target_fps", targetFps)) return false;

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
    p_weedTracker = new ObjectTracker(distanceTolerance, targetFps, maxTimeDisappeared, minTimeValid);
    weedTrackerLock.unlock();

    // Initialize cropTracker
    cropTrackerLock.lock();
    p_cropTracker = new ObjectTracker(distanceTolerance, targetFps, maxTimeDisappeared, minTimeValid, ObjectType::CROP);
    cropTrackerLock.unlock();

    // Subscriber to the weed publisher (from urVision)
    ros::Subscriber weedSub = nodeHandle.subscribe(weedPublisherName, 1000, new_weed_callback);
    ros::Subscriber cropSub = nodeHandle.subscribe(cropPublisherName, 1000, new_crop_callback);
    ros::Subscriber framerateSub = nodeHandle.subscribe(frameratePublisherName, 1, new_framerate_callback);

    // Services to provide to controller (and vision node)
    ros::ServiceServer fetchWeedService = nodeHandle.advertiseService(fetchWeedServiceName, fetch_weed);
    ros::ServiceServer queryWeedService = nodeHandle.advertiseService(queryWeedServiceName, query_weeds);
    ros::ServiceServer markUprootedService = nodeHandle.advertiseService(markUprootedServiceName, mark_uprooted);

    ros::spin();

    weedTrackerLock.lock();
    delete p_weedTracker;
    weedTrackerLock.unlock();
}