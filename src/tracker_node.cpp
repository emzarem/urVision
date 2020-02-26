#include <ros/ros.h>

#include "urVision/ObjectTracker.h"

// Srv and msg types
#include <urGovernor/FetchWeed.h>
#include <urGovernor/MarkUprooted.h>
#include <urVision/weedDataArray.h>

#include <vector>

static ObjectTracker* p_tracker;

// Parameters to read from configs
std::string weedPublisherName;
std::string fetchWeedServiceName;
std::string queryWeedServiceName;
std::string markUprootedServiceName;

Distance distanceTolerance;
float maxTimeDisappeared;
int minTimeValid;

float targetFps;

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

/* Service Definitions */

/* fetch_weed_service (called by controller):
 *      Response includes a valid weed to be uprooted.
 *      Note that this indicates the object is IN_PROGRESS or 'being uprooted'
 */
bool fetch_weed(urGovernor::FetchWeed::Request &req, urGovernor::FetchWeed::Response &res)
{
    Object top_valid_obj;
    ObjectID obj_id = 0;

    bool retValue = p_tracker->topValidAndUproot(top_valid_obj, obj_id);

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
bool query_weed(urGovernor::FetchWeed::Request &req, urGovernor::FetchWeed::Response &res)
{
    Object top_valid_obj;

    bool retValue = p_tracker->topValid(top_valid_obj);

    if (retValue)
    {
        // Set response data to include weed fetched from the top
        object_to_weed(top_valid_obj, res.weed);
    }
    else
    {
        ROS_DEBUG("query_weed_service: Issues with calls to tracker.");
    }

    return retValue;
}

// mark_uprooted_service
bool mark_uprooted(urGovernor::MarkUprooted::Request &req, urGovernor::MarkUprooted::Response &res)
{
    bool retValue = p_tracker->markUprooted(req.tracking_id, req.success);

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
        ROS_DEBUG("\tNew weed: x- %f    y- %f      z- %f      size- %f", weed.x_cm, weed.y_cm, weed.z_cm, weed.size_cm);
        new_objs.push_back(weed_to_object(weed));
    }

    p_tracker->update(new_objs);
}

// General parameters for this node
bool readGeneralParameters(ros::NodeHandle nodeHandle)
{
    if (!nodeHandle.getParam("weed_data_publisher", weedPublisherName)) return false;
    if (!nodeHandle.getParam("fetch_weed_service", fetchWeedServiceName)) return false;
    if (!nodeHandle.getParam("query_weed_service", queryWeedServiceName)) return false;
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

    // Set minValidFrames and maxTimeDisappeared based on frame rate
    int maxDisappearedFrms = (int)(floor((1.0*targetFps) * maxTimeDisappeared));
    int minValidFrames = (int)(ceil((1.0*targetFps) * minTimeValid));
    ROS_INFO("Tracker -- maxDisappearedFrms == %i; minValidFrames == %i", maxDisappearedFrms, minValidFrames);
    p_tracker = new ObjectTracker(distanceTolerance, maxDisappearedFrms, minValidFrames);

    // Subscriber to the weed publisher (from urVision)
    ros::Subscriber sub = nodeHandle.subscribe(weedPublisherName, 1000, new_weed_callback);

    // Services to provide to controller (and vision node)
    ros::ServiceServer fetchWeedService = nodeHandle.advertiseService(fetchWeedServiceName, fetch_weed);
    ros::ServiceServer queryWeedService = nodeHandle.advertiseService(queryWeedServiceName, query_weed);
    ros::ServiceServer markUprootedService = nodeHandle.advertiseService(markUprootedServiceName, mark_uprooted);

    ros::spin();

    delete p_tracker;
}