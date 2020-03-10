/* @file ObjectTracker.cpp
 *      @author emzarem
 *      
 *      This class handles the following functionality:
 *              -> Maintains sorted list of 'active' weeds
 *              -> Removes weeds from the list once they are out of scope
 *              -> Chooses the next weed to target
 */

#include "urVision/ObjectTracker.h"
#include <algorithm>    // sort
#include <numeric>      // iota
#include <math.h>       // sqrt

#include <ros/ros.h>

/* euclidean_distance
 *      @brief Calculates euclidean distance between two objects
 */
static Distance euclidean_distance(const Object& a, const Object& b)
{
    Distance delt_x = (Distance)a.x - (Distance)b.x;
    Distance delt_y = (Distance)a.y - (Distance)b.y;
    Distance delt_z = (Distance)a.z - (Distance)b.z;
    /*TODO: Do we want size to be in comparison? */
    // Distance delt_size = (Distance)a.size - (Distance)b.size;
    return sqrt(delt_x*delt_x + delt_y*delt_y + delt_z*delt_z);
}

/* ObjectTracker
 *      @brief constructor
 *
 *      @param max_dissapeared_frms : 
 *              max number of missed frames before object removed
 */
ObjectTracker::ObjectTracker(Distance distTol, float velLpfCutoff, float targetFps, float maxTimeDisappeared, float minTimeValid, ObjectType trackerType) :
    m_dist_tol(distTol), m_lpfTau(1.0/velLpfCutoff), m_maxTimeDisappeared(maxTimeDisappeared), m_minTimeValid(minTimeValid), m_framerate(targetFps), m_type(trackerType)
{
    m_next_id = 0;
    m_xVelocity = 0;
    m_yVelocity = 0;
    m_max_dissapeared_frms = (int)(floor(targetFps * m_maxTimeDisappeared));
    m_min_framecount = (int)(ceil(targetFps * m_minTimeValid));
    ROS_INFO("Tracker -- maxDisappearedFrms == %i; minValidFrames == %i", m_max_dissapeared_frms, m_min_framecount);
}

/* ~ObjectTracker
 *      @brief destructor
 */
ObjectTracker::~ObjectTracker()
{}

/* active_objects
 *      @brief returns vector of active objects
 */
std::vector<Object> ObjectTracker::active_objects()
{
    std::vector<Object> to_ret;

    for (auto id: m_id_list)
    {
        to_ret.push_back(m_active_objects[id]);
    }

    return to_ret;
}

bool ObjectTracker::getReadyObjects(std::vector<std::pair<ObjectID, Object>>& ret_objs)
{
    ret_objs.clear();

    for (auto id: m_id_list)
    {
        // IF object is READY or IN_PROGRESS
        if (m_status[id] == READY || m_status[id] == IN_PROGRESS)
        {
            ret_objs.push_back(std::make_pair(id, m_active_objects[id]));
        }
    }

    return true;
}

bool ObjectTracker::getCompletedObjects(std::vector<std::pair<ObjectID, Object>>& ret_objs)
{
    ret_objs.clear();

    for (auto id: m_id_list)
    {
        // IF COMPLETED and currently  IN FRAME
        if (m_status[id] == COMPLETED)
        {
            ret_objs.push_back(std::make_pair(id, m_active_objects[id]));
        }
    }

    return true;
}

float ObjectTracker::getXVelocity()
{
    size_t listSize = m_xVelAccumulator.size();
    if (listSize > 0)
        // return the average from the last update
        return std::accumulate(m_xVelAccumulator.begin(), m_xVelAccumulator.end(), 0.0) / (float)listSize;
    
    return 0;
}

float ObjectTracker::getYVelocity()
{
    size_t listSize = m_yVelAccumulator.size();
    if (listSize > 0)
        // return the average from the last update
        return std::accumulate(m_yVelAccumulator.begin(), m_yVelAccumulator.end(), 0.0) / (float)listSize;

    return 0;
}

/* object_count
 *      @brief returns number of currently tracked objects
 */
size_t ObjectTracker::object_count()
{
    return m_active_objects.size();
}

/* markUprooted
 *      @brief mark the given object as COMPLETED
 */
bool ObjectTracker::markUprooted(ObjectID uprootedId, bool success)
{
    try {
        if (m_status.at(uprootedId) == IN_PROGRESS) {
            m_status.at(uprootedId) = (success ? COMPLETED : READY);
            return true;
        }
    } catch (const std::out_of_range& oor) {
        // Not a valid key
    }
    return false;
}

// Gets an object by it's specific ID
bool ObjectTracker::getObjectByID(Object& to_ret, ObjectID objectID)
{
    if (object_count() == 0)
        return false;

    auto itr =  m_active_objects.find(objectID);

    if (itr == m_active_objects.end()) 
    {
        return false;
    } 
    else 
    {
        to_ret = itr->second;
        return true;
    }
}

/* topValidAndUproot
 *      @brief returns the largest Object with other valid parameters (as defined by operator>)
 *                  and marks it as uprooted
 *
 *      @param  to_ret  : The top object is returned through this
 *      @returns   bool : True if top object exists, false otherwise
 */
bool ObjectTracker::topValidAndUproot(Object& to_ret, ObjectID& ret_id)
{
    if (object_count() == 0)
        return false;

    /* Go through all active objects and check for 'valid' conditions */
    for (auto itr = m_id_list.begin(); itr != m_id_list.end(); itr++)
    {
        // IF status is READY or IN_PROGRESS
        if (m_status[*itr] == READY)
        {
            // Set status to be IN_PROGRESS
            m_status[*itr] = IN_PROGRESS;
            to_ret = m_active_objects[*itr];
            ret_id = *itr;
            return true;
        }
    }

    return false;
}

/* topValid
 *      @brief returns the largest Object with other valid parameters (as defined by operator>)
 *
 *      @param  to_ret  : The top object is returned through this
 *      @returns   bool : True if top object exists, false otherwise
 */
bool ObjectTracker::topValid(Object& to_ret)
{
    if (object_count() == 0)
        return false;

    /* Go through all active objects (sorted in m_id_list) and check for 'valid' conditions */
    for (auto itr = m_id_list.begin(); itr != m_id_list.end(); itr++)
    {
        // If status is READY OR in progress (because calls to this function are just to show valid objects)
        if (m_status[*itr] == READY ||
            m_status[*itr] == IN_PROGRESS)
        {
            to_ret = m_active_objects[*itr];
            return true;
        }
    }

    return false;
}

/* top
 *      @brief returns the largest Object (as defined by operator>)
 *
 *      @param  to_ret  : The top object is returned through this
 *      @returns   bool : True if top object exists, false otherwise
 */
bool ObjectTracker::top(Object& to_ret)
{
    if (object_count() == 0)
        return false;

    to_ret = m_active_objects[m_id_list[0]];
    return true;
}

/* update
 *      @brief updates the active list of objects
 *
 *      @details
 *              1) calculates a distance matrix between current objects and new:
 *                             __  new_objects --->  __
 *             active_objects | D_1,1 . . . . . . D_1,n|
 *                  |         |  .       .             |
 *                  |         |  .           .         |
 *                  |         | D_m,1             D_m,n|
 *                  V         |__                    __|
 *
 *               2) sorts through each row to have min in 1st col
 *                     -Then the rows are sorted by the min distance in each
 *
 *      @param new_objs : list of centroids
 */
void ObjectTracker::update(const std::vector<Object>& new_objs)
{
    m_xVelAccumulator.clear();
    m_yVelAccumulator.clear();

    if (new_objs.size() == 0) 
    {
        // If we didn't get any new objects, all are dissapeared
        for (auto itr = m_disappeared.begin(); itr != m_disappeared.end(); itr++)
        {
            m_disappeared[itr->first]++;
            m_framecount[itr->first] = 0;
            m_status[itr->first] = DEFAULT;
        }
    }

    else if (m_active_objects.size() == 0)
    {
        ROS_DEBUG("Tracker -- no current objects, registering all objects");
        
        // If we don't have any current objects register all the new ones
        for (auto itr = new_objs.begin(); itr != new_objs.end(); itr++)
        {
            register_object(*itr);
        }
    }

    else
    {
        /* 1. Calculate distance between each current point and each new point */
        int32_t m = m_active_objects.size();
        int32_t n = new_objs.size();
        std::vector< std::vector<Distance> > dist_matrix(m, std::vector<Distance>(n));

        for (int i = 0; i < m; i++)
        {
            for (int j = 0; j < n; j++) 
            {
                Object crnt_obj = m_active_objects[m_id_list[i]];
                dist_matrix[i][j] = euclidean_distance(crnt_obj, new_objs[j]);
            }
        }

        /* 2. Sort by distance */  
        // Keep track of indices -- new objs
        // | 0 1 - - - - - n |
        // | 0 1 - - - - - n |
        // | - - - - - - - - |
        // | - - - - - - - - |
        // | 0 1 - - - - - n |
        std::vector<std::vector<size_t> > sorted_ids(m, std::vector<size_t>(n));
        for (int i = 0; i < m; i++)
            std::iota(sorted_ids[i].begin(), sorted_ids[i].end(), 0);

        // Keep track of indices -- active objs
        // | 0 1 - - - - - m |
        std::vector<size_t> active_obj_ids(m);
        std::iota(active_obj_ids.begin(), active_obj_ids.end(), 0);

        // Sort intra-row (find closest new object to each old object)
        // | 0 1 - <---> - n |
        // | 0 1 - <---> - n |
        // | - - - <---> - - |
        // | - - - <---> - - |
        // | 0 1 - <---> - n |
        for (int i = 0; i < m; i++)
        {
             std::sort(sorted_ids[i].begin(), sorted_ids[i].end(), 
                    [&dist_matrix, &i](const size_t& a, const size_t& b) -> bool
                    {
                        return dist_matrix[i][a] < dist_matrix[i][b];
                    });
        }
        
        // Sort the rows based on min distance in each row
        // | 0 1 - <---> - m |
        std::sort(active_obj_ids.begin(), active_obj_ids.end(), 
                [&dist_matrix, &sorted_ids](const size_t& a, const size_t& b) -> bool
                {
                    return dist_matrix[a][sorted_ids[a][0]] < dist_matrix[b][sorted_ids[b][0]];
                });
        
        /* 3. Find matching objects */
        std::map<size_t, int> used_cols;

        // Loop over each active object
        for (auto itr = active_obj_ids.begin(); itr != active_obj_ids.end(); itr++)
        {
            bool found_update = false;
            
            // loop over each new object in the row 
            for (auto sub_itr = sorted_ids[*itr].begin(); sub_itr != sorted_ids[*itr].end(); sub_itr++)
            {
                // If this matches
                if (used_cols.count(*sub_itr) == 0 && dist_matrix[*itr][*sub_itr] < m_dist_tol)
                {
                    // We found our tracked object!
                    used_cols[*sub_itr] = 1; // update that we used this object
                    // Increment the number of consecutive frames this was found in
                    // This ensures the first found object is stored (it should be closest in dist_)
                    if (!found_update)
                    {
                        update_active_object(m_id_list[*itr], new_objs[*sub_itr]);
                        //m_active_objects[m_id_list[*itr]] = new_objs[*sub_itr];
                        m_framecount[m_id_list[*itr]]++;
                        // Mark as ready if framecount is appropriate
                        if (m_status[m_id_list[*itr]] == DEFAULT && m_framecount[m_id_list[*itr]] >= m_min_framecount)
                        {
                            m_status[m_id_list[*itr]] = READY;
                            Object obj = m_active_objects[m_id_list[*itr]];

                            if (m_type == ObjectType::WEED)
                            {
                                ROS_INFO("WEED READY @ (x,y,z,size) = (%.2f,%.2f,%.2f,%.2f)", obj.x, obj.y, obj.z, obj.size);
                            }
                            else if (m_type == ObjectType::CROP)
                            {
                                ROS_DEBUG("CROP READY @ (x,y,z,size) = (%.2f,%.2f,%.2f,%.2f)", obj.x, obj.y, obj.z, obj.size);
                            }
                        }
                        found_update = true;
                    }
                    else
                    {
                        // Take the average
                        m_active_objects[m_id_list[*itr]].x =  (m_active_objects[m_id_list[*itr]].x + new_objs[*sub_itr].x) / 2;
                        m_active_objects[m_id_list[*itr]].y =  (m_active_objects[m_id_list[*itr]].y + new_objs[*sub_itr].y) / 2;
                    }
                }
            }

            // If not updated its missing this frame
            if (!found_update)
            {
                // Estimate new position
                estimate_new_position(m_id_list[*itr]);

                // Mark as dissapeared and reset framecount
                m_disappeared[m_id_list[*itr]]++;

                if (m_status[m_id_list[*itr]] == DEFAULT)
                {
                    m_framecount[m_id_list[*itr]] = 0;
                }
            }
        }

        // Add any new objects in the scene
        for (auto x : sorted_ids[0])
        {
           if (used_cols.count(x) == 0)
           {
               register_object(new_objs[x]);
           }
        }
    }

    /* 4. Update missing objects */
    cleanup_dissapeared();
}

void ObjectTracker::updateFramerate(float framerate)
{
    m_max_dissapeared_frms = (int)(floor(framerate * m_maxTimeDisappeared));
    m_min_framecount = (int)(ceil(framerate * m_minTimeValid));
    m_framerate = framerate;
}

void ObjectTracker::updateVelocity(float xVelocity, float yVelocity)
{
    m_xVelocity = xVelocity;
    m_yVelocity = yVelocity;
}

/* register_object
 *      @brief registers an object as active
 */
ObjectID ObjectTracker::register_object(const Object& obj)
{
    // Insertion sort on IDs
    auto id_itr = m_id_list.begin();
    
    // This effectively does the sorting based on y-value
    while (id_itr != m_id_list.end() && m_active_objects[*id_itr].y < obj.y) {id_itr++;}
    m_id_list.insert(id_itr, m_next_id);
    
    m_active_objects[m_next_id] = obj;
    m_disappeared[m_next_id] = 0;
    m_framecount[m_next_id] = 1;
    m_status[m_next_id] = DEFAULT;

    return m_next_id++;
}

/* deregister_object
 *      @brief removes object with id from registry
 */
void ObjectTracker::deregister_object(const ObjectID id)
{
    m_active_objects.erase(id);
    m_disappeared.erase(id);
    m_framecount.erase(id);
    m_status.erase(id);
    for (auto itr = m_id_list.begin(); itr != m_id_list.end(); itr++)
    {
        if (*itr == id)
        {
            m_id_list.erase(itr);
            break;
        }
    }
}


/* cleanup_dissapeared
 *      @brief removes any objects whose dissapeared value > max_dissapeared
 */
void ObjectTracker::cleanup_dissapeared()
{
    for (auto itr = m_disappeared.begin(); itr != m_disappeared.end(); itr++)
    {
        // Only if it never became ready
        if (m_status[itr->first] == DEFAULT && itr->second > m_max_dissapeared_frms)
        {
            deregister_object(itr->first);
        }
    }
}


/* estimate_new_position
 *      @brief Estimate the position based on velocity
 */
void ObjectTracker::estimate_new_position(ObjectID id)
{
    Object& toUpdate = m_active_objects[id];
    
    // Use global velocity estimate!
    double currTime = ros::Time::now().toSec();
    double dt = currTime - toUpdate.timestamp;
    float dx = m_xVelocity * dt;
    float dy = m_yVelocity * dt;

    toUpdate.x += dx;
    toUpdate.y += dy;
    toUpdate.timestamp = currTime; 
}

/* update_active_object
 *      @brief Update the values of an active object given a new object
 */
void ObjectTracker::update_active_object(ObjectID id, const Object& new_obj)
{
    Object old = m_active_objects[id];
    Object& toUpdate = m_active_objects[id];

    toUpdate = new_obj;
    double dt = toUpdate.timestamp - old.timestamp;
    float new_x_vel = (toUpdate.x - old.x)/dt;
    float new_y_vel = (toUpdate.y - old.y)/dt;

    // LPF
    toUpdate.x_vel = lowpass(new_x_vel, old.x_vel, dt);
    toUpdate.y_vel = lowpass(new_y_vel, old.y_vel, dt);

    // Push to the accumulators
    m_xVelAccumulator.push_back(toUpdate.x_vel);
    m_yVelAccumulator.push_back(toUpdate.y_vel);
}







