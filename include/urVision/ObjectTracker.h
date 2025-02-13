/* @file ObjectTracker.h
 *      @author emzarem
 *      
 *      This class handles the following functionality:
 *              -> Maintains sorted list of 'active' weeds
 *              -> Removes weeds from the list once they are out of scope
 *              -> Chooses the next weed to target
 */

#ifndef TRACKER_H
#define TRACKER_H

#include <gtest/gtest.h>

#include <map>
#include <sstream>
#include <vector>

/* Object
 *      @brief Struct holding centroid of object to track
 */
struct Object {
    // All values in cm
    float x;
    float y;
    float z;
    float size;
    double timestamp;

    // Internally used in tracker (cm/s)
    float x_vel;
    float y_vel;

    operator std::string() const
    {
        std::ostringstream oss;
        oss << "Object " << this << ":" << std::endl
            << "    x: " << this->x << std::endl
            << "    y: " << this->y << std::endl
            << "    z: " << this->z << std::endl;
        return oss.str();
    }
};

/* Describes the status of an object (i.e. if it is uprooted, if it is in progress, etc.) */
enum ObjectStatus
{
    DEFAULT = 0,
    READY = 1,
    IN_PROGRESS = 2,
    COMPLETED = 3
};

enum ObjectType
{
    WEED = 0,
    CROP = 1
}; 

/* ObjectID
 *      @brief Object ID type used to track
 */
typedef uint32_t ObjectID;
/* Distance
 *      @brief type used for centroid distances
 */
typedef float Distance;


/* ObjectTracker
 *      @brief Class to perform centroid tracking given bounding boxes
 *
 *      @note Constructor accepts both number of missing frames before removing object
 *            and 
 */
class ObjectTracker {
    public:
        ObjectTracker(Distance distTol, float velLpfCutoff, float targetFps, float maxTimeDisappeared, float minTimeValid, ObjectType trackerType = ObjectType::WEED);
        ~ObjectTracker();
       
        // Getters
        std::vector<Object> active_objects();
        size_t object_count();
        
        // Gets READY (and IN_PROGRESS) objects and IDS
        bool getReadyObjects(std::vector<std::pair<ObjectID, Object>>& ret_objs);
        // Gets COMPLETED objects and IDS
        bool getCompletedObjects(std::vector<std::pair<ObjectID, Object>>& ret_objs);

        float getXVelocity();
        float getYVelocity();

        // Sorted operations
        bool top(Object& to_ret);

        bool topValid(Object& to_ret);
        
        bool getObjectByID(Object& to_ret, ObjectID objectID);

        bool topValidAndUproot(Object& to_ret, ObjectID& ret_id);

        // Used to mark an object as uprooted (get object ide from call to topValidAndUproot)
        bool markUprooted(ObjectID uprootedId, bool success);

        // Modifiers
        void update(const std::vector<Object>& new_objs);

        void updateFramerate(float framerate);
        void updateVelocity(float xVelocity, float yVelocity);

        bool remove_object(const ObjectID id);

    private:
        void deregister_object(const ObjectID id);
        ObjectID register_object(const Object& obj);
        void cleanup_dissapeared();
        void update_active_object(ObjectID id, const Object& new_obj);
        void estimate_new_position(ObjectID id);
        inline float lowpass(float newVal, float oldVal, double dt) {
            return oldVal + (dt/m_lpfTau)*(newVal - oldVal);
        };

    private:
        Distance m_dist_tol;

        ObjectID m_next_id;
        /* Max num. frames an object can disappear in before being deregistered */
        uint32_t m_max_dissapeared_frms;
        /* Min num. of consecutive frames to be considered valid */
        uint32_t m_min_framecount;

        float m_framerate;
        float m_maxTimeDisappeared, m_minTimeValid;

        // Object type
        ObjectType m_type;

        double m_lastTimestamp;
  
        /* For Low pass filtering velocity */
        float m_lpfTau;
  
        std::vector<float> m_xVelAccumulator;
        std::vector<float> m_yVelAccumulator;

        // Global velocity to be used for all objects
        float m_xVelocity;
        float m_yVelocity;

        /* Main tracker lists */
        std::map<ObjectID, Object> m_active_objects;

        /* Each registered object will have an associated:
         *      framecount: the number of consecutive frames this object has appeared in
         *      dissapeared: total number of frames this alleged "object" has not appeared in
         *      uprooted: true if this weed has already been processed  
         */
        std::map<ObjectID, uint32_t> m_framecount;
        std::map<ObjectID, uint32_t> m_disappeared;
        std::map<ObjectID, ObjectStatus> m_status;

        std::vector<ObjectID> m_id_list;
};

#endif
