#ifndef TF2_SILO_BUFFER_CORE_H
#define TF2_SILO_BUFFER_CORE_H

#include "../include/tf2/transform_storage.h"
#include "../include/tf2/write_stat.h"

#include <boost/signals2.hpp>

#include <string>
#include <tbb/concurrent_unordered_map.h>
#include <tbb/concurrent_hash_map.h>
#include <tbb/concurrent_vector.h>

#include "ros/duration.h"
#include <ctime>
//#include "geometry_msgs/TwistStamped.h"
#include "geometry_msgs/TransformStamped.h"
#include "virtual_rwlock.h"
#include "../include/tf2/read_stat.h"
#include "read_checker.h"

//////////////////////////backwards startup for porting
//#include "tf/tf.h"

#include <boost/unordered_map.hpp>
#include <boost/thread/mutex.hpp>
#include <boost/function.hpp>

// what you need for virtual lock?
// local read area: trivial
// local write: no need to do.
// version number for

namespace tf2{
  class TimeCacheInterface;
}

namespace silo_tf2
{

  typedef std::pair<ros::Time, tf2::CompactFrameID> P_TimeAndFrameID;
  typedef uint32_t TransformableCallbackHandle;
  typedef uint64_t TransformableRequestHandle;

  typedef tf2::TransformStorage* TimeCacheInterfacePtr;

  enum TransformableResult
  {
    TransformAvailable,
    TransformFailure,
  };

/** \brief A Class which provides coordinate transforms between any two frames in a system.
 *
 * This class provides a simple interface to allow recording and lookup of
 * relationships between arbitrary frames of the system.
 *
 * libTF assumes that there is a tree of coordinate frame transforms which define the relationship between all coordinate frames.
 * For example your typical robot would have a transform from global to real world.  And then from base to hand, and from base to head.
 * But Base to Hand really is composed of base to shoulder to elbow to wrist to hand.
 * libTF is designed to take care of all the intermediate steps for you.
 *
 * Internal Representation
 * libTF will store frames with the parameters necessary for generating the transform into that frame from it's parent and a reference to the parent frame.
 * Frames are designated using an std::string
 * 0 is a frame without a parent (the top of a tree)
 * The positions of frames over time must be pushed in.
 *
 * All function calls which pass frame ids can potentially throw the exception tf::LookupException
 */
  class SiloBufferCore
  {
  public:
    /************* Constants ***********************/
    static const int DEFAULT_CACHE_TIME = INT_MAX;  //!< The default amount of time to cache data in seconds
    static const uint64_t MAX_GRAPH_DEPTH = 1000'000'000'000UL;  //!< Maximum graph search depth (deeper graphs will be assumed to have loops)

    /** Constructor
     * \param interpolating Whether to interpolate, if this is false the closest value will be returned
     * \param cache_time How long to keep a history of transforms in nanoseconds
     *
     */
    SiloBufferCore(ros::Duration cache_time_ = ros::Duration(DEFAULT_CACHE_TIME));
    virtual ~SiloBufferCore(void);

    /** \brief Add transform information to the tf data structure
     * \param transform The transform to store
     * \param authority The source of the information for this transform
     * \param is_static Record this transform as a static transform.  It will be good across all time.  (This cannot be changed after the first call.)
     * \return True unless an error occured
     */
    bool setTransform(const geometry_msgs::TransformStamped& transform, const std::string & authority, bool is_static = false) noexcept;

    /** \brief Add transform information to the tf data structure
     * \param transforms The transform to store
     * \param authority The source of the information for this transform
     * \param is_static Record this transform as a static transform.  It will be good across all time.  (This cannot be changed after the first call.)
     * \return True unless an error occured
     */
    bool setTransforms(const std::vector<geometry_msgs::TransformStamped> &transforms, const std::string & authority, bool is_static = false, WriteStat *result = nullptr) noexcept;

    geometry_msgs::TransformStamped
    lookupLatestTransform(const std::string& target_frame, const std::string& source_frame, ReadStat *stat = nullptr) const noexcept(false);

  private:

    /******************** Internal Storage ****************/

    /** \brief The pointers to potential frames that the tree can be made of.
     * The frames will be dynamically allocated at run time when set the first time. */
    std::vector<TimeCacheInterfacePtr> frames_;
    /** \brief Used for high-granularity locking. */
//  mutable tbb::concurrent_vector<RWLockPtr> frame_each_mutex_{};
    mutable std::array<VRWLock, 1'000'005> *frame_each_mutex_ = nullptr;

    /** \brief A map from string frame ids to CompactFrameID */
    boost::unordered_map<std::string, tf2::CompactFrameID> frameIDs_;
    /** \brief A map from CompactFrameID frame_id_numbers to string for debugging and output */
    std::vector<std::string> frameIDs_reverse;
    /** \brief A map to lookup the most recent authority for a given frame */
    boost::unordered_map<tf2::CompactFrameID, std::string> frame_authority_;

    /// How long to cache transform history
    ros::Duration cache_time_;

    struct TransformableRequest
    {
      ros::Time time;
      TransformableRequestHandle request_handle;
      TransformableCallbackHandle cb_handle;
      tf2::CompactFrameID target_id;
      tf2::CompactFrameID source_id;
      std::string target_string;
      std::string source_string;
    };
    // NOTE: to support erase, we can't use tbb::concurrent_vector!!
    std::vector<TransformableRequest> transformable_requests_{};
    VRWLock transformable_requests_mutex_{};
    std::atomic_uint64_t transformable_requests_counter_{};

    struct RemoveRequestByCallback;
    struct RemoveRequestByID;

    // Backwards compatability for tf message_filter
    typedef boost::signals2::signal<void(void)> TransformsChangedSignal;
    /// Signal which is fired whenever new transform data has arrived, from the thread the data arrived in
    TransformsChangedSignal _transforms_changed_;


    /************************* Internal Functions ****************************/

    /** \brief An accessor to get a frame, which will throw an exception if the frame is no there.
     * \param frame_number The frameID of the desired Reference Frame
     *
     * This is an internal function which will get the pointer to the frame associated with the frame id
     * Possible Exception: tf::LookupException
     */
    TimeCacheInterfacePtr getFrame(tf2::CompactFrameID c_frame_id) const noexcept;

    TimeCacheInterfacePtr allocateFrame(tf2::CompactFrameID cfid, bool is_static) noexcept;


    bool warnFrameId(const char* function_name_arg, const std::string& frame_id) const noexcept;
    tf2::CompactFrameID validateFrameId(const char* function_name_arg, const std::string& frame_id) const;

    /// String to number for frame lookup with dynamic allocation of new frames
    tf2::CompactFrameID lookupFrameNumber(const std::string& frameid_str) const noexcept;

    /// String to number for frame lookup with dynamic allocation of new frames
    tf2::CompactFrameID lookupOrInsertFrameNumber(const std::string& frameid_str) noexcept;

    ///Number to string frame lookup may throw LookupException if number invalid
    const std::string& lookupFrameString(tf2::CompactFrameID frame_id_num) const noexcept(false);

    void createConnectivityErrorString(tf2::CompactFrameID source_frame, tf2::CompactFrameID target_frame, std::string* out) const noexcept(false);

    template<typename F>
    int walkToTopParentLatest(F& f, tf2::CompactFrameID target_id, tf2::CompactFrameID source_id, std::string* error_string, ReadChecker &read_checker, ReadStat *stat = nullptr) const noexcept;

    //Whether it is safe to use canTransform with a timeout. (If another thread is not provided it will always timeout.)
    bool using_dedicated_thread_;

  public:
    friend class TestBufferCore; // For unit testing

  };
}

#endif //TF2_SILO_BUFFER_CORE_H
