#include "time_cache.h"

namespace new_tf2{

namespace cache { // Avoid ODR collisions https://github.com/ros/geometry2/issues/175
// hoisting these into separate functions causes an ~8% speedup.  Removing calling them altogether adds another ~10%
    void createExtrapolationException1(ros::Time t0, ros::Time t1, std::string* error_str)
    {
      if (error_str)
      {
        std::stringstream ss;
        ss << "Lookup would require extrapolation at time " << t0 << ", but only time " << t1 << " is in the buffer";
        *error_str = ss.str();
      }
    }

    void createExtrapolationException2(ros::Time t0, ros::Time t1, std::string* error_str)
    {
      if (error_str)
      {
        std::stringstream ss;
        ss << "Lookup would require extrapolation into the future.  Requested time " << t0 << " but the latest data is at time " << t1;
        *error_str = ss.str();
      }
    }

    void createExtrapolationException3(ros::Time t0, ros::Time t1, std::string* error_str)
    {
      if (error_str)
      {
        std::stringstream ss;
        ss << "Lookup would require extrapolation into the past.  Requested time " << t0 << " but the earliest data is at time " << t1;
        *error_str = ss.str();
      }
    }
  } // namespace cache

bool TimeCache::insertData(const TransformStorage& new_data){
  auto storage_it = storage.begin();

  if(storage_it != storage.end())
  {
    if (storage_it->stamp > new_data.stamp + max_storage_time)
    {
      return false;
    }
  }

  while(storage_it != storage.end())
  {
    if (storage_it->stamp <= new_data.stamp)
      break;
    storage_it++;
  }
  storage.insert(storage_it, new_data);

  pruneList();
  return true;
}

TimeCache* TimeCache::getParent(const ros::Time& time, std::string* error_str){
  TransformStorage* p1;
  TransformStorage* p2;

  int num_nodes = findClosest(p1, p2, time, error_str);
  if(num_nodes == 0){
    return nullptr;
  }

  return p1->parent;
}

uint8_t TimeCache::findClosest(TransformStorage* &one, TransformStorage* &two,
                    ros::Time target_time, std::string* error_str){
  if(storage.empty()){
    return 0;
  }

  // If time == 0 return the latest
  if(target_time.isZero()){
    one = &storage.front();
    return 1;
  }

  // One value stored
  if(++storage.begin() == storage.end()){
    auto &ts = *storage.begin();
    if(ts.stamp == target_time){
      one = &ts;
      return 1;
    }else{
      cache::createExtrapolationException1(target_time, ts.stamp, error_str);
      return 0;
    }
  }

  ros::Time latest_time = (*storage.begin()).stamp;
  ros::Time earliest_time = (*(storage.rbegin())).stamp;

  if(target_time == latest_time){
    one = &(*storage.begin());
    return 1;
  }else if(target_time == earliest_time){
    one = &(*storage.rbegin());
    return 1;
  }
  // Catch cases that would require extrapolation
  else if(target_time > latest_time){
    cache::createExtrapolationException2(target_time, latest_time, error_str);
    return 0;
  }
  else if(target_time < earliest_time){
    cache::createExtrapolationException3(target_time, earliest_time, error_str);
    return 0;
  }

  //At least 2 values stored
  //Find the first value less than the target value
  Deque::iterator storage_it;
  TransformStorage storage_target_time;
  storage_target_time.stamp = target_time;
  storage_it = std::lower_bound(
    storage.begin(),
    storage.end(),
    storage_target_time, std::greater<TransformStorage>());

  one = &*(storage_it); // Older
  two = &*(--storage_it); // Newer
  return 2;
}


void TimeCache::pruneList(){
  ros::Time latest_time = storage.begin()->stamp;

  while(!storage.empty() && storage.back().stamp + max_storage_time < latest_time)
  {
    storage.pop_back();
  }
}


}