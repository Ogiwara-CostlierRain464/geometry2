#ifndef NEW_TIME_CACHE_H
#define NEW_TIME_CACHE_H

#include <cstdint>
#include <stdexcept>
#include <deque>
#include "transform_storage.h"
#include "rwlock.h"
#include "virtual_rwlock.h"
#include <boost/circular_buffer.hpp>

namespace new_tf2{

// https://gist.github.com/donny-dont/1471329#file-aligned_allocator-cpp-L51
  template <typename T, std::size_t Alignment>
  class aligned_allocator
  {
  public:

    // The following will be the same for virtually all allocators.
    typedef T * pointer;
    typedef const T * const_pointer;
    typedef T& reference;
    typedef const T& const_reference;
    typedef T value_type;
    typedef std::size_t size_type;
    typedef std::ptrdiff_t difference_type;

    T * address(T& r) const
    {
      return &r;
    }

    const T * address(const T& s) const
    {
      return &s;
    }

    std::size_t max_size() const
    {
      // The following has been carefully written to be independent of
      // the definition of size_t and to avoid signed/unsigned warnings.
      return (static_cast<std::size_t>(0) - static_cast<std::size_t>(1)) / sizeof(T);
    }


    // The following must be the same for all allocators.
    template <typename U>
    struct rebind
    {
      typedef aligned_allocator<U, Alignment> other;
    } ;

    bool operator!=(const aligned_allocator& other) const
    {
      return *this != other;
    }

    void construct(T * const p, const T& t) const
    {
      void * const pv = static_cast<void *>(p);

      new (pv) T(t);
    }

    void destroy(T * const p) const
    {
      p->~T();
    }

    // Returns true if and only if storage allocated from *this
    // can be deallocated from other, and vice versa.
    // Always returns true for stateless allocators.
    bool operator==(const aligned_allocator& other) const
    {
      return true;
    }


    // Default constructor, copy constructor, rebinding constructor, and destructor.
    // Empty for stateless allocators.
    aligned_allocator() { }

    aligned_allocator(const aligned_allocator&) { }

    template <typename U> aligned_allocator(const aligned_allocator<U, Alignment>&) { }

    ~aligned_allocator() { }


    // The following will be different for each allocator.
    T * allocate(const std::size_t n) const
    {
      // The return value of allocate(0) is unspecified.
      // Mallocator returns NULL in order to avoid depending
      // on malloc(0)'s implementation-defined behavior
      // (the implementation can define malloc(0) to return NULL,
      // in which case the bad_alloc check below would fire).
      // All allocators can return NULL in this case.
      if (n == 0) {
        return nullptr;
      }

      // All allocators should contain an integer overflow check.
      // The Standardization Committee recommends that std::length_error
      // be thrown in the case of integer overflow.
      if (n > max_size())
      {
        throw std::length_error("aligned_allocator<T>::allocate() - Integer overflow.");
      }

      // Mallocator wraps malloc().
      void * const pv = aligned_alloc(Alignment, n * sizeof(T));

      // Allocators should throw std::bad_alloc in the case of memory allocation failure.
      if (pv == nullptr)
      {
        throw std::bad_alloc();
      }

      return static_cast<T *>(pv);
    }

    void deallocate(T * const p, const std::size_t n) const
    {
      free(p);
    }


    // The following will be the same for all allocators that ignore hints.
    template <typename U>
    T * allocate(const std::size_t n, const U * /* const hint */) const
    {
      return allocate(n);
    }


    // Allocators are not required to be assignable, so
    // all allocators should have a private unimplemented
    // assignment operator. Note that this will trigger the
    // off-by-default (enabled under /Wall) warning C4626
    // "assignment operator could not be generated because a
    // base class assignment operator is inaccessible" within
    // the STL headers, but that warning is useless.
  private:
    aligned_allocator& operator=(const aligned_allocator&) = delete;
  };


  class TimeCache{
  public:
    static const int64_t DEFAULT_MAX_STORAGE_TIME = 60ULL * 1'000'000'000LL;

    explicit TimeCache(const ros::Duration &max_storage_time_ = ros::Duration().fromNSec(DEFAULT_MAX_STORAGE_TIME))
    : max_storage_time(max_storage_time_){}

    RWLock lock{};
    VRWLock v_lock{};

    typedef boost::circular_buffer<TransformStorage,
      aligned_allocator<TransformStorage, 128>> Deque;

    Deque storage{50};
    std::string authority;
    std::string frameName; // may need to comment out for performance
    ros::Duration max_storage_time;

    bool getData(const ros::Time &time,
                        TransformStorage & data_out,
                        std::string* error_str = nullptr){
      TransformStorage* p_temp_1;
      TransformStorage* p_temp_2;

      int num_nodes = findClosest(p_temp_1, p_temp_2, time, error_str);
      if (num_nodes == 0)
      {
        return false;
      }
      else if (num_nodes == 1)
      {
        data_out = *p_temp_1;
      }
      else if (num_nodes == 2)
      {
        if( p_temp_1->parent == p_temp_2->parent)
        {
          interpolate(*p_temp_1, *p_temp_2, time, data_out);
        }
        else
        {
          data_out = *p_temp_1;
        }
      }
      else
      {
        assert(0);
      }

      return true;
    }

    bool insertData(const TransformStorage& new_data);
    TimeCache* getParent(const ros::Time& time, std::string* error_str);
    inline std::pair<ros::Time, TimeCache*> getLatestTimeAndParent(){
      if(storage.empty()){
        return std::make_pair(ros::Time(), nullptr);
      }

      const auto &ts = storage.front();
      return std::make_pair(ts.stamp, ts.parent);
    }

    inline ros::Time getLatestTimestamp(){
      if (storage.empty()) return {}; //empty list case
      return storage.front().stamp;
    }

    uint8_t findClosest(TransformStorage* &one, TransformStorage* &two,
                               ros::Time target_time, std::string* error_str);
    inline void interpolate(const TransformStorage& one,
                            const TransformStorage& two,
                            const ros::Time &time,
                            TransformStorage& output){
      // Check for zero distance case
      if(two.stamp == one.stamp){
        output = two;
        return;
      }

      double ratio = (time - one.stamp).toSec() / (two.stamp - one.stamp).toSec();
      tf2::Vector3 tmp;
      tf2::Vector3 one_vec(one.vec[0], one.vec[1], one.vec[2]);
      tf2::Vector3 two_vec(two.vec[0], two.vec[1], two.vec[2]);

      tmp.setInterpolate3(one_vec, two_vec, ratio);
      output.vec[0] = tmp[0];
      output.vec[1] = tmp[1];
      output.vec[2] = tmp[2];

      output.rotation = tf2::slerp(one.rotation, two.rotation, ratio);
      output.stamp = time;
      output.parent = one.parent;
    }
    void pruneList();
  };
}

#endif //NEW_TIME_CACHE_H
