/// Copyright (C) 2026 Mitsubishi Electric Research Laboratories (MERL)
/// SPDX-License-Identifier: AGPL-3.0-or-later
///
/// @file queue.hpp
/// @brief Thread-safe queue wrapper based on std::list.
/// @details Adds lock-aware methods for insertion, lookup, iteration-friendly
/// copying, and safe popping. This container is intended for shared
/// producer-consumer style data flows in the pipeline.
/// @author Pedro Miraldo
/// @author Valter Piedade
#ifndef QUEUE_HPP
#define QUEUE_HPP

#include <types/mutex.hpp>
#include <types/output.hpp>
#include <types/vector.hpp>

#include <list>
#include <shared_mutex>

namespace slam_mer {

/// @brief Thread-safe queue wrapper around `std::list`.
/// @tparam type Type of elements stored in the queue.
template <typename type>
class queue_t : public std::list<type> {
public:
  /// Inherit constructors
  using std::list<type>::list;

  /// Typedefs
  using shared_ptr = std::shared_ptr<queue_t<type>>;

  /// Simple thread-safe function that inserts an element at the end of the
  /// queue
  /// @param item_ Element appended to the back of the queue.
  /// @return Queue size after the insertion.
  inline size_t push_back_safe(const type item_) {
    write_lock_t lock(_mutex);
    this->push_back(item_);
    return this->size();
  }

  /// Simple thread-safe function that checks if the queue is empty
  /// @return `true` when no elements are stored, `false` otherwise.
  [[nodiscard("Use to check if the queue is empty.")]]
  inline bool empty_safe() const {
    read_lock_t lock(_mutex);
    return this->empty();
  }

  /// Simple thread-safe function that returns the size of the queue
  /// @return Number of elements currently stored in the queue.
  [[nodiscard("Use to get the size of the queue.")]]
  inline size_t size_safe() const {
    read_lock_t lock(_mutex);
    return this->size();
  }

  /// Non thread-safe function to pop the front (first) element of the queue
  /// @return Front element wrapped in `output_t`, or `fail_t::IS_EMPTY`.
  [[nodiscard("Use to pop the front (first) element of the queue.")]]
  inline output_t<type> pop_front() {
    if (this->empty()) return fail_t::IS_EMPTY;
    auto item_ptr = this->begin();
    type item = *item_ptr;  // Do a copy of the item
    this->erase(item_ptr);
    return item;
  }

  /// Thread-safe function to pop the front (first) element of the queue
  /// @return Front element wrapped in `output_t`, or `fail_t::IS_EMPTY`.
  [[nodiscard("Use to pop the front (first) element of the queue.")]]
  inline output_t<type> pop_front_safe() {
    write_lock_t lock(_mutex);
    return this->pop_front();
  }

  /// @brief Non-thread-safe function to pop the front (first) n elements.
  /// @param numberElements_ Maximum number of elements to remove.
  /// @return Output vector with the popped elements.
  [[nodiscard("Use to pop the front (first) n elements of the queue.")]]
  vector_t<type> pop_front(const size_t numberElements_) {
    vector_t<type> items;
    for (size_t item = 0; item < numberElements_; ++item) {
      if (this->empty()) break;
      items.push_back(this->pop_front());
    }
    return items;
  }

  /// @brief Thread-safe function to pop the front (first) n elements.
  /// @param numberElements_ Maximum number of elements to remove.
  /// @return Output vector with the popped elements.
  [[nodiscard("Use to pop the front (first) n elements of the queue.")]]
  vector_t<type> pop_front_safe(const size_t numberElements_) {
    write_lock_t lock(_mutex);
    return this->pop_front(numberElements_);
  }

  /// Shared mutex lock functions
  inline void lock_read() const { _mutex.lock_shared(); }

  /// Shared mutex unlock functions
  inline void unlock_read() const { _mutex.unlock_shared(); }

  /// Exclusive mutex lock functions
  inline void lock() const { _mutex.lock(); }

  /// Exclusive mutex unlock functions
  inline void unlock() const { _mutex.unlock(); }

  /// Print all elements in the queue (for debugging purposes)
  /// @param prefix_ Prefix printed before the queue contents.
  void print(const std::string& prefix_) const {
    using std::print;
    read_lock_t lock(_mutex);
    print("Queue elements {}: ", prefix_);
    for (const type& item : *this)
      print("{}; ", item);
    print("\n");
  }

private:
  /// Mutex for thread safety
  mutable std::shared_mutex _mutex;
};

}  // namespace slam_mer

#endif  // QUEUE_HPP
