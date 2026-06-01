/// Copyright (C) 2026 Mitsubishi Electric Research Laboratories (MERL)
/// SPDX-License-Identifier: AGPL-3.0-or-later
///
/// @file vector.hpp
/// @brief Thread-safe vector wrapper built on std::vector.
/// @details Adds synchronized read/write access, helper methods, and conversion
/// utilities for shared dynamic arrays used across SLAM modules. The
/// interface is designed for frequent concurrent reads with guarded
/// writes.
/// @author Pedro Miraldo
/// @author Valter Piedade
#ifndef TYPE_SAFE_VECTOR_HPP
#define TYPE_SAFE_VECTOR_HPP

// the necessary typedefs
#include <types/mutex.hpp>

// Standard libraries
#include <memory>
#include <shared_mutex>
#include <vector>

namespace slam_mer {

/// @brief Thread-safe wrapper around `std::vector`.
///
/// This container is used across SLAM modules and protects concurrent access
/// with a shared mutex.
template <typename type>
class vector_t : public std::vector<type> {

public:
  // Inerhit constructors
  using std::vector<type>::vector;

  /// Shared pointer type
  using shared_ptr = std::shared_ptr<vector_t<type>>;

  /// Make shared pointer function
  /// @return Shared pointer owning a new empty thread-safe vector.
  static vector_t<type>::shared_ptr make_shared() {
    return std::make_shared<vector_t<type>>();
  }

  /// Make shared pointer function from data
  /// @param data_ Initial contents copied into the new shared vector.
  /// @return Shared pointer owning a copy of `data_`.
  static vector_t<type>::shared_ptr make_shared(vector_t<type> data_) {
    return std::make_shared<vector_t<type>>(data_);
  }

  /// Make shared pointer function from std::vector
  /// @param data_ Initial standard vector contents to copy.
  /// @return Shared pointer owning a vector initialized from `data_`.
  static vector_t<type>::shared_ptr make_shared(std::vector<type> data_) {
    return std::make_shared<vector_t<type>>(data_);
  }

  /// Empty constructor for the Visualization Vector object.
  vector_t() : std::vector<type>() {}

  /// Constructor with initial data
  /// @param data_ First element inserted during construction.
  vector_t(type data_) : std::vector<type>() { this->push_back(data_); }

  /// Copy constructor
  /// @param other_ Source vector copied while holding a shared read lock.
  vector_t(const vector_t<type>& other_) : std::vector<type>() {
    write_lock_t lock(this->_mutex);
    if (this != &other_) {
      other_.shared_lock();
      std::vector<type>::operator=(other_);
      other_.unlock_shared();
    }
  }

  /// Destroy the Visualization Vector object.
  ~vector_t() {}

  /// Equal operator
  /// @param other_ Source vector whose contents replace the current vector.
  /// @return Reference to this vector after synchronized assignment.
  vector_t<type>& operator=(const vector_t<type>& other_) {
    write_lock_t lock(this->_mutex);
    if (this != &other_) {
      other_.shared_lock();
      std::vector<type>::operator=(other_);
      other_.unlock_shared();
    }
    return *this;
  }

  /// Assignment operator from std::vector
  /// @param other_ Standard vector copied into this container.
  /// @return Reference to this vector after assignment.
  vector_t<type>& operator=(std::vector<type> other_) {
    write_lock_t lock(this->_mutex);
    std::vector<type>::operator=(other_);
    return *this;
  }

  /// Add new item to the vector in a thread-safe manner
  /// @param item_ Element appended to the end of the vector.
  void push_back_(type item_) {
    write_lock_t lock(this->_mutex);
    this->push_back(item_);
  }

  /// Remove specific items from the vector in a thread-safe manner
  /// @param itemsToRemove_ Elements to erase; empty input clears all elements.
  void clear_(const std::vector<type>& itemsToRemove_ = std::vector<type>()) {
    write_lock_t lock(this->_mutex);
    if (itemsToRemove_.size() > 0) {
      for (const type& item : itemsToRemove_) {
        this->erase(std::remove(this->begin(), this->end(), item), this->end());
      }
    } else {
      // Otherwise clear the entire vector.
      this->clear();
    }
  }

  /// Remove a specific item from the vector in a thread-safe manner
  /// @param itemToRemove_ Element value to erase from the vector.
  void clear_(type itemToRemove_) {
    write_lock_t lock(this->_mutex);
    this->erase(
        std::remove(this->begin(), this->end(), itemToRemove_),
        this->end());
  }

  /// Access an element in a thread-safe manner
  /// @param pos_ Zero-based index of the element to read.
  /// @return Copy of the element stored at `pos_`.
  type at_(size_t pos_) {
    read_lock_t lock(this->_mutex);
    return this->at(pos_);
  }

  /// Assign from iterators in a thread-safe manner
  /// @param first_ Iterator to the first element of the source range.
  /// @param last_ Iterator one past the last element of the source range.
  void assign_(
      typename vector_t<type>::const_iterator first_,
      typename vector_t<type>::const_iterator last_) {
    write_lock_t lock(this->_mutex);
    this->assign(first_, last_);
  }

  /// This function is used to lock the entire vector for reading
  void shared_lock() const { this->_mutex.lock_shared(); }

  /// This function is used to unlock the vector after it was
  void unlock_shared() const { this->_mutex.unlock_shared(); }

  /// This function is used to lock the entire vector for writing
  void unique_lock() const { this->_mutex.lock(); }

  /// This function is used to unlock the vector after it was
  void unlock_unique() const { this->_mutex.unlock(); }

  /// Get the size of the vector in a thread-safe manner
  /// @return Number of elements currently stored in the vector.
  int get_size_safe() const {
    read_lock_t lock(this->_mutex);
    return this->size();
  }

  /// Convert the vector to a standard vector in a thread-safe manner
  /// @param size_ Maximum number of elements to copy; `0` means copy all.
  /// @return New vector containing up to `size_` elements from the front.
  vector_t<type> to_vector(const size_t size_ = 0) const {
    read_lock_t lock(this->_mutex);
    vector_t<type> output;
    size_t numberElements = size_ == 0 ? this->size() : size_;
    for (size_t i = 0; i < numberElements && i < this->size(); ++i) {
      output.push_back(this->at(i));
    }
    return output;
  }

private:
  //! Mutex that is used to lock the vector for reading and writing.
  mutable std::shared_mutex _mutex;
};

}  // namespace slam_mer

#endif  // TYPE_SAFE_VECTOR_HPP
