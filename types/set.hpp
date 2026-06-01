/// Copyright (C) 2026 Mitsubishi Electric Research Laboratories (MERL)
/// SPDX-License-Identifier: AGPL-3.0-or-later
///
/// @file set.hpp
/// @brief Thread-safe wrappers for unordered and ordered set containers.
/// @details Provides lock-aware APIs for both hash-based and ordered set
/// storage with conversion helpers and synchronized access. These
/// types are used for shared ID collections and filtered subsets.
/// @author Pedro Miraldo
/// @author Valter Piedade
#ifndef TYPE_SAFE_SET_HPP
#define TYPE_SAFE_SET_HPP

// the necessary typedefs
#include <types/mutex.hpp>
#include <types/vector.hpp>

// Standard libraries
#include <memory>
#include <set>
#include <shared_mutex>
#include <unordered_set>

namespace slam_mer {

/// This simple class is used to store unordered sets of data.
/// It is a simple wrapper for the std::unordered_set class,
/// but it uses a shared mutex to lock the access to the set
/// among other useful functions.
template <typename type>
class set_t : public std::unordered_set<type> {
public:
  /// Instantiate the constructors
  using std::unordered_set<type>::unordered_set;

  /// Shared pointer type
  using shared_ptr = std::shared_ptr<set_t<type>>;

  /// Make shared pointer function
  /// @return Shared pointer owning a new empty thread-safe unordered set.
  static set_t<type>::shared_ptr make_shared() {
    return std::make_shared<set_t<type>>();
  }

  /// Empty constructor for the Visualization Set object.
  set_t() : std::unordered_set<type>() {}

  /// Construct a new set safe object from a single data point
  /// @param data_ Initial element inserted into the set during construction.
  set_t(type data_) : std::unordered_set<type>() { this->insert(data_); }

  /// Copy constructor
  /// @param other_ Source set to copy from while holding a shared read lock.
  set_t(const set_t<type>& other_) : std::unordered_set<type>() {
    write_lock_t lock(this->_mutex);
    other_.lock_read();
    std::unordered_set<type>::operator=(other_);
    other_.unlock_read();
  }

  /// Destroy the Visualization Set object.
  ~set_t() {}

  /// Equal operator
  /// @param other_ Source set whose contents replace the current set.
  /// @return Reference to this set after the synchronized assignment.
  set_t<type>& operator=(const set_t<type>& other_) {
    if (this != &other_) {
      write_lock_t lock(this->_mutex);
      other_.lock_read();
      std::unordered_set<type>::operator=(other_);
      other_.unlock_read();
    }
    return *this;
  }

  /// Safe add a point to the set.
  /// @param id_ Element to insert while holding the write lock.
  void insert_(type id_) {
    write_lock_t lock(this->_mutex);
    this->insert(id_);
  }

  /// Safe remove a point from the set.
  /// @param id_ Element to remove while holding the write lock.
  void erase_(type id_) {
    write_lock_t lock(this->_mutex);
    this->erase(id_);
  }

  /// Safe erase multiple points from the set. [all if empty]
  /// @param itemsToRemove_ Elements to erase; when empty, the whole set is cleared.
  void clear_(const vector_t<type>& itemsToRemove_ = vector_t<type>()) {
    write_lock_t lock(this->_mutex);
    if (itemsToRemove_.size() > 0)
      for (const type& item : itemsToRemove_)
        this->erase(item);
    else
      this->clear();
  }

  /// Safe check if the set contains a specific value.
  /// @param value_ Element to search for in the set.
  /// @return True when `value_` is present, otherwise false.
  bool contains_(const type& value_) const {
    read_lock_t lock(this->_mutex);
    return this->find(value_) != this->end();
  }

  /// @brief This function is used to lock the entire set for reading purposes.
  void lock_read() const { this->_mutex.lock_shared(); }

  /// @brief To unlock the set after it was locked for reading.
  void unlock_read() const { this->_mutex.unlock_shared(); }

  /// @brief This function is used to lock the entire set for writing purposes.
  void lock() const { this->_mutex.lock(); }

  /// @brief To unlock the set after it was locked for writing.
  void unlock() const { this->_mutex.unlock(); }

  /// @brief Get the size of the set in a thread-safe manner
  /// @return Number of elements currently stored in the set.
  int get_size_safe() const {
    read_lock_t lock(this->_mutex);
    return this->size();
  }

  /// @brief  Get all elements in a thread-safe manner
  /// @return Copy of the full unordered set at the time of the call.
  std::unordered_set<type> get_all() const {
    read_lock_t lock(this->_mutex);
    return *this;
  }

  /// @brief Convert to vector in a thread-safe manner
  /// @param size_ Maximum number of elements to copy; `0` means copy al
  /// elements.
  /// @return Vector with up to `size_` elements copied from the set.
  vector_t<type> to_vector(const size_t size_ = 0) const {
    read_lock_t lock(this->_mutex);
    vector_t<type> output;
    const size_t numberElements = size_ == 0 ? this->size() : size_;
    for (const auto& item : *this) {
      if (output.size() >= numberElements) break;
      output.push_back(item);
    }
    return output;
  }

  /// @brief Conversion operator to vector
  inline operator vector_t<type>() const { return this->to_vector(); }

private:
  /// @brief Mutex that is used to lock the set for reading and writing.
  mutable std::shared_mutex _mutex;
};

/// This simple class is used to store ordered sets of data.
/// It is a simple wrapper for the std::set class,
/// but it uses a shared mutex to lock the access to the set
/// among other useful functions.
template <typename type>
class ordered_t : public std::set<type> {
public:
public:
  // Instantiate the constructors
  using std::set<type>::set;

  /// Shared pointer type
  using shared_ptr = std::shared_ptr<ordered_t<type>>;

  /// Make shared pointer function
  /// @return Shared pointer owning a new empty thread-safe ordered set.
  static ordered_t<type>::shared_ptr make_shared() {
    return std::make_shared<ordered_t<type>>();
  }

  /// Empty constructor for the Visualization Set object.
  ordered_t() : std::set<type>() {}

  /// Construct a new set safe object from a single data point
  /// @param data_ Initial element inserted into the ordered set during
  /// construction.
  ordered_t(type data_) : std::set<type>() { this->insert(data_); }

  /// Copy constructor
  /// @param other_ Source ordered set to copy from while holding a shared
  /// read lock.
  ordered_t(const ordered_t<type>& other_) : std::set<type>() {
    write_lock_t lock(this->_mutex);
    other_.lock_read();
    std::set<type>::operator=(other_);
    other_.unlock_read();
  }

  /// Destroy the Visualization Set object.
  ~ordered_t() {}

  /// Equal operator
  /// @param other_ Source ordered set whose contents replace the current set.
  /// @return Reference to this ordered set after synchronized assignment.
  ordered_t<type>& operator=(const ordered_t<type>& other_) {
    if (this != &other_) {
      write_lock_t lock(this->_mutex);
      other_.lock_read();
      std::set<type>::operator=(other_);
      other_.unlock_read();
    }
    return *this;
  }

  /// Safe add a point to the set.
  /// @param id_ Element to insert while holding the write lock.
  void insert_(type id_) {
    write_lock_t lock(this->_mutex);
    this->insert(id_);
  }

  /// Safe remove a point from the set.
  /// @param id_ Element to remove while holding the write lock.
  void erase_(type id_) {
    write_lock_t lock(this->_mutex);
    this->erase(id_);
  }

  /// Safe erase multiple points from the set. [all if empty]
  /// @param itemsToRemove_ Elements to erase; when empty, the whole set
  /// is cleared.
  void clear_(const vector_t<type>& itemsToRemove_ = vector_t<type>()) {
    write_lock_t lock(this->_mutex);
    if (itemsToRemove_.size() > 0)
      for (const type& item : itemsToRemove_)
        this->erase(item);
    else
      this->clear();
  }

  /// Safe check if the set contains a specific value.
  /// @param value_ Element to search for in the ordered set.
  /// @return True when `value_` is present, otherwise false.
  bool contains_(const type& value_) const {
    read_lock_t lock(this->_mutex);
    return this->find(value_) != this->end();
  }

  /// @brief This function is used to lock the entire set for reading purposes.
  void lock_read() const { this->_mutex.lock_shared(); }

  /// @brief To unlock the set after it was locked for reading.
  void unlock_read() const { this->_mutex.unlock_shared(); }

  /// @brief This function is used to lock the entire set for writing purposes.
  void lock() const { this->_mutex.lock(); }

  /// @brief To unlock the set after it was locked for writing.
  void unlock() const { this->_mutex.unlock(); }

  /// @brief Get the size of the set in a thread-safe manner
  /// @return Number of elements currently stored in the ordered set.
  int get_size_safe() const {
    read_lock_t lock(this->_mutex);
    return this->size();
  }

  /// @brief  Get all elements in a thread-safe manner
  /// @return Copy of the full ordered set at the time of the call.
  std::set<type> get_all() const {
    read_lock_t lock(this->_mutex);
    return *this;
  }

  /// @brief Convert to vector in a thread-safe manner
  /// @param size_ Maximum number of elements to copy; `0` means copy all elements.
  /// @return Vector with up to `size_` elements copied in sorted order.
  vector_t<type> to_vector(const size_t size_ = 0) const {
    read_lock_t lock(this->_mutex);
    vector_t<type> output;
    const size_t numberElements = size_ == 0 ? this->size() : size_;
    for (const auto& item : *this) {
      if (output.size() >= numberElements) break;
      output.push_back(item);
    }
    return output;
  }

  /// @brief Conversion operator to vector
  inline operator vector_t<type>() const { return this->to_vector(); }

private:
  /// @brief Mutex that is used to lock the set for reading and writing.
  mutable std::shared_mutex _mutex;
};

}  // namespace slam_mer

#endif  // TYPE_SAFE_SET_HPP
