/// Copyright (C) 2026 Mitsubishi Electric Research Laboratories (MERL)
/// SPDX-License-Identifier: AGPL-3.0-or-later
///
/// @file dictionary.hpp
/// @brief Thread-safe dictionary wrapper built on std::map.
/// @details Adds synchronized read and write access patterns around ordered
/// key-value storage. The wrapper is designed for shared SLAM state
/// where deterministic key ordering and concurrency safety matter.
/// @author Pedro Miraldo
/// @author Valter Piedade

#ifndef DICTIONARY_T_HPP
#define DICTIONARY_T_HPP

// the necessary typedefs
#include <types/mutex.hpp>

// Standard libraries
#include <map>
#include <memory>
#include <shared_mutex>

namespace slam_mer {

/// @brief Thread-safe dictionary wrapper around `std::map`.
///
/// It supports concurrent reads and synchronized writes.
/// @tparam key Type of the dictionary key.
/// @tparam value Type of the dictionary value.
template <typename key, typename value>
class dictionary_t : public std::map<key, value> {
public:
  /// Typedef for a shared pointer to a dictionary
  using shared_ptr = std::shared_ptr<dictionary_t<key, value>>;

  /// Default constructor initializes an empty dictionary.
  dictionary_t() : std::map<key, value>() {}

  /// Construct a new dictionary_t object from a single key-value pair
  /// @param data_ Initial key-value pair inserted at construction time.
  dictionary_t(const std::pair<const key, value>& data_)
      : std::map<key, value>() {
    this->insert(data_);
  }

  /// Copy constructor
  /// @param other_ Source dictionary copied while holding a read lock.
  dictionary_t(const dictionary_t<key, value>& other_)
      : std::map<key, value>() {
    write_lock_t lock(_mutex);
    if (this != &other_) {
      other_.lock_read();
      std::map<key, value>::operator=(other_);
      other_.unlock_read();
    }
  }

  /// Construct a new dictionary_t object from an initializer list of
  /// key-value pairs
  /// @param init_ Initial key-value pairs used to populate the dictionary.
  dictionary_t(std::initializer_list<std::pair<const key, value>> init_)
      : std::map<key, value>(init_) {}

  /// Get the size of the dictionary in a thread-safe manner
  /// @return Number of key-value entries currently stored.
  [[nodiscard("Retrieving dictionary size.")]]
  inline size_t size_() const {
    read_lock_t lock(_mutex);
    return this->size();
  }

  /// Check if the dictionary contains a key in a thread-safe manner
  /// @param key_ Key tested for existence in the dictionary.
  /// @return `true` when `key_` exists, `false` otherwise.
  [[nodiscard("Checking if dictionary contains key.")]]
  inline bool contains_(const key& key_) const {
    read_lock_t lock(_mutex);
    return this->find(key_) != this->end();
  }

  /// Get the value associated with a key in a thread-safe manner
  /// @param key_ Key whose associated value is retrieved.
  /// @return Copy of the value stored for `key_`.
  [[nodiscard("Retrieving value associated with key.")]]
  inline value at_(const key& key_) const {
    read_lock_t lock(_mutex);
    return this->at(key_);
  }

  /// Insert a key-value pair into the dictionary in a thread-safe manner
  /// @param data_ Key-value pair inserted into the dictionary.
  inline void insert_(const std::pair<const key, value>& data_) {
    write_lock_t lock(_mutex);
    this->insert(data_);
  }

  /// Remove a key from the dictionary in a thread-safe manner
  /// @param key_ key to be erased.
  inline void erase_(const key& key_) {
    write_lock_t lock(_mutex);
    this->erase(key_);
  }

  /// Equal operator
  /// @param other_ Source dictionary whose entries replace this dictionary.
  /// @return Reference to this dictionary after assignment.
  dictionary_t<key, value>& operator=(const dictionary_t<key, value>& other_) {
    write_lock_t lock(_mutex);
    if (this != &other_) {
      other_.lock_read();
      std::map<key, value>::operator=(other_);
      other_.unlock_read();
    }
    return *this;
  }

  /// Lock the object for shared reading
  void lock_read() const { _mutex.lock_shared(); }

  /// Unlock the shared reading lock
  void unlock_read() const { _mutex.unlock_shared(); }

  /// Lock the object for writing
  void lock() const { _mutex.lock(); }

  /// Unlock the writing lock
  void unlock() const { _mutex.unlock(); }

private:
  /// Mutex that is used to lock the dictionary for reading and writing.
  mutable std::shared_mutex _mutex;
};

}  // namespace slam_mer

#endif  // DICTIONARY_T_HPP
