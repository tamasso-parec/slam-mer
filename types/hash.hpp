/// Copyright (C) 2026 Mitsubishi Electric Research Laboratories (MERL)
/// SPDX-License-Identifier: AGPL-3.0-or-later
///
/// @file hash.hpp
/// @brief Thread-safe hash-table wrapper built on std::unordered_map.
/// @details Provides synchronized access to unordered key-value storage for
/// high-frequency lookup paths. The API mirrors common map usage while
/// enforcing explicit lock discipline in shared contexts.
/// @author Pedro Miraldo
/// @author Valter Piedade
#ifndef HASH_T_HPP
#define HASH_T_HPP

// the necessary typedefs
#include <types/mutex.hpp>

// Standard libraries
#include <memory>
#include <shared_mutex>
#include <unordered_map>

namespace slam_mer {

/// @brief Thread-safe hash table wrapper around `std::unordered_map`.
///
/// It supports concurrent reads and synchronized writes.
/// @tparam key Type of the hash-table key.
/// @tparam value Type of the hash-table value.
template <typename key, typename value>
class hash_t : public std::unordered_map<key, value> {

public:
  /// Shared pointer type
  using shared_ptr = std::shared_ptr<hash_t<key, value>>;
  /// Weak pointer type
  using weak_ptr = std::weak_ptr<hash_t<key, value>>;
  /// Make shared pointer function
  /// @return Shared pointer owning a new empty thread-safe hash table.
  static hash_t<key, value>::shared_ptr make_shared() {
    return std::make_shared<hash_t<key, value>>();
  }

  /// Default constructor
  hash_t() : std::unordered_map<key, value>() {}

  /// Constructor from a single pair
  /// @param data_ Initial key-value pair inserted at construction time.
  hash_t(const std::pair<const key, value>& data_)
      : std::unordered_map<key, value>() {
    this->insert(data_);
  }

  /// Copy constructor
  /// @param other_ Source hash table copied while holding a read lock.
  hash_t(const hash_t<key, value>& other_) : std::unordered_map<key, value>() {
    write_lock_t lock(_mutex);
    if (this != &other_) {
      other_.lock_read();
      std::unordered_map<key, value>::operator=(other_);
      other_.unlock_read();
    }
  }

  /// Get the size of the hash table
  /// @return Number of key-value entries currently stored.
  [[nodiscard("Use to Retrieve hash table size.")]]
  inline size_t size_() const {
    read_lock_t lock(_mutex);
    return this->size();
  }

  /// Thread-safe check whether a key exists in the hash table.
  /// @param key_ Key tested for existence in the hash table.
  /// @return `true` when `key_` exists, `false` otherwise.
  [[nodiscard("Use to check if hash table contains key.")]]
  inline bool contains_(const key& key_) const {
    read_lock_t lock(_mutex);
    return this->find(key_) != this->end();
  }

  /// Thread-safe access to a value by key.
  /// @param key_ Key whose associated value is retrieved.
  /// @return Copy of the value stored for `key_`.
  [[nodiscard("Use to access value by key in hash table.")]]
  inline value at_(const key& key_) const {
    read_lock_t lock(_mutex);
    return this->at(key_);
  }

  /// Thread-safe insertion of a key-value pair.
  /// @param data_ Key-value pair inserted into the hash table.
  inline void insert_(const std::pair<const key, value>& data_) {
    write_lock_t lock(_mutex);
    this->insert(data_);
  }

  /// Thread-safe emplacement of a key-value pair.
  /// @param key_ Key of the entry to insert or update in the hash table.
  /// @param value_ Value associated with `key_` to insert or update.
  inline void try_emplace_(const key& key_, const value& value_) {
    write_lock_t lock(_mutex);
    this->try_emplace(key_, value_);
  }

  /// Thread-safe erasure of a key-value pair by key.
  /// @param key_ Key of the entry to erase from the hash table.
  inline void erase_(const key& key_) {
    write_lock_t lock(_mutex);
    this->erase(key_);
  }

  /// Thread-safe conditional erasure of a key-value pair by key.
  /// @param key_ Key of the entry to conditionally erase from the hash table.
  inline void conditional_erase_(const key& key_) {
    write_lock_t lock(_mutex);
    if (this->find(key_) != this->end()) this->erase(key_);
  }

  /// Find the value associated with a key in the hash table.
  /// @param key_ Key whose associated value is searched for.
  /// @return Pointer to the value if `key_` exists, `nullptr` otherwise.
  inline value* find_(const key& key_) {
    write_lock_t lock(_mutex);
    auto it = this->find(key_);
    return (it != this->end()) ? &(it->second) : nullptr;
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

#endif  // HASH_T_HPP
