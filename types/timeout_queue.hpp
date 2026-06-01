/// Copyright (C) 2026 Mitsubishi Electric Research Laboratories (MERL)
/// SPDX-License-Identifier: AGPL-3.0-or-later
///
/// @file timeout_queue.hpp
/// @brief Timeout-aware queue with filtered iterators and expiration handling.
/// @details Implements a list-based container that tracks insertion time,
/// exposes iterators that skip expired entries, and buffers timed-out
/// elements for post-processing. Useful for time-bounded caches and
/// asynchronous queues.
/// @author Pedro Miraldo
/// @author Valter Piedade
#ifndef TIMEOUT_QUEUE_HPP
#define TIMEOUT_QUEUE_HPP

#include <type_traits>
#include <types/mutex.hpp>
#include <types/output.hpp>
#include <types/pair.hpp>
#include <types/timer.hpp>
#include <types/vector.hpp>

#include <list>
#include <print>
#include <shared_mutex>
#include <sstream>

namespace slam_mer {

/// @brief Timeout-aware queue wrapper around `std::list`.
///
/// Timeout checks are performed during iteration. Expired items are collected
/// in a side buffer and can be retrieved with `timed_out()`.
///
/// Thread-safety is managed manually via `read_lock()`/`unlock_read()` and
/// `lock()`/`unlock()`.
///
/// Example:
/// `timeout_queue_t<item_t> q(duration_t::from_s(5));`
/// `q.read_lock(); for (const auto& v : q) { ... } q.unlock_read();`
/// `auto expired = q.timed_out();`
///
/// @tparam type Type of elements stored in the queue.
template <typename type>
class timeout_queue_t : public std::list<pair_t<type, timer_t>> {

  // base type
  using base_t = std::list<pair_t<type, timer_t>>;

public:
  // inherit constructors
  using base_t::base_t;
  /// Shared pointer type
  using shared_ptr = std::shared_ptr<timeout_queue_t<type>>;

  /// @brief Construct a new timeout_queue_t object
  /// @param timeOut_ Maximum allowed age for each element in the queue.
  timeout_queue_t(duration_t timeOut_ = duration_t::from_s(10))
      : base_t(), timeOut(timeOut_) {}

  // ----------------------------------------------------------------------
  // Iterators (const and non const)
  /// Filtered iterator (yields only non-expired type)
  // ----------------------------------------------------------------------
  // Iterator (yields only non-expired type)
  /// @brief Iterator that yields only non-expired elements in the queue.
  struct iterator {
    using base_iter = typename base_t::iterator;

    using iterator_category = std::forward_iterator_tag;
    using value_type = type;
    using difference_type =
        typename std::iterator_traits<base_iter>::difference_type;
    using reference = type&;
    using pointer = type*;

    timeout_queue_t* owner = nullptr;  ///< Pointer to the owning queue
    base_iter it{}, end{};             ///< Underlying iterators

    /// @brief Default constructor.
    iterator() = default;
    /// @brief Construct an iterator
    /// @param o_ Queue instance that owns this iterator.
    /// @param i_ Current list iterator position.
    /// @param e_ Sentinel iterator marking the end position.
    iterator(timeout_queue_t* o_, base_iter i_, base_iter e_)
        : owner(o_), it(i_), end(e_) {
      __skip_expired();
    }

    /// @brief Dereferences the iterator.
    reference operator*() const { return it->first; }
    /// @brief Accesses the current element pointer.
    pointer operator->() const { return std::addressof(it->first); }

    /// @brief Pre-increment operator
    iterator& operator++() {
      ++it;
      __skip_expired();
      return *this;
    }

    /// @brief Post-increment operator
    /// @param int_ Dummy integer used to select postfix increment syntax.
    /// @return Iterator snapshot before incrementing.
    iterator operator++(int int_) {
      auto tmp = *this;
      ++(*this);
      return tmp;
    }

    /// @brief Equality comparison
    /// @param a_ Left iterator in the comparison.
    /// @param b_ Right iterator in the comparison.
    /// @return `true` when both iterators point to the same list position.
    friend bool operator==(const iterator& a_, const iterator& b_) {
      return a_.it == b_.it;
    }

    /// @brief Inequality comparison
    /// @param a_ Left iterator in the comparison.
    /// @param b_ Right iterator in the comparison.
    /// @return `true` when iterators point to different list positions.
    friend bool operator!=(const iterator& a_, const iterator& b_) {
      return !(a_ == b_);
    }

  private:
    /// @brief Skip expired elements and collect them in timedOutItems
    void __skip_expired() {
      while (it != end && it->second.over(owner->timeOut)) {
        owner->timedOutItems.push_back_(it->first);
        ++it;
      }
    }
  };

  // Const iterator (yields only non-expired type)
  /// @brief Const iterator that yields only non-expired elements in the queue.
  struct const_iterator {
    using base_iter = typename base_t::const_iterator;

    using iterator_category = std::forward_iterator_tag;
    using value_type = type;
    using difference_type =
        typename std::iterator_traits<base_iter>::difference_type;
    using reference = const type&;
    using pointer = const type*;

    const timeout_queue_t* owner = nullptr;  ///< Pointer to the owning queue
    base_iter it{}, end{};                   ///< Underlying iterators

    /// @brief Default constructor.
    const_iterator() = default;
    /// @brief Construct a const_iterator
    /// @param o_ Queue instance that owns this iterator.
    /// @param i_ Current list iterator position.
    /// @param e_ Sentinel iterator marking the end position.
    const_iterator(const timeout_queue_t* o_, base_iter i_, base_iter e_)
        : owner(o_), it(i_), end(e_) {
      __skip_expired();
    }

    /// @brief Allow converting iterator to const_iterator
    /// @param other_ Mutable iterator whose state is copied.
    const_iterator(const iterator& other_)
        : owner(other_.owner), it(other_.it), end(other_.end) {}

    /// @brief Dereferences the iterator.
    reference operator*() const { return it->first; }
    /// @brief Accesses the current element pointer.
    pointer operator->() const { return std::addressof(it->first); }

    /// @brief Pre-increment operator
    const_iterator& operator++() {
      ++it;
      __skip_expired();
      return *this;
    }
    /// @brief Post-increment operator
    /// @param int_ Dummy integer used to select postfix increment syntax.
    /// @return Iterator snapshot before incrementing.
    const_iterator operator++(int int_) {
      auto tmp = *this;
      ++(*this);
      return tmp;
    }

    /// @brief Equality comparison
    /// @param a_ Left iterator in the comparison.
    /// @param b_ Right iterator in the comparison.
    /// @return `true` when both iterators point to the same list position.
    friend bool operator==(const const_iterator& a_, const const_iterator& b_) {
      return a_.it == b_.it;
    }
    /// @brief Inequality comparison
    /// @param a_ Left iterator in the comparison.
    /// @param b_ Right iterator in the comparison.
    /// @return `true` when iterators point to different list positions.
    friend bool operator!=(const const_iterator& a_, const const_iterator& b_) {
      return !(a_ == b_);
    }

  private:
    /// @brief Skip expired elements and collect them in timedOutItems
    void __skip_expired() {
      while (it != end && it->second.over(owner->timeOut)) {
        owner->timedOutItems.push_back_(it->first);  // timedOutItems is mutable
        ++it;
      }
    }
  };
  // ----------------------------------------------------------------------

  /// @brief Returns iterator to the beginning (first non-expired element)
  /// @return Iterator to the first non-expired element.
  iterator begin() { return iterator{this, base_t::begin(), base_t::end()}; }

  /// @brief Returns iterator to the end
  /// @return Iterator representing the end sentinel.
  iterator end() { return iterator{this, base_t::end(), base_t::end()}; }

  /// @brief Returns const_iterator to the beginning (first non-expired element)
  /// @return Const iterator to the first non-expired element.
  const_iterator begin() const {
    return const_iterator{this, base_t::cbegin(), base_t::cend()};
  }

  /// @brief Returns const_iterator to the end
  /// @return Const iterator representing the end sentinel.
  const_iterator end() const {
    return const_iterator{this, base_t::cend(), base_t::cend()};
  }

  /// @brief Returns const_iterator to the beginning (first non-expired element)
  /// @return Const iterator to the first non-expired element.
  const_iterator cbegin() const { return begin(); }

  /// @brief Returns const_iterator to the end
  /// @return Const iterator representing the end sentinel.
  const_iterator cend() const { return end(); }

  // Methods

  /// @brief Insert item into the queue with current time
  /// @param item_ Element inserted with a fresh timestamp.
  void insert(const type& item_) { base_t::emplace_back(item_, timer_t()); }

  /// @brief Remove elements from the queue that match the given elements
  /// @param elements_ Values to remove from the queue when matched by equality.
  void clear(const vector_t<type>& elements_) {
    write_lock_t lock(this->mutex);
    for (const type& item : elements_) {
      this->remove_if([&item](const pair_t<type, timer_t>& p__) {
        return p__.first == item;
      });
    }
  }

  /// @brief Clear all elements from the queue.
  void clear() { base_t::clear(); }

  /// @brief Get the vector of timed out items (and clear the internal buffer)
  /// @return Reference to the vector of timed out items
  vector_t<type>& timed_out() { return timedOutItems; }

  /// @brief Clear timed out items. If elements are given, clear only those.
  /// @param elements_ Timed-out items that should be removed from the buffer.
  /// @param dumpVerbose_ Enables per-item timeout messages before removal.
  /// @param prefix_ Text prefix prepended to each timeout message.
  void clear_timed_out(
      const vector_t<type>& elements_,
      bool dumpVerbose_ = false,
      const std::string& prefix_ = "") {
    if (dumpVerbose_)
      for (const type& item : elements_)
        __timedout_message(prefix_, item);
    timedOutItems.clear_(elements_);
  }

  /// @brief Clear all timed out items.
  /// @param dumpVerbose_ Enables per-item timeout messages before clearing.
  /// @param prefix_ Text prefix prepended to each timeout message.
  void clear_timed_out(
      bool dumpVerbose_ = false,
      const std::string& prefix_ = "") {
    if (dumpVerbose_)
      for (const type& item : timedOutItems)
        __timedout_message(prefix_, item);

    timedOutItems.clear_();
  }

  /// @brief Acquire a shared (read) lock for thread safety
  void read_lock() const { this->mutex.lock_shared(); }

  /// @brief Acquire an exclusive (write) lock for thread safety
  void lock() const { this->mutex.lock(); }

  /// @brief Release a shared (read) lock
  void unlock_read() const { this->mutex.unlock_shared(); }

  /// @brief Release an exclusive (write) lock
  void unlock() const { this->mutex.unlock(); }

private:
  /// Timeout duration for elements in the queue
  duration_t timeOut;
  /// Store timed out items (mutable for const access in iterators)
  mutable vector_t<type> timedOutItems;
  /// Mutex for thread safety (shared/exclusive)
  mutable std::shared_mutex mutex;

  /// @brief Print a message for a timed out item
  /// @param prefix_ Text prefix prepended to the timeout message.
  /// @param item_ Item value that just timed out.
  inline void __timedout_message(
      const std::string& prefix_,
      const type& item_) {
    using std::println;
    println(
        "{}[Timeout Queue] Item timed out: {}",
        prefix_,
        __item_to_string(item_));
  }

  /// @brief Convert an item to string for printing
  /// @param item_ Item value converted to text for diagnostic output.
  /// @return String representation of the item
  [[nodiscard("Use to get string representation of the item.")]]
  inline std::string __item_to_string(const type& item_) const {
    if constexpr (std::is_convertible_v<type, std::string>) {
      return static_cast<std::string>(item_);
    }

    std::ostringstream stream;
    stream << item_;
    return stream.str();
  }
};

}  // namespace slam_mer

#endif  // TIMEOUT_QUEUE_HPP
