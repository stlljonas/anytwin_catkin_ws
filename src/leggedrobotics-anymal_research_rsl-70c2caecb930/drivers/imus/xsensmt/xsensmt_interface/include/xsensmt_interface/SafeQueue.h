/** \file SafeQueue.h
    \brief This file defines the SafeQueue class, which represents a
           thread-safe queue.
 */

#ifndef BASE_SAFEQUEUE_H
#define BASE_SAFEQUEUE_H

#include <deque>
#include <limits>

#include <mutex>

namespace xsensmt {

/** The class SafeQueue represents a thread-safe queue.
      \brief Thread-safe queue
 */
template <typename T> class SafeQueue {
public:
  /** \name Types definitions
      @{
   */
  /// Container type
  typedef std::deque<T> Container;
  /** @}
   */

  /** \name Constructors/destructor
      @{
   */
  /// Constructs queue with capacity
  SafeQueue(size_t capacity = std::numeric_limits<size_t>::max());
  /// Copy constructor
  SafeQueue(const SafeQueue& other);
  /// Assignment operator
  SafeQueue& operator = (const SafeQueue& other);
  /// Destructor
  ~SafeQueue();
  /** @}
   */

  /** \name Accessors
        @{
   */
  /// Returns the capacity of the queue
  size_t getCapacity() const;
  /// Sets the capacity of the queue
  void setCapacity(size_t capacity);
  /// Returns the statistics about dropped elements
  size_t getNumDroppedElements() const;
  /// Check whether the queue is empty
  bool isEmpty() const;
  /// Get size of the queue
  size_t getSize() const;
  /// Get the queue
  Container getQueue() const;
  /** @}
   */

  /** \name Methods
        @{
   */
  /// Enqueue an element
  void enqueue(const T& value);
  /// Dequeue an element
  T dequeue();
  /** @}
   */

protected:
  /** \name Protected members
      @{
   */
  /// Queue
  Container queue_;
  /// Capacity
  size_t capacity_;
  /// Statistics about the dropped elements
  size_t numDroppedElements_;
  /// Mutex protecting the queue
  mutable std::mutex mutex_;
  /** @}
   */
};

}

#include "xsensmt_interface/SafeQueue.tpp"

#endif // BASE_SAFEQUEUE_H
