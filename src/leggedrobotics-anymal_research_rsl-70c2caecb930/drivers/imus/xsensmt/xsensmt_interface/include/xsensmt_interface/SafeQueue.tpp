//#include "exceptions/InvalidOperationException.h"

namespace xsensmt {

/******************************************************************************/
/* Constructors and Destructor                                                */
/******************************************************************************/

template <typename T>
SafeQueue<T>::SafeQueue(size_t capacity)
: capacity_(capacity),
  numDroppedElements_(0) {
}

template <typename T>
SafeQueue<T>::SafeQueue(const SafeQueue& other) {
  *this = other;
}

template <typename T>
SafeQueue<T>& SafeQueue<T>::operator=(const SafeQueue& other) {
  if (this != &other) {
    std::unique_lock<std::mutex> lock(mutex_);
    queue_ = other.getQueue();
    capacity_ = other.getCapacity();
    numDroppedElements_ = other.getNumDroppedElements();
  }
  return *this;
}

template <typename T>
SafeQueue<T>::~SafeQueue() {
}

/******************************************************************************/
/* Accessors                                                                  */
/******************************************************************************/

template <typename T>
size_t SafeQueue<T>::getCapacity() const {
  std::unique_lock<std::mutex> lock(mutex_);
  return capacity_;
}

template <typename T>
void SafeQueue<T>::setCapacity(size_t capacity) {
  std::unique_lock<std::mutex> lock(mutex_);
  capacity_ = capacity;
}

template <typename T>
size_t SafeQueue<T>::getNumDroppedElements() const {
  std::unique_lock<std::mutex> lock(mutex_);
  return numDroppedElements_;
}

template <typename T>
bool SafeQueue<T>::isEmpty() const {
  std::unique_lock<std::mutex> lock(mutex_);
  return queue_.empty();
}

template <typename T>
size_t SafeQueue<T>::getSize() const {
  std::unique_lock<std::mutex> lock(mutex_);
  return queue_.size();
}

template <typename T>
typename SafeQueue<T>::Container SafeQueue<T>::getQueue() const {
  std::unique_lock<std::mutex> lock(mutex_);
  return queue_;
}


/******************************************************************************/
/* Methods                                                                    */
/******************************************************************************/

template <typename T>
void SafeQueue<T>::enqueue(const T& value) {
  std::unique_lock<std::mutex> lock(mutex_);
  queue_.push_back(value);
  if (queue_.size() > capacity_) {
    queue_.pop_front();
    numDroppedElements_++;
  }
}

template <typename T>
T SafeQueue<T>::dequeue() {
  std::unique_lock<std::mutex> lock(mutex_);
//  if (queue_.empty())
//    throw InvalidOperationException("SafeQueue<T>::dequeue(): empty queue");
  T front = queue_.front();
  queue_.pop_front();
  return front;
}

}
