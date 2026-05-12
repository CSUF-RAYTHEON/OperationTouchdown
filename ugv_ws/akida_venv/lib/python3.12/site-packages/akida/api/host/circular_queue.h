#pragma once

#include <cstddef>
#include <stdexcept>
#include <vector>

namespace akida {

template<typename T>
class CircularQueue {
 public:
  explicit CircularQueue(size_t size);
  void push(const T& elem);
  bool empty() const;
  bool full() const;
  size_t size() const;
  std::vector<T> get(
      bool flush);  // returns items in the order they were inserted
  T latest();       // returns the latest inserted item
  void clear();     // clear the queue
 private:
  std::vector<T> buffer_;
  T* head_;             // pointer to the latest inserted item
  T* tail_;             // pointer to the oldest inserted item
  const T* const end_;  // pointer to the end of the buffer
  bool full_;
};

template<typename T>
CircularQueue<T>::CircularQueue(size_t size)
    : buffer_(size),          // dimension buffer to expected size
      head_(buffer_.data()),  // initialize both head & tail to beginning of the
                              // buffer
      tail_(buffer_.data()),
      end_(buffer_.data() + size - 1),  // initialize end to the last element
      full_(false) {
  if (size == 0) {
    throw std::invalid_argument("Size must be > 0.");
  }
}

template<typename T>
static T* advance_ptr(T* const ptr, T* const begin, const T* const end) {
  T* result;
  if (ptr == end) {
    result = begin;
  } else {
    result = ptr + 1;
  }
  return result;
}

template<typename T>
void CircularQueue<T>::push(const T& elem) {
  // insert elem at the head
  *head_ = elem;
  // if buffer is full, we just erased tail, so advance it
  if (full()) {
    tail_ = advance_ptr(tail_, buffer_.data(), end_);
  }
  // advance head
  head_ = advance_ptr(head_, buffer_.data(), end_);

  full_ = head_ == tail_;
}

template<typename T>
bool CircularQueue<T>::empty() const {
  return !full_ && head_ == tail_;
}

template<typename T>
bool CircularQueue<T>::full() const {
  return full_;
}

template<typename T>
size_t CircularQueue<T>::size() const {
  size_t size;

  if (empty()) {
    size = 0;
  } else if (full()) {
    size = buffer_.size();
  } else {
    if (head_ > tail_) {
      // If head is ahead of tail, just return diff between them
      size = head_ - tail_;
    } else {
      // else adds max size to "force" head to be ahead of tail
      size = head_ + buffer_.size() - tail_;
    }
  }
  return size;
}

template<typename T>
std::vector<T> CircularQueue<T>::get(bool flush) {
  std::vector<T> result;
  const auto nb_elems = size();
  result.reserve(nb_elems);
  // get elems from tail to head
  auto begin = tail_;
  for (size_t i = 0; i < nb_elems; ++i) {
    result.push_back(*begin);
    begin = advance_ptr(begin, buffer_.data(), end_);
  }

  if (flush) {
    clear();
  }

  return result;
}

template<typename T>
void CircularQueue<T>::clear() {
  // clear buffer
  head_ = buffer_.data();
  tail_ = buffer_.data();
  full_ = false;
}

template<typename T>
T CircularQueue<T>::latest() {
  if (head_ == buffer_.data()) {
    return *end_;
  } else {
    return *(head_ - 1);
  }
}

}  // namespace akida
