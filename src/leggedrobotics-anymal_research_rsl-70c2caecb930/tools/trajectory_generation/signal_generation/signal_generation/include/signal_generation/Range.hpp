#pragma once


// std
#include <limits>


namespace signal_generation {


template <typename PrimT_>
class Range
{
protected:
  using PrimT = PrimT_;

  // The range is defined as the interval [start, end),
  // which means that start is included, end is excluded.
  PrimT start_ = -std::numeric_limits<PrimT>::infinity();
  PrimT end_ = std::numeric_limits<PrimT>::infinity();

public:
  Range() {}

  Range(const PrimT start, const PrimT end)
  : start_(start),
    end_(end) {}

  virtual ~Range() {}

  PrimT getStart() const
  {
    return start_;
  }

  PrimT& getStart()
  {
    return start_;
  }

  PrimT getEnd() const
  {
    return end_;
  }

  PrimT& getEnd()
  {
    return end_;
  }

  PrimT getLength() const
  {
    return end_ - start_;
  }

  bool isEmpty() const
  {
    return start_ > end_;
  }

  bool contains(const PrimT value) const
  {
    return (value >= start_) && (value < end_);
  }

  Range unifiedWith(const Range& other) const
  {
    return Range(
        std::min(this->getStart(), other.getStart()),
        std::max(this->getEnd(), other.getEnd()));
  }

  Range& unifyWith(const Range& other)
  {
    *this = unifiedWith(other);
    return *this;
  }

  Range intersectedWith(const Range& other) const
  {
    return Range(
        std::max(this->getStart(), other.getStart()),
        std::min(this->getEnd(), other.getEnd()));
  }

  Range& intersectWith(const Range& other)
  {
    *this = intersectedWith(other);
    return *this;
  }
};

using RangeF = Range<float>;
using RangeD = Range<double>;


} // signal_generation
