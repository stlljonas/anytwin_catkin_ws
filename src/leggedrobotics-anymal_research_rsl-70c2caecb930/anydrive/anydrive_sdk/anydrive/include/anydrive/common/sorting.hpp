#include <algorithm>
#include <numeric>
#include <vector>

namespace anydrive {
namespace common {

/**
 * Compute the permutation to order the vec according to the provided compare function.
 * @tparam T Type.
 * @tparam Compare
 * @param vec Vector to get the sort permutation for.
 * @param compare Comparison function object.
 * @return Permutation vector.
 */
template <typename T, typename Compare>
std::vector<int> sortPermutation(std::vector<T> const& vec, Compare compare) {
  std::vector<int> p(vec.size());
  std::iota(p.begin(), p.end(), 0);
  std::sort(p.begin(), p.end(), [&](int i, int j) { return compare(vec[i], vec[j]); });
  return p;
}

/**
 * Sort the vec according to the permutation vector p.
 * @tparam T Type.
 * @param vec Vector to order.
 * @param p Permutation vector.
 * @return Ordered vector.
 */
template <typename T>
std::vector<T> applyPermutation(std::vector<T> const& vec, std::vector<int> const& p) {
  std::vector<T> sortedVec(p.size());
  std::transform(p.begin(), p.end(), sortedVec.begin(), [&](int i) { return vec[i]; });
  return sortedVec;
}

}  // namespace common
}  // namespace anydrive
