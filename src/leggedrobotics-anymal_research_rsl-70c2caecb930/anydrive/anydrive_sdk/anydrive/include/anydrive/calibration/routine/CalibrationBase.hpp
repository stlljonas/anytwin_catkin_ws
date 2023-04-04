#pragma once

#include <atomic>
#include <cmath>

#include "anydrive/Anydrive.hpp"
#include "anydrive/calibration/routine/OptimizationNelderMead.hpp"

namespace anydrive {
namespace calibration {
namespace routine {

class CalibrationBase {
 protected:
  using LogReading = Log<ReadingExtended>;

  static constexpr double timeStep_ = 0.1;

  // Both ANYdrive 2 and 3 use 18-bit gear/joint encoders.
  // Ideally, this value is read from the drive through an SDO. TODO(remo).
  static constexpr int64_t ticks_ = 262144;
  static constexpr int64_t ticksHalf_ = ticks_ / 2;
  static constexpr double omega_ = 2.0 * M_PI / ticks_;
  // The allowed gear ratios. Currently, 50:1 and 100:1 gearboxes are used in ANYdrive 2/3.
  const std::vector<uint32_t> gearRatios_ = {50, 100};
  // The default gear ratio in case the readout fails:
  static constexpr uint32_t gearRatioDefault_ = 50;

  std::atomic<bool> isRunning_;
  std::atomic<bool> shutdownRequested_;

  AnydrivePtr anydrive_;
  std::string filesFolder_;

  const bool writeToFiles_ = false;

 protected:
  explicit CalibrationBase(const AnydrivePtr& anydrive);
  virtual ~CalibrationBase() = default;

 public:
  bool isRunning();
  void requestShutdown();

  bool run();
  void shutdown();

 protected:
  static std::string getHomeDirectory();

  void startupDataCollection();
  virtual bool collectData() = 0;
  void shutdownDataCollection();
  virtual bool postProcessData() = 0;

  bool stopRequested();

  bool preemptableSleep(const double duration);

  void stageCommandDisable();
  void stageCommandMotorVelocity(const double motorVelocity);
  void stageCommandJointVelocity(const double jointVelocity);
  bool getGearRatio(uint32_t& gearRatio);

  /*!
   * Calculate the modulo of two floating-point variables.
   * The result (the remainder) has same sign as the divisor.
   * - Similar to Matlab's mod(..).
   * - Not similar to fmod(..):
   *   FloatingPointModulo(-3, 4) = 1
   *   fmod(-3, 4) = -3
   */
  template <typename T>
  static T floatingPointModulo(const T x, const T y) {
    static_assert(!std::numeric_limits<T>::is_exact, "Floating-point type expected.");

    if (y == 0.0) {
      return x;
    }

    const double m = x - y * std::floor(x / y);

    // Handle boundary cases resulted from floating-point cut off:
    if (y > 0.0)  // modulo range: [0, y)
    {
      if (m >= y) {  // mod(-1e-16, 360.0): m = 360.0
        return 0.0;
      }

      if (m < 0) {
        if (y + m == y) {
          return 0.0;  // just in case ...
        } else {
          return y + m;  // mod(106.81415022205296 , 2*M_PI ): m = -1.421e-14
        }
      }
    } else {         // modulo range: (y, 0]
      if (m <= y) {  // mod(1e-16 , -360.0): m = -360.0
        return 0.0;
      }

      if (m > 0) {
        if (y + m == y) {
          return 0.0;  // just in case ...
        } else {
          return y + m;  // mod(-106.81415022205296, -2*M_PI): m = 1.421e-14
        }
      }
    }

    return m;
  }

  /*!
   * Wrap an angle into [-PI, PI).
   * @param angle Angle to wrap.
   * @return Wrapped angle.
   */
  template <typename T>
  static inline T wrapPosNegPI(const T angle) {
    return floatingPointModulo(angle + T(M_PI), T(2.0 * M_PI)) - M_PI;
  }

  /*!
   * Wrap an angle into [0, 2*PI).
   * @param angle Angle to wrap.
   * @return Wrapped angle.
   */
  template <typename T>
  static inline T wrapTwoPI(const T angle) {
    return floatingPointModulo(angle, T(2.0 * M_PI));
  }
};

using CalibrationBasePtr = std::shared_ptr<CalibrationBase>;

}  // namespace routine
}  // namespace calibration
}  // namespace anydrive
