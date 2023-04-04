/******************************************************************************
 * Authors:                                                                   *
 *    C. Dario Bellicoso                                                      *
 ******************************************************************************/

#pragma once

#include <vector>

namespace rqt_loco_visualizer {

class GaitPatternContainer {
public:

  /* ======================================================================== */
  /* Constructor/Destructor                                                   */
  /* ======================================================================== */

  GaitPatternContainer();

  ~GaitPatternContainer();

  /* ======================================================================== */
  /* Accessors                                                                */
  /* ======================================================================== */

  bool isInitialized() const;

  double getFootLiftOffPhase(int leg) const;

  double getFootTouchDownPhase(int leg) const;

  double getStancePhaseForLeg(int leg) const;

  double getSwingPhaseForLeg(int leg) const;

  double getStridePhase() const;

  bool shouldBeGrounded(int leg) const;

  void setLiftOffPhase(double phase, int leg);

  void setTouchDownPhase(double phase, int leg);

  void setStancePhase(double phase, int leg);

  void setSwingPhase(double phase, int leg);

  void setStridePhase(double phase);

  void setShouldBeGrounded(bool shouldBeGrounded, int leg);

  void setDuration(double duration);

  double getDuration() const;

  void setStartPhase(double startPhase);

  double getStartPhase() const;

protected:

  /* ======================================================================== */
  /* Variables                                                                */
  /* ======================================================================== */

  bool isInitialized_;

  std::vector<double> liftOffPhase_;

  std::vector<double> touchDownPhase_;

  double duration_;

  double startPhase_;

  std::vector<double> stancePhase_;

  std::vector<double> swingPhase_;

  std::vector<bool> shouldBeGrounded_;

  double stridePhase_;
};

} // namespace
