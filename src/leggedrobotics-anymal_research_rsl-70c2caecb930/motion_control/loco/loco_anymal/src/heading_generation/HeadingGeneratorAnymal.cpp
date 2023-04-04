/*
 * HeadingGeneratorAnymal.cpp
 *
 *  Created on: Jan 26, 2017
 *      Author: dbellicoso
 */

#include "loco_anymal/heading_generation/HeadingGeneratorAnymal.hpp"

namespace loco_anymal {

HeadingGeneratorAnymal::HeadingGeneratorAnymal(const loco::WholeBody& wholeBody)
: HeadingGenerator(),
  wholeBody_(wholeBody)
{

}

void HeadingGeneratorAnymal::getTorsoHeadingDirectionInWorldFrame(loco::Vector& headingDirectionInWorldFrame) const {
  const loco::Position positionForeHipsMidPointInWorldFrame = (wholeBody_.getLegs().get(0).getLimbStateMeasured().getPositionWorldToLimbBaseInWorldFrame()
                                                            +  wholeBody_.getLegs().get(1).getLimbStateMeasured().getPositionWorldToLimbBaseInWorldFrame()) * 0.5;
  const loco::Position positionHindHipsMidPointInWorldFrame = (wholeBody_.getLegs().get(2).getLimbStateMeasured().getPositionWorldToLimbBaseInWorldFrame()
                                                            +  wholeBody_.getLegs().get(3).getLimbStateMeasured().getPositionWorldToLimbBaseInWorldFrame()) * 0.5;
  headingDirectionInWorldFrame = loco::Vector(positionForeHipsMidPointInWorldFrame - positionHindHipsMidPointInWorldFrame);
}

void HeadingGeneratorAnymal::getLegsHeadingDirectionFromCurrentFeetInWorldFrame(loco::Vector& headingDirectionInWorldFrame) const {
  const loco::Position positionForeFeetMidPointInWorldFrame = (wholeBody_.getLegs().get(0).getFoot().getStateMeasured().getPositionWorldToEndEffectorInWorldFrame()
                                                            +  wholeBody_.getLegs().get(1).getFoot().getStateMeasured().getPositionWorldToEndEffectorInWorldFrame()) * 0.5;
  const loco::Position positionHindFeetMidPointInWorldFrame = (wholeBody_.getLegs().get(2).getFoot().getStateMeasured().getPositionWorldToEndEffectorInWorldFrame()
                                                            +  wholeBody_.getLegs().get(3).getFoot().getStateMeasured().getPositionWorldToEndEffectorInWorldFrame()) * 0.5;
  headingDirectionInWorldFrame = loco::Vector(positionForeFeetMidPointInWorldFrame - positionHindFeetMidPointInWorldFrame);
}

void HeadingGeneratorAnymal::getLegsHeadingDirectionFromLastContactsInWorldFrame(loco::Vector& headingDirectionInWorldFrame) const {
  const loco::Position positionForeFeetMidPointInWorldFrame = (wholeBody_.getLegs().get(0).getPositionWorldToLastOrCurrentContactInWorldFrame()
                                                            +  wholeBody_.getLegs().get(1).getPositionWorldToLastOrCurrentContactInWorldFrame()) * 0.5;
  const loco::Position positionHindFeetMidPointInWorldFrame = (wholeBody_.getLegs().get(2).getPositionWorldToLastOrCurrentContactInWorldFrame()
                                                            +  wholeBody_.getLegs().get(3).getPositionWorldToLastOrCurrentContactInWorldFrame()) * 0.5;
  headingDirectionInWorldFrame = loco::Vector(positionForeFeetMidPointInWorldFrame - positionHindFeetMidPointInWorldFrame);
}

void HeadingGeneratorAnymal::getLegsHeadingDirectionFromPlannedContactsInWorldFrame(loco::Vector& headingDirectionInWorldFrame) const {
  const loco::Position positionForeFeetMidPointInWorldFrame = (wholeBody_.getLegs().get(0).getFoot().getStateDesired().getPositionWorldToEndEffectorInWorldFrame()
                                                            +  wholeBody_.getLegs().get(1).getFoot().getStateDesired().getPositionWorldToEndEffectorInWorldFrame()) * 0.5;
  const loco::Position positionHindFeetMidPointInWorldFrame = (wholeBody_.getLegs().get(2).getFoot().getStateDesired().getPositionWorldToEndEffectorInWorldFrame()
                                                            +  wholeBody_.getLegs().get(3).getFoot().getStateDesired().getPositionWorldToEndEffectorInWorldFrame()) * 0.5;
  headingDirectionInWorldFrame = loco::Vector(positionForeFeetMidPointInWorldFrame - positionHindFeetMidPointInWorldFrame);
}

bool HeadingGeneratorAnymal::computeLastFootPrintCenterInWorldFrame(loco::Position& footPrintCenterInWorldFrame) const {
  footPrintCenterInWorldFrame.setZero();

  for (const auto& leg : wholeBody_.getLegs()) {
    footPrintCenterInWorldFrame += leg->getPositionWorldToLastOrCurrentContactInWorldFrame();
  }

  if (wholeBody_.getLegs().size()>0) {
    footPrintCenterInWorldFrame /= wholeBody_.getLegs().size();
  } else { return false; }

  return true;
}

bool HeadingGeneratorAnymal::computeCurrentFootPrintCenterInWorldFrame(loco::Position& footPrintCenterInWorldFrame) const {
  footPrintCenterInWorldFrame.setZero();

  for (const auto& leg : wholeBody_.getLegs()) {
    footPrintCenterInWorldFrame += leg->getFoot().getStateMeasured().getPositionWorldToEndEffectorInWorldFrame();
  }

  if (wholeBody_.getLegs().size()>0u) {
    footPrintCenterInWorldFrame /= wholeBody_.getLegs().size();
  } else { return false; }

  return true;
}

bool HeadingGeneratorAnymal::computePlannedFootPrintCenterInWorldFrame(loco::Position& footPrintCenterInWorldFrame) const {
  footPrintCenterInWorldFrame.setZero();

  for (const auto& leg : wholeBody_.getLegs()) {
    footPrintCenterInWorldFrame += leg->getFoot().getStateDesired().getPositionWorldToFootholdInWorldFrame();
  }

  if (wholeBody_.getLegs().size()>0u) {
    footPrintCenterInWorldFrame /= wholeBody_.getLegs().size();
  } else { return false; }

  return true;
}

} /* namespace loco */
