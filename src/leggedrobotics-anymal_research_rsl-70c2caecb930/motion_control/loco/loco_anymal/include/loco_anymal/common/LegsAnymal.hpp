/*
 * LegGroup.hpp
 *
 *  Created on: Feb 25, 2014
 *      Author: Christian Gehring, Dario Bellicoso, Gabriel Hottiger
 */

#pragma once

// loco
#include "loco/common/GroupContainer.hpp"

// loco anymal
#include "loco_anymal/common/LegAnymal.hpp"

namespace loco_anymal {

//! Container of legs for easy access
/*! Allows to iterate over all legs:
 *  LegGroup* legs = new LegGroup();
 *  leg* leg = new LegBase()
 *  legs.addleg(leg);
 *  legs.addLeg()
 *  for (auto leg : *legs) {
 *    leg->...
 *  }
 */
class LegsAnymal : public loco::GroupContainer<LegAnymal, std::unique_ptr> {
 public:
  using Base = GroupContainer<LegAnymal>;
  using ItemSmartPtr = typename Base::ItemSmartPtr;

  //! Constructor
  LegsAnymal() = default;

  /*! Constructor
   *
   * Create leg group with four legs
   * @param leftForeLeg
   * @param rightForeLeg
   * @param leftHindLeg
   * @param rightHindLeg
   */
  LegsAnymal(ItemSmartPtr&& leftForeLeg,
                ItemSmartPtr&& rightForeLeg,
                ItemSmartPtr&& leftHindLeg,
                ItemSmartPtr&& rightHindLeg);

  //! Destructor
  ~LegsAnymal() override = default;

  //! @return const ref to left fore leg
  const LegAnymal& getLeftForeLeg() const   { return *(items_[0]); }
  //! @return const ref to right fore leg
  const LegAnymal& getRightForeLeg() const  { return *(items_[1]); }
  //! @return const ref to left hind leg
  const LegAnymal& getLeftHindLeg() const   { return *(items_[2]); }
  //! @return const ref to right hind leg
  const LegAnymal& getRightHindLeg() const  { return *(items_[3]); }

  //! @return ptr to left fore leg
  LegAnymal* getLeftForeLegPtr()   { return items_[0].get(); }
  //! @return ptr to right fore leg
  LegAnymal* getRightForeLegPtr()  { return items_[1].get(); }
  //! @return ptr to left hind leg
  LegAnymal* getLeftHindLegPtr()   { return items_[2].get(); }
  //! @return ptr to right hind leg
  LegAnymal* getRightHindLegPtr()  { return items_[3].get(); }

  /*! Gets leg by identifier
    *
    * @param legId  id of the leg (LF = 0, RF = 1, LH = 2, RH = 3)
    * @return  const ref to leg
    */
  const LegAnymal& getLegById(int legId) const;

  /*! Gets leg by identifier
    *
    * @param legId  id of the leg (LF = 0, RF = 1, LH = 2, RH = 3)
    * @return  ptr to leg
    */
  LegAnymal* getLegPtrById(int legId);

  /*! Get leg from a LimbEnum identifier.
   *
   * @param limbEnum Leg identifier.
   * @return Const reference to leg.
   */
  const LegAnymal& getLeg(anymal_description::AnymalDescription::LimbEnum limbEnum) const {
    return getLegById(static_cast<int>(limbEnum));
  }

  /*! Get leg from a LimbEnum identifier.
   *
   * @param limbEnum Leg identifier.
   * @return Pointer to leg.
   */
  LegAnymal* getLegPtr(anymal_description::AnymalDescription::LimbEnum limbEnum) {
    return getLegPtrById(static_cast<int>(limbEnum));
  }
};

} /* namespace loco_anymal */
