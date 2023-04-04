/*
 * EndEffectorEnum.hpp
 *
 *  Created on: Mar 20, 2017
 *      Author: Gabriel Hottiger
 */

#pragma once

namespace loco {

class EndEffectorEnum {
 public:
  constexpr static unsigned int Origin = 0;
};

class EndEffectorContactEnum : public EndEffectorEnum {
 public:
  constexpr static unsigned int Contact = 1;
};

}  // namespace loco