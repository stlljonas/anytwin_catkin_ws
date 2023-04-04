/*
 * loco_tests.hpp
 *
 *  Created on: Jan 22, 2018
 *      Author: Gabriel Hottiger
 */

#pragma once

// gtest
#include <gtest/gtest.h>

#define FIXTURE_TEST_TYPE LOCO_TEST_TYPE
#include "WholeBody_tests.hpp"
#undef FIXTURE_TEST_TYPE
