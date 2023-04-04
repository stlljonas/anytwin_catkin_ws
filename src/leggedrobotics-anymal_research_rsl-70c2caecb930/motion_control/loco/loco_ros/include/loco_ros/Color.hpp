/*
 * Color.hpp
 *
 *  Created on: Mar 1, 2017
 *  Author: Gabriel Hottiger
 */

#pragma once

namespace loco_ros {

// See http://www.colourlovers.com/blog/2007/07/24/32-common-color-names-for-easy-reference
enum class ColorEnum : unsigned int {
  BLACK = 0,
  WHITE,
  RED,
  GREEN,
  BLUE,
  YELLOW,
  IVORY,
  BEIGE,
  WHEAT,
  TAN,
  KHAKI,
  SILVER,
  GRAY,
  CHARCOAL,
  NAVY_BLUE,
  ROYAL_BLUE,
  MEDIUM_BLUE,
  AZURE,
  CYAN,
  AQUAMARINE,
  TEAL,
  FOREST_GREEN,
  OLIVE,
  CHARTREUSE,
  LIME,
  GOLDEN,
  GOLDENROD,
  CORAL,
  SALMON,
  HOT_PINK,
  FUCHSIA,
  PUCE,
  MAUVE,
  LAVENDER,
  PLUM,
  INDIGO,
  MAROON,
  CRIMSON
};

class Color {
 public:
  Color(): Color(0.0, 0.0, 0.0) { }
  Color(int r, int g, int b, double a = 1.0): r_(r), g_(g), b_(b), a_(a) { }
  Color(ColorEnum color, double a = 1.0): a_(a) {
    switch(color) {
      case ColorEnum::BLACK:
        r_ = 0; g_ = 0; b_ = 0;
        break;
      case ColorEnum::WHITE:
        r_ = 255; g_ = 255; b_ = 255;
        break;
      case ColorEnum::RED:
        r_ = 255; g_ = 0; b_ = 0;
        break;
      case ColorEnum::GREEN:
        r_ = 0; g_ = 255; b_ = 0;
        break;
      case ColorEnum::BLUE:
        r_ = 0; g_ = 0; b_ = 255;
        break;
      case ColorEnum::YELLOW:
        r_ = 255; g_ = 255; b_ = 0;
        break;
      case ColorEnum::IVORY:
        r_ = 255; g_ = 255; b_ = 240;
        break;
      case ColorEnum::BEIGE:
        r_ = 245; g_ = 245; b_ = 220;
        break;
      case ColorEnum::WHEAT:
        r_ = 255; g_ = 222; b_ = 179;
        break;
      case ColorEnum::TAN:
        r_ = 210; g_ = 180; b_ = 140;
        break;
      case ColorEnum::KHAKI:
        r_ = 195; g_ = 176; b_ = 145;
        break;
      case ColorEnum::SILVER:
        r_ = 192; g_ = 192; b_ = 192;
        break;
      case ColorEnum::GRAY:
        r_ = 128; g_ = 128; b_ = 128;
        break;
      case ColorEnum::CHARCOAL:
        r_ = 70; g_ = 70; b_ = 70;
        break;
      case ColorEnum::NAVY_BLUE:
        r_ = 0; g_ = 0; b_ = 128;
        break;
      case ColorEnum::ROYAL_BLUE:
        r_ = 8; g_ = 76; b_ = 158;
        break;
      case ColorEnum::MEDIUM_BLUE:
        r_ = 0; g_ = 0; b_ = 205;
        break;
      case ColorEnum::AZURE:
        r_ = 0; g_ = 127; b_ = 255;
        break;
      case ColorEnum::CYAN:
        r_ = 0; g_ = 255; b_ = 255;
        break;
      case ColorEnum::AQUAMARINE:
        r_ = 127; g_ = 255; b_ = 212;
        break;
      case ColorEnum::TEAL:
        r_ = 0; g_ = 128; b_ = 128;
        break;
      case ColorEnum::FOREST_GREEN:
        r_ = 34; g_ = 139; b_ = 34;
        break;
      case ColorEnum::OLIVE:
        r_ = 128; g_ = 128; b_ = 0;
        break;
      case ColorEnum::CHARTREUSE:
        r_ = 127; g_ = 255; b_ = 0;
        break;
      case ColorEnum::LIME:
        r_ = 191; g_ = 255; b_ = 0;
        break;
      case ColorEnum::GOLDEN:
        r_ = 255; g_ = 215; b_ = 0;
        break;
      case ColorEnum::GOLDENROD:
        r_ = 218; g_ = 165; b_ = 32;
        break;
      case ColorEnum::CORAL:
        r_ = 255; g_ = 127; b_ = 80;
        break;
      case ColorEnum::SALMON:
        r_ = 250; g_ = 128; b_ = 114;
        break;
      case ColorEnum::HOT_PINK:
        r_ = 252; g_ = 15; b_ = 192;
        break;
      case ColorEnum::FUCHSIA:
        r_ = 255; g_ = 119; b_ = 255;
        break;
      case ColorEnum::PUCE:
        r_ = 204; g_ = 136; b_ = 153;
        break;
      case ColorEnum::MAUVE:
        r_ = 224; g_ = 176; b_ = 255;
        break;
      case ColorEnum::LAVENDER:
        r_ = 181; g_ = 126; b_ = 220;
        break;
      case ColorEnum::PLUM:
        r_ = 132; g_ = 49; b_ = 121;
        break;
      case ColorEnum::INDIGO:
        r_ = 75; g_ = 0; b_ = 130;
        break;
      case ColorEnum::MAROON:
        r_ = 128; g_ = 0; b_ = 0;
        break;
      case ColorEnum::CRIMSON:
        r_ = 220; g_ = 20; b_ = 60;
        break;
      default:
        r_ = 0; g_ = 0; b_ = 0;
    }
  }

  int r() const { return r_; }
  int g() const { return g_; }
  int b() const { return b_; }
  int a() const { return a_; }

  double rNormalized() const { return static_cast<double>(r_)/255.0; }
  double gNormalized() const { return static_cast<double>(g_)/255.0; }
  double bNormalized() const { return static_cast<double>(b_)/255.0; }

  void setR(int r) { r_ = r; }
  void setG(int g) { g_ = g; }
  void setB(int b) { b_ = b; }
  void setA(int a) { a_ = a; }

 private:
  int    r_, g_, b_;
  double a_;
};

}


