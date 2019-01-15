/****************************************************************************
 *  Copyright (C) 2019 RoboMaster.
 *
 *  This program is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of 
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program. If not, see <http://www.gnu.org/licenses/>.
 ***************************************************************************/

#ifndef ROBORTS_DECISION_LINE_ITERATOR_H
#define ROBORTS_DECISION_LINE_ITERATOR_H

#include <stdlib.h>

namespace roborts_decision {

class FastLineIterator {
  // this method is a modified version of base_local_planner/line_iterator.h
 public:
  FastLineIterator( int x0, int y0, int x1, int y1 )
      : x0_( x0 )
      , y0_( y0 )
      , x1_( x1 )
      , y1_( y1 )
      , x_( x0 )
      , y_( y0 )
      , deltax_(abs(x1 - x0))
      , deltay_(abs(y1 - y0))
      , curpixel_( 0 ) {

    xinc1_ = (x1 - x0) >0 ?1:-1;
    xinc2_ = (x1 - x0) >0 ?1:-1;

    yinc1_ = (y1 - y0) >0 ?1:-1;
    yinc2_ = (y1 - y0) >0 ?1:-1;

    if( deltax_ >= deltay_ ) {
      xinc1_ = 0;
      yinc2_ = 0;
      den_ = 2 * deltax_;
      num_ = deltax_;
      numadd_ = 2 * deltay_;
      numpixels_ = deltax_;
    } else {
      xinc2_ = 0;
      yinc1_ = 0;
      den_ = 2 * deltay_;
      num_ = deltay_;
      numadd_ = 2 * deltax_;
      numpixels_ = deltay_;
    }
  }
  ~FastLineIterator() = default;
  bool IsValid() const {
    return curpixel_ <= numpixels_;
  }

  void Advance() {
    num_ += numadd_;
    if( num_ >= den_ ) {
      num_ -= den_;
      x_ += xinc1_;
      y_ += yinc1_;
    }
    x_ += xinc2_;
    y_ += yinc2_;

    curpixel_++;
  }

  int GetX() const { return x_; }
  int GetY() const { return y_; }

  int GetX0() const { return x0_; }
  int GetY0() const { return y0_; }

  int GetX1() const { return x1_; }
  int GetY1() const { return y1_; }

 private:
  int x0_;
  int y0_;
  int x1_;
  int y1_;

  int x_;
  int y_;

  int deltax_;
  int deltay_;

  int curpixel_;

  int xinc1_, xinc2_, yinc1_, yinc2_;
  int den_, num_, numadd_, numpixels_;
};

} // namespace roborts_decision

#endif //ROBORTS_DECISION_LINE_ITERATOR_H

