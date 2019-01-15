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

#ifndef TEST_TRACKING_UTILITY_H
#define TEST_TRACKING_UTILITY_H
#include "opencv2/opencv.hpp"
/*! @brief
 * The TrackingUtility Class handles simple start and stop tracking logic
 */
class TrackingUtility
{
public:
  TrackingUtility()
          : P1(0,0),
            P2(0,0),
            roi(0,0,0,0),
            mouseClicked(false),
            roiSelected(false),
            state(STATE_IDLE)
  {
  }

  typedef enum TrackingState
  {
    STATE_IDLE,
    STATE_INIT,
    STATE_ONGOING,
    STATE_STOP
  } TrackingState;

  static void mouseCallback(int event, int x, int y, int f, void *p);

  cv::Point P1;
  cv::Point P2;
  bool mouseClicked;
  cv::Rect roi;

  /*!
   * start_tracking is set true when you select a region and press 'g'
   * is set to false when you press 's'
   */
  bool roiSelected;
  TrackingState state;
  TrackingState getState();

  void startTracker();
  void stopTracker();

  cv::Rect getROI();
  void getKey(char c);
};
#endif //TEST_TRACKING_UTILITY_H
