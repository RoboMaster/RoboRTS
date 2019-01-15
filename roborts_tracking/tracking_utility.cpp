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

#include "tracking_utility.h"
#include <iostream>
void TrackingUtility::mouseCallback(int event, int x, int y, int f, void *p)
{
  TrackingUtility *u = reinterpret_cast<TrackingUtility*>(p);
  if(u->state != STATE_IDLE)
  {
    std::cout << "Currently tracking, press s key to stop" << std::endl;
    return;
  }

  switch(event)
  {
    case  CV_EVENT_LBUTTONDOWN  :
      u->mouseClicked = true;
      u->roiSelected = false;
      u->P1 = cv::Point(x,y);
      u->P2 = cv::Point(x,y);
      break;

    case  CV_EVENT_LBUTTONUP    :
      u->P2 = cv::Point(x,y);
      u->mouseClicked=false;
      if(u->P2 != u->P1)
      {
        u->roiSelected = true;
      }
      break;

    case  CV_EVENT_MOUSEMOVE    :
      if(u->mouseClicked)
      {
        u->P2 = cv::Point(x,y);
      }
      break;

    default                     :
      break;
  }

  if(u->mouseClicked)
  {
    u->roi = cv::Rect(u->P1, u->P2);
    printf("Current Region of Interest: %d, %d, %d, %d\n", u->roi.tl().x, u->roi.tl().y, u->roi.br().x, u->roi.br().y);
  }
}

cv::Rect TrackingUtility::getROI()
{
  return roi;
}

TrackingUtility::TrackingState TrackingUtility::getState()
{
  return state;
}

void TrackingUtility::startTracker()
{
  state = STATE_ONGOING;
}

void TrackingUtility::stopTracker()
{
  state = STATE_IDLE;
}

void TrackingUtility::getKey(char c)
{
  switch(c)
  {
    case 'g':
      if( (state == STATE_IDLE) && (roiSelected == true))
      {
        state = STATE_INIT;
      }
      break;

    case 's':
      if( state == STATE_ONGOING )
      {
        state = STATE_STOP;
        roi = cv::Rect(0,0,0,0); //when we press s, should clear bounding box
      }
      break;

    default:
      break;
  }
}