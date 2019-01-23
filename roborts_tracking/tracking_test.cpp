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

#include <iostream>
#include <fstream>
#include <string>
#include <cstdio>
#include <chrono>
#include <ros/ros.h>
#include "roborts_msgs/GimbalAngle.h"

#include "tracking_utility.h"

#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "KCFcpp/src/kcftracker.hpp"

#define HOG 1
#define FIXEDWINDOW 1
#define MULTISCALE 1
#define LAB 1

typedef std::chrono::time_point<std::chrono::high_resolution_clock> timer;
typedef std::chrono::duration<float> duration;

using namespace std;
using namespace cv;

int main(int argc, char** argv)
{
  ros::init(argc, argv, "roborts_tracking_node");
  ros::NodeHandle nh;
  auto  pub= nh.advertise<roborts_msgs::GimbalAngle>("cmd_gimbal_angle", 30);
  const char winName[]="My Camera";
  char message1[100];
  char message2[100];
  Rect roi(0,0,0,0);
  TrackingUtility tu;
  KCFTracker *tracker = NULL;

  cv::namedWindow(winName,1);
  cv::setMouseCallback(winName,TrackingUtility::mouseCallback, (void*)&tu);


  VideoCapture video(0);
  int img_width = 640;
  int img_height = 480;
  video.set(CV_CAP_PROP_FRAME_WIDTH,img_width);
  video.set(CV_CAP_PROP_FRAME_HEIGHT,img_height);
  img_width = video.get(CV_CAP_PROP_FRAME_WIDTH);
  img_height = video.get(CV_CAP_PROP_FRAME_HEIGHT);

  if (!video.isOpened()) {
    cout << "cannot read video!" << endl;
    return -1;
  }
  //可选BOOSTING, MIL, KCF, TLD, MEDIANFLOW, or GOTURN

  while(ros::ok())
  {
    char c = cv::waitKey(10);
    if(c==27)
    {
      if(tracker != NULL)
      {
        delete tracker;
        tracker = NULL;
      }
      break; // Quit if ESC is pressed
    }

    tu.getKey(c); //Internal states will be updated based on key pressed.

    Mat frame;
    if(video.read(frame)){
      int dx = 0;
      int dy = 0;
      int yawRate = 0;
      int pitchRate = 0;
      timer trackerStartTime, trackerFinishTime;
      duration trackerTimeDiff;

      roborts_msgs::GimbalAngle gimbal_angle;
      int k = 1920/img_width;
      switch(tu.getState())
      {
        case TrackingUtility::STATE_IDLE:
          roi = tu.getROI();
          sprintf(message2, "Please select ROI and press g");
          break;

        case TrackingUtility::STATE_INIT:
          cout << "g pressed, initialize tracker" << endl;
          sprintf(message2, "g pressed, initialize tracker");
          roi = tu.getROI();
          tracker = new KCFTracker(HOG, FIXEDWINDOW, MULTISCALE, LAB);
          tracker->init(roi, frame);
          tu.startTracker();
          break;

        case TrackingUtility::STATE_ONGOING:
          trackerStartTime  = std::chrono::high_resolution_clock::now();
          roi = tracker->update(frame);
          trackerFinishTime = std::chrono::high_resolution_clock::now();
          trackerTimeDiff = trackerFinishTime - trackerStartTime;
          sprintf(message2, "Tracking: bounding box update time = %.2f ms\n", trackerTimeDiff.count()*1000.0);

          // send gimbal speed command
          dx = (int)(roi.x + roi.width/2  - img_width/2);
          dy = (int)(roi.y + roi.height/2 - img_height/2);

          yawRate   = -dx;
          pitchRate = dy;
          cout<<"yaw_rate:"<<yawRate<<endl;
          cout<<"pitch_rate:"<<pitchRate<<endl;
          if(abs(yawRate) < 10/k)
          {
            yawRate = 0;
          }
          else if(abs(yawRate)>500/k) {
            yawRate = ((yawRate>0)?1:-1)*500/k;
          }

          if(abs(pitchRate) < 10/k)
          {
            pitchRate = 0;
          }
          else if(abs(pitchRate)>500/k) {
            pitchRate = ((pitchRate>0)?1:-1)*500/k;
          }

          gimbal_angle.pitch_mode = true;
          gimbal_angle.pitch_angle = pitchRate/180.*M_PI/110.*k;

          gimbal_angle.yaw_mode = true;
          gimbal_angle.yaw_angle = yawRate/180.*M_PI/160.*k;
          pub.publish(gimbal_angle);

          break;

        case TrackingUtility::STATE_STOP:
          cout << "s pressed, stop tracker" << endl;
          sprintf(message2, "s pressed, stop tracker");
          delete tracker;
          tracker = NULL;
          tu.stopTracker();
          roi = tu.getROI();
          break;

        default:
          break;
      }
      dx = roi.x + roi.width/2  - img_width/2;
      dy = roi.y + roi.height/2 - img_height/2;

      cv::circle(frame, Point(img_width/2, img_height/2), 5, cv::Scalar(255,0,0), 2, 8);
      if(roi.width != 0)
      {
        cv::circle(frame, Point(roi.x + roi.width/2, roi.y + roi.height/2), 3, cv::Scalar(0,0,255), 1, 8);

        cv::line(frame,  Point(img_width/2, img_height/2),
                 Point(roi.x + roi.width/2, roi.y + roi.height/2),
                 cv::Scalar(0,255,255));
      }

      cv::rectangle(frame, roi, cv::Scalar(0,255,0), 1, 8, 0 );
      sprintf(message1,"dx=%04d, dy=%04d",dx, dy);
      putText(frame, message1, Point2f(20,30), FONT_HERSHEY_SIMPLEX, 0.5,  Scalar(0,255,0));
      putText(frame, message2, Point2f(20,60), FONT_HERSHEY_SIMPLEX, 0.5,  Scalar(0,255,0));
      cv::imshow(winName, frame);

    }

  }

  if(tracker)
  {
    delete tracker;
  }

  return 0;
}