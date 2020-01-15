#include "lightBlob_test.h"
#include "timer/timer.h"
#include "io/io.h"
#include <string>
#include <sstream>

namespace roborts_detection
{
    bool compare(LBInfo &l1, LBInfo &l2)
    {
        return l1.radius > l2.radius;
    }

    LightBlob::LightBlob(std::shared_ptr<CVToolbox> cv_toolbox):
    ArmorDetectionBase(cv_toolbox){
        filter_x_count_ = 0;
        filter_y_count_ = 0;
        filter_distance_count_ = 0;
        filter_pitch_count_ = 0;
        filter_yaw_count_ = 0;
        read_index_ = -1; 
        detection_time_ = 0;
        thread_running_ = false;
        enable_debug_ = true; // Tobo Modified
        enemy_color_ = 1; // Means blue
        int get_intr_state = -1;
        int get_dist_state = -1;
        LoadParam();
        while((get_dist_state<0) || (get_intr_state<0))
        {
            ROS_WARN("wait for camera driver launch %d", get_intr_state);
            usleep(50000);
            ros::spinOnce();
            get_intr_state = cv_toolbox_->GetCameraMatrix(intrinsic_matrix_);
            get_dist_state = cv_toolbox_->GetCameraMatrix(distortion_coeffs_);
            
        }
        error_info_ = ErrorInfo(roborts_common::OK);
    }

    ErrorInfo LightBlob::DetectArmor(bool &detected, cv::Point3f &target_3d)
    {
        auto img_begin = std::chrono::high_resolution_clock::now();
        bool sleep_by_diff_flag = true;
        // Below is a time match which make sure real time performance as well as 
        // Breakable thread
        while(true)
        {
            if(!thread_running_)
            {
                ErrorInfo error_info(ErrorCode::STOP_DETECTION);
                return error_info;
            }
            read_index_ = cv_toolbox_->NextImage(src_img_);
            if(read_index_ < 0)
            {
                if(detection_time_ == 0)
                {
                    usleep(20000);
                    continue;
                }
                else
                {
                    double capture_time = 0;
                    cv_toolbox_->GetCaptureTime(capture_time);
                    if(capture_time == 0) // The very beginning
                    {
                        usleep(20000);
                        continue;
                    }
                    else if(capture_time > detection_time_ && sleep_by_diff_flag)
                    {
                        usleep((unsigned int)(capture_time - detection_time_));
                        sleep_by_diff_flag = false;
                        continue;
                    }
                    else
                    {
                        usleep(500);
                        continue;
                    }
                }
            }
            else
            {
                break;
            }
                
        }

        auto detection_begin = std::chrono::high_resolution_clock::now();
        if(enable_debug_)
        {
            show_minus = src_img_.clone();
            show_thresh= src_img_.clone();
            show_raw   = src_img_.clone(); 
        }
        show_minus = cv_toolbox_->DistillationColor(src_img_,enemy_color_,false);
        if(enable_debug_)
            cv::imshow("minus",show_minus);
        cv::threshold(show_minus,show_thresh,80,255,cv::THRESH_BINARY);
        if(enable_debug_)
            cv::imshow("thresh",show_thresh);
        std::vector<std::vector<cv::Point>> cnts;
        cnts = cv_toolbox_->FindContours(show_thresh);
        // Using list of lb_info to represent light blob positional data
        std::list<LBInfo> candidates; 
        LBInfo target;
        // Constrain operation filtered proper armor box
        if(cnts.size() > 0)
        {
            for(int i=0;i<cnts.size();i++)
            {
                std::vector<cv::Point> cnt = cnts[i];
                double lb_area = cv::contourArea(cnt);
                if (lb_area >=500 && lb_area <= 30000 )
                {
                    LBInfo lb;
                    cv::minEnclosingCircle(cnt,lb.center,lb.radius);
                    if(lb.radius*lb.radius*3.141592 <= 1.2*lb_area) 
                    {
                        for(int j=0;j<cnt.size();j++)
                        {
                            lb.radius += sqrt(  (cnt[j].x-lb.center.x)*(cnt[j].x-lb.center.x) + (cnt[j].y-lb.center.y)*(cnt[j].y-lb.center.y) );
                        }
                        lb.radius /= cnt.size()+1;
                        candidates.push_back(lb);
                    }
                }
            }
            std::cout<<candidates.size()<<std::endl;

            if(candidates.size() > 0)
            {
                candidates.sort(compare);
                detected = true;
                target = *candidates.begin();
            
                double df = ENEMY_SIZE/tan(MAX_ANGLE);
                double yaw   = atan( (target.center.x-cv_toolbox_->GetCameraWidth()/2) *tan(MAX_ANGLE)/(cv_toolbox_->GetCameraHeight()/2));
                // Need to be determined 
                double pitch = -atan( (target.center.y-cv_toolbox_->GetCameraHeight()/2) *tan(MAX_ANGLE)/(cv_toolbox_->GetCameraHeight()/2));
                target_3d.z = (cv_toolbox_->GetCameraHeight()/2*df)/target.radius;
                target_3d.x = target_3d.z * tan(yaw);
                target_3d.y = target_3d.z * tan(pitch);
                if(enable_debug_)
                {
                    cv::Point p(int(target.center.x),int(target.center.y));
                    cv::circle(show_raw,p,target.radius,cv::Scalar(0,255,0));
                    //std::stringstream ss;
                    //ss<<"The Distance is";
                    //ss<<sqrt((target_3d.x*target_3d.x)+(target_3d.y*target_3d.y)+(target_3d.z*target_3d.z));
                    std::cout<<"Distance: "<<sqrt((target_3d.x*target_3d.x)+(target_3d.y*target_3d.y)+(target_3d.z*target_3d.z))<<std::endl;
                    cv::imshow("result image",show_raw);
                }
            }
            else
            {
                detected = false;
            }
        }
        else
        {
            detected = false;
        }
        auto c = cv::waitKey(1);
        if(c=='a')
        {
            cv::waitKey(0);
        }
        
        cv_toolbox_->ReadComplete(read_index_);
        ROS_INFO("read complete");
        detection_time_ = std::chrono::duration<double, std::ratio<1, 1000000>>
        (std::chrono::high_resolution_clock::now() - detection_begin).count();
        return error_info_;
    }

    void LightBlob::SignalFilter(double &new_num, double &old_num, unsigned int &filter_count, double max_diff)
    {
        if(fabs(new_num - old_num) > max_diff && filter_count < 2)
        {
            filter_count++;
            new_num+= max_diff;
        }
        else
        {
            filter_count = 0;
            old_num = new_num;
        }
    }

    void LightBlob::LoadParam()
    {
        return;
    }

    void LightBlob::SetThreadState(bool thread_state)
    {
        thread_running_ = thread_state;
    }

    LightBlob::~LightBlob()
    {
        // Temperorily do nothing.
    }

}