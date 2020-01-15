#ifndef ROBORTS_DETECTION_ARMOR_LIGHT_BLOB_
#define ROBORTS_DETECTION_ARMOR_LIGHT_BLOB_

#include <list>
#include <vector>
#include <algorithm>
#include <functional>

#include <opencv2/opencv.hpp>
#include <alg_factory/algorithm_factory.h>
#include "state/error_code.h"

#include "cv_toolbox.h"
#include "../armor_detection_base.h"

#define ENEMY_SIZE 7.45 // Plate radius size in cm
#define MAX_ANGLE  0.3191713446 //Max Vertical viewing angle in Radius

namespace roborts_detection
{
    using roborts_common::ErrorCode;
    using roborts_common::ErrorInfo;

    // Over load the operator for LBInfo compare
  
   class LBInfo
   {
       public:
       cv::Point2f center;
       float radius;
   };

    class LightBlob: public ArmorDetectionBase
    {
    public:
        LightBlob(std::shared_ptr<CVToolbox> cv_toolbox);
        void LoadParam() override;
        ErrorInfo DetectArmor(bool &detected,cv::Point3f &target_3d) override;
        void CalcControlInfo(const LBInfo & lightblob,cv::Point3f &target_3d);
        void SignalFilter(double &new_num, double &old_num,unsigned int &filter_count, double max_diff);
        void SetThreadState(bool thread_state) override;
        ~LightBlob() final;
    private:
        ErrorInfo error_info_;
        unsigned int filter_x_count_;
        unsigned int filter_y_count_;
        unsigned int filter_z_count_;
        unsigned int filter_distance_count_;
        unsigned int filter_pitch_count_;
        unsigned int filter_yaw_count_;

        cv::Mat src_img_;
        cv::Mat intrinsic_matrix_;
        cv::Mat distortion_coeffs_;
        int read_index_;
        double detection_time_;

        bool enable_debug_;
        unsigned int enemy_color_;

        cv::Mat show_raw;
        cv::Mat show_minus;
        cv::Mat show_thresh;
        cv::Mat show_result;

        ros::NodeHandle nh;
        bool thread_running_;

    };
    roborts_common::REGISTER_ALGORITHM(ArmorDetectionBase,"light_blob_test",LightBlob,std::shared_ptr<CVToolbox>);

}

#endif

