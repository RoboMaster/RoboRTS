#include <time.h>
#include <stdlib.h>
#include <stdio.h>

#include "darknet.h"
#include "c2cpp.h"

//for cvRound error
int cvRound(double value) {return(ceil(value));}

#ifdef __cplusplus
extern "C" {
#endif

void InitYOLO(const char* datacfg, const char* cfg, const char* weights, float thresh, bool enable_debug) {
#ifndef GPU
  gpu_index = -1;
#else
  if(gpu_index >= 0){
    cuda_set_device(gpu_index);
  }
#endif
  list *options = read_data_cfg(datacfg);
  int classes = option_find_int(options, "classes", 20);
  char *name_list = option_find_str(options, "names", "modules/perception/detection/yolo/config/mobilenet.names");
  char **names = get_labels(name_list);
  demo_init(cfg, weights, thresh, names, classes, enable_debug);
}

void RunYOLO(bool enable, int camera_id, int **x_offset, int *object_num){
  demo(enable, camera_id, x_offset, object_num);
}

#ifdef __cplusplus
};
#endif