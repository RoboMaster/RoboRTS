#include <sys/time.h>

#include "network.h"
#include "cpp2c.hpp"
#include "detection_layer.h"
#include "region_layer.h"
#include "cost_layer.h"
#include "utils.h"
#include "parser.h"
#include "box.h"
#include "image.h"
#include "demo.h"
#include "layer.h"

#define DEMO 1

#ifdef OPENCV

static char **demo_names;
static image **demo_alphabet;
static int demo_classes;

static network *net;
static image buff;
static image buff_letter;
static CvCapture * cap;
static IplImage  * ipl;
static bool Initialized = false;
static double fps = 0;
static float demo_thresh = 0;
static float demo_hier = .5;
static int running = 0;

static int demo_frame = 1;
static int demo_index = 0;
static float **predictions;
static float *avg;
static int demo_total = 0;
int camera_index = 0;
double demo_time;

detection *get_network_boxes(network *net, int w, int h, float thresh, float hier, int *map, int relative, int *num);

int size_network(network *net)
{
  int i;
  int count = 0;
  for(i = 0; i < net->n; ++i){
    layer l = net->layers[i];
    if(l.type == YOLO || l.type == REGION || l.type == DETECTION){
      count += l.outputs;
    }
  }
  return count;
}

void remember_network(network *net)
{
  int i;
  int count = 0;
  for(i = 0; i < net->n; ++i){
    layer l = net->layers[i];
    if(l.type == YOLO || l.type == REGION || l.type == DETECTION){
      memcpy(predictions[demo_index] + count, net->layers[i].output, sizeof(float) * l.outputs);
      count += l.outputs;
    }
  }
}

detection *avg_predictions(network *net, int *nboxes)
{
  int i, j;
  int count = 0;
  fill_cpu(demo_total, 0, avg, 1);
  for(j = 0; j < demo_frame; ++j){
    axpy_cpu(demo_total, 1./demo_frame, predictions[j], 1, avg, 1);
  }
  for(i = 0; i < net->n; ++i){
    layer l = net->layers[i];
    if(l.type == YOLO || l.type == REGION || l.type == DETECTION){
      memcpy(l.output, avg + count, sizeof(float) * l.outputs);
      count += l.outputs;
    }
  }
  detection *dets = get_network_boxes(net, buff.w, buff.h, demo_thresh, demo_hier, 0, 1, nboxes);
  return dets;
}

void detect_in_thread(int **x_offset, int *object_num)
{
  running = 1;
  float nms = .4;

  layer l = net->layers[net->n-1];
  float *X = buff_letter.data;
  network_predict(net, X);

  /*
     if(l.type == DETECTION){
     get_detection_boxes(l, 1, 1, demo_thresh, probs, boxes, 0);
     } else */
  remember_network(net);
  detection *dets = 0;
  int nboxes = 0;
  dets = avg_predictions(net, &nboxes);

  if (nms > 0) do_nms_obj(dets, nboxes, l.classes, nms);

  printf("\033[2J");
  printf("\033[1;1H");
  printf("\nFPS:%.1f\n",fps);
  printf("Objects:\n\n");
  image display = buff;

  *object_num = nboxes;
  *x_offset = NULL;
  *x_offset = calloc(nboxes, sizeof(int));
  draw_detections(display, dets, nboxes, demo_thresh, demo_names, demo_alphabet, demo_classes, x_offset);
  free_detections(dets, nboxes);

  demo_index = (demo_index + 1)%demo_frame;
  running = 0;
}

void display_in_thread()
{
  show_image_cv(buff, "Demo", ipl);
  int c = cvWaitKey(1);
  if (c != -1) c = c%256;
  if (c == 27) {
    return;
  } else if (c == 82) {
    demo_thresh += .02;
  } else if (c == 84) {
    demo_thresh -= .02;
    if(demo_thresh <= .02) demo_thresh = .02;
  } else if (c == 83) {
    demo_hier += .02;
  } else if (c == 81) {
    demo_hier -= .02;
    if(demo_hier <= .0) demo_hier = .0;
  }
}

void demo_init(char *cfgfile, char *weightfile, float thresh, char **names, int classes, bool enable_debug)
{
  //image **alphabet = load_alphabet();
  demo_names = names;
  //demo_alphabet = alphabet;
  demo_classes = classes;
  demo_thresh = thresh;
  demo_hier = 0.5;
  printf("Demo\n");
  net = load_network(cfgfile, weightfile, 0);
  set_batch_network(net, 1);
  srand(2222222);

  int i;
  demo_total = size_network(net);
  predictions = calloc(demo_frame, sizeof(float*));
  for (i = 0; i < demo_frame; ++i){
    predictions[i] = calloc(demo_total, sizeof(float));
  }
  avg = calloc(demo_total, sizeof(float));
}

void demo(bool enable, int camera_id, int **x_offset, int *object_num)
{
  ipl = GetNextImage(camera_id);
  if(ipl != NULL) {
    static char start_message[] = "YOLO started!";
    Notice(start_message);
    demo_time = what_time_is_it_now();
    if(!Initialized){
      buff = ipl_to_image(ipl);
      buff_letter = letterbox_image(buff, net->w, net->h);
      Initialized = true;
    }

    ipl_into_image(ipl, buff);
    rgbgr_image(buff);
    letterbox_image_into(buff, net->w, net->h, buff_letter);
    //free_image(buff);
    //free_image(buff_letter);
    camera_index %= 1;
    detect_in_thread(x_offset, object_num);
    fps = 1. / (what_time_is_it_now() - demo_time);
    if(enable)
      display_in_thread();
    cvRelease(&ipl);
  } else {
    char message[] = "Waiting for camera driver...";
    Notice(message);
  }
  //cvRelease(ipl);
}
#else
void demo(char *cfgfile, char *weightfile, float thresh, int cam_index, const char *filename, char **names, int classes, int delay, char *prefix, int avg, float hier, int w, int h, int frames, int fullscreen)
{
    fprintf(stderr, "Demo needs OpenCV for webcam images.\n");
}
#endif

