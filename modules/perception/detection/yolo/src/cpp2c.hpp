#ifndef RRTS_CPP2C_H
#define RRTS_CPP2C_H
#include "opencv2/highgui/highgui_c.h"

#ifdef __cplusplus
extern "C" {
#endif
IplImage* GetNextImage(int camera_id);
void Notice(char* message);
void ShowImage(IplImage *img);
#ifdef __cplusplus
};
#endif

#endif //RRTS_CPP2C_H