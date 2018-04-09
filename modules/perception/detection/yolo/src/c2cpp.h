#ifndef RRTS_C2CPP_H
#define RRTS_C2CPP_H

#ifdef __cplusplus
extern "C" {
#endif
void InitYOLO(const char* datacfg, const char* cfg, const char* weights, float thresh, bool enable_debug);
void RunYOLO(bool enable, int camera_id, int **x_offset, int *object_num);
#ifdef __cplusplus
};
#endif

#endif //RRTS_C2CPP_H