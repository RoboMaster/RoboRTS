#ifndef DEPTHWISE_CONVOLUTIONAL_LAYER_H
#define DEPTHWISE_CONVOLUTIONAL_LAYER_H

#include "cuda.h"
#include "image.h"
#include "activations.h"
#include "layer.h"
#include "network.h"

typedef layer depthwise_convolutional_layer;

#ifdef GPU
void forward_depthwise_convolutional_layer_gpu(depthwise_convolutional_layer layer, network net);
void backward_depthwise_convolutional_layer_gpu(depthwise_convolutional_layer layer, network net);
void update_depthwise_convolutional_layer_gpu(depthwise_convolutional_layer layer, update_args a);

void push_depthwise_convolutional_layer(depthwise_convolutional_layer layer);
void pull_depthwise_convolutional_layer(depthwise_convolutional_layer layer);

void add_bias_gpu(float *output, float *biases, int batch, int n, int size);
void backward_bias_gpu(float *bias_updates, float *delta, int batch, int n, int size);
void adam_update_gpu(float *w, float *d, float *m, float *v, float B1, float B2, float eps, float decay, float rate, int n, int batch, int t);
#ifdef CUDNN
void cudnn_depthwise_convolutional_setup(layer *l);
#endif
#endif

depthwise_convolutional_layer make_depthwise_convolutional_layer(int batch, int h, int w, int c, int size, int stride, int padding, ACTIVATION activation, int batch_normalize);
void resize_depthwise_convolutional_layer(depthwise_convolutional_layer *layer, int w, int h);
void forward_depthwise_convolutional_layer(const depthwise_convolutional_layer layer, network net);
void update_depthwise_convolutional_layer(depthwise_convolutional_layer layer, update_args a);


void denormalize_depthwise_convolutional_layer(depthwise_convolutional_layer l);
void backward_depthwise_convolutional_layer(depthwise_convolutional_layer layer, network net);

void add_bias(float *output, float *biases, int batch, int n, int size);
void backward_bias(float *bias_updates, float *delta, int batch, int n, int size);



int depthwise_convolutional_out_height(depthwise_convolutional_layer layer);
int depthwise_convolutional_out_width(depthwise_convolutional_layer layer);

#endif

