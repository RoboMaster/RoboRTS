cmake_minimum_required(VERSION 3.1)
project(alg_factory)

#common::io
add_library(alg_factory INTERFACE)

target_include_directories(alg_factory INTERFACE include/)

#add_library(roborts_common::alg_factory ALIAS alg_factory)