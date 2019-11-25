cmake_minimum_required(VERSION 3.5.1)
project(src C)

set(CMAKE_C_STANDARD 11)

include_directories(.)

#set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -O0")
#set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O0")


add_executable(src
		cf_feat_ext.c
		cf_feat_ext.h
		cf_feat_ext_initialize.c
		cf_feat_ext_initialize.h
		cf_feat_ext_terminate.c
		cf_feat_ext_terminate.h
		cf_feat_ext_types.h
		init.c
		init.h
		myRansac.c
		myRansac.h
		rtGetInf.c
		rtGetInf.h
		rtGetNaN.c
		rtGetNaN.h
		rt_nonfinite.c
		rt_nonfinite.h
		rtwtypes.h
		main.c
                )



target_link_libraries(src m)
