/**
  ******************************************************************************
  * @file    network_handc5.c
  * @author  AST Embedded Analytics Research Platform
  * @date    Wed Nov  4 17:40:34 2020
  * @brief   AI Tool Automatic Code Generator for Embedded NN computing
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2018 STMicroelectronics.
  * All rights reserved.
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  */


#include "network_handc5.h"

#include "ai_platform_interface.h"
#include "ai_math_helpers.h"

#include "core_common.h"
#include "layers.h"

#undef AI_TOOLS_VERSION_MAJOR
#undef AI_TOOLS_VERSION_MINOR
#undef AI_TOOLS_VERSION_MICRO
#define AI_TOOLS_VERSION_MAJOR 5
#define AI_TOOLS_VERSION_MINOR 1
#define AI_TOOLS_VERSION_MICRO 2


#undef AI_TOOLS_API_VERSION_MAJOR
#undef AI_TOOLS_API_VERSION_MINOR
#undef AI_TOOLS_API_VERSION_MICRO
#define AI_TOOLS_API_VERSION_MAJOR 1
#define AI_TOOLS_API_VERSION_MINOR 3
#define AI_TOOLS_API_VERSION_MICRO 0

#undef AI_NET_OBJ_INSTANCE
#define AI_NET_OBJ_INSTANCE g_network_handc5
 
#undef AI_NETWORK_HANDC5_MODEL_SIGNATURE
#define AI_NETWORK_HANDC5_MODEL_SIGNATURE     "f4db67affba2dfdcb7edd2ce4a9b514b"

#ifndef AI_TOOLS_REVISION_ID
#define AI_TOOLS_REVISION_ID     "(rev-5.1.2)"
#endif

#undef AI_TOOLS_DATE_TIME
#define AI_TOOLS_DATE_TIME   "Wed Nov  4 17:40:34 2020"

#undef AI_TOOLS_COMPILE_TIME
#define AI_TOOLS_COMPILE_TIME    __DATE__ " " __TIME__

#undef AI_NETWORK_HANDC5_N_BATCHES
#define AI_NETWORK_HANDC5_N_BATCHES         (1)

/**  Forward network declaration section  *************************************/
AI_STATIC ai_network AI_NET_OBJ_INSTANCE;


/**  Forward network array declarations  **************************************/
AI_STATIC ai_array conv2d_18_scratch0_array;   /* Array #0 */
AI_STATIC ai_array conv2d_20_bias_array;   /* Array #1 */
AI_STATIC ai_array conv2d_20_weights_array;   /* Array #2 */
AI_STATIC ai_array conv2d_19_bias_array;   /* Array #3 */
AI_STATIC ai_array conv2d_19_weights_array;   /* Array #4 */
AI_STATIC ai_array conv2d_18_bias_array;   /* Array #5 */
AI_STATIC ai_array conv2d_18_weights_array;   /* Array #6 */
AI_STATIC ai_array conv2d_17_bias_array;   /* Array #7 */
AI_STATIC ai_array conv2d_17_weights_array;   /* Array #8 */
AI_STATIC ai_array dense_12_bias_array;   /* Array #9 */
AI_STATIC ai_array dense_12_weights_array;   /* Array #10 */
AI_STATIC ai_array dense_11_bias_array;   /* Array #11 */
AI_STATIC ai_array dense_11_weights_array;   /* Array #12 */
AI_STATIC ai_array depthwise_conv2d_8_bias_array;   /* Array #13 */
AI_STATIC ai_array depthwise_conv2d_8_weights_array;   /* Array #14 */
AI_STATIC ai_array conv2d_16_bias_array;   /* Array #15 */
AI_STATIC ai_array conv2d_16_weights_array;   /* Array #16 */
AI_STATIC ai_array conv2d_15_bias_array;   /* Array #17 */
AI_STATIC ai_array conv2d_15_weights_array;   /* Array #18 */
AI_STATIC ai_array dense_10_bias_array;   /* Array #19 */
AI_STATIC ai_array dense_10_weights_array;   /* Array #20 */
AI_STATIC ai_array dense_9_bias_array;   /* Array #21 */
AI_STATIC ai_array dense_9_weights_array;   /* Array #22 */
AI_STATIC ai_array depthwise_conv2d_7_bias_array;   /* Array #23 */
AI_STATIC ai_array depthwise_conv2d_7_weights_array;   /* Array #24 */
AI_STATIC ai_array conv2d_14_bias_array;   /* Array #25 */
AI_STATIC ai_array conv2d_14_weights_array;   /* Array #26 */
AI_STATIC ai_array conv2d_13_bias_array;   /* Array #27 */
AI_STATIC ai_array conv2d_13_weights_array;   /* Array #28 */
AI_STATIC ai_array dense_8_bias_array;   /* Array #29 */
AI_STATIC ai_array dense_8_weights_array;   /* Array #30 */
AI_STATIC ai_array dense_7_bias_array;   /* Array #31 */
AI_STATIC ai_array dense_7_weights_array;   /* Array #32 */
AI_STATIC ai_array depthwise_conv2d_6_bias_array;   /* Array #33 */
AI_STATIC ai_array depthwise_conv2d_6_weights_array;   /* Array #34 */
AI_STATIC ai_array conv2d_12_bias_array;   /* Array #35 */
AI_STATIC ai_array conv2d_12_weights_array;   /* Array #36 */
AI_STATIC ai_array conv2d_11_bias_array;   /* Array #37 */
AI_STATIC ai_array conv2d_11_weights_array;   /* Array #38 */
AI_STATIC ai_array dense_6_bias_array;   /* Array #39 */
AI_STATIC ai_array dense_6_weights_array;   /* Array #40 */
AI_STATIC ai_array dense_5_bias_array;   /* Array #41 */
AI_STATIC ai_array dense_5_weights_array;   /* Array #42 */
AI_STATIC ai_array depthwise_conv2d_5_bias_array;   /* Array #43 */
AI_STATIC ai_array depthwise_conv2d_5_weights_array;   /* Array #44 */
AI_STATIC ai_array conv2d_10_bias_array;   /* Array #45 */
AI_STATIC ai_array conv2d_10_weights_array;   /* Array #46 */
AI_STATIC ai_array conv2d_9_bias_array;   /* Array #47 */
AI_STATIC ai_array conv2d_9_weights_array;   /* Array #48 */
AI_STATIC ai_array dense_4_bias_array;   /* Array #49 */
AI_STATIC ai_array dense_4_weights_array;   /* Array #50 */
AI_STATIC ai_array dense_3_bias_array;   /* Array #51 */
AI_STATIC ai_array dense_3_weights_array;   /* Array #52 */
AI_STATIC ai_array depthwise_conv2d_4_bias_array;   /* Array #53 */
AI_STATIC ai_array depthwise_conv2d_4_weights_array;   /* Array #54 */
AI_STATIC ai_array conv2d_8_bias_array;   /* Array #55 */
AI_STATIC ai_array conv2d_8_weights_array;   /* Array #56 */
AI_STATIC ai_array conv2d_7_bias_array;   /* Array #57 */
AI_STATIC ai_array conv2d_7_weights_array;   /* Array #58 */
AI_STATIC ai_array depthwise_conv2d_3_bias_array;   /* Array #59 */
AI_STATIC ai_array depthwise_conv2d_3_weights_array;   /* Array #60 */
AI_STATIC ai_array conv2d_6_bias_array;   /* Array #61 */
AI_STATIC ai_array conv2d_6_weights_array;   /* Array #62 */
AI_STATIC ai_array conv2d_5_bias_array;   /* Array #63 */
AI_STATIC ai_array conv2d_5_weights_array;   /* Array #64 */
AI_STATIC ai_array depthwise_conv2d_2_bias_array;   /* Array #65 */
AI_STATIC ai_array depthwise_conv2d_2_weights_array;   /* Array #66 */
AI_STATIC ai_array conv2d_4_bias_array;   /* Array #67 */
AI_STATIC ai_array conv2d_4_weights_array;   /* Array #68 */
AI_STATIC ai_array conv2d_3_bias_array;   /* Array #69 */
AI_STATIC ai_array conv2d_3_weights_array;   /* Array #70 */
AI_STATIC ai_array dense_2_bias_array;   /* Array #71 */
AI_STATIC ai_array dense_2_weights_array;   /* Array #72 */
AI_STATIC ai_array dense_1_bias_array;   /* Array #73 */
AI_STATIC ai_array dense_1_weights_array;   /* Array #74 */
AI_STATIC ai_array depthwise_conv2d_1_bias_array;   /* Array #75 */
AI_STATIC ai_array depthwise_conv2d_1_weights_array;   /* Array #76 */
AI_STATIC ai_array conv2d_2_bias_array;   /* Array #77 */
AI_STATIC ai_array conv2d_2_weights_array;   /* Array #78 */
AI_STATIC ai_array conv2d_1_bias_array;   /* Array #79 */
AI_STATIC ai_array conv2d_1_weights_array;   /* Array #80 */
AI_STATIC ai_array input_1_output_array;   /* Array #81 */
AI_STATIC ai_array conv2d_1_output_array;   /* Array #82 */
AI_STATIC ai_array conv2d_2_output_array;   /* Array #83 */
AI_STATIC ai_array depthwise_conv2d_1_output_array;   /* Array #84 */
AI_STATIC ai_array global_average_pooling2d_1_output_array;   /* Array #85 */
AI_STATIC ai_array dense_1_output_array;   /* Array #86 */
AI_STATIC ai_array dense_1_nl_output_array;   /* Array #87 */
AI_STATIC ai_array dense_2_output_array;   /* Array #88 */
AI_STATIC ai_array dense_2_nl_output_array;   /* Array #89 */
AI_STATIC ai_array multiply_1_output_array;   /* Array #90 */
AI_STATIC ai_array conv2d_3_output_array;   /* Array #91 */
AI_STATIC ai_array conv2d_4_output_array;   /* Array #92 */
AI_STATIC ai_array depthwise_conv2d_2_output_array;   /* Array #93 */
AI_STATIC ai_array conv2d_5_output_array;   /* Array #94 */
AI_STATIC ai_array conv2d_6_output_array;   /* Array #95 */
AI_STATIC ai_array depthwise_conv2d_3_output_array;   /* Array #96 */
AI_STATIC ai_array conv2d_7_output_array;   /* Array #97 */
AI_STATIC ai_array add_1_output_array;   /* Array #98 */
AI_STATIC ai_array conv2d_8_output_array;   /* Array #99 */
AI_STATIC ai_array depthwise_conv2d_4_output_array;   /* Array #100 */
AI_STATIC ai_array global_average_pooling2d_2_output_array;   /* Array #101 */
AI_STATIC ai_array dense_3_output_array;   /* Array #102 */
AI_STATIC ai_array dense_3_nl_output_array;   /* Array #103 */
AI_STATIC ai_array dense_4_output_array;   /* Array #104 */
AI_STATIC ai_array dense_4_nl_output_array;   /* Array #105 */
AI_STATIC ai_array multiply_2_output_array;   /* Array #106 */
AI_STATIC ai_array conv2d_9_output_array;   /* Array #107 */
AI_STATIC ai_array conv2d_10_output_array;   /* Array #108 */
AI_STATIC ai_array depthwise_conv2d_5_output_array;   /* Array #109 */
AI_STATIC ai_array global_average_pooling2d_3_output_array;   /* Array #110 */
AI_STATIC ai_array dense_5_output_array;   /* Array #111 */
AI_STATIC ai_array dense_5_nl_output_array;   /* Array #112 */
AI_STATIC ai_array dense_6_output_array;   /* Array #113 */
AI_STATIC ai_array dense_6_nl_output_array;   /* Array #114 */
AI_STATIC ai_array multiply_3_output_array;   /* Array #115 */
AI_STATIC ai_array conv2d_11_output_array;   /* Array #116 */
AI_STATIC ai_array add_2_output_array;   /* Array #117 */
AI_STATIC ai_array conv2d_12_output_array;   /* Array #118 */
AI_STATIC ai_array depthwise_conv2d_6_output_array;   /* Array #119 */
AI_STATIC ai_array global_average_pooling2d_4_output_array;   /* Array #120 */
AI_STATIC ai_array dense_7_output_array;   /* Array #121 */
AI_STATIC ai_array dense_7_nl_output_array;   /* Array #122 */
AI_STATIC ai_array dense_8_output_array;   /* Array #123 */
AI_STATIC ai_array dense_8_nl_output_array;   /* Array #124 */
AI_STATIC ai_array multiply_4_output_array;   /* Array #125 */
AI_STATIC ai_array conv2d_13_output_array;   /* Array #126 */
AI_STATIC ai_array add_3_output_array;   /* Array #127 */
AI_STATIC ai_array conv2d_14_output_array;   /* Array #128 */
AI_STATIC ai_array depthwise_conv2d_7_output_array;   /* Array #129 */
AI_STATIC ai_array global_average_pooling2d_5_output_array;   /* Array #130 */
AI_STATIC ai_array dense_9_output_array;   /* Array #131 */
AI_STATIC ai_array dense_9_nl_output_array;   /* Array #132 */
AI_STATIC ai_array dense_10_output_array;   /* Array #133 */
AI_STATIC ai_array dense_10_nl_output_array;   /* Array #134 */
AI_STATIC ai_array multiply_5_output_array;   /* Array #135 */
AI_STATIC ai_array conv2d_15_output_array;   /* Array #136 */
AI_STATIC ai_array conv2d_16_output_array;   /* Array #137 */
AI_STATIC ai_array depthwise_conv2d_8_output_array;   /* Array #138 */
AI_STATIC ai_array global_average_pooling2d_6_output_array;   /* Array #139 */
AI_STATIC ai_array dense_11_output_array;   /* Array #140 */
AI_STATIC ai_array dense_11_nl_output_array;   /* Array #141 */
AI_STATIC ai_array dense_12_output_array;   /* Array #142 */
AI_STATIC ai_array dense_12_nl_output_array;   /* Array #143 */
AI_STATIC ai_array multiply_6_output_array;   /* Array #144 */
AI_STATIC ai_array conv2d_17_output_array;   /* Array #145 */
AI_STATIC ai_array add_4_output_array;   /* Array #146 */
AI_STATIC ai_array conv2d_18_output_array;   /* Array #147 */
AI_STATIC ai_array conv2d_19_output_array;   /* Array #148 */
AI_STATIC ai_array conv2d_20_output_array;   /* Array #149 */


/**  Forward network tensor declarations  *************************************/
AI_STATIC ai_tensor conv2d_18_scratch0;   /* Tensor #0 */
AI_STATIC ai_tensor conv2d_20_bias;   /* Tensor #1 */
AI_STATIC ai_tensor conv2d_20_weights;   /* Tensor #2 */
AI_STATIC ai_tensor conv2d_19_bias;   /* Tensor #3 */
AI_STATIC ai_tensor conv2d_19_weights;   /* Tensor #4 */
AI_STATIC ai_tensor conv2d_18_bias;   /* Tensor #5 */
AI_STATIC ai_tensor conv2d_18_weights;   /* Tensor #6 */
AI_STATIC ai_tensor conv2d_17_bias;   /* Tensor #7 */
AI_STATIC ai_tensor conv2d_17_weights;   /* Tensor #8 */
AI_STATIC ai_tensor dense_12_bias;   /* Tensor #9 */
AI_STATIC ai_tensor dense_12_weights;   /* Tensor #10 */
AI_STATIC ai_tensor dense_11_bias;   /* Tensor #11 */
AI_STATIC ai_tensor dense_11_weights;   /* Tensor #12 */
AI_STATIC ai_tensor depthwise_conv2d_8_bias;   /* Tensor #13 */
AI_STATIC ai_tensor depthwise_conv2d_8_weights;   /* Tensor #14 */
AI_STATIC ai_tensor conv2d_16_bias;   /* Tensor #15 */
AI_STATIC ai_tensor conv2d_16_weights;   /* Tensor #16 */
AI_STATIC ai_tensor conv2d_15_bias;   /* Tensor #17 */
AI_STATIC ai_tensor conv2d_15_weights;   /* Tensor #18 */
AI_STATIC ai_tensor dense_10_bias;   /* Tensor #19 */
AI_STATIC ai_tensor dense_10_weights;   /* Tensor #20 */
AI_STATIC ai_tensor dense_9_bias;   /* Tensor #21 */
AI_STATIC ai_tensor dense_9_weights;   /* Tensor #22 */
AI_STATIC ai_tensor depthwise_conv2d_7_bias;   /* Tensor #23 */
AI_STATIC ai_tensor depthwise_conv2d_7_weights;   /* Tensor #24 */
AI_STATIC ai_tensor conv2d_14_bias;   /* Tensor #25 */
AI_STATIC ai_tensor conv2d_14_weights;   /* Tensor #26 */
AI_STATIC ai_tensor conv2d_13_bias;   /* Tensor #27 */
AI_STATIC ai_tensor conv2d_13_weights;   /* Tensor #28 */
AI_STATIC ai_tensor dense_8_bias;   /* Tensor #29 */
AI_STATIC ai_tensor dense_8_weights;   /* Tensor #30 */
AI_STATIC ai_tensor dense_7_bias;   /* Tensor #31 */
AI_STATIC ai_tensor dense_7_weights;   /* Tensor #32 */
AI_STATIC ai_tensor depthwise_conv2d_6_bias;   /* Tensor #33 */
AI_STATIC ai_tensor depthwise_conv2d_6_weights;   /* Tensor #34 */
AI_STATIC ai_tensor conv2d_12_bias;   /* Tensor #35 */
AI_STATIC ai_tensor conv2d_12_weights;   /* Tensor #36 */
AI_STATIC ai_tensor conv2d_11_bias;   /* Tensor #37 */
AI_STATIC ai_tensor conv2d_11_weights;   /* Tensor #38 */
AI_STATIC ai_tensor dense_6_bias;   /* Tensor #39 */
AI_STATIC ai_tensor dense_6_weights;   /* Tensor #40 */
AI_STATIC ai_tensor dense_5_bias;   /* Tensor #41 */
AI_STATIC ai_tensor dense_5_weights;   /* Tensor #42 */
AI_STATIC ai_tensor depthwise_conv2d_5_bias;   /* Tensor #43 */
AI_STATIC ai_tensor depthwise_conv2d_5_weights;   /* Tensor #44 */
AI_STATIC ai_tensor conv2d_10_bias;   /* Tensor #45 */
AI_STATIC ai_tensor conv2d_10_weights;   /* Tensor #46 */
AI_STATIC ai_tensor conv2d_9_bias;   /* Tensor #47 */
AI_STATIC ai_tensor conv2d_9_weights;   /* Tensor #48 */
AI_STATIC ai_tensor dense_4_bias;   /* Tensor #49 */
AI_STATIC ai_tensor dense_4_weights;   /* Tensor #50 */
AI_STATIC ai_tensor dense_3_bias;   /* Tensor #51 */
AI_STATIC ai_tensor dense_3_weights;   /* Tensor #52 */
AI_STATIC ai_tensor depthwise_conv2d_4_bias;   /* Tensor #53 */
AI_STATIC ai_tensor depthwise_conv2d_4_weights;   /* Tensor #54 */
AI_STATIC ai_tensor conv2d_8_bias;   /* Tensor #55 */
AI_STATIC ai_tensor conv2d_8_weights;   /* Tensor #56 */
AI_STATIC ai_tensor conv2d_7_bias;   /* Tensor #57 */
AI_STATIC ai_tensor conv2d_7_weights;   /* Tensor #58 */
AI_STATIC ai_tensor depthwise_conv2d_3_bias;   /* Tensor #59 */
AI_STATIC ai_tensor depthwise_conv2d_3_weights;   /* Tensor #60 */
AI_STATIC ai_tensor conv2d_6_bias;   /* Tensor #61 */
AI_STATIC ai_tensor conv2d_6_weights;   /* Tensor #62 */
AI_STATIC ai_tensor conv2d_5_bias;   /* Tensor #63 */
AI_STATIC ai_tensor conv2d_5_weights;   /* Tensor #64 */
AI_STATIC ai_tensor depthwise_conv2d_2_bias;   /* Tensor #65 */
AI_STATIC ai_tensor depthwise_conv2d_2_weights;   /* Tensor #66 */
AI_STATIC ai_tensor conv2d_4_bias;   /* Tensor #67 */
AI_STATIC ai_tensor conv2d_4_weights;   /* Tensor #68 */
AI_STATIC ai_tensor conv2d_3_bias;   /* Tensor #69 */
AI_STATIC ai_tensor conv2d_3_weights;   /* Tensor #70 */
AI_STATIC ai_tensor dense_2_bias;   /* Tensor #71 */
AI_STATIC ai_tensor dense_2_weights;   /* Tensor #72 */
AI_STATIC ai_tensor dense_1_bias;   /* Tensor #73 */
AI_STATIC ai_tensor dense_1_weights;   /* Tensor #74 */
AI_STATIC ai_tensor depthwise_conv2d_1_bias;   /* Tensor #75 */
AI_STATIC ai_tensor depthwise_conv2d_1_weights;   /* Tensor #76 */
AI_STATIC ai_tensor conv2d_2_bias;   /* Tensor #77 */
AI_STATIC ai_tensor conv2d_2_weights;   /* Tensor #78 */
AI_STATIC ai_tensor conv2d_1_bias;   /* Tensor #79 */
AI_STATIC ai_tensor conv2d_1_weights;   /* Tensor #80 */
AI_STATIC ai_tensor input_1_output;   /* Tensor #81 */
AI_STATIC ai_tensor conv2d_1_output;   /* Tensor #82 */
AI_STATIC ai_tensor conv2d_2_output;   /* Tensor #83 */
AI_STATIC ai_tensor depthwise_conv2d_1_output;   /* Tensor #84 */
AI_STATIC ai_tensor global_average_pooling2d_1_output;   /* Tensor #85 */
AI_STATIC ai_tensor dense_1_output;   /* Tensor #86 */
AI_STATIC ai_tensor dense_1_nl_output;   /* Tensor #87 */
AI_STATIC ai_tensor dense_2_output;   /* Tensor #88 */
AI_STATIC ai_tensor dense_2_nl_output;   /* Tensor #89 */
AI_STATIC ai_tensor multiply_1_output;   /* Tensor #90 */
AI_STATIC ai_tensor conv2d_3_output;   /* Tensor #91 */
AI_STATIC ai_tensor conv2d_4_output;   /* Tensor #92 */
AI_STATIC ai_tensor depthwise_conv2d_2_output;   /* Tensor #93 */
AI_STATIC ai_tensor conv2d_5_output;   /* Tensor #94 */
AI_STATIC ai_tensor conv2d_6_output;   /* Tensor #95 */
AI_STATIC ai_tensor depthwise_conv2d_3_output;   /* Tensor #96 */
AI_STATIC ai_tensor conv2d_7_output;   /* Tensor #97 */
AI_STATIC ai_tensor add_1_output;   /* Tensor #98 */
AI_STATIC ai_tensor conv2d_8_output;   /* Tensor #99 */
AI_STATIC ai_tensor depthwise_conv2d_4_output;   /* Tensor #100 */
AI_STATIC ai_tensor global_average_pooling2d_2_output;   /* Tensor #101 */
AI_STATIC ai_tensor dense_3_output;   /* Tensor #102 */
AI_STATIC ai_tensor dense_3_nl_output;   /* Tensor #103 */
AI_STATIC ai_tensor dense_4_output;   /* Tensor #104 */
AI_STATIC ai_tensor dense_4_nl_output;   /* Tensor #105 */
AI_STATIC ai_tensor multiply_2_output;   /* Tensor #106 */
AI_STATIC ai_tensor conv2d_9_output;   /* Tensor #107 */
AI_STATIC ai_tensor conv2d_10_output;   /* Tensor #108 */
AI_STATIC ai_tensor depthwise_conv2d_5_output;   /* Tensor #109 */
AI_STATIC ai_tensor global_average_pooling2d_3_output;   /* Tensor #110 */
AI_STATIC ai_tensor dense_5_output;   /* Tensor #111 */
AI_STATIC ai_tensor dense_5_nl_output;   /* Tensor #112 */
AI_STATIC ai_tensor dense_6_output;   /* Tensor #113 */
AI_STATIC ai_tensor dense_6_nl_output;   /* Tensor #114 */
AI_STATIC ai_tensor multiply_3_output;   /* Tensor #115 */
AI_STATIC ai_tensor conv2d_11_output;   /* Tensor #116 */
AI_STATIC ai_tensor add_2_output;   /* Tensor #117 */
AI_STATIC ai_tensor conv2d_12_output;   /* Tensor #118 */
AI_STATIC ai_tensor depthwise_conv2d_6_output;   /* Tensor #119 */
AI_STATIC ai_tensor global_average_pooling2d_4_output;   /* Tensor #120 */
AI_STATIC ai_tensor dense_7_output;   /* Tensor #121 */
AI_STATIC ai_tensor dense_7_nl_output;   /* Tensor #122 */
AI_STATIC ai_tensor dense_8_output;   /* Tensor #123 */
AI_STATIC ai_tensor dense_8_nl_output;   /* Tensor #124 */
AI_STATIC ai_tensor multiply_4_output;   /* Tensor #125 */
AI_STATIC ai_tensor conv2d_13_output;   /* Tensor #126 */
AI_STATIC ai_tensor add_3_output;   /* Tensor #127 */
AI_STATIC ai_tensor conv2d_14_output;   /* Tensor #128 */
AI_STATIC ai_tensor depthwise_conv2d_7_output;   /* Tensor #129 */
AI_STATIC ai_tensor global_average_pooling2d_5_output;   /* Tensor #130 */
AI_STATIC ai_tensor dense_9_output;   /* Tensor #131 */
AI_STATIC ai_tensor dense_9_nl_output;   /* Tensor #132 */
AI_STATIC ai_tensor dense_10_output;   /* Tensor #133 */
AI_STATIC ai_tensor dense_10_nl_output;   /* Tensor #134 */
AI_STATIC ai_tensor multiply_5_output;   /* Tensor #135 */
AI_STATIC ai_tensor conv2d_15_output;   /* Tensor #136 */
AI_STATIC ai_tensor conv2d_16_output;   /* Tensor #137 */
AI_STATIC ai_tensor depthwise_conv2d_8_output;   /* Tensor #138 */
AI_STATIC ai_tensor global_average_pooling2d_6_output;   /* Tensor #139 */
AI_STATIC ai_tensor dense_11_output;   /* Tensor #140 */
AI_STATIC ai_tensor dense_11_nl_output;   /* Tensor #141 */
AI_STATIC ai_tensor dense_12_output;   /* Tensor #142 */
AI_STATIC ai_tensor dense_12_nl_output;   /* Tensor #143 */
AI_STATIC ai_tensor multiply_6_output;   /* Tensor #144 */
AI_STATIC ai_tensor conv2d_17_output;   /* Tensor #145 */
AI_STATIC ai_tensor add_4_output;   /* Tensor #146 */
AI_STATIC ai_tensor conv2d_18_output;   /* Tensor #147 */
AI_STATIC ai_tensor conv2d_19_output;   /* Tensor #148 */
AI_STATIC ai_tensor conv2d_20_output;   /* Tensor #149 */


/**  Forward network tensor chain declarations  *******************************/
AI_STATIC_CONST ai_tensor_chain conv2d_1_chain;   /* Chain #0 */
AI_STATIC_CONST ai_tensor_chain conv2d_2_chain;   /* Chain #1 */
AI_STATIC_CONST ai_tensor_chain depthwise_conv2d_1_chain;   /* Chain #2 */
AI_STATIC_CONST ai_tensor_chain global_average_pooling2d_1_chain;   /* Chain #3 */
AI_STATIC_CONST ai_tensor_chain dense_1_chain;   /* Chain #4 */
AI_STATIC_CONST ai_tensor_chain dense_1_nl_chain;   /* Chain #5 */
AI_STATIC_CONST ai_tensor_chain dense_2_chain;   /* Chain #6 */
AI_STATIC_CONST ai_tensor_chain dense_2_nl_chain;   /* Chain #7 */
AI_STATIC_CONST ai_tensor_chain multiply_1_chain;   /* Chain #8 */
AI_STATIC_CONST ai_tensor_chain conv2d_3_chain;   /* Chain #9 */
AI_STATIC_CONST ai_tensor_chain conv2d_4_chain;   /* Chain #10 */
AI_STATIC_CONST ai_tensor_chain depthwise_conv2d_2_chain;   /* Chain #11 */
AI_STATIC_CONST ai_tensor_chain conv2d_5_chain;   /* Chain #12 */
AI_STATIC_CONST ai_tensor_chain conv2d_6_chain;   /* Chain #13 */
AI_STATIC_CONST ai_tensor_chain depthwise_conv2d_3_chain;   /* Chain #14 */
AI_STATIC_CONST ai_tensor_chain conv2d_7_chain;   /* Chain #15 */
AI_STATIC_CONST ai_tensor_chain add_1_chain;   /* Chain #16 */
AI_STATIC_CONST ai_tensor_chain conv2d_8_chain;   /* Chain #17 */
AI_STATIC_CONST ai_tensor_chain depthwise_conv2d_4_chain;   /* Chain #18 */
AI_STATIC_CONST ai_tensor_chain global_average_pooling2d_2_chain;   /* Chain #19 */
AI_STATIC_CONST ai_tensor_chain dense_3_chain;   /* Chain #20 */
AI_STATIC_CONST ai_tensor_chain dense_3_nl_chain;   /* Chain #21 */
AI_STATIC_CONST ai_tensor_chain dense_4_chain;   /* Chain #22 */
AI_STATIC_CONST ai_tensor_chain dense_4_nl_chain;   /* Chain #23 */
AI_STATIC_CONST ai_tensor_chain multiply_2_chain;   /* Chain #24 */
AI_STATIC_CONST ai_tensor_chain conv2d_9_chain;   /* Chain #25 */
AI_STATIC_CONST ai_tensor_chain conv2d_10_chain;   /* Chain #26 */
AI_STATIC_CONST ai_tensor_chain depthwise_conv2d_5_chain;   /* Chain #27 */
AI_STATIC_CONST ai_tensor_chain global_average_pooling2d_3_chain;   /* Chain #28 */
AI_STATIC_CONST ai_tensor_chain dense_5_chain;   /* Chain #29 */
AI_STATIC_CONST ai_tensor_chain dense_5_nl_chain;   /* Chain #30 */
AI_STATIC_CONST ai_tensor_chain dense_6_chain;   /* Chain #31 */
AI_STATIC_CONST ai_tensor_chain dense_6_nl_chain;   /* Chain #32 */
AI_STATIC_CONST ai_tensor_chain multiply_3_chain;   /* Chain #33 */
AI_STATIC_CONST ai_tensor_chain conv2d_11_chain;   /* Chain #34 */
AI_STATIC_CONST ai_tensor_chain add_2_chain;   /* Chain #35 */
AI_STATIC_CONST ai_tensor_chain conv2d_12_chain;   /* Chain #36 */
AI_STATIC_CONST ai_tensor_chain depthwise_conv2d_6_chain;   /* Chain #37 */
AI_STATIC_CONST ai_tensor_chain global_average_pooling2d_4_chain;   /* Chain #38 */
AI_STATIC_CONST ai_tensor_chain dense_7_chain;   /* Chain #39 */
AI_STATIC_CONST ai_tensor_chain dense_7_nl_chain;   /* Chain #40 */
AI_STATIC_CONST ai_tensor_chain dense_8_chain;   /* Chain #41 */
AI_STATIC_CONST ai_tensor_chain dense_8_nl_chain;   /* Chain #42 */
AI_STATIC_CONST ai_tensor_chain multiply_4_chain;   /* Chain #43 */
AI_STATIC_CONST ai_tensor_chain conv2d_13_chain;   /* Chain #44 */
AI_STATIC_CONST ai_tensor_chain add_3_chain;   /* Chain #45 */
AI_STATIC_CONST ai_tensor_chain conv2d_14_chain;   /* Chain #46 */
AI_STATIC_CONST ai_tensor_chain depthwise_conv2d_7_chain;   /* Chain #47 */
AI_STATIC_CONST ai_tensor_chain global_average_pooling2d_5_chain;   /* Chain #48 */
AI_STATIC_CONST ai_tensor_chain dense_9_chain;   /* Chain #49 */
AI_STATIC_CONST ai_tensor_chain dense_9_nl_chain;   /* Chain #50 */
AI_STATIC_CONST ai_tensor_chain dense_10_chain;   /* Chain #51 */
AI_STATIC_CONST ai_tensor_chain dense_10_nl_chain;   /* Chain #52 */
AI_STATIC_CONST ai_tensor_chain multiply_5_chain;   /* Chain #53 */
AI_STATIC_CONST ai_tensor_chain conv2d_15_chain;   /* Chain #54 */
AI_STATIC_CONST ai_tensor_chain conv2d_16_chain;   /* Chain #55 */
AI_STATIC_CONST ai_tensor_chain depthwise_conv2d_8_chain;   /* Chain #56 */
AI_STATIC_CONST ai_tensor_chain global_average_pooling2d_6_chain;   /* Chain #57 */
AI_STATIC_CONST ai_tensor_chain dense_11_chain;   /* Chain #58 */
AI_STATIC_CONST ai_tensor_chain dense_11_nl_chain;   /* Chain #59 */
AI_STATIC_CONST ai_tensor_chain dense_12_chain;   /* Chain #60 */
AI_STATIC_CONST ai_tensor_chain dense_12_nl_chain;   /* Chain #61 */
AI_STATIC_CONST ai_tensor_chain multiply_6_chain;   /* Chain #62 */
AI_STATIC_CONST ai_tensor_chain conv2d_17_chain;   /* Chain #63 */
AI_STATIC_CONST ai_tensor_chain add_4_chain;   /* Chain #64 */
AI_STATIC_CONST ai_tensor_chain conv2d_18_chain;   /* Chain #65 */
AI_STATIC_CONST ai_tensor_chain conv2d_19_chain;   /* Chain #66 */
AI_STATIC_CONST ai_tensor_chain conv2d_20_chain;   /* Chain #67 */


/**  Forward network layer declarations  **************************************/
AI_STATIC ai_layer_conv2d conv2d_1_layer; /* Layer #0 */
AI_STATIC ai_layer_conv2d conv2d_2_layer; /* Layer #1 */
AI_STATIC ai_layer_conv2d depthwise_conv2d_1_layer; /* Layer #2 */
AI_STATIC ai_layer_pool global_average_pooling2d_1_layer; /* Layer #3 */
AI_STATIC ai_layer_dense dense_1_layer; /* Layer #4 */
AI_STATIC ai_layer_nl dense_1_nl_layer; /* Layer #5 */
AI_STATIC ai_layer_dense dense_2_layer; /* Layer #6 */
AI_STATIC ai_layer_nl dense_2_nl_layer; /* Layer #7 */
AI_STATIC ai_layer_eltwise multiply_1_layer; /* Layer #8 */
AI_STATIC ai_layer_conv2d conv2d_3_layer; /* Layer #9 */
AI_STATIC ai_layer_conv2d conv2d_4_layer; /* Layer #10 */
AI_STATIC ai_layer_conv2d depthwise_conv2d_2_layer; /* Layer #11 */
AI_STATIC ai_layer_conv2d conv2d_5_layer; /* Layer #12 */
AI_STATIC ai_layer_conv2d conv2d_6_layer; /* Layer #13 */
AI_STATIC ai_layer_conv2d depthwise_conv2d_3_layer; /* Layer #14 */
AI_STATIC ai_layer_conv2d conv2d_7_layer; /* Layer #15 */
AI_STATIC ai_layer_eltwise add_1_layer; /* Layer #16 */
AI_STATIC ai_layer_conv2d conv2d_8_layer; /* Layer #17 */
AI_STATIC ai_layer_conv2d depthwise_conv2d_4_layer; /* Layer #18 */
AI_STATIC ai_layer_pool global_average_pooling2d_2_layer; /* Layer #19 */
AI_STATIC ai_layer_dense dense_3_layer; /* Layer #20 */
AI_STATIC ai_layer_nl dense_3_nl_layer; /* Layer #21 */
AI_STATIC ai_layer_dense dense_4_layer; /* Layer #22 */
AI_STATIC ai_layer_nl dense_4_nl_layer; /* Layer #23 */
AI_STATIC ai_layer_eltwise multiply_2_layer; /* Layer #24 */
AI_STATIC ai_layer_conv2d conv2d_9_layer; /* Layer #25 */
AI_STATIC ai_layer_conv2d conv2d_10_layer; /* Layer #26 */
AI_STATIC ai_layer_conv2d depthwise_conv2d_5_layer; /* Layer #27 */
AI_STATIC ai_layer_pool global_average_pooling2d_3_layer; /* Layer #28 */
AI_STATIC ai_layer_dense dense_5_layer; /* Layer #29 */
AI_STATIC ai_layer_nl dense_5_nl_layer; /* Layer #30 */
AI_STATIC ai_layer_dense dense_6_layer; /* Layer #31 */
AI_STATIC ai_layer_nl dense_6_nl_layer; /* Layer #32 */
AI_STATIC ai_layer_eltwise multiply_3_layer; /* Layer #33 */
AI_STATIC ai_layer_conv2d conv2d_11_layer; /* Layer #34 */
AI_STATIC ai_layer_eltwise add_2_layer; /* Layer #35 */
AI_STATIC ai_layer_conv2d conv2d_12_layer; /* Layer #36 */
AI_STATIC ai_layer_conv2d depthwise_conv2d_6_layer; /* Layer #37 */
AI_STATIC ai_layer_pool global_average_pooling2d_4_layer; /* Layer #38 */
AI_STATIC ai_layer_dense dense_7_layer; /* Layer #39 */
AI_STATIC ai_layer_nl dense_7_nl_layer; /* Layer #40 */
AI_STATIC ai_layer_dense dense_8_layer; /* Layer #41 */
AI_STATIC ai_layer_nl dense_8_nl_layer; /* Layer #42 */
AI_STATIC ai_layer_eltwise multiply_4_layer; /* Layer #43 */
AI_STATIC ai_layer_conv2d conv2d_13_layer; /* Layer #44 */
AI_STATIC ai_layer_eltwise add_3_layer; /* Layer #45 */
AI_STATIC ai_layer_conv2d conv2d_14_layer; /* Layer #46 */
AI_STATIC ai_layer_conv2d depthwise_conv2d_7_layer; /* Layer #47 */
AI_STATIC ai_layer_pool global_average_pooling2d_5_layer; /* Layer #48 */
AI_STATIC ai_layer_dense dense_9_layer; /* Layer #49 */
AI_STATIC ai_layer_nl dense_9_nl_layer; /* Layer #50 */
AI_STATIC ai_layer_dense dense_10_layer; /* Layer #51 */
AI_STATIC ai_layer_nl dense_10_nl_layer; /* Layer #52 */
AI_STATIC ai_layer_eltwise multiply_5_layer; /* Layer #53 */
AI_STATIC ai_layer_conv2d conv2d_15_layer; /* Layer #54 */
AI_STATIC ai_layer_conv2d conv2d_16_layer; /* Layer #55 */
AI_STATIC ai_layer_conv2d depthwise_conv2d_8_layer; /* Layer #56 */
AI_STATIC ai_layer_pool global_average_pooling2d_6_layer; /* Layer #57 */
AI_STATIC ai_layer_dense dense_11_layer; /* Layer #58 */
AI_STATIC ai_layer_nl dense_11_nl_layer; /* Layer #59 */
AI_STATIC ai_layer_dense dense_12_layer; /* Layer #60 */
AI_STATIC ai_layer_nl dense_12_nl_layer; /* Layer #61 */
AI_STATIC ai_layer_eltwise multiply_6_layer; /* Layer #62 */
AI_STATIC ai_layer_conv2d conv2d_17_layer; /* Layer #63 */
AI_STATIC ai_layer_eltwise add_4_layer; /* Layer #64 */
AI_STATIC ai_layer_conv2d_nl_pool conv2d_18_layer; /* Layer #65 */
AI_STATIC ai_layer_conv2d conv2d_19_layer; /* Layer #66 */
AI_STATIC ai_layer_conv2d conv2d_20_layer; /* Layer #67 */


/**  Array declarations section  **********************************************/
/* Array#0 */
AI_ARRAY_OBJ_DECLARE(
  conv2d_18_scratch0_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 240, AI_STATIC)

/* Array#1 */
AI_ARRAY_OBJ_DECLARE(
  conv2d_20_bias_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 4, AI_STATIC)

/* Array#2 */
AI_ARRAY_OBJ_DECLARE(
  conv2d_20_weights_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 480, AI_STATIC)

/* Array#3 */
AI_ARRAY_OBJ_DECLARE(
  conv2d_19_bias_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 120, AI_STATIC)

/* Array#4 */
AI_ARRAY_OBJ_DECLARE(
  conv2d_19_weights_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 7200, AI_STATIC)

/* Array#5 */
AI_ARRAY_OBJ_DECLARE(
  conv2d_18_bias_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 60, AI_STATIC)

/* Array#6 */
AI_ARRAY_OBJ_DECLARE(
  conv2d_18_weights_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 1200, AI_STATIC)

/* Array#7 */
AI_ARRAY_OBJ_DECLARE(
  conv2d_17_bias_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 20, AI_STATIC)

/* Array#8 */
AI_ARRAY_OBJ_DECLARE(
  conv2d_17_weights_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 1200, AI_STATIC)

/* Array#9 */
AI_ARRAY_OBJ_DECLARE(
  dense_12_bias_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 60, AI_STATIC)

/* Array#10 */
AI_ARRAY_OBJ_DECLARE(
  dense_12_weights_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 3600, AI_STATIC)

/* Array#11 */
AI_ARRAY_OBJ_DECLARE(
  dense_11_bias_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 60, AI_STATIC)

/* Array#12 */
AI_ARRAY_OBJ_DECLARE(
  dense_11_weights_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 3600, AI_STATIC)

/* Array#13 */
AI_ARRAY_OBJ_DECLARE(
  depthwise_conv2d_8_bias_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 60, AI_STATIC)

/* Array#14 */
AI_ARRAY_OBJ_DECLARE(
  depthwise_conv2d_8_weights_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 1500, AI_STATIC)

/* Array#15 */
AI_ARRAY_OBJ_DECLARE(
  conv2d_16_bias_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 60, AI_STATIC)

/* Array#16 */
AI_ARRAY_OBJ_DECLARE(
  conv2d_16_weights_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 1200, AI_STATIC)

/* Array#17 */
AI_ARRAY_OBJ_DECLARE(
  conv2d_15_bias_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 20, AI_STATIC)

/* Array#18 */
AI_ARRAY_OBJ_DECLARE(
  conv2d_15_weights_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 1200, AI_STATIC)

/* Array#19 */
AI_ARRAY_OBJ_DECLARE(
  dense_10_bias_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 60, AI_STATIC)

/* Array#20 */
AI_ARRAY_OBJ_DECLARE(
  dense_10_weights_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 3600, AI_STATIC)

/* Array#21 */
AI_ARRAY_OBJ_DECLARE(
  dense_9_bias_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 60, AI_STATIC)

/* Array#22 */
AI_ARRAY_OBJ_DECLARE(
  dense_9_weights_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 3600, AI_STATIC)

/* Array#23 */
AI_ARRAY_OBJ_DECLARE(
  depthwise_conv2d_7_bias_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 60, AI_STATIC)

/* Array#24 */
AI_ARRAY_OBJ_DECLARE(
  depthwise_conv2d_7_weights_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 1500, AI_STATIC)

/* Array#25 */
AI_ARRAY_OBJ_DECLARE(
  conv2d_14_bias_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 60, AI_STATIC)

/* Array#26 */
AI_ARRAY_OBJ_DECLARE(
  conv2d_14_weights_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 720, AI_STATIC)

/* Array#27 */
AI_ARRAY_OBJ_DECLARE(
  conv2d_13_bias_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 12, AI_STATIC)

/* Array#28 */
AI_ARRAY_OBJ_DECLARE(
  conv2d_13_weights_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 720, AI_STATIC)

/* Array#29 */
AI_ARRAY_OBJ_DECLARE(
  dense_8_bias_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 60, AI_STATIC)

/* Array#30 */
AI_ARRAY_OBJ_DECLARE(
  dense_8_weights_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 3600, AI_STATIC)

/* Array#31 */
AI_ARRAY_OBJ_DECLARE(
  dense_7_bias_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 60, AI_STATIC)

/* Array#32 */
AI_ARRAY_OBJ_DECLARE(
  dense_7_weights_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 3600, AI_STATIC)

/* Array#33 */
AI_ARRAY_OBJ_DECLARE(
  depthwise_conv2d_6_bias_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 60, AI_STATIC)

/* Array#34 */
AI_ARRAY_OBJ_DECLARE(
  depthwise_conv2d_6_weights_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 1500, AI_STATIC)

/* Array#35 */
AI_ARRAY_OBJ_DECLARE(
  conv2d_12_bias_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 60, AI_STATIC)

/* Array#36 */
AI_ARRAY_OBJ_DECLARE(
  conv2d_12_weights_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 720, AI_STATIC)

/* Array#37 */
AI_ARRAY_OBJ_DECLARE(
  conv2d_11_bias_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 12, AI_STATIC)

/* Array#38 */
AI_ARRAY_OBJ_DECLARE(
  conv2d_11_weights_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 720, AI_STATIC)

/* Array#39 */
AI_ARRAY_OBJ_DECLARE(
  dense_6_bias_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 60, AI_STATIC)

/* Array#40 */
AI_ARRAY_OBJ_DECLARE(
  dense_6_weights_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 3600, AI_STATIC)

/* Array#41 */
AI_ARRAY_OBJ_DECLARE(
  dense_5_bias_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 60, AI_STATIC)

/* Array#42 */
AI_ARRAY_OBJ_DECLARE(
  dense_5_weights_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 3600, AI_STATIC)

/* Array#43 */
AI_ARRAY_OBJ_DECLARE(
  depthwise_conv2d_5_bias_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 60, AI_STATIC)

/* Array#44 */
AI_ARRAY_OBJ_DECLARE(
  depthwise_conv2d_5_weights_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 1500, AI_STATIC)

/* Array#45 */
AI_ARRAY_OBJ_DECLARE(
  conv2d_10_bias_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 60, AI_STATIC)

/* Array#46 */
AI_ARRAY_OBJ_DECLARE(
  conv2d_10_weights_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 720, AI_STATIC)

/* Array#47 */
AI_ARRAY_OBJ_DECLARE(
  conv2d_9_bias_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 12, AI_STATIC)

/* Array#48 */
AI_ARRAY_OBJ_DECLARE(
  conv2d_9_weights_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 240, AI_STATIC)

/* Array#49 */
AI_ARRAY_OBJ_DECLARE(
  dense_4_bias_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 20, AI_STATIC)

/* Array#50 */
AI_ARRAY_OBJ_DECLARE(
  dense_4_weights_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 400, AI_STATIC)

/* Array#51 */
AI_ARRAY_OBJ_DECLARE(
  dense_3_bias_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 20, AI_STATIC)

/* Array#52 */
AI_ARRAY_OBJ_DECLARE(
  dense_3_weights_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 400, AI_STATIC)

/* Array#53 */
AI_ARRAY_OBJ_DECLARE(
  depthwise_conv2d_4_bias_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 20, AI_STATIC)

/* Array#54 */
AI_ARRAY_OBJ_DECLARE(
  depthwise_conv2d_4_weights_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 500, AI_STATIC)

/* Array#55 */
AI_ARRAY_OBJ_DECLARE(
  conv2d_8_bias_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 20, AI_STATIC)

/* Array#56 */
AI_ARRAY_OBJ_DECLARE(
  conv2d_8_weights_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 160, AI_STATIC)

/* Array#57 */
AI_ARRAY_OBJ_DECLARE(
  conv2d_7_bias_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 8, AI_STATIC)

/* Array#58 */
AI_ARRAY_OBJ_DECLARE(
  conv2d_7_weights_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 160, AI_STATIC)

/* Array#59 */
AI_ARRAY_OBJ_DECLARE(
  depthwise_conv2d_3_bias_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 20, AI_STATIC)

/* Array#60 */
AI_ARRAY_OBJ_DECLARE(
  depthwise_conv2d_3_weights_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 180, AI_STATIC)

/* Array#61 */
AI_ARRAY_OBJ_DECLARE(
  conv2d_6_bias_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 20, AI_STATIC)

/* Array#62 */
AI_ARRAY_OBJ_DECLARE(
  conv2d_6_weights_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 160, AI_STATIC)

/* Array#63 */
AI_ARRAY_OBJ_DECLARE(
  conv2d_5_bias_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 8, AI_STATIC)

/* Array#64 */
AI_ARRAY_OBJ_DECLARE(
  conv2d_5_weights_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 160, AI_STATIC)

/* Array#65 */
AI_ARRAY_OBJ_DECLARE(
  depthwise_conv2d_2_bias_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 20, AI_STATIC)

/* Array#66 */
AI_ARRAY_OBJ_DECLARE(
  depthwise_conv2d_2_weights_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 180, AI_STATIC)

/* Array#67 */
AI_ARRAY_OBJ_DECLARE(
  conv2d_4_bias_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 20, AI_STATIC)

/* Array#68 */
AI_ARRAY_OBJ_DECLARE(
  conv2d_4_weights_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 120, AI_STATIC)

/* Array#69 */
AI_ARRAY_OBJ_DECLARE(
  conv2d_3_bias_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 6, AI_STATIC)

/* Array#70 */
AI_ARRAY_OBJ_DECLARE(
  conv2d_3_weights_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 36, AI_STATIC)

/* Array#71 */
AI_ARRAY_OBJ_DECLARE(
  dense_2_bias_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 6, AI_STATIC)

/* Array#72 */
AI_ARRAY_OBJ_DECLARE(
  dense_2_weights_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 36, AI_STATIC)

/* Array#73 */
AI_ARRAY_OBJ_DECLARE(
  dense_1_bias_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 6, AI_STATIC)

/* Array#74 */
AI_ARRAY_OBJ_DECLARE(
  dense_1_weights_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 36, AI_STATIC)

/* Array#75 */
AI_ARRAY_OBJ_DECLARE(
  depthwise_conv2d_1_bias_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 6, AI_STATIC)

/* Array#76 */
AI_ARRAY_OBJ_DECLARE(
  depthwise_conv2d_1_weights_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 54, AI_STATIC)

/* Array#77 */
AI_ARRAY_OBJ_DECLARE(
  conv2d_2_bias_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 6, AI_STATIC)

/* Array#78 */
AI_ARRAY_OBJ_DECLARE(
  conv2d_2_weights_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 36, AI_STATIC)

/* Array#79 */
AI_ARRAY_OBJ_DECLARE(
  conv2d_1_bias_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 6, AI_STATIC)

/* Array#80 */
AI_ARRAY_OBJ_DECLARE(
  conv2d_1_weights_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 54, AI_STATIC)

/* Array#81 */
AI_ARRAY_OBJ_DECLARE(
  input_1_output_array, AI_ARRAY_FORMAT_FLOAT|AI_FMT_FLAG_IS_IO,
  NULL, NULL, 3000, AI_STATIC)

/* Array#82 */
AI_ARRAY_OBJ_DECLARE(
  conv2d_1_output_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 4500, AI_STATIC)

/* Array#83 */
AI_ARRAY_OBJ_DECLARE(
  conv2d_2_output_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 4500, AI_STATIC)

/* Array#84 */
AI_ARRAY_OBJ_DECLARE(
  depthwise_conv2d_1_output_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 1170, AI_STATIC)

/* Array#85 */
AI_ARRAY_OBJ_DECLARE(
  global_average_pooling2d_1_output_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 6, AI_STATIC)

/* Array#86 */
AI_ARRAY_OBJ_DECLARE(
  dense_1_output_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 6, AI_STATIC)

/* Array#87 */
AI_ARRAY_OBJ_DECLARE(
  dense_1_nl_output_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 6, AI_STATIC)

/* Array#88 */
AI_ARRAY_OBJ_DECLARE(
  dense_2_output_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 6, AI_STATIC)

/* Array#89 */
AI_ARRAY_OBJ_DECLARE(
  dense_2_nl_output_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 6, AI_STATIC)

/* Array#90 */
AI_ARRAY_OBJ_DECLARE(
  multiply_1_output_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 1170, AI_STATIC)

/* Array#91 */
AI_ARRAY_OBJ_DECLARE(
  conv2d_3_output_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 1170, AI_STATIC)

/* Array#92 */
AI_ARRAY_OBJ_DECLARE(
  conv2d_4_output_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 3900, AI_STATIC)

/* Array#93 */
AI_ARRAY_OBJ_DECLARE(
  depthwise_conv2d_2_output_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 1120, AI_STATIC)

/* Array#94 */
AI_ARRAY_OBJ_DECLARE(
  conv2d_5_output_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 448, AI_STATIC)

/* Array#95 */
AI_ARRAY_OBJ_DECLARE(
  conv2d_6_output_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 1120, AI_STATIC)

/* Array#96 */
AI_ARRAY_OBJ_DECLARE(
  depthwise_conv2d_3_output_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 1120, AI_STATIC)

/* Array#97 */
AI_ARRAY_OBJ_DECLARE(
  conv2d_7_output_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 448, AI_STATIC)

/* Array#98 */
AI_ARRAY_OBJ_DECLARE(
  add_1_output_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 448, AI_STATIC)

/* Array#99 */
AI_ARRAY_OBJ_DECLARE(
  conv2d_8_output_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 1120, AI_STATIC)

/* Array#100 */
AI_ARRAY_OBJ_DECLARE(
  depthwise_conv2d_4_output_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 320, AI_STATIC)

/* Array#101 */
AI_ARRAY_OBJ_DECLARE(
  global_average_pooling2d_2_output_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 20, AI_STATIC)

/* Array#102 */
AI_ARRAY_OBJ_DECLARE(
  dense_3_output_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 20, AI_STATIC)

/* Array#103 */
AI_ARRAY_OBJ_DECLARE(
  dense_3_nl_output_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 20, AI_STATIC)

/* Array#104 */
AI_ARRAY_OBJ_DECLARE(
  dense_4_output_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 20, AI_STATIC)

/* Array#105 */
AI_ARRAY_OBJ_DECLARE(
  dense_4_nl_output_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 20, AI_STATIC)

/* Array#106 */
AI_ARRAY_OBJ_DECLARE(
  multiply_2_output_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 320, AI_STATIC)

/* Array#107 */
AI_ARRAY_OBJ_DECLARE(
  conv2d_9_output_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 192, AI_STATIC)

/* Array#108 */
AI_ARRAY_OBJ_DECLARE(
  conv2d_10_output_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 960, AI_STATIC)

/* Array#109 */
AI_ARRAY_OBJ_DECLARE(
  depthwise_conv2d_5_output_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 960, AI_STATIC)

/* Array#110 */
AI_ARRAY_OBJ_DECLARE(
  global_average_pooling2d_3_output_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 60, AI_STATIC)

/* Array#111 */
AI_ARRAY_OBJ_DECLARE(
  dense_5_output_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 60, AI_STATIC)

/* Array#112 */
AI_ARRAY_OBJ_DECLARE(
  dense_5_nl_output_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 60, AI_STATIC)

/* Array#113 */
AI_ARRAY_OBJ_DECLARE(
  dense_6_output_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 60, AI_STATIC)

/* Array#114 */
AI_ARRAY_OBJ_DECLARE(
  dense_6_nl_output_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 60, AI_STATIC)

/* Array#115 */
AI_ARRAY_OBJ_DECLARE(
  multiply_3_output_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 960, AI_STATIC)

/* Array#116 */
AI_ARRAY_OBJ_DECLARE(
  conv2d_11_output_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 192, AI_STATIC)

/* Array#117 */
AI_ARRAY_OBJ_DECLARE(
  add_2_output_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 192, AI_STATIC)

/* Array#118 */
AI_ARRAY_OBJ_DECLARE(
  conv2d_12_output_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 960, AI_STATIC)

/* Array#119 */
AI_ARRAY_OBJ_DECLARE(
  depthwise_conv2d_6_output_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 960, AI_STATIC)

/* Array#120 */
AI_ARRAY_OBJ_DECLARE(
  global_average_pooling2d_4_output_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 60, AI_STATIC)

/* Array#121 */
AI_ARRAY_OBJ_DECLARE(
  dense_7_output_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 60, AI_STATIC)

/* Array#122 */
AI_ARRAY_OBJ_DECLARE(
  dense_7_nl_output_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 60, AI_STATIC)

/* Array#123 */
AI_ARRAY_OBJ_DECLARE(
  dense_8_output_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 60, AI_STATIC)

/* Array#124 */
AI_ARRAY_OBJ_DECLARE(
  dense_8_nl_output_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 60, AI_STATIC)

/* Array#125 */
AI_ARRAY_OBJ_DECLARE(
  multiply_4_output_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 960, AI_STATIC)

/* Array#126 */
AI_ARRAY_OBJ_DECLARE(
  conv2d_13_output_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 192, AI_STATIC)

/* Array#127 */
AI_ARRAY_OBJ_DECLARE(
  add_3_output_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 192, AI_STATIC)

/* Array#128 */
AI_ARRAY_OBJ_DECLARE(
  conv2d_14_output_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 960, AI_STATIC)

/* Array#129 */
AI_ARRAY_OBJ_DECLARE(
  depthwise_conv2d_7_output_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 240, AI_STATIC)

/* Array#130 */
AI_ARRAY_OBJ_DECLARE(
  global_average_pooling2d_5_output_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 60, AI_STATIC)

/* Array#131 */
AI_ARRAY_OBJ_DECLARE(
  dense_9_output_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 60, AI_STATIC)

/* Array#132 */
AI_ARRAY_OBJ_DECLARE(
  dense_9_nl_output_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 60, AI_STATIC)

/* Array#133 */
AI_ARRAY_OBJ_DECLARE(
  dense_10_output_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 60, AI_STATIC)

/* Array#134 */
AI_ARRAY_OBJ_DECLARE(
  dense_10_nl_output_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 60, AI_STATIC)

/* Array#135 */
AI_ARRAY_OBJ_DECLARE(
  multiply_5_output_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 240, AI_STATIC)

/* Array#136 */
AI_ARRAY_OBJ_DECLARE(
  conv2d_15_output_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 80, AI_STATIC)

/* Array#137 */
AI_ARRAY_OBJ_DECLARE(
  conv2d_16_output_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 240, AI_STATIC)

/* Array#138 */
AI_ARRAY_OBJ_DECLARE(
  depthwise_conv2d_8_output_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 240, AI_STATIC)

/* Array#139 */
AI_ARRAY_OBJ_DECLARE(
  global_average_pooling2d_6_output_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 60, AI_STATIC)

/* Array#140 */
AI_ARRAY_OBJ_DECLARE(
  dense_11_output_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 60, AI_STATIC)

/* Array#141 */
AI_ARRAY_OBJ_DECLARE(
  dense_11_nl_output_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 60, AI_STATIC)

/* Array#142 */
AI_ARRAY_OBJ_DECLARE(
  dense_12_output_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 60, AI_STATIC)

/* Array#143 */
AI_ARRAY_OBJ_DECLARE(
  dense_12_nl_output_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 60, AI_STATIC)

/* Array#144 */
AI_ARRAY_OBJ_DECLARE(
  multiply_6_output_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 240, AI_STATIC)

/* Array#145 */
AI_ARRAY_OBJ_DECLARE(
  conv2d_17_output_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 80, AI_STATIC)

/* Array#146 */
AI_ARRAY_OBJ_DECLARE(
  add_4_output_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 80, AI_STATIC)

/* Array#147 */
AI_ARRAY_OBJ_DECLARE(
  conv2d_18_output_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 60, AI_STATIC)

/* Array#148 */
AI_ARRAY_OBJ_DECLARE(
  conv2d_19_output_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 120, AI_STATIC)

/* Array#149 */
AI_ARRAY_OBJ_DECLARE(
  conv2d_20_output_array, AI_ARRAY_FORMAT_FLOAT|AI_FMT_FLAG_IS_IO,
  NULL, NULL, 4, AI_STATIC)

/**  Tensor declarations section  *********************************************/
/* Tensor #0 */
AI_TENSOR_OBJ_DECLARE(
  conv2d_18_scratch0, AI_STATIC,
  0x0, 0x0,
  AI_SHAPE_INIT(4, 1, 60, 2, 2), AI_STRIDE_INIT(4, 4, 4, 240, 480),
  1, &conv2d_18_scratch0_array, NULL)

/* Tensor #1 */
AI_TENSOR_OBJ_DECLARE(
  conv2d_20_bias, AI_STATIC,
  0x0, 0x0,
  AI_SHAPE_INIT(4, 1, 4, 1, 1), AI_STRIDE_INIT(4, 4, 4, 16, 16),
  1, &conv2d_20_bias_array, NULL)

/* Tensor #2 */
AI_TENSOR_OBJ_DECLARE(
  conv2d_20_weights, AI_STATIC,
  0x0, 0x0,
  AI_SHAPE_INIT(4, 120, 1, 1, 4), AI_STRIDE_INIT(4, 4, 480, 480, 480),
  1, &conv2d_20_weights_array, NULL)

/* Tensor #3 */
AI_TENSOR_OBJ_DECLARE(
  conv2d_19_bias, AI_STATIC,
  0x0, 0x0,
  AI_SHAPE_INIT(4, 1, 120, 1, 1), AI_STRIDE_INIT(4, 4, 4, 480, 480),
  1, &conv2d_19_bias_array, NULL)

/* Tensor #4 */
AI_TENSOR_OBJ_DECLARE(
  conv2d_19_weights, AI_STATIC,
  0x0, 0x0,
  AI_SHAPE_INIT(4, 60, 1, 1, 120), AI_STRIDE_INIT(4, 4, 240, 240, 240),
  1, &conv2d_19_weights_array, NULL)

/* Tensor #5 */
AI_TENSOR_OBJ_DECLARE(
  conv2d_18_bias, AI_STATIC,
  0x0, 0x0,
  AI_SHAPE_INIT(4, 1, 60, 1, 1), AI_STRIDE_INIT(4, 4, 4, 240, 240),
  1, &conv2d_18_bias_array, NULL)

/* Tensor #6 */
AI_TENSOR_OBJ_DECLARE(
  conv2d_18_weights, AI_STATIC,
  0x0, 0x0,
  AI_SHAPE_INIT(4, 20, 1, 1, 60), AI_STRIDE_INIT(4, 4, 80, 80, 80),
  1, &conv2d_18_weights_array, NULL)

/* Tensor #7 */
AI_TENSOR_OBJ_DECLARE(
  conv2d_17_bias, AI_STATIC,
  0x0, 0x0,
  AI_SHAPE_INIT(4, 1, 20, 1, 1), AI_STRIDE_INIT(4, 4, 4, 80, 80),
  1, &conv2d_17_bias_array, NULL)

/* Tensor #8 */
AI_TENSOR_OBJ_DECLARE(
  conv2d_17_weights, AI_STATIC,
  0x0, 0x0,
  AI_SHAPE_INIT(4, 60, 1, 1, 20), AI_STRIDE_INIT(4, 4, 240, 240, 240),
  1, &conv2d_17_weights_array, NULL)

/* Tensor #9 */
AI_TENSOR_OBJ_DECLARE(
  dense_12_bias, AI_STATIC,
  0x0, 0x0,
  AI_SHAPE_INIT(4, 1, 60, 1, 1), AI_STRIDE_INIT(4, 4, 4, 240, 240),
  1, &dense_12_bias_array, NULL)

/* Tensor #10 */
AI_TENSOR_OBJ_DECLARE(
  dense_12_weights, AI_STATIC,
  0x0, 0x0,
  AI_SHAPE_INIT(4, 60, 60, 1, 1), AI_STRIDE_INIT(4, 4, 240, 14400, 14400),
  1, &dense_12_weights_array, NULL)

/* Tensor #11 */
AI_TENSOR_OBJ_DECLARE(
  dense_11_bias, AI_STATIC,
  0x0, 0x0,
  AI_SHAPE_INIT(4, 1, 60, 1, 1), AI_STRIDE_INIT(4, 4, 4, 240, 240),
  1, &dense_11_bias_array, NULL)

/* Tensor #12 */
AI_TENSOR_OBJ_DECLARE(
  dense_11_weights, AI_STATIC,
  0x0, 0x0,
  AI_SHAPE_INIT(4, 60, 60, 1, 1), AI_STRIDE_INIT(4, 4, 240, 14400, 14400),
  1, &dense_11_weights_array, NULL)

/* Tensor #13 */
AI_TENSOR_OBJ_DECLARE(
  depthwise_conv2d_8_bias, AI_STATIC,
  0x0, 0x0,
  AI_SHAPE_INIT(4, 1, 60, 1, 1), AI_STRIDE_INIT(4, 4, 4, 240, 240),
  1, &depthwise_conv2d_8_bias_array, NULL)

/* Tensor #14 */
AI_TENSOR_OBJ_DECLARE(
  depthwise_conv2d_8_weights, AI_STATIC,
  0x0, 0x0,
  AI_SHAPE_INIT(4, 1, 5, 5, 60), AI_STRIDE_INIT(4, 4, 4, 20, 100),
  1, &depthwise_conv2d_8_weights_array, NULL)

/* Tensor #15 */
AI_TENSOR_OBJ_DECLARE(
  conv2d_16_bias, AI_STATIC,
  0x0, 0x0,
  AI_SHAPE_INIT(4, 1, 60, 1, 1), AI_STRIDE_INIT(4, 4, 4, 240, 240),
  1, &conv2d_16_bias_array, NULL)

/* Tensor #16 */
AI_TENSOR_OBJ_DECLARE(
  conv2d_16_weights, AI_STATIC,
  0x0, 0x0,
  AI_SHAPE_INIT(4, 20, 1, 1, 60), AI_STRIDE_INIT(4, 4, 80, 80, 80),
  1, &conv2d_16_weights_array, NULL)

/* Tensor #17 */
AI_TENSOR_OBJ_DECLARE(
  conv2d_15_bias, AI_STATIC,
  0x0, 0x0,
  AI_SHAPE_INIT(4, 1, 20, 1, 1), AI_STRIDE_INIT(4, 4, 4, 80, 80),
  1, &conv2d_15_bias_array, NULL)

/* Tensor #18 */
AI_TENSOR_OBJ_DECLARE(
  conv2d_15_weights, AI_STATIC,
  0x0, 0x0,
  AI_SHAPE_INIT(4, 60, 1, 1, 20), AI_STRIDE_INIT(4, 4, 240, 240, 240),
  1, &conv2d_15_weights_array, NULL)

/* Tensor #19 */
AI_TENSOR_OBJ_DECLARE(
  dense_10_bias, AI_STATIC,
  0x0, 0x0,
  AI_SHAPE_INIT(4, 1, 60, 1, 1), AI_STRIDE_INIT(4, 4, 4, 240, 240),
  1, &dense_10_bias_array, NULL)

/* Tensor #20 */
AI_TENSOR_OBJ_DECLARE(
  dense_10_weights, AI_STATIC,
  0x0, 0x0,
  AI_SHAPE_INIT(4, 60, 60, 1, 1), AI_STRIDE_INIT(4, 4, 240, 14400, 14400),
  1, &dense_10_weights_array, NULL)

/* Tensor #21 */
AI_TENSOR_OBJ_DECLARE(
  dense_9_bias, AI_STATIC,
  0x0, 0x0,
  AI_SHAPE_INIT(4, 1, 60, 1, 1), AI_STRIDE_INIT(4, 4, 4, 240, 240),
  1, &dense_9_bias_array, NULL)

/* Tensor #22 */
AI_TENSOR_OBJ_DECLARE(
  dense_9_weights, AI_STATIC,
  0x0, 0x0,
  AI_SHAPE_INIT(4, 60, 60, 1, 1), AI_STRIDE_INIT(4, 4, 240, 14400, 14400),
  1, &dense_9_weights_array, NULL)

/* Tensor #23 */
AI_TENSOR_OBJ_DECLARE(
  depthwise_conv2d_7_bias, AI_STATIC,
  0x0, 0x0,
  AI_SHAPE_INIT(4, 1, 60, 1, 1), AI_STRIDE_INIT(4, 4, 4, 240, 240),
  1, &depthwise_conv2d_7_bias_array, NULL)

/* Tensor #24 */
AI_TENSOR_OBJ_DECLARE(
  depthwise_conv2d_7_weights, AI_STATIC,
  0x0, 0x0,
  AI_SHAPE_INIT(4, 1, 5, 5, 60), AI_STRIDE_INIT(4, 4, 4, 20, 100),
  1, &depthwise_conv2d_7_weights_array, NULL)

/* Tensor #25 */
AI_TENSOR_OBJ_DECLARE(
  conv2d_14_bias, AI_STATIC,
  0x0, 0x0,
  AI_SHAPE_INIT(4, 1, 60, 1, 1), AI_STRIDE_INIT(4, 4, 4, 240, 240),
  1, &conv2d_14_bias_array, NULL)

/* Tensor #26 */
AI_TENSOR_OBJ_DECLARE(
  conv2d_14_weights, AI_STATIC,
  0x0, 0x0,
  AI_SHAPE_INIT(4, 12, 1, 1, 60), AI_STRIDE_INIT(4, 4, 48, 48, 48),
  1, &conv2d_14_weights_array, NULL)

/* Tensor #27 */
AI_TENSOR_OBJ_DECLARE(
  conv2d_13_bias, AI_STATIC,
  0x0, 0x0,
  AI_SHAPE_INIT(4, 1, 12, 1, 1), AI_STRIDE_INIT(4, 4, 4, 48, 48),
  1, &conv2d_13_bias_array, NULL)

/* Tensor #28 */
AI_TENSOR_OBJ_DECLARE(
  conv2d_13_weights, AI_STATIC,
  0x0, 0x0,
  AI_SHAPE_INIT(4, 60, 1, 1, 12), AI_STRIDE_INIT(4, 4, 240, 240, 240),
  1, &conv2d_13_weights_array, NULL)

/* Tensor #29 */
AI_TENSOR_OBJ_DECLARE(
  dense_8_bias, AI_STATIC,
  0x0, 0x0,
  AI_SHAPE_INIT(4, 1, 60, 1, 1), AI_STRIDE_INIT(4, 4, 4, 240, 240),
  1, &dense_8_bias_array, NULL)

/* Tensor #30 */
AI_TENSOR_OBJ_DECLARE(
  dense_8_weights, AI_STATIC,
  0x0, 0x0,
  AI_SHAPE_INIT(4, 60, 60, 1, 1), AI_STRIDE_INIT(4, 4, 240, 14400, 14400),
  1, &dense_8_weights_array, NULL)

/* Tensor #31 */
AI_TENSOR_OBJ_DECLARE(
  dense_7_bias, AI_STATIC,
  0x0, 0x0,
  AI_SHAPE_INIT(4, 1, 60, 1, 1), AI_STRIDE_INIT(4, 4, 4, 240, 240),
  1, &dense_7_bias_array, NULL)

/* Tensor #32 */
AI_TENSOR_OBJ_DECLARE(
  dense_7_weights, AI_STATIC,
  0x0, 0x0,
  AI_SHAPE_INIT(4, 60, 60, 1, 1), AI_STRIDE_INIT(4, 4, 240, 14400, 14400),
  1, &dense_7_weights_array, NULL)

/* Tensor #33 */
AI_TENSOR_OBJ_DECLARE(
  depthwise_conv2d_6_bias, AI_STATIC,
  0x0, 0x0,
  AI_SHAPE_INIT(4, 1, 60, 1, 1), AI_STRIDE_INIT(4, 4, 4, 240, 240),
  1, &depthwise_conv2d_6_bias_array, NULL)

/* Tensor #34 */
AI_TENSOR_OBJ_DECLARE(
  depthwise_conv2d_6_weights, AI_STATIC,
  0x0, 0x0,
  AI_SHAPE_INIT(4, 1, 5, 5, 60), AI_STRIDE_INIT(4, 4, 4, 20, 100),
  1, &depthwise_conv2d_6_weights_array, NULL)

/* Tensor #35 */
AI_TENSOR_OBJ_DECLARE(
  conv2d_12_bias, AI_STATIC,
  0x0, 0x0,
  AI_SHAPE_INIT(4, 1, 60, 1, 1), AI_STRIDE_INIT(4, 4, 4, 240, 240),
  1, &conv2d_12_bias_array, NULL)

/* Tensor #36 */
AI_TENSOR_OBJ_DECLARE(
  conv2d_12_weights, AI_STATIC,
  0x0, 0x0,
  AI_SHAPE_INIT(4, 12, 1, 1, 60), AI_STRIDE_INIT(4, 4, 48, 48, 48),
  1, &conv2d_12_weights_array, NULL)

/* Tensor #37 */
AI_TENSOR_OBJ_DECLARE(
  conv2d_11_bias, AI_STATIC,
  0x0, 0x0,
  AI_SHAPE_INIT(4, 1, 12, 1, 1), AI_STRIDE_INIT(4, 4, 4, 48, 48),
  1, &conv2d_11_bias_array, NULL)

/* Tensor #38 */
AI_TENSOR_OBJ_DECLARE(
  conv2d_11_weights, AI_STATIC,
  0x0, 0x0,
  AI_SHAPE_INIT(4, 60, 1, 1, 12), AI_STRIDE_INIT(4, 4, 240, 240, 240),
  1, &conv2d_11_weights_array, NULL)

/* Tensor #39 */
AI_TENSOR_OBJ_DECLARE(
  dense_6_bias, AI_STATIC,
  0x0, 0x0,
  AI_SHAPE_INIT(4, 1, 60, 1, 1), AI_STRIDE_INIT(4, 4, 4, 240, 240),
  1, &dense_6_bias_array, NULL)

/* Tensor #40 */
AI_TENSOR_OBJ_DECLARE(
  dense_6_weights, AI_STATIC,
  0x0, 0x0,
  AI_SHAPE_INIT(4, 60, 60, 1, 1), AI_STRIDE_INIT(4, 4, 240, 14400, 14400),
  1, &dense_6_weights_array, NULL)

/* Tensor #41 */
AI_TENSOR_OBJ_DECLARE(
  dense_5_bias, AI_STATIC,
  0x0, 0x0,
  AI_SHAPE_INIT(4, 1, 60, 1, 1), AI_STRIDE_INIT(4, 4, 4, 240, 240),
  1, &dense_5_bias_array, NULL)

/* Tensor #42 */
AI_TENSOR_OBJ_DECLARE(
  dense_5_weights, AI_STATIC,
  0x0, 0x0,
  AI_SHAPE_INIT(4, 60, 60, 1, 1), AI_STRIDE_INIT(4, 4, 240, 14400, 14400),
  1, &dense_5_weights_array, NULL)

/* Tensor #43 */
AI_TENSOR_OBJ_DECLARE(
  depthwise_conv2d_5_bias, AI_STATIC,
  0x0, 0x0,
  AI_SHAPE_INIT(4, 1, 60, 1, 1), AI_STRIDE_INIT(4, 4, 4, 240, 240),
  1, &depthwise_conv2d_5_bias_array, NULL)

/* Tensor #44 */
AI_TENSOR_OBJ_DECLARE(
  depthwise_conv2d_5_weights, AI_STATIC,
  0x0, 0x0,
  AI_SHAPE_INIT(4, 1, 5, 5, 60), AI_STRIDE_INIT(4, 4, 4, 20, 100),
  1, &depthwise_conv2d_5_weights_array, NULL)

/* Tensor #45 */
AI_TENSOR_OBJ_DECLARE(
  conv2d_10_bias, AI_STATIC,
  0x0, 0x0,
  AI_SHAPE_INIT(4, 1, 60, 1, 1), AI_STRIDE_INIT(4, 4, 4, 240, 240),
  1, &conv2d_10_bias_array, NULL)

/* Tensor #46 */
AI_TENSOR_OBJ_DECLARE(
  conv2d_10_weights, AI_STATIC,
  0x0, 0x0,
  AI_SHAPE_INIT(4, 12, 1, 1, 60), AI_STRIDE_INIT(4, 4, 48, 48, 48),
  1, &conv2d_10_weights_array, NULL)

/* Tensor #47 */
AI_TENSOR_OBJ_DECLARE(
  conv2d_9_bias, AI_STATIC,
  0x0, 0x0,
  AI_SHAPE_INIT(4, 1, 12, 1, 1), AI_STRIDE_INIT(4, 4, 4, 48, 48),
  1, &conv2d_9_bias_array, NULL)

/* Tensor #48 */
AI_TENSOR_OBJ_DECLARE(
  conv2d_9_weights, AI_STATIC,
  0x0, 0x0,
  AI_SHAPE_INIT(4, 20, 1, 1, 12), AI_STRIDE_INIT(4, 4, 80, 80, 80),
  1, &conv2d_9_weights_array, NULL)

/* Tensor #49 */
AI_TENSOR_OBJ_DECLARE(
  dense_4_bias, AI_STATIC,
  0x0, 0x0,
  AI_SHAPE_INIT(4, 1, 20, 1, 1), AI_STRIDE_INIT(4, 4, 4, 80, 80),
  1, &dense_4_bias_array, NULL)

/* Tensor #50 */
AI_TENSOR_OBJ_DECLARE(
  dense_4_weights, AI_STATIC,
  0x0, 0x0,
  AI_SHAPE_INIT(4, 20, 20, 1, 1), AI_STRIDE_INIT(4, 4, 80, 1600, 1600),
  1, &dense_4_weights_array, NULL)

/* Tensor #51 */
AI_TENSOR_OBJ_DECLARE(
  dense_3_bias, AI_STATIC,
  0x0, 0x0,
  AI_SHAPE_INIT(4, 1, 20, 1, 1), AI_STRIDE_INIT(4, 4, 4, 80, 80),
  1, &dense_3_bias_array, NULL)

/* Tensor #52 */
AI_TENSOR_OBJ_DECLARE(
  dense_3_weights, AI_STATIC,
  0x0, 0x0,
  AI_SHAPE_INIT(4, 20, 20, 1, 1), AI_STRIDE_INIT(4, 4, 80, 1600, 1600),
  1, &dense_3_weights_array, NULL)

/* Tensor #53 */
AI_TENSOR_OBJ_DECLARE(
  depthwise_conv2d_4_bias, AI_STATIC,
  0x0, 0x0,
  AI_SHAPE_INIT(4, 1, 20, 1, 1), AI_STRIDE_INIT(4, 4, 4, 80, 80),
  1, &depthwise_conv2d_4_bias_array, NULL)

/* Tensor #54 */
AI_TENSOR_OBJ_DECLARE(
  depthwise_conv2d_4_weights, AI_STATIC,
  0x0, 0x0,
  AI_SHAPE_INIT(4, 1, 5, 5, 20), AI_STRIDE_INIT(4, 4, 4, 20, 100),
  1, &depthwise_conv2d_4_weights_array, NULL)

/* Tensor #55 */
AI_TENSOR_OBJ_DECLARE(
  conv2d_8_bias, AI_STATIC,
  0x0, 0x0,
  AI_SHAPE_INIT(4, 1, 20, 1, 1), AI_STRIDE_INIT(4, 4, 4, 80, 80),
  1, &conv2d_8_bias_array, NULL)

/* Tensor #56 */
AI_TENSOR_OBJ_DECLARE(
  conv2d_8_weights, AI_STATIC,
  0x0, 0x0,
  AI_SHAPE_INIT(4, 8, 1, 1, 20), AI_STRIDE_INIT(4, 4, 32, 32, 32),
  1, &conv2d_8_weights_array, NULL)

/* Tensor #57 */
AI_TENSOR_OBJ_DECLARE(
  conv2d_7_bias, AI_STATIC,
  0x0, 0x0,
  AI_SHAPE_INIT(4, 1, 8, 1, 1), AI_STRIDE_INIT(4, 4, 4, 32, 32),
  1, &conv2d_7_bias_array, NULL)

/* Tensor #58 */
AI_TENSOR_OBJ_DECLARE(
  conv2d_7_weights, AI_STATIC,
  0x0, 0x0,
  AI_SHAPE_INIT(4, 20, 1, 1, 8), AI_STRIDE_INIT(4, 4, 80, 80, 80),
  1, &conv2d_7_weights_array, NULL)

/* Tensor #59 */
AI_TENSOR_OBJ_DECLARE(
  depthwise_conv2d_3_bias, AI_STATIC,
  0x0, 0x0,
  AI_SHAPE_INIT(4, 1, 20, 1, 1), AI_STRIDE_INIT(4, 4, 4, 80, 80),
  1, &depthwise_conv2d_3_bias_array, NULL)

/* Tensor #60 */
AI_TENSOR_OBJ_DECLARE(
  depthwise_conv2d_3_weights, AI_STATIC,
  0x0, 0x0,
  AI_SHAPE_INIT(4, 1, 3, 3, 20), AI_STRIDE_INIT(4, 4, 4, 12, 36),
  1, &depthwise_conv2d_3_weights_array, NULL)

/* Tensor #61 */
AI_TENSOR_OBJ_DECLARE(
  conv2d_6_bias, AI_STATIC,
  0x0, 0x0,
  AI_SHAPE_INIT(4, 1, 20, 1, 1), AI_STRIDE_INIT(4, 4, 4, 80, 80),
  1, &conv2d_6_bias_array, NULL)

/* Tensor #62 */
AI_TENSOR_OBJ_DECLARE(
  conv2d_6_weights, AI_STATIC,
  0x0, 0x0,
  AI_SHAPE_INIT(4, 8, 1, 1, 20), AI_STRIDE_INIT(4, 4, 32, 32, 32),
  1, &conv2d_6_weights_array, NULL)

/* Tensor #63 */
AI_TENSOR_OBJ_DECLARE(
  conv2d_5_bias, AI_STATIC,
  0x0, 0x0,
  AI_SHAPE_INIT(4, 1, 8, 1, 1), AI_STRIDE_INIT(4, 4, 4, 32, 32),
  1, &conv2d_5_bias_array, NULL)

/* Tensor #64 */
AI_TENSOR_OBJ_DECLARE(
  conv2d_5_weights, AI_STATIC,
  0x0, 0x0,
  AI_SHAPE_INIT(4, 20, 1, 1, 8), AI_STRIDE_INIT(4, 4, 80, 80, 80),
  1, &conv2d_5_weights_array, NULL)

/* Tensor #65 */
AI_TENSOR_OBJ_DECLARE(
  depthwise_conv2d_2_bias, AI_STATIC,
  0x0, 0x0,
  AI_SHAPE_INIT(4, 1, 20, 1, 1), AI_STRIDE_INIT(4, 4, 4, 80, 80),
  1, &depthwise_conv2d_2_bias_array, NULL)

/* Tensor #66 */
AI_TENSOR_OBJ_DECLARE(
  depthwise_conv2d_2_weights, AI_STATIC,
  0x0, 0x0,
  AI_SHAPE_INIT(4, 1, 3, 3, 20), AI_STRIDE_INIT(4, 4, 4, 12, 36),
  1, &depthwise_conv2d_2_weights_array, NULL)

/* Tensor #67 */
AI_TENSOR_OBJ_DECLARE(
  conv2d_4_bias, AI_STATIC,
  0x0, 0x0,
  AI_SHAPE_INIT(4, 1, 20, 1, 1), AI_STRIDE_INIT(4, 4, 4, 80, 80),
  1, &conv2d_4_bias_array, NULL)

/* Tensor #68 */
AI_TENSOR_OBJ_DECLARE(
  conv2d_4_weights, AI_STATIC,
  0x0, 0x0,
  AI_SHAPE_INIT(4, 6, 1, 1, 20), AI_STRIDE_INIT(4, 4, 24, 24, 24),
  1, &conv2d_4_weights_array, NULL)

/* Tensor #69 */
AI_TENSOR_OBJ_DECLARE(
  conv2d_3_bias, AI_STATIC,
  0x0, 0x0,
  AI_SHAPE_INIT(4, 1, 6, 1, 1), AI_STRIDE_INIT(4, 4, 4, 24, 24),
  1, &conv2d_3_bias_array, NULL)

/* Tensor #70 */
AI_TENSOR_OBJ_DECLARE(
  conv2d_3_weights, AI_STATIC,
  0x0, 0x0,
  AI_SHAPE_INIT(4, 6, 1, 1, 6), AI_STRIDE_INIT(4, 4, 24, 24, 24),
  1, &conv2d_3_weights_array, NULL)

/* Tensor #71 */
AI_TENSOR_OBJ_DECLARE(
  dense_2_bias, AI_STATIC,
  0x0, 0x0,
  AI_SHAPE_INIT(4, 1, 6, 1, 1), AI_STRIDE_INIT(4, 4, 4, 24, 24),
  1, &dense_2_bias_array, NULL)

/* Tensor #72 */
AI_TENSOR_OBJ_DECLARE(
  dense_2_weights, AI_STATIC,
  0x0, 0x0,
  AI_SHAPE_INIT(4, 6, 6, 1, 1), AI_STRIDE_INIT(4, 4, 24, 144, 144),
  1, &dense_2_weights_array, NULL)

/* Tensor #73 */
AI_TENSOR_OBJ_DECLARE(
  dense_1_bias, AI_STATIC,
  0x0, 0x0,
  AI_SHAPE_INIT(4, 1, 6, 1, 1), AI_STRIDE_INIT(4, 4, 4, 24, 24),
  1, &dense_1_bias_array, NULL)

/* Tensor #74 */
AI_TENSOR_OBJ_DECLARE(
  dense_1_weights, AI_STATIC,
  0x0, 0x0,
  AI_SHAPE_INIT(4, 6, 6, 1, 1), AI_STRIDE_INIT(4, 4, 24, 144, 144),
  1, &dense_1_weights_array, NULL)

/* Tensor #75 */
AI_TENSOR_OBJ_DECLARE(
  depthwise_conv2d_1_bias, AI_STATIC,
  0x0, 0x0,
  AI_SHAPE_INIT(4, 1, 6, 1, 1), AI_STRIDE_INIT(4, 4, 4, 24, 24),
  1, &depthwise_conv2d_1_bias_array, NULL)

/* Tensor #76 */
AI_TENSOR_OBJ_DECLARE(
  depthwise_conv2d_1_weights, AI_STATIC,
  0x0, 0x0,
  AI_SHAPE_INIT(4, 1, 3, 3, 6), AI_STRIDE_INIT(4, 4, 4, 12, 36),
  1, &depthwise_conv2d_1_weights_array, NULL)

/* Tensor #77 */
AI_TENSOR_OBJ_DECLARE(
  conv2d_2_bias, AI_STATIC,
  0x0, 0x0,
  AI_SHAPE_INIT(4, 1, 6, 1, 1), AI_STRIDE_INIT(4, 4, 4, 24, 24),
  1, &conv2d_2_bias_array, NULL)

/* Tensor #78 */
AI_TENSOR_OBJ_DECLARE(
  conv2d_2_weights, AI_STATIC,
  0x0, 0x0,
  AI_SHAPE_INIT(4, 6, 1, 1, 6), AI_STRIDE_INIT(4, 4, 24, 24, 24),
  1, &conv2d_2_weights_array, NULL)

/* Tensor #79 */
AI_TENSOR_OBJ_DECLARE(
  conv2d_1_bias, AI_STATIC,
  0x0, 0x0,
  AI_SHAPE_INIT(4, 1, 6, 1, 1), AI_STRIDE_INIT(4, 4, 4, 24, 24),
  1, &conv2d_1_bias_array, NULL)

/* Tensor #80 */
AI_TENSOR_OBJ_DECLARE(
  conv2d_1_weights, AI_STATIC,
  0x0, 0x0,
  AI_SHAPE_INIT(4, 1, 3, 3, 6), AI_STRIDE_INIT(4, 4, 4, 12, 36),
  1, &conv2d_1_weights_array, NULL)

/* Tensor #81 */
AI_TENSOR_OBJ_DECLARE(
  input_1_output, AI_STATIC,
  0x0, 0x0,
  AI_SHAPE_INIT(4, 1, 1, 60, 50), AI_STRIDE_INIT(4, 4, 4, 4, 240),
  1, &input_1_output_array, NULL)

/* Tensor #82 */
AI_TENSOR_OBJ_DECLARE(
  conv2d_1_output, AI_STATIC,
  0x0, 0x0,
  AI_SHAPE_INIT(4, 1, 6, 30, 25), AI_STRIDE_INIT(4, 4, 4, 24, 720),
  1, &conv2d_1_output_array, NULL)

/* Tensor #83 */
AI_TENSOR_OBJ_DECLARE(
  conv2d_2_output, AI_STATIC,
  0x0, 0x0,
  AI_SHAPE_INIT(4, 1, 6, 30, 25), AI_STRIDE_INIT(4, 4, 4, 24, 720),
  1, &conv2d_2_output_array, NULL)

/* Tensor #84 */
AI_TENSOR_OBJ_DECLARE(
  depthwise_conv2d_1_output, AI_STATIC,
  0x0, 0x0,
  AI_SHAPE_INIT(4, 1, 6, 15, 13), AI_STRIDE_INIT(4, 4, 4, 24, 360),
  1, &depthwise_conv2d_1_output_array, NULL)

/* Tensor #85 */
AI_TENSOR_OBJ_DECLARE(
  global_average_pooling2d_1_output, AI_STATIC,
  0x0, 0x0,
  AI_SHAPE_INIT(4, 1, 6, 1, 1), AI_STRIDE_INIT(4, 4, 4, 24, 24),
  1, &global_average_pooling2d_1_output_array, NULL)

/* Tensor #86 */
AI_TENSOR_OBJ_DECLARE(
  dense_1_output, AI_STATIC,
  0x0, 0x0,
  AI_SHAPE_INIT(4, 1, 6, 1, 1), AI_STRIDE_INIT(4, 4, 4, 24, 24),
  1, &dense_1_output_array, NULL)

/* Tensor #87 */
AI_TENSOR_OBJ_DECLARE(
  dense_1_nl_output, AI_STATIC,
  0x0, 0x0,
  AI_SHAPE_INIT(4, 1, 6, 1, 1), AI_STRIDE_INIT(4, 4, 4, 24, 24),
  1, &dense_1_nl_output_array, NULL)

/* Tensor #88 */
AI_TENSOR_OBJ_DECLARE(
  dense_2_output, AI_STATIC,
  0x0, 0x0,
  AI_SHAPE_INIT(4, 1, 6, 1, 1), AI_STRIDE_INIT(4, 4, 4, 24, 24),
  1, &dense_2_output_array, NULL)

/* Tensor #89 */
AI_TENSOR_OBJ_DECLARE(
  dense_2_nl_output, AI_STATIC,
  0x0, 0x0,
  AI_SHAPE_INIT(4, 1, 6, 1, 1), AI_STRIDE_INIT(4, 4, 4, 24, 24),
  1, &dense_2_nl_output_array, NULL)

/* Tensor #90 */
AI_TENSOR_OBJ_DECLARE(
  multiply_1_output, AI_STATIC,
  0x0, 0x0,
  AI_SHAPE_INIT(4, 1, 6, 15, 13), AI_STRIDE_INIT(4, 4, 4, 24, 360),
  1, &multiply_1_output_array, NULL)

/* Tensor #91 */
AI_TENSOR_OBJ_DECLARE(
  conv2d_3_output, AI_STATIC,
  0x0, 0x0,
  AI_SHAPE_INIT(4, 1, 6, 15, 13), AI_STRIDE_INIT(4, 4, 4, 24, 360),
  1, &conv2d_3_output_array, NULL)

/* Tensor #92 */
AI_TENSOR_OBJ_DECLARE(
  conv2d_4_output, AI_STATIC,
  0x0, 0x0,
  AI_SHAPE_INIT(4, 1, 20, 15, 13), AI_STRIDE_INIT(4, 4, 4, 80, 1200),
  1, &conv2d_4_output_array, NULL)

/* Tensor #93 */
AI_TENSOR_OBJ_DECLARE(
  depthwise_conv2d_2_output, AI_STATIC,
  0x0, 0x0,
  AI_SHAPE_INIT(4, 1, 20, 8, 7), AI_STRIDE_INIT(4, 4, 4, 80, 640),
  1, &depthwise_conv2d_2_output_array, NULL)

/* Tensor #94 */
AI_TENSOR_OBJ_DECLARE(
  conv2d_5_output, AI_STATIC,
  0x0, 0x0,
  AI_SHAPE_INIT(4, 1, 8, 8, 7), AI_STRIDE_INIT(4, 4, 4, 32, 256),
  1, &conv2d_5_output_array, NULL)

/* Tensor #95 */
AI_TENSOR_OBJ_DECLARE(
  conv2d_6_output, AI_STATIC,
  0x0, 0x0,
  AI_SHAPE_INIT(4, 1, 20, 8, 7), AI_STRIDE_INIT(4, 4, 4, 80, 640),
  1, &conv2d_6_output_array, NULL)

/* Tensor #96 */
AI_TENSOR_OBJ_DECLARE(
  depthwise_conv2d_3_output, AI_STATIC,
  0x0, 0x0,
  AI_SHAPE_INIT(4, 1, 20, 8, 7), AI_STRIDE_INIT(4, 4, 4, 80, 640),
  1, &depthwise_conv2d_3_output_array, NULL)

/* Tensor #97 */
AI_TENSOR_OBJ_DECLARE(
  conv2d_7_output, AI_STATIC,
  0x0, 0x0,
  AI_SHAPE_INIT(4, 1, 8, 8, 7), AI_STRIDE_INIT(4, 4, 4, 32, 256),
  1, &conv2d_7_output_array, NULL)

/* Tensor #98 */
AI_TENSOR_OBJ_DECLARE(
  add_1_output, AI_STATIC,
  0x0, 0x0,
  AI_SHAPE_INIT(4, 1, 8, 8, 7), AI_STRIDE_INIT(4, 4, 4, 32, 256),
  1, &add_1_output_array, NULL)

/* Tensor #99 */
AI_TENSOR_OBJ_DECLARE(
  conv2d_8_output, AI_STATIC,
  0x0, 0x0,
  AI_SHAPE_INIT(4, 1, 20, 8, 7), AI_STRIDE_INIT(4, 4, 4, 80, 640),
  1, &conv2d_8_output_array, NULL)

/* Tensor #100 */
AI_TENSOR_OBJ_DECLARE(
  depthwise_conv2d_4_output, AI_STATIC,
  0x0, 0x0,
  AI_SHAPE_INIT(4, 1, 20, 4, 4), AI_STRIDE_INIT(4, 4, 4, 80, 320),
  1, &depthwise_conv2d_4_output_array, NULL)

/* Tensor #101 */
AI_TENSOR_OBJ_DECLARE(
  global_average_pooling2d_2_output, AI_STATIC,
  0x0, 0x0,
  AI_SHAPE_INIT(4, 1, 20, 1, 1), AI_STRIDE_INIT(4, 4, 4, 80, 80),
  1, &global_average_pooling2d_2_output_array, NULL)

/* Tensor #102 */
AI_TENSOR_OBJ_DECLARE(
  dense_3_output, AI_STATIC,
  0x0, 0x0,
  AI_SHAPE_INIT(4, 1, 20, 1, 1), AI_STRIDE_INIT(4, 4, 4, 80, 80),
  1, &dense_3_output_array, NULL)

/* Tensor #103 */
AI_TENSOR_OBJ_DECLARE(
  dense_3_nl_output, AI_STATIC,
  0x0, 0x0,
  AI_SHAPE_INIT(4, 1, 20, 1, 1), AI_STRIDE_INIT(4, 4, 4, 80, 80),
  1, &dense_3_nl_output_array, NULL)

/* Tensor #104 */
AI_TENSOR_OBJ_DECLARE(
  dense_4_output, AI_STATIC,
  0x0, 0x0,
  AI_SHAPE_INIT(4, 1, 20, 1, 1), AI_STRIDE_INIT(4, 4, 4, 80, 80),
  1, &dense_4_output_array, NULL)

/* Tensor #105 */
AI_TENSOR_OBJ_DECLARE(
  dense_4_nl_output, AI_STATIC,
  0x0, 0x0,
  AI_SHAPE_INIT(4, 1, 20, 1, 1), AI_STRIDE_INIT(4, 4, 4, 80, 80),
  1, &dense_4_nl_output_array, NULL)

/* Tensor #106 */
AI_TENSOR_OBJ_DECLARE(
  multiply_2_output, AI_STATIC,
  0x0, 0x0,
  AI_SHAPE_INIT(4, 1, 20, 4, 4), AI_STRIDE_INIT(4, 4, 4, 80, 320),
  1, &multiply_2_output_array, NULL)

/* Tensor #107 */
AI_TENSOR_OBJ_DECLARE(
  conv2d_9_output, AI_STATIC,
  0x0, 0x0,
  AI_SHAPE_INIT(4, 1, 12, 4, 4), AI_STRIDE_INIT(4, 4, 4, 48, 192),
  1, &conv2d_9_output_array, NULL)

/* Tensor #108 */
AI_TENSOR_OBJ_DECLARE(
  conv2d_10_output, AI_STATIC,
  0x0, 0x0,
  AI_SHAPE_INIT(4, 1, 60, 4, 4), AI_STRIDE_INIT(4, 4, 4, 240, 960),
  1, &conv2d_10_output_array, NULL)

/* Tensor #109 */
AI_TENSOR_OBJ_DECLARE(
  depthwise_conv2d_5_output, AI_STATIC,
  0x0, 0x0,
  AI_SHAPE_INIT(4, 1, 60, 4, 4), AI_STRIDE_INIT(4, 4, 4, 240, 960),
  1, &depthwise_conv2d_5_output_array, NULL)

/* Tensor #110 */
AI_TENSOR_OBJ_DECLARE(
  global_average_pooling2d_3_output, AI_STATIC,
  0x0, 0x0,
  AI_SHAPE_INIT(4, 1, 60, 1, 1), AI_STRIDE_INIT(4, 4, 4, 240, 240),
  1, &global_average_pooling2d_3_output_array, NULL)

/* Tensor #111 */
AI_TENSOR_OBJ_DECLARE(
  dense_5_output, AI_STATIC,
  0x0, 0x0,
  AI_SHAPE_INIT(4, 1, 60, 1, 1), AI_STRIDE_INIT(4, 4, 4, 240, 240),
  1, &dense_5_output_array, NULL)

/* Tensor #112 */
AI_TENSOR_OBJ_DECLARE(
  dense_5_nl_output, AI_STATIC,
  0x0, 0x0,
  AI_SHAPE_INIT(4, 1, 60, 1, 1), AI_STRIDE_INIT(4, 4, 4, 240, 240),
  1, &dense_5_nl_output_array, NULL)

/* Tensor #113 */
AI_TENSOR_OBJ_DECLARE(
  dense_6_output, AI_STATIC,
  0x0, 0x0,
  AI_SHAPE_INIT(4, 1, 60, 1, 1), AI_STRIDE_INIT(4, 4, 4, 240, 240),
  1, &dense_6_output_array, NULL)

/* Tensor #114 */
AI_TENSOR_OBJ_DECLARE(
  dense_6_nl_output, AI_STATIC,
  0x0, 0x0,
  AI_SHAPE_INIT(4, 1, 60, 1, 1), AI_STRIDE_INIT(4, 4, 4, 240, 240),
  1, &dense_6_nl_output_array, NULL)

/* Tensor #115 */
AI_TENSOR_OBJ_DECLARE(
  multiply_3_output, AI_STATIC,
  0x0, 0x0,
  AI_SHAPE_INIT(4, 1, 60, 4, 4), AI_STRIDE_INIT(4, 4, 4, 240, 960),
  1, &multiply_3_output_array, NULL)

/* Tensor #116 */
AI_TENSOR_OBJ_DECLARE(
  conv2d_11_output, AI_STATIC,
  0x0, 0x0,
  AI_SHAPE_INIT(4, 1, 12, 4, 4), AI_STRIDE_INIT(4, 4, 4, 48, 192),
  1, &conv2d_11_output_array, NULL)

/* Tensor #117 */
AI_TENSOR_OBJ_DECLARE(
  add_2_output, AI_STATIC,
  0x0, 0x0,
  AI_SHAPE_INIT(4, 1, 12, 4, 4), AI_STRIDE_INIT(4, 4, 4, 48, 192),
  1, &add_2_output_array, NULL)

/* Tensor #118 */
AI_TENSOR_OBJ_DECLARE(
  conv2d_12_output, AI_STATIC,
  0x0, 0x0,
  AI_SHAPE_INIT(4, 1, 60, 4, 4), AI_STRIDE_INIT(4, 4, 4, 240, 960),
  1, &conv2d_12_output_array, NULL)

/* Tensor #119 */
AI_TENSOR_OBJ_DECLARE(
  depthwise_conv2d_6_output, AI_STATIC,
  0x0, 0x0,
  AI_SHAPE_INIT(4, 1, 60, 4, 4), AI_STRIDE_INIT(4, 4, 4, 240, 960),
  1, &depthwise_conv2d_6_output_array, NULL)

/* Tensor #120 */
AI_TENSOR_OBJ_DECLARE(
  global_average_pooling2d_4_output, AI_STATIC,
  0x0, 0x0,
  AI_SHAPE_INIT(4, 1, 60, 1, 1), AI_STRIDE_INIT(4, 4, 4, 240, 240),
  1, &global_average_pooling2d_4_output_array, NULL)

/* Tensor #121 */
AI_TENSOR_OBJ_DECLARE(
  dense_7_output, AI_STATIC,
  0x0, 0x0,
  AI_SHAPE_INIT(4, 1, 60, 1, 1), AI_STRIDE_INIT(4, 4, 4, 240, 240),
  1, &dense_7_output_array, NULL)

/* Tensor #122 */
AI_TENSOR_OBJ_DECLARE(
  dense_7_nl_output, AI_STATIC,
  0x0, 0x0,
  AI_SHAPE_INIT(4, 1, 60, 1, 1), AI_STRIDE_INIT(4, 4, 4, 240, 240),
  1, &dense_7_nl_output_array, NULL)

/* Tensor #123 */
AI_TENSOR_OBJ_DECLARE(
  dense_8_output, AI_STATIC,
  0x0, 0x0,
  AI_SHAPE_INIT(4, 1, 60, 1, 1), AI_STRIDE_INIT(4, 4, 4, 240, 240),
  1, &dense_8_output_array, NULL)

/* Tensor #124 */
AI_TENSOR_OBJ_DECLARE(
  dense_8_nl_output, AI_STATIC,
  0x0, 0x0,
  AI_SHAPE_INIT(4, 1, 60, 1, 1), AI_STRIDE_INIT(4, 4, 4, 240, 240),
  1, &dense_8_nl_output_array, NULL)

/* Tensor #125 */
AI_TENSOR_OBJ_DECLARE(
  multiply_4_output, AI_STATIC,
  0x0, 0x0,
  AI_SHAPE_INIT(4, 1, 60, 4, 4), AI_STRIDE_INIT(4, 4, 4, 240, 960),
  1, &multiply_4_output_array, NULL)

/* Tensor #126 */
AI_TENSOR_OBJ_DECLARE(
  conv2d_13_output, AI_STATIC,
  0x0, 0x0,
  AI_SHAPE_INIT(4, 1, 12, 4, 4), AI_STRIDE_INIT(4, 4, 4, 48, 192),
  1, &conv2d_13_output_array, NULL)

/* Tensor #127 */
AI_TENSOR_OBJ_DECLARE(
  add_3_output, AI_STATIC,
  0x0, 0x0,
  AI_SHAPE_INIT(4, 1, 12, 4, 4), AI_STRIDE_INIT(4, 4, 4, 48, 192),
  1, &add_3_output_array, NULL)

/* Tensor #128 */
AI_TENSOR_OBJ_DECLARE(
  conv2d_14_output, AI_STATIC,
  0x0, 0x0,
  AI_SHAPE_INIT(4, 1, 60, 4, 4), AI_STRIDE_INIT(4, 4, 4, 240, 960),
  1, &conv2d_14_output_array, NULL)

/* Tensor #129 */
AI_TENSOR_OBJ_DECLARE(
  depthwise_conv2d_7_output, AI_STATIC,
  0x0, 0x0,
  AI_SHAPE_INIT(4, 1, 60, 2, 2), AI_STRIDE_INIT(4, 4, 4, 240, 480),
  1, &depthwise_conv2d_7_output_array, NULL)

/* Tensor #130 */
AI_TENSOR_OBJ_DECLARE(
  global_average_pooling2d_5_output, AI_STATIC,
  0x0, 0x0,
  AI_SHAPE_INIT(4, 1, 60, 1, 1), AI_STRIDE_INIT(4, 4, 4, 240, 240),
  1, &global_average_pooling2d_5_output_array, NULL)

/* Tensor #131 */
AI_TENSOR_OBJ_DECLARE(
  dense_9_output, AI_STATIC,
  0x0, 0x0,
  AI_SHAPE_INIT(4, 1, 60, 1, 1), AI_STRIDE_INIT(4, 4, 4, 240, 240),
  1, &dense_9_output_array, NULL)

/* Tensor #132 */
AI_TENSOR_OBJ_DECLARE(
  dense_9_nl_output, AI_STATIC,
  0x0, 0x0,
  AI_SHAPE_INIT(4, 1, 60, 1, 1), AI_STRIDE_INIT(4, 4, 4, 240, 240),
  1, &dense_9_nl_output_array, NULL)

/* Tensor #133 */
AI_TENSOR_OBJ_DECLARE(
  dense_10_output, AI_STATIC,
  0x0, 0x0,
  AI_SHAPE_INIT(4, 1, 60, 1, 1), AI_STRIDE_INIT(4, 4, 4, 240, 240),
  1, &dense_10_output_array, NULL)

/* Tensor #134 */
AI_TENSOR_OBJ_DECLARE(
  dense_10_nl_output, AI_STATIC,
  0x0, 0x0,
  AI_SHAPE_INIT(4, 1, 60, 1, 1), AI_STRIDE_INIT(4, 4, 4, 240, 240),
  1, &dense_10_nl_output_array, NULL)

/* Tensor #135 */
AI_TENSOR_OBJ_DECLARE(
  multiply_5_output, AI_STATIC,
  0x0, 0x0,
  AI_SHAPE_INIT(4, 1, 60, 2, 2), AI_STRIDE_INIT(4, 4, 4, 240, 480),
  1, &multiply_5_output_array, NULL)

/* Tensor #136 */
AI_TENSOR_OBJ_DECLARE(
  conv2d_15_output, AI_STATIC,
  0x0, 0x0,
  AI_SHAPE_INIT(4, 1, 20, 2, 2), AI_STRIDE_INIT(4, 4, 4, 80, 160),
  1, &conv2d_15_output_array, NULL)

/* Tensor #137 */
AI_TENSOR_OBJ_DECLARE(
  conv2d_16_output, AI_STATIC,
  0x0, 0x0,
  AI_SHAPE_INIT(4, 1, 60, 2, 2), AI_STRIDE_INIT(4, 4, 4, 240, 480),
  1, &conv2d_16_output_array, NULL)

/* Tensor #138 */
AI_TENSOR_OBJ_DECLARE(
  depthwise_conv2d_8_output, AI_STATIC,
  0x0, 0x0,
  AI_SHAPE_INIT(4, 1, 60, 2, 2), AI_STRIDE_INIT(4, 4, 4, 240, 480),
  1, &depthwise_conv2d_8_output_array, NULL)

/* Tensor #139 */
AI_TENSOR_OBJ_DECLARE(
  global_average_pooling2d_6_output, AI_STATIC,
  0x0, 0x0,
  AI_SHAPE_INIT(4, 1, 60, 1, 1), AI_STRIDE_INIT(4, 4, 4, 240, 240),
  1, &global_average_pooling2d_6_output_array, NULL)

/* Tensor #140 */
AI_TENSOR_OBJ_DECLARE(
  dense_11_output, AI_STATIC,
  0x0, 0x0,
  AI_SHAPE_INIT(4, 1, 60, 1, 1), AI_STRIDE_INIT(4, 4, 4, 240, 240),
  1, &dense_11_output_array, NULL)

/* Tensor #141 */
AI_TENSOR_OBJ_DECLARE(
  dense_11_nl_output, AI_STATIC,
  0x0, 0x0,
  AI_SHAPE_INIT(4, 1, 60, 1, 1), AI_STRIDE_INIT(4, 4, 4, 240, 240),
  1, &dense_11_nl_output_array, NULL)

/* Tensor #142 */
AI_TENSOR_OBJ_DECLARE(
  dense_12_output, AI_STATIC,
  0x0, 0x0,
  AI_SHAPE_INIT(4, 1, 60, 1, 1), AI_STRIDE_INIT(4, 4, 4, 240, 240),
  1, &dense_12_output_array, NULL)

/* Tensor #143 */
AI_TENSOR_OBJ_DECLARE(
  dense_12_nl_output, AI_STATIC,
  0x0, 0x0,
  AI_SHAPE_INIT(4, 1, 60, 1, 1), AI_STRIDE_INIT(4, 4, 4, 240, 240),
  1, &dense_12_nl_output_array, NULL)

/* Tensor #144 */
AI_TENSOR_OBJ_DECLARE(
  multiply_6_output, AI_STATIC,
  0x0, 0x0,
  AI_SHAPE_INIT(4, 1, 60, 2, 2), AI_STRIDE_INIT(4, 4, 4, 240, 480),
  1, &multiply_6_output_array, NULL)

/* Tensor #145 */
AI_TENSOR_OBJ_DECLARE(
  conv2d_17_output, AI_STATIC,
  0x0, 0x0,
  AI_SHAPE_INIT(4, 1, 20, 2, 2), AI_STRIDE_INIT(4, 4, 4, 80, 160),
  1, &conv2d_17_output_array, NULL)

/* Tensor #146 */
AI_TENSOR_OBJ_DECLARE(
  add_4_output, AI_STATIC,
  0x0, 0x0,
  AI_SHAPE_INIT(4, 1, 20, 2, 2), AI_STRIDE_INIT(4, 4, 4, 80, 160),
  1, &add_4_output_array, NULL)

/* Tensor #147 */
AI_TENSOR_OBJ_DECLARE(
  conv2d_18_output, AI_STATIC,
  0x0, 0x0,
  AI_SHAPE_INIT(4, 1, 60, 1, 1), AI_STRIDE_INIT(4, 4, 4, 240, 240),
  1, &conv2d_18_output_array, NULL)

/* Tensor #148 */
AI_TENSOR_OBJ_DECLARE(
  conv2d_19_output, AI_STATIC,
  0x0, 0x0,
  AI_SHAPE_INIT(4, 1, 120, 1, 1), AI_STRIDE_INIT(4, 4, 4, 480, 480),
  1, &conv2d_19_output_array, NULL)

/* Tensor #149 */
AI_TENSOR_OBJ_DECLARE(
  conv2d_20_output, AI_STATIC,
  0x0, 0x0,
  AI_SHAPE_INIT(4, 1, 4, 1, 1), AI_STRIDE_INIT(4, 4, 4, 16, 16),
  1, &conv2d_20_output_array, NULL)



/**  Layer declarations section  **********************************************/



AI_STATIC_CONST ai_float conv2d_1_nl_params_data[] = { 0, 6.0 };
AI_ARRAY_OBJ_DECLARE(
    conv2d_1_nl_params, AI_ARRAY_FORMAT_FLOAT,
    conv2d_1_nl_params_data, conv2d_1_nl_params_data, 2, AI_STATIC_CONST)
AI_TENSOR_CHAIN_OBJ_DECLARE(
  conv2d_1_chain, AI_STATIC_CONST, 4,
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &input_1_output),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &conv2d_1_output),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 3, &conv2d_1_weights, &conv2d_1_bias, NULL),
  AI_TENSOR_LIST_OBJ_EMPTY
)

AI_LAYER_OBJ_DECLARE(
  conv2d_1_layer, 1,
  CONV2D_TYPE,
  conv2d, forward_conv2d,
  &AI_NET_OBJ_INSTANCE, &conv2d_2_layer, AI_STATIC,
  .tensors = &conv2d_1_chain, 
  .groups = 1, 
  .nl_params = &conv2d_1_nl_params, 
  .nl_func = nl_func_clip_array_f32, 
  .filter_stride = AI_SHAPE_2D_INIT(2, 2), 
  .dilation = AI_SHAPE_2D_INIT(1, 1), 
  .filter_pad = AI_SHAPE_INIT(4, 0, 0, 2, 2), 
)


AI_STATIC_CONST ai_float conv2d_2_nl_params_data[] = { 0, 6.0 };
AI_ARRAY_OBJ_DECLARE(
    conv2d_2_nl_params, AI_ARRAY_FORMAT_FLOAT,
    conv2d_2_nl_params_data, conv2d_2_nl_params_data, 2, AI_STATIC_CONST)
AI_TENSOR_CHAIN_OBJ_DECLARE(
  conv2d_2_chain, AI_STATIC_CONST, 4,
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &conv2d_1_output),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &conv2d_2_output),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 3, &conv2d_2_weights, &conv2d_2_bias, NULL),
  AI_TENSOR_LIST_OBJ_EMPTY
)

AI_LAYER_OBJ_DECLARE(
  conv2d_2_layer, 4,
  CONV2D_TYPE,
  conv2d, forward_conv2d,
  &AI_NET_OBJ_INSTANCE, &depthwise_conv2d_1_layer, AI_STATIC,
  .tensors = &conv2d_2_chain, 
  .groups = 1, 
  .nl_params = &conv2d_2_nl_params, 
  .nl_func = nl_func_clip_array_f32, 
  .filter_stride = AI_SHAPE_2D_INIT(1, 1), 
  .dilation = AI_SHAPE_2D_INIT(1, 1), 
  .filter_pad = AI_SHAPE_INIT(4, 0, 0, 0, 0), 
)


AI_STATIC_CONST ai_float depthwise_conv2d_1_nl_params_data[] = { 0, 6.0 };
AI_ARRAY_OBJ_DECLARE(
    depthwise_conv2d_1_nl_params, AI_ARRAY_FORMAT_FLOAT,
    depthwise_conv2d_1_nl_params_data, depthwise_conv2d_1_nl_params_data, 2, AI_STATIC_CONST)
AI_TENSOR_CHAIN_OBJ_DECLARE(
  depthwise_conv2d_1_chain, AI_STATIC_CONST, 4,
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &conv2d_2_output),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &depthwise_conv2d_1_output),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 3, &depthwise_conv2d_1_weights, &depthwise_conv2d_1_bias, NULL),
  AI_TENSOR_LIST_OBJ_EMPTY
)

AI_LAYER_OBJ_DECLARE(
  depthwise_conv2d_1_layer, 7,
  CONV2D_TYPE,
  conv2d, forward_conv2d,
  &AI_NET_OBJ_INSTANCE, &global_average_pooling2d_1_layer, AI_STATIC,
  .tensors = &depthwise_conv2d_1_chain, 
  .groups = 6, 
  .nl_params = &depthwise_conv2d_1_nl_params, 
  .nl_func = nl_func_clip_array_f32, 
  .filter_stride = AI_SHAPE_2D_INIT(2, 2), 
  .dilation = AI_SHAPE_2D_INIT(1, 1), 
  .filter_pad = AI_SHAPE_INIT(4, 1, 0, 1, 2), 
)

AI_TENSOR_CHAIN_OBJ_DECLARE(
  global_average_pooling2d_1_chain, AI_STATIC_CONST, 4,
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &depthwise_conv2d_1_output),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &global_average_pooling2d_1_output),
  AI_TENSOR_LIST_OBJ_EMPTY,
  AI_TENSOR_LIST_OBJ_EMPTY
)

AI_LAYER_OBJ_DECLARE(
  global_average_pooling2d_1_layer, 10,
  POOL_TYPE,
  pool, forward_ap,
  &AI_NET_OBJ_INSTANCE, &dense_1_layer, AI_STATIC,
  .tensors = &global_average_pooling2d_1_chain, 
  .pool_size = AI_SHAPE_2D_INIT(15, 13), 
  .pool_stride = AI_SHAPE_2D_INIT(15, 13), 
  .pool_pad = AI_SHAPE_INIT(4, 0, 0, 0, 0), 
)

AI_TENSOR_CHAIN_OBJ_DECLARE(
  dense_1_chain, AI_STATIC_CONST, 4,
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &global_average_pooling2d_1_output),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &dense_1_output),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 2, &dense_1_weights, &dense_1_bias),
  AI_TENSOR_LIST_OBJ_EMPTY
)

AI_LAYER_OBJ_DECLARE(
  dense_1_layer, 11,
  DENSE_TYPE,
  dense, forward_dense,
  &AI_NET_OBJ_INSTANCE, &dense_1_nl_layer, AI_STATIC,
  .tensors = &dense_1_chain, 
)

AI_TENSOR_CHAIN_OBJ_DECLARE(
  dense_1_nl_chain, AI_STATIC_CONST, 4,
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &dense_1_output),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &dense_1_nl_output),
  AI_TENSOR_LIST_OBJ_EMPTY,
  AI_TENSOR_LIST_OBJ_EMPTY
)

AI_LAYER_OBJ_DECLARE(
  dense_1_nl_layer, 11,
  NL_TYPE,
  nl, forward_relu,
  &AI_NET_OBJ_INSTANCE, &dense_2_layer, AI_STATIC,
  .tensors = &dense_1_nl_chain, 
)

AI_TENSOR_CHAIN_OBJ_DECLARE(
  dense_2_chain, AI_STATIC_CONST, 4,
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &dense_1_nl_output),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &dense_2_output),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 2, &dense_2_weights, &dense_2_bias),
  AI_TENSOR_LIST_OBJ_EMPTY
)

AI_LAYER_OBJ_DECLARE(
  dense_2_layer, 12,
  DENSE_TYPE,
  dense, forward_dense,
  &AI_NET_OBJ_INSTANCE, &dense_2_nl_layer, AI_STATIC,
  .tensors = &dense_2_chain, 
)

AI_TENSOR_CHAIN_OBJ_DECLARE(
  dense_2_nl_chain, AI_STATIC_CONST, 4,
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &dense_2_output),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &dense_2_nl_output),
  AI_TENSOR_LIST_OBJ_EMPTY,
  AI_TENSOR_LIST_OBJ_EMPTY
)

AI_LAYER_OBJ_DECLARE(
  dense_2_nl_layer, 12,
  NL_TYPE,
  nl, forward_hard_sigmoid,
  &AI_NET_OBJ_INSTANCE, &multiply_1_layer, AI_STATIC,
  .tensors = &dense_2_nl_chain, 
)

AI_TENSOR_CHAIN_OBJ_DECLARE(
  multiply_1_chain, AI_STATIC_CONST, 4,
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 2, &depthwise_conv2d_1_output, &dense_2_nl_output),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &multiply_1_output),
  AI_TENSOR_LIST_OBJ_EMPTY,
  AI_TENSOR_LIST_OBJ_EMPTY
)

AI_LAYER_OBJ_DECLARE(
  multiply_1_layer, 14,
  ELTWISE_TYPE,
  eltwise, forward_eltwise,
  &AI_NET_OBJ_INSTANCE, &conv2d_3_layer, AI_STATIC,
  .tensors = &multiply_1_chain, 
  .operation = ai_mul, 
)

AI_TENSOR_CHAIN_OBJ_DECLARE(
  conv2d_3_chain, AI_STATIC_CONST, 4,
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &multiply_1_output),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &conv2d_3_output),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 3, &conv2d_3_weights, &conv2d_3_bias, NULL),
  AI_TENSOR_LIST_OBJ_EMPTY
)

AI_LAYER_OBJ_DECLARE(
  conv2d_3_layer, 15,
  CONV2D_TYPE,
  conv2d, forward_conv2d,
  &AI_NET_OBJ_INSTANCE, &conv2d_4_layer, AI_STATIC,
  .tensors = &conv2d_3_chain, 
  .groups = 1, 
  .nl_func = NULL, 
  .filter_stride = AI_SHAPE_2D_INIT(1, 1), 
  .dilation = AI_SHAPE_2D_INIT(1, 1), 
  .filter_pad = AI_SHAPE_INIT(4, 0, 0, 0, 0), 
)


AI_STATIC_CONST ai_float conv2d_4_nl_params_data[] = { 0, 6.0 };
AI_ARRAY_OBJ_DECLARE(
    conv2d_4_nl_params, AI_ARRAY_FORMAT_FLOAT,
    conv2d_4_nl_params_data, conv2d_4_nl_params_data, 2, AI_STATIC_CONST)
AI_TENSOR_CHAIN_OBJ_DECLARE(
  conv2d_4_chain, AI_STATIC_CONST, 4,
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &conv2d_3_output),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &conv2d_4_output),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 3, &conv2d_4_weights, &conv2d_4_bias, NULL),
  AI_TENSOR_LIST_OBJ_EMPTY
)

AI_LAYER_OBJ_DECLARE(
  conv2d_4_layer, 17,
  CONV2D_TYPE,
  conv2d, forward_conv2d,
  &AI_NET_OBJ_INSTANCE, &depthwise_conv2d_2_layer, AI_STATIC,
  .tensors = &conv2d_4_chain, 
  .groups = 1, 
  .nl_params = &conv2d_4_nl_params, 
  .nl_func = nl_func_clip_array_f32, 
  .filter_stride = AI_SHAPE_2D_INIT(1, 1), 
  .dilation = AI_SHAPE_2D_INIT(1, 1), 
  .filter_pad = AI_SHAPE_INIT(4, 0, 0, 0, 0), 
)


AI_STATIC_CONST ai_float depthwise_conv2d_2_nl_params_data[] = { 0, 6.0 };
AI_ARRAY_OBJ_DECLARE(
    depthwise_conv2d_2_nl_params, AI_ARRAY_FORMAT_FLOAT,
    depthwise_conv2d_2_nl_params_data, depthwise_conv2d_2_nl_params_data, 2, AI_STATIC_CONST)
AI_TENSOR_CHAIN_OBJ_DECLARE(
  depthwise_conv2d_2_chain, AI_STATIC_CONST, 4,
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &conv2d_4_output),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &depthwise_conv2d_2_output),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 3, &depthwise_conv2d_2_weights, &depthwise_conv2d_2_bias, NULL),
  AI_TENSOR_LIST_OBJ_EMPTY
)

AI_LAYER_OBJ_DECLARE(
  depthwise_conv2d_2_layer, 20,
  CONV2D_TYPE,
  conv2d, forward_conv2d,
  &AI_NET_OBJ_INSTANCE, &conv2d_5_layer, AI_STATIC,
  .tensors = &depthwise_conv2d_2_chain, 
  .groups = 20, 
  .nl_params = &depthwise_conv2d_2_nl_params, 
  .nl_func = nl_func_clip_array_f32, 
  .filter_stride = AI_SHAPE_2D_INIT(2, 2), 
  .dilation = AI_SHAPE_2D_INIT(1, 1), 
  .filter_pad = AI_SHAPE_INIT(4, 1, 1, 1, 1), 
)

AI_TENSOR_CHAIN_OBJ_DECLARE(
  conv2d_5_chain, AI_STATIC_CONST, 4,
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &depthwise_conv2d_2_output),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &conv2d_5_output),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 3, &conv2d_5_weights, &conv2d_5_bias, NULL),
  AI_TENSOR_LIST_OBJ_EMPTY
)

AI_LAYER_OBJ_DECLARE(
  conv2d_5_layer, 23,
  CONV2D_TYPE,
  conv2d, forward_conv2d,
  &AI_NET_OBJ_INSTANCE, &conv2d_6_layer, AI_STATIC,
  .tensors = &conv2d_5_chain, 
  .groups = 1, 
  .nl_func = NULL, 
  .filter_stride = AI_SHAPE_2D_INIT(1, 1), 
  .dilation = AI_SHAPE_2D_INIT(1, 1), 
  .filter_pad = AI_SHAPE_INIT(4, 0, 0, 0, 0), 
)


AI_STATIC_CONST ai_float conv2d_6_nl_params_data[] = { 0, 6.0 };
AI_ARRAY_OBJ_DECLARE(
    conv2d_6_nl_params, AI_ARRAY_FORMAT_FLOAT,
    conv2d_6_nl_params_data, conv2d_6_nl_params_data, 2, AI_STATIC_CONST)
AI_TENSOR_CHAIN_OBJ_DECLARE(
  conv2d_6_chain, AI_STATIC_CONST, 4,
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &conv2d_5_output),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &conv2d_6_output),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 3, &conv2d_6_weights, &conv2d_6_bias, NULL),
  AI_TENSOR_LIST_OBJ_EMPTY
)

AI_LAYER_OBJ_DECLARE(
  conv2d_6_layer, 25,
  CONV2D_TYPE,
  conv2d, forward_conv2d,
  &AI_NET_OBJ_INSTANCE, &depthwise_conv2d_3_layer, AI_STATIC,
  .tensors = &conv2d_6_chain, 
  .groups = 1, 
  .nl_params = &conv2d_6_nl_params, 
  .nl_func = nl_func_clip_array_f32, 
  .filter_stride = AI_SHAPE_2D_INIT(1, 1), 
  .dilation = AI_SHAPE_2D_INIT(1, 1), 
  .filter_pad = AI_SHAPE_INIT(4, 0, 0, 0, 0), 
)


AI_STATIC_CONST ai_float depthwise_conv2d_3_nl_params_data[] = { 0, 6.0 };
AI_ARRAY_OBJ_DECLARE(
    depthwise_conv2d_3_nl_params, AI_ARRAY_FORMAT_FLOAT,
    depthwise_conv2d_3_nl_params_data, depthwise_conv2d_3_nl_params_data, 2, AI_STATIC_CONST)
AI_TENSOR_CHAIN_OBJ_DECLARE(
  depthwise_conv2d_3_chain, AI_STATIC_CONST, 4,
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &conv2d_6_output),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &depthwise_conv2d_3_output),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 3, &depthwise_conv2d_3_weights, &depthwise_conv2d_3_bias, NULL),
  AI_TENSOR_LIST_OBJ_EMPTY
)

AI_LAYER_OBJ_DECLARE(
  depthwise_conv2d_3_layer, 28,
  CONV2D_TYPE,
  conv2d, forward_conv2d,
  &AI_NET_OBJ_INSTANCE, &conv2d_7_layer, AI_STATIC,
  .tensors = &depthwise_conv2d_3_chain, 
  .groups = 20, 
  .nl_params = &depthwise_conv2d_3_nl_params, 
  .nl_func = nl_func_clip_array_f32, 
  .filter_stride = AI_SHAPE_2D_INIT(1, 1), 
  .dilation = AI_SHAPE_2D_INIT(1, 1), 
  .filter_pad = AI_SHAPE_INIT(4, 1, 1, 1, 1), 
)

AI_TENSOR_CHAIN_OBJ_DECLARE(
  conv2d_7_chain, AI_STATIC_CONST, 4,
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &depthwise_conv2d_3_output),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &conv2d_7_output),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 3, &conv2d_7_weights, &conv2d_7_bias, NULL),
  AI_TENSOR_LIST_OBJ_EMPTY
)

AI_LAYER_OBJ_DECLARE(
  conv2d_7_layer, 31,
  CONV2D_TYPE,
  conv2d, forward_conv2d,
  &AI_NET_OBJ_INSTANCE, &add_1_layer, AI_STATIC,
  .tensors = &conv2d_7_chain, 
  .groups = 1, 
  .nl_func = NULL, 
  .filter_stride = AI_SHAPE_2D_INIT(1, 1), 
  .dilation = AI_SHAPE_2D_INIT(1, 1), 
  .filter_pad = AI_SHAPE_INIT(4, 0, 0, 0, 0), 
)

AI_TENSOR_CHAIN_OBJ_DECLARE(
  add_1_chain, AI_STATIC_CONST, 4,
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 2, &conv2d_7_output, &conv2d_5_output),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &add_1_output),
  AI_TENSOR_LIST_OBJ_EMPTY,
  AI_TENSOR_LIST_OBJ_EMPTY
)

AI_LAYER_OBJ_DECLARE(
  add_1_layer, 33,
  ELTWISE_TYPE,
  eltwise, forward_eltwise,
  &AI_NET_OBJ_INSTANCE, &conv2d_8_layer, AI_STATIC,
  .tensors = &add_1_chain, 
  .operation = ai_sum, 
)


AI_STATIC_CONST ai_float conv2d_8_nl_params_data[] = { 0, 6.0 };
AI_ARRAY_OBJ_DECLARE(
    conv2d_8_nl_params, AI_ARRAY_FORMAT_FLOAT,
    conv2d_8_nl_params_data, conv2d_8_nl_params_data, 2, AI_STATIC_CONST)
AI_TENSOR_CHAIN_OBJ_DECLARE(
  conv2d_8_chain, AI_STATIC_CONST, 4,
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &add_1_output),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &conv2d_8_output),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 3, &conv2d_8_weights, &conv2d_8_bias, NULL),
  AI_TENSOR_LIST_OBJ_EMPTY
)

AI_LAYER_OBJ_DECLARE(
  conv2d_8_layer, 34,
  CONV2D_TYPE,
  conv2d, forward_conv2d,
  &AI_NET_OBJ_INSTANCE, &depthwise_conv2d_4_layer, AI_STATIC,
  .tensors = &conv2d_8_chain, 
  .groups = 1, 
  .nl_params = &conv2d_8_nl_params, 
  .nl_func = nl_func_clip_array_f32, 
  .filter_stride = AI_SHAPE_2D_INIT(1, 1), 
  .dilation = AI_SHAPE_2D_INIT(1, 1), 
  .filter_pad = AI_SHAPE_INIT(4, 0, 0, 0, 0), 
)


AI_STATIC_CONST ai_float depthwise_conv2d_4_nl_params_data[] = { 0, 6.0 };
AI_ARRAY_OBJ_DECLARE(
    depthwise_conv2d_4_nl_params, AI_ARRAY_FORMAT_FLOAT,
    depthwise_conv2d_4_nl_params_data, depthwise_conv2d_4_nl_params_data, 2, AI_STATIC_CONST)
AI_TENSOR_CHAIN_OBJ_DECLARE(
  depthwise_conv2d_4_chain, AI_STATIC_CONST, 4,
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &conv2d_8_output),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &depthwise_conv2d_4_output),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 3, &depthwise_conv2d_4_weights, &depthwise_conv2d_4_bias, NULL),
  AI_TENSOR_LIST_OBJ_EMPTY
)

AI_LAYER_OBJ_DECLARE(
  depthwise_conv2d_4_layer, 37,
  CONV2D_TYPE,
  conv2d, forward_conv2d,
  &AI_NET_OBJ_INSTANCE, &global_average_pooling2d_2_layer, AI_STATIC,
  .tensors = &depthwise_conv2d_4_chain, 
  .groups = 20, 
  .nl_params = &depthwise_conv2d_4_nl_params, 
  .nl_func = nl_func_clip_array_f32, 
  .filter_stride = AI_SHAPE_2D_INIT(2, 2), 
  .dilation = AI_SHAPE_2D_INIT(1, 1), 
  .filter_pad = AI_SHAPE_INIT(4, 2, 1, 2, 3), 
)

AI_TENSOR_CHAIN_OBJ_DECLARE(
  global_average_pooling2d_2_chain, AI_STATIC_CONST, 4,
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &depthwise_conv2d_4_output),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &global_average_pooling2d_2_output),
  AI_TENSOR_LIST_OBJ_EMPTY,
  AI_TENSOR_LIST_OBJ_EMPTY
)

AI_LAYER_OBJ_DECLARE(
  global_average_pooling2d_2_layer, 40,
  POOL_TYPE,
  pool, forward_ap,
  &AI_NET_OBJ_INSTANCE, &dense_3_layer, AI_STATIC,
  .tensors = &global_average_pooling2d_2_chain, 
  .pool_size = AI_SHAPE_2D_INIT(4, 4), 
  .pool_stride = AI_SHAPE_2D_INIT(4, 4), 
  .pool_pad = AI_SHAPE_INIT(4, 0, 0, 0, 0), 
)

AI_TENSOR_CHAIN_OBJ_DECLARE(
  dense_3_chain, AI_STATIC_CONST, 4,
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &global_average_pooling2d_2_output),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &dense_3_output),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 2, &dense_3_weights, &dense_3_bias),
  AI_TENSOR_LIST_OBJ_EMPTY
)

AI_LAYER_OBJ_DECLARE(
  dense_3_layer, 41,
  DENSE_TYPE,
  dense, forward_dense,
  &AI_NET_OBJ_INSTANCE, &dense_3_nl_layer, AI_STATIC,
  .tensors = &dense_3_chain, 
)

AI_TENSOR_CHAIN_OBJ_DECLARE(
  dense_3_nl_chain, AI_STATIC_CONST, 4,
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &dense_3_output),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &dense_3_nl_output),
  AI_TENSOR_LIST_OBJ_EMPTY,
  AI_TENSOR_LIST_OBJ_EMPTY
)

AI_LAYER_OBJ_DECLARE(
  dense_3_nl_layer, 41,
  NL_TYPE,
  nl, forward_relu,
  &AI_NET_OBJ_INSTANCE, &dense_4_layer, AI_STATIC,
  .tensors = &dense_3_nl_chain, 
)

AI_TENSOR_CHAIN_OBJ_DECLARE(
  dense_4_chain, AI_STATIC_CONST, 4,
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &dense_3_nl_output),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &dense_4_output),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 2, &dense_4_weights, &dense_4_bias),
  AI_TENSOR_LIST_OBJ_EMPTY
)

AI_LAYER_OBJ_DECLARE(
  dense_4_layer, 42,
  DENSE_TYPE,
  dense, forward_dense,
  &AI_NET_OBJ_INSTANCE, &dense_4_nl_layer, AI_STATIC,
  .tensors = &dense_4_chain, 
)

AI_TENSOR_CHAIN_OBJ_DECLARE(
  dense_4_nl_chain, AI_STATIC_CONST, 4,
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &dense_4_output),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &dense_4_nl_output),
  AI_TENSOR_LIST_OBJ_EMPTY,
  AI_TENSOR_LIST_OBJ_EMPTY
)

AI_LAYER_OBJ_DECLARE(
  dense_4_nl_layer, 42,
  NL_TYPE,
  nl, forward_hard_sigmoid,
  &AI_NET_OBJ_INSTANCE, &multiply_2_layer, AI_STATIC,
  .tensors = &dense_4_nl_chain, 
)

AI_TENSOR_CHAIN_OBJ_DECLARE(
  multiply_2_chain, AI_STATIC_CONST, 4,
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 2, &depthwise_conv2d_4_output, &dense_4_nl_output),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &multiply_2_output),
  AI_TENSOR_LIST_OBJ_EMPTY,
  AI_TENSOR_LIST_OBJ_EMPTY
)

AI_LAYER_OBJ_DECLARE(
  multiply_2_layer, 44,
  ELTWISE_TYPE,
  eltwise, forward_eltwise,
  &AI_NET_OBJ_INSTANCE, &conv2d_9_layer, AI_STATIC,
  .tensors = &multiply_2_chain, 
  .operation = ai_mul, 
)

AI_TENSOR_CHAIN_OBJ_DECLARE(
  conv2d_9_chain, AI_STATIC_CONST, 4,
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &multiply_2_output),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &conv2d_9_output),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 3, &conv2d_9_weights, &conv2d_9_bias, NULL),
  AI_TENSOR_LIST_OBJ_EMPTY
)

AI_LAYER_OBJ_DECLARE(
  conv2d_9_layer, 45,
  CONV2D_TYPE,
  conv2d, forward_conv2d,
  &AI_NET_OBJ_INSTANCE, &conv2d_10_layer, AI_STATIC,
  .tensors = &conv2d_9_chain, 
  .groups = 1, 
  .nl_func = NULL, 
  .filter_stride = AI_SHAPE_2D_INIT(1, 1), 
  .dilation = AI_SHAPE_2D_INIT(1, 1), 
  .filter_pad = AI_SHAPE_INIT(4, 0, 0, 0, 0), 
)


AI_STATIC_CONST ai_float conv2d_10_nl_params_data[] = { 0, 6.0 };
AI_ARRAY_OBJ_DECLARE(
    conv2d_10_nl_params, AI_ARRAY_FORMAT_FLOAT,
    conv2d_10_nl_params_data, conv2d_10_nl_params_data, 2, AI_STATIC_CONST)
AI_TENSOR_CHAIN_OBJ_DECLARE(
  conv2d_10_chain, AI_STATIC_CONST, 4,
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &conv2d_9_output),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &conv2d_10_output),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 3, &conv2d_10_weights, &conv2d_10_bias, NULL),
  AI_TENSOR_LIST_OBJ_EMPTY
)

AI_LAYER_OBJ_DECLARE(
  conv2d_10_layer, 47,
  CONV2D_TYPE,
  conv2d, forward_conv2d,
  &AI_NET_OBJ_INSTANCE, &depthwise_conv2d_5_layer, AI_STATIC,
  .tensors = &conv2d_10_chain, 
  .groups = 1, 
  .nl_params = &conv2d_10_nl_params, 
  .nl_func = nl_func_clip_array_f32, 
  .filter_stride = AI_SHAPE_2D_INIT(1, 1), 
  .dilation = AI_SHAPE_2D_INIT(1, 1), 
  .filter_pad = AI_SHAPE_INIT(4, 0, 0, 0, 0), 
)


AI_STATIC_CONST ai_float depthwise_conv2d_5_nl_params_data[] = { 0, 6.0 };
AI_ARRAY_OBJ_DECLARE(
    depthwise_conv2d_5_nl_params, AI_ARRAY_FORMAT_FLOAT,
    depthwise_conv2d_5_nl_params_data, depthwise_conv2d_5_nl_params_data, 2, AI_STATIC_CONST)
AI_TENSOR_CHAIN_OBJ_DECLARE(
  depthwise_conv2d_5_chain, AI_STATIC_CONST, 4,
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &conv2d_10_output),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &depthwise_conv2d_5_output),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 3, &depthwise_conv2d_5_weights, &depthwise_conv2d_5_bias, NULL),
  AI_TENSOR_LIST_OBJ_EMPTY
)

AI_LAYER_OBJ_DECLARE(
  depthwise_conv2d_5_layer, 50,
  CONV2D_TYPE,
  conv2d, forward_conv2d,
  &AI_NET_OBJ_INSTANCE, &global_average_pooling2d_3_layer, AI_STATIC,
  .tensors = &depthwise_conv2d_5_chain, 
  .groups = 60, 
  .nl_params = &depthwise_conv2d_5_nl_params, 
  .nl_func = nl_func_clip_array_f32, 
  .filter_stride = AI_SHAPE_2D_INIT(1, 1), 
  .dilation = AI_SHAPE_2D_INIT(1, 1), 
  .filter_pad = AI_SHAPE_INIT(4, 2, 2, 2, 2), 
)

AI_TENSOR_CHAIN_OBJ_DECLARE(
  global_average_pooling2d_3_chain, AI_STATIC_CONST, 4,
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &depthwise_conv2d_5_output),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &global_average_pooling2d_3_output),
  AI_TENSOR_LIST_OBJ_EMPTY,
  AI_TENSOR_LIST_OBJ_EMPTY
)

AI_LAYER_OBJ_DECLARE(
  global_average_pooling2d_3_layer, 53,
  POOL_TYPE,
  pool, forward_ap,
  &AI_NET_OBJ_INSTANCE, &dense_5_layer, AI_STATIC,
  .tensors = &global_average_pooling2d_3_chain, 
  .pool_size = AI_SHAPE_2D_INIT(4, 4), 
  .pool_stride = AI_SHAPE_2D_INIT(4, 4), 
  .pool_pad = AI_SHAPE_INIT(4, 0, 0, 0, 0), 
)

AI_TENSOR_CHAIN_OBJ_DECLARE(
  dense_5_chain, AI_STATIC_CONST, 4,
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &global_average_pooling2d_3_output),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &dense_5_output),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 2, &dense_5_weights, &dense_5_bias),
  AI_TENSOR_LIST_OBJ_EMPTY
)

AI_LAYER_OBJ_DECLARE(
  dense_5_layer, 54,
  DENSE_TYPE,
  dense, forward_dense,
  &AI_NET_OBJ_INSTANCE, &dense_5_nl_layer, AI_STATIC,
  .tensors = &dense_5_chain, 
)

AI_TENSOR_CHAIN_OBJ_DECLARE(
  dense_5_nl_chain, AI_STATIC_CONST, 4,
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &dense_5_output),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &dense_5_nl_output),
  AI_TENSOR_LIST_OBJ_EMPTY,
  AI_TENSOR_LIST_OBJ_EMPTY
)

AI_LAYER_OBJ_DECLARE(
  dense_5_nl_layer, 54,
  NL_TYPE,
  nl, forward_relu,
  &AI_NET_OBJ_INSTANCE, &dense_6_layer, AI_STATIC,
  .tensors = &dense_5_nl_chain, 
)

AI_TENSOR_CHAIN_OBJ_DECLARE(
  dense_6_chain, AI_STATIC_CONST, 4,
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &dense_5_nl_output),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &dense_6_output),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 2, &dense_6_weights, &dense_6_bias),
  AI_TENSOR_LIST_OBJ_EMPTY
)

AI_LAYER_OBJ_DECLARE(
  dense_6_layer, 55,
  DENSE_TYPE,
  dense, forward_dense,
  &AI_NET_OBJ_INSTANCE, &dense_6_nl_layer, AI_STATIC,
  .tensors = &dense_6_chain, 
)

AI_TENSOR_CHAIN_OBJ_DECLARE(
  dense_6_nl_chain, AI_STATIC_CONST, 4,
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &dense_6_output),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &dense_6_nl_output),
  AI_TENSOR_LIST_OBJ_EMPTY,
  AI_TENSOR_LIST_OBJ_EMPTY
)

AI_LAYER_OBJ_DECLARE(
  dense_6_nl_layer, 55,
  NL_TYPE,
  nl, forward_hard_sigmoid,
  &AI_NET_OBJ_INSTANCE, &multiply_3_layer, AI_STATIC,
  .tensors = &dense_6_nl_chain, 
)

AI_TENSOR_CHAIN_OBJ_DECLARE(
  multiply_3_chain, AI_STATIC_CONST, 4,
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 2, &depthwise_conv2d_5_output, &dense_6_nl_output),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &multiply_3_output),
  AI_TENSOR_LIST_OBJ_EMPTY,
  AI_TENSOR_LIST_OBJ_EMPTY
)

AI_LAYER_OBJ_DECLARE(
  multiply_3_layer, 57,
  ELTWISE_TYPE,
  eltwise, forward_eltwise,
  &AI_NET_OBJ_INSTANCE, &conv2d_11_layer, AI_STATIC,
  .tensors = &multiply_3_chain, 
  .operation = ai_mul, 
)

AI_TENSOR_CHAIN_OBJ_DECLARE(
  conv2d_11_chain, AI_STATIC_CONST, 4,
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &multiply_3_output),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &conv2d_11_output),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 3, &conv2d_11_weights, &conv2d_11_bias, NULL),
  AI_TENSOR_LIST_OBJ_EMPTY
)

AI_LAYER_OBJ_DECLARE(
  conv2d_11_layer, 58,
  CONV2D_TYPE,
  conv2d, forward_conv2d,
  &AI_NET_OBJ_INSTANCE, &add_2_layer, AI_STATIC,
  .tensors = &conv2d_11_chain, 
  .groups = 1, 
  .nl_func = NULL, 
  .filter_stride = AI_SHAPE_2D_INIT(1, 1), 
  .dilation = AI_SHAPE_2D_INIT(1, 1), 
  .filter_pad = AI_SHAPE_INIT(4, 0, 0, 0, 0), 
)

AI_TENSOR_CHAIN_OBJ_DECLARE(
  add_2_chain, AI_STATIC_CONST, 4,
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 2, &conv2d_11_output, &conv2d_9_output),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &add_2_output),
  AI_TENSOR_LIST_OBJ_EMPTY,
  AI_TENSOR_LIST_OBJ_EMPTY
)

AI_LAYER_OBJ_DECLARE(
  add_2_layer, 60,
  ELTWISE_TYPE,
  eltwise, forward_eltwise,
  &AI_NET_OBJ_INSTANCE, &conv2d_12_layer, AI_STATIC,
  .tensors = &add_2_chain, 
  .operation = ai_sum, 
)


AI_STATIC_CONST ai_float conv2d_12_nl_params_data[] = { 0, 6.0 };
AI_ARRAY_OBJ_DECLARE(
    conv2d_12_nl_params, AI_ARRAY_FORMAT_FLOAT,
    conv2d_12_nl_params_data, conv2d_12_nl_params_data, 2, AI_STATIC_CONST)
AI_TENSOR_CHAIN_OBJ_DECLARE(
  conv2d_12_chain, AI_STATIC_CONST, 4,
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &add_2_output),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &conv2d_12_output),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 3, &conv2d_12_weights, &conv2d_12_bias, NULL),
  AI_TENSOR_LIST_OBJ_EMPTY
)

AI_LAYER_OBJ_DECLARE(
  conv2d_12_layer, 61,
  CONV2D_TYPE,
  conv2d, forward_conv2d,
  &AI_NET_OBJ_INSTANCE, &depthwise_conv2d_6_layer, AI_STATIC,
  .tensors = &conv2d_12_chain, 
  .groups = 1, 
  .nl_params = &conv2d_12_nl_params, 
  .nl_func = nl_func_clip_array_f32, 
  .filter_stride = AI_SHAPE_2D_INIT(1, 1), 
  .dilation = AI_SHAPE_2D_INIT(1, 1), 
  .filter_pad = AI_SHAPE_INIT(4, 0, 0, 0, 0), 
)


AI_STATIC_CONST ai_float depthwise_conv2d_6_nl_params_data[] = { 0, 6.0 };
AI_ARRAY_OBJ_DECLARE(
    depthwise_conv2d_6_nl_params, AI_ARRAY_FORMAT_FLOAT,
    depthwise_conv2d_6_nl_params_data, depthwise_conv2d_6_nl_params_data, 2, AI_STATIC_CONST)
AI_TENSOR_CHAIN_OBJ_DECLARE(
  depthwise_conv2d_6_chain, AI_STATIC_CONST, 4,
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &conv2d_12_output),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &depthwise_conv2d_6_output),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 3, &depthwise_conv2d_6_weights, &depthwise_conv2d_6_bias, NULL),
  AI_TENSOR_LIST_OBJ_EMPTY
)

AI_LAYER_OBJ_DECLARE(
  depthwise_conv2d_6_layer, 64,
  CONV2D_TYPE,
  conv2d, forward_conv2d,
  &AI_NET_OBJ_INSTANCE, &global_average_pooling2d_4_layer, AI_STATIC,
  .tensors = &depthwise_conv2d_6_chain, 
  .groups = 60, 
  .nl_params = &depthwise_conv2d_6_nl_params, 
  .nl_func = nl_func_clip_array_f32, 
  .filter_stride = AI_SHAPE_2D_INIT(1, 1), 
  .dilation = AI_SHAPE_2D_INIT(1, 1), 
  .filter_pad = AI_SHAPE_INIT(4, 2, 2, 2, 2), 
)

AI_TENSOR_CHAIN_OBJ_DECLARE(
  global_average_pooling2d_4_chain, AI_STATIC_CONST, 4,
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &depthwise_conv2d_6_output),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &global_average_pooling2d_4_output),
  AI_TENSOR_LIST_OBJ_EMPTY,
  AI_TENSOR_LIST_OBJ_EMPTY
)

AI_LAYER_OBJ_DECLARE(
  global_average_pooling2d_4_layer, 67,
  POOL_TYPE,
  pool, forward_ap,
  &AI_NET_OBJ_INSTANCE, &dense_7_layer, AI_STATIC,
  .tensors = &global_average_pooling2d_4_chain, 
  .pool_size = AI_SHAPE_2D_INIT(4, 4), 
  .pool_stride = AI_SHAPE_2D_INIT(4, 4), 
  .pool_pad = AI_SHAPE_INIT(4, 0, 0, 0, 0), 
)

AI_TENSOR_CHAIN_OBJ_DECLARE(
  dense_7_chain, AI_STATIC_CONST, 4,
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &global_average_pooling2d_4_output),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &dense_7_output),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 2, &dense_7_weights, &dense_7_bias),
  AI_TENSOR_LIST_OBJ_EMPTY
)

AI_LAYER_OBJ_DECLARE(
  dense_7_layer, 68,
  DENSE_TYPE,
  dense, forward_dense,
  &AI_NET_OBJ_INSTANCE, &dense_7_nl_layer, AI_STATIC,
  .tensors = &dense_7_chain, 
)

AI_TENSOR_CHAIN_OBJ_DECLARE(
  dense_7_nl_chain, AI_STATIC_CONST, 4,
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &dense_7_output),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &dense_7_nl_output),
  AI_TENSOR_LIST_OBJ_EMPTY,
  AI_TENSOR_LIST_OBJ_EMPTY
)

AI_LAYER_OBJ_DECLARE(
  dense_7_nl_layer, 68,
  NL_TYPE,
  nl, forward_relu,
  &AI_NET_OBJ_INSTANCE, &dense_8_layer, AI_STATIC,
  .tensors = &dense_7_nl_chain, 
)

AI_TENSOR_CHAIN_OBJ_DECLARE(
  dense_8_chain, AI_STATIC_CONST, 4,
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &dense_7_nl_output),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &dense_8_output),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 2, &dense_8_weights, &dense_8_bias),
  AI_TENSOR_LIST_OBJ_EMPTY
)

AI_LAYER_OBJ_DECLARE(
  dense_8_layer, 69,
  DENSE_TYPE,
  dense, forward_dense,
  &AI_NET_OBJ_INSTANCE, &dense_8_nl_layer, AI_STATIC,
  .tensors = &dense_8_chain, 
)

AI_TENSOR_CHAIN_OBJ_DECLARE(
  dense_8_nl_chain, AI_STATIC_CONST, 4,
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &dense_8_output),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &dense_8_nl_output),
  AI_TENSOR_LIST_OBJ_EMPTY,
  AI_TENSOR_LIST_OBJ_EMPTY
)

AI_LAYER_OBJ_DECLARE(
  dense_8_nl_layer, 69,
  NL_TYPE,
  nl, forward_hard_sigmoid,
  &AI_NET_OBJ_INSTANCE, &multiply_4_layer, AI_STATIC,
  .tensors = &dense_8_nl_chain, 
)

AI_TENSOR_CHAIN_OBJ_DECLARE(
  multiply_4_chain, AI_STATIC_CONST, 4,
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 2, &depthwise_conv2d_6_output, &dense_8_nl_output),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &multiply_4_output),
  AI_TENSOR_LIST_OBJ_EMPTY,
  AI_TENSOR_LIST_OBJ_EMPTY
)

AI_LAYER_OBJ_DECLARE(
  multiply_4_layer, 71,
  ELTWISE_TYPE,
  eltwise, forward_eltwise,
  &AI_NET_OBJ_INSTANCE, &conv2d_13_layer, AI_STATIC,
  .tensors = &multiply_4_chain, 
  .operation = ai_mul, 
)

AI_TENSOR_CHAIN_OBJ_DECLARE(
  conv2d_13_chain, AI_STATIC_CONST, 4,
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &multiply_4_output),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &conv2d_13_output),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 3, &conv2d_13_weights, &conv2d_13_bias, NULL),
  AI_TENSOR_LIST_OBJ_EMPTY
)

AI_LAYER_OBJ_DECLARE(
  conv2d_13_layer, 72,
  CONV2D_TYPE,
  conv2d, forward_conv2d,
  &AI_NET_OBJ_INSTANCE, &add_3_layer, AI_STATIC,
  .tensors = &conv2d_13_chain, 
  .groups = 1, 
  .nl_func = NULL, 
  .filter_stride = AI_SHAPE_2D_INIT(1, 1), 
  .dilation = AI_SHAPE_2D_INIT(1, 1), 
  .filter_pad = AI_SHAPE_INIT(4, 0, 0, 0, 0), 
)

AI_TENSOR_CHAIN_OBJ_DECLARE(
  add_3_chain, AI_STATIC_CONST, 4,
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 2, &conv2d_13_output, &add_2_output),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &add_3_output),
  AI_TENSOR_LIST_OBJ_EMPTY,
  AI_TENSOR_LIST_OBJ_EMPTY
)

AI_LAYER_OBJ_DECLARE(
  add_3_layer, 74,
  ELTWISE_TYPE,
  eltwise, forward_eltwise,
  &AI_NET_OBJ_INSTANCE, &conv2d_14_layer, AI_STATIC,
  .tensors = &add_3_chain, 
  .operation = ai_sum, 
)


AI_STATIC_CONST ai_float conv2d_14_nl_params_data[] = { 0, 6.0 };
AI_ARRAY_OBJ_DECLARE(
    conv2d_14_nl_params, AI_ARRAY_FORMAT_FLOAT,
    conv2d_14_nl_params_data, conv2d_14_nl_params_data, 2, AI_STATIC_CONST)
AI_TENSOR_CHAIN_OBJ_DECLARE(
  conv2d_14_chain, AI_STATIC_CONST, 4,
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &add_3_output),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &conv2d_14_output),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 3, &conv2d_14_weights, &conv2d_14_bias, NULL),
  AI_TENSOR_LIST_OBJ_EMPTY
)

AI_LAYER_OBJ_DECLARE(
  conv2d_14_layer, 75,
  CONV2D_TYPE,
  conv2d, forward_conv2d,
  &AI_NET_OBJ_INSTANCE, &depthwise_conv2d_7_layer, AI_STATIC,
  .tensors = &conv2d_14_chain, 
  .groups = 1, 
  .nl_params = &conv2d_14_nl_params, 
  .nl_func = nl_func_clip_array_f32, 
  .filter_stride = AI_SHAPE_2D_INIT(1, 1), 
  .dilation = AI_SHAPE_2D_INIT(1, 1), 
  .filter_pad = AI_SHAPE_INIT(4, 0, 0, 0, 0), 
)


AI_STATIC_CONST ai_float depthwise_conv2d_7_nl_params_data[] = { 0, 6.0 };
AI_ARRAY_OBJ_DECLARE(
    depthwise_conv2d_7_nl_params, AI_ARRAY_FORMAT_FLOAT,
    depthwise_conv2d_7_nl_params_data, depthwise_conv2d_7_nl_params_data, 2, AI_STATIC_CONST)
AI_TENSOR_CHAIN_OBJ_DECLARE(
  depthwise_conv2d_7_chain, AI_STATIC_CONST, 4,
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &conv2d_14_output),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &depthwise_conv2d_7_output),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 3, &depthwise_conv2d_7_weights, &depthwise_conv2d_7_bias, NULL),
  AI_TENSOR_LIST_OBJ_EMPTY
)

AI_LAYER_OBJ_DECLARE(
  depthwise_conv2d_7_layer, 78,
  CONV2D_TYPE,
  conv2d, forward_conv2d,
  &AI_NET_OBJ_INSTANCE, &global_average_pooling2d_5_layer, AI_STATIC,
  .tensors = &depthwise_conv2d_7_chain, 
  .groups = 60, 
  .nl_params = &depthwise_conv2d_7_nl_params, 
  .nl_func = nl_func_clip_array_f32, 
  .filter_stride = AI_SHAPE_2D_INIT(2, 2), 
  .dilation = AI_SHAPE_2D_INIT(1, 1), 
  .filter_pad = AI_SHAPE_INIT(4, 1, 1, 3, 3), 
)

AI_TENSOR_CHAIN_OBJ_DECLARE(
  global_average_pooling2d_5_chain, AI_STATIC_CONST, 4,
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &depthwise_conv2d_7_output),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &global_average_pooling2d_5_output),
  AI_TENSOR_LIST_OBJ_EMPTY,
  AI_TENSOR_LIST_OBJ_EMPTY
)

AI_LAYER_OBJ_DECLARE(
  global_average_pooling2d_5_layer, 81,
  POOL_TYPE,
  pool, forward_ap,
  &AI_NET_OBJ_INSTANCE, &dense_9_layer, AI_STATIC,
  .tensors = &global_average_pooling2d_5_chain, 
  .pool_size = AI_SHAPE_2D_INIT(2, 2), 
  .pool_stride = AI_SHAPE_2D_INIT(2, 2), 
  .pool_pad = AI_SHAPE_INIT(4, 0, 0, 0, 0), 
)

AI_TENSOR_CHAIN_OBJ_DECLARE(
  dense_9_chain, AI_STATIC_CONST, 4,
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &global_average_pooling2d_5_output),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &dense_9_output),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 2, &dense_9_weights, &dense_9_bias),
  AI_TENSOR_LIST_OBJ_EMPTY
)

AI_LAYER_OBJ_DECLARE(
  dense_9_layer, 82,
  DENSE_TYPE,
  dense, forward_dense,
  &AI_NET_OBJ_INSTANCE, &dense_9_nl_layer, AI_STATIC,
  .tensors = &dense_9_chain, 
)

AI_TENSOR_CHAIN_OBJ_DECLARE(
  dense_9_nl_chain, AI_STATIC_CONST, 4,
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &dense_9_output),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &dense_9_nl_output),
  AI_TENSOR_LIST_OBJ_EMPTY,
  AI_TENSOR_LIST_OBJ_EMPTY
)

AI_LAYER_OBJ_DECLARE(
  dense_9_nl_layer, 82,
  NL_TYPE,
  nl, forward_relu,
  &AI_NET_OBJ_INSTANCE, &dense_10_layer, AI_STATIC,
  .tensors = &dense_9_nl_chain, 
)

AI_TENSOR_CHAIN_OBJ_DECLARE(
  dense_10_chain, AI_STATIC_CONST, 4,
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &dense_9_nl_output),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &dense_10_output),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 2, &dense_10_weights, &dense_10_bias),
  AI_TENSOR_LIST_OBJ_EMPTY
)

AI_LAYER_OBJ_DECLARE(
  dense_10_layer, 83,
  DENSE_TYPE,
  dense, forward_dense,
  &AI_NET_OBJ_INSTANCE, &dense_10_nl_layer, AI_STATIC,
  .tensors = &dense_10_chain, 
)

AI_TENSOR_CHAIN_OBJ_DECLARE(
  dense_10_nl_chain, AI_STATIC_CONST, 4,
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &dense_10_output),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &dense_10_nl_output),
  AI_TENSOR_LIST_OBJ_EMPTY,
  AI_TENSOR_LIST_OBJ_EMPTY
)

AI_LAYER_OBJ_DECLARE(
  dense_10_nl_layer, 83,
  NL_TYPE,
  nl, forward_hard_sigmoid,
  &AI_NET_OBJ_INSTANCE, &multiply_5_layer, AI_STATIC,
  .tensors = &dense_10_nl_chain, 
)

AI_TENSOR_CHAIN_OBJ_DECLARE(
  multiply_5_chain, AI_STATIC_CONST, 4,
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 2, &depthwise_conv2d_7_output, &dense_10_nl_output),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &multiply_5_output),
  AI_TENSOR_LIST_OBJ_EMPTY,
  AI_TENSOR_LIST_OBJ_EMPTY
)

AI_LAYER_OBJ_DECLARE(
  multiply_5_layer, 85,
  ELTWISE_TYPE,
  eltwise, forward_eltwise,
  &AI_NET_OBJ_INSTANCE, &conv2d_15_layer, AI_STATIC,
  .tensors = &multiply_5_chain, 
  .operation = ai_mul, 
)

AI_TENSOR_CHAIN_OBJ_DECLARE(
  conv2d_15_chain, AI_STATIC_CONST, 4,
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &multiply_5_output),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &conv2d_15_output),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 3, &conv2d_15_weights, &conv2d_15_bias, NULL),
  AI_TENSOR_LIST_OBJ_EMPTY
)

AI_LAYER_OBJ_DECLARE(
  conv2d_15_layer, 86,
  CONV2D_TYPE,
  conv2d, forward_conv2d,
  &AI_NET_OBJ_INSTANCE, &conv2d_16_layer, AI_STATIC,
  .tensors = &conv2d_15_chain, 
  .groups = 1, 
  .nl_func = NULL, 
  .filter_stride = AI_SHAPE_2D_INIT(1, 1), 
  .dilation = AI_SHAPE_2D_INIT(1, 1), 
  .filter_pad = AI_SHAPE_INIT(4, 0, 0, 0, 0), 
)


AI_STATIC_CONST ai_float conv2d_16_nl_params_data[] = { 0, 6.0 };
AI_ARRAY_OBJ_DECLARE(
    conv2d_16_nl_params, AI_ARRAY_FORMAT_FLOAT,
    conv2d_16_nl_params_data, conv2d_16_nl_params_data, 2, AI_STATIC_CONST)
AI_TENSOR_CHAIN_OBJ_DECLARE(
  conv2d_16_chain, AI_STATIC_CONST, 4,
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &conv2d_15_output),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &conv2d_16_output),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 3, &conv2d_16_weights, &conv2d_16_bias, NULL),
  AI_TENSOR_LIST_OBJ_EMPTY
)

AI_LAYER_OBJ_DECLARE(
  conv2d_16_layer, 88,
  CONV2D_TYPE,
  conv2d, forward_conv2d,
  &AI_NET_OBJ_INSTANCE, &depthwise_conv2d_8_layer, AI_STATIC,
  .tensors = &conv2d_16_chain, 
  .groups = 1, 
  .nl_params = &conv2d_16_nl_params, 
  .nl_func = nl_func_clip_array_f32, 
  .filter_stride = AI_SHAPE_2D_INIT(1, 1), 
  .dilation = AI_SHAPE_2D_INIT(1, 1), 
  .filter_pad = AI_SHAPE_INIT(4, 0, 0, 0, 0), 
)


AI_STATIC_CONST ai_float depthwise_conv2d_8_nl_params_data[] = { 0, 6.0 };
AI_ARRAY_OBJ_DECLARE(
    depthwise_conv2d_8_nl_params, AI_ARRAY_FORMAT_FLOAT,
    depthwise_conv2d_8_nl_params_data, depthwise_conv2d_8_nl_params_data, 2, AI_STATIC_CONST)
AI_TENSOR_CHAIN_OBJ_DECLARE(
  depthwise_conv2d_8_chain, AI_STATIC_CONST, 4,
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &conv2d_16_output),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &depthwise_conv2d_8_output),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 3, &depthwise_conv2d_8_weights, &depthwise_conv2d_8_bias, NULL),
  AI_TENSOR_LIST_OBJ_EMPTY
)

AI_LAYER_OBJ_DECLARE(
  depthwise_conv2d_8_layer, 91,
  CONV2D_TYPE,
  conv2d, forward_conv2d,
  &AI_NET_OBJ_INSTANCE, &global_average_pooling2d_6_layer, AI_STATIC,
  .tensors = &depthwise_conv2d_8_chain, 
  .groups = 60, 
  .nl_params = &depthwise_conv2d_8_nl_params, 
  .nl_func = nl_func_clip_array_f32, 
  .filter_stride = AI_SHAPE_2D_INIT(1, 1), 
  .dilation = AI_SHAPE_2D_INIT(1, 1), 
  .filter_pad = AI_SHAPE_INIT(4, 2, 2, 2, 2), 
)

AI_TENSOR_CHAIN_OBJ_DECLARE(
  global_average_pooling2d_6_chain, AI_STATIC_CONST, 4,
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &depthwise_conv2d_8_output),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &global_average_pooling2d_6_output),
  AI_TENSOR_LIST_OBJ_EMPTY,
  AI_TENSOR_LIST_OBJ_EMPTY
)

AI_LAYER_OBJ_DECLARE(
  global_average_pooling2d_6_layer, 94,
  POOL_TYPE,
  pool, forward_ap,
  &AI_NET_OBJ_INSTANCE, &dense_11_layer, AI_STATIC,
  .tensors = &global_average_pooling2d_6_chain, 
  .pool_size = AI_SHAPE_2D_INIT(2, 2), 
  .pool_stride = AI_SHAPE_2D_INIT(2, 2), 
  .pool_pad = AI_SHAPE_INIT(4, 0, 0, 0, 0), 
)

AI_TENSOR_CHAIN_OBJ_DECLARE(
  dense_11_chain, AI_STATIC_CONST, 4,
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &global_average_pooling2d_6_output),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &dense_11_output),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 2, &dense_11_weights, &dense_11_bias),
  AI_TENSOR_LIST_OBJ_EMPTY
)

AI_LAYER_OBJ_DECLARE(
  dense_11_layer, 95,
  DENSE_TYPE,
  dense, forward_dense,
  &AI_NET_OBJ_INSTANCE, &dense_11_nl_layer, AI_STATIC,
  .tensors = &dense_11_chain, 
)

AI_TENSOR_CHAIN_OBJ_DECLARE(
  dense_11_nl_chain, AI_STATIC_CONST, 4,
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &dense_11_output),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &dense_11_nl_output),
  AI_TENSOR_LIST_OBJ_EMPTY,
  AI_TENSOR_LIST_OBJ_EMPTY
)

AI_LAYER_OBJ_DECLARE(
  dense_11_nl_layer, 95,
  NL_TYPE,
  nl, forward_relu,
  &AI_NET_OBJ_INSTANCE, &dense_12_layer, AI_STATIC,
  .tensors = &dense_11_nl_chain, 
)

AI_TENSOR_CHAIN_OBJ_DECLARE(
  dense_12_chain, AI_STATIC_CONST, 4,
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &dense_11_nl_output),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &dense_12_output),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 2, &dense_12_weights, &dense_12_bias),
  AI_TENSOR_LIST_OBJ_EMPTY
)

AI_LAYER_OBJ_DECLARE(
  dense_12_layer, 96,
  DENSE_TYPE,
  dense, forward_dense,
  &AI_NET_OBJ_INSTANCE, &dense_12_nl_layer, AI_STATIC,
  .tensors = &dense_12_chain, 
)

AI_TENSOR_CHAIN_OBJ_DECLARE(
  dense_12_nl_chain, AI_STATIC_CONST, 4,
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &dense_12_output),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &dense_12_nl_output),
  AI_TENSOR_LIST_OBJ_EMPTY,
  AI_TENSOR_LIST_OBJ_EMPTY
)

AI_LAYER_OBJ_DECLARE(
  dense_12_nl_layer, 96,
  NL_TYPE,
  nl, forward_hard_sigmoid,
  &AI_NET_OBJ_INSTANCE, &multiply_6_layer, AI_STATIC,
  .tensors = &dense_12_nl_chain, 
)

AI_TENSOR_CHAIN_OBJ_DECLARE(
  multiply_6_chain, AI_STATIC_CONST, 4,
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 2, &depthwise_conv2d_8_output, &dense_12_nl_output),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &multiply_6_output),
  AI_TENSOR_LIST_OBJ_EMPTY,
  AI_TENSOR_LIST_OBJ_EMPTY
)

AI_LAYER_OBJ_DECLARE(
  multiply_6_layer, 98,
  ELTWISE_TYPE,
  eltwise, forward_eltwise,
  &AI_NET_OBJ_INSTANCE, &conv2d_17_layer, AI_STATIC,
  .tensors = &multiply_6_chain, 
  .operation = ai_mul, 
)

AI_TENSOR_CHAIN_OBJ_DECLARE(
  conv2d_17_chain, AI_STATIC_CONST, 4,
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &multiply_6_output),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &conv2d_17_output),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 3, &conv2d_17_weights, &conv2d_17_bias, NULL),
  AI_TENSOR_LIST_OBJ_EMPTY
)

AI_LAYER_OBJ_DECLARE(
  conv2d_17_layer, 99,
  CONV2D_TYPE,
  conv2d, forward_conv2d,
  &AI_NET_OBJ_INSTANCE, &add_4_layer, AI_STATIC,
  .tensors = &conv2d_17_chain, 
  .groups = 1, 
  .nl_func = NULL, 
  .filter_stride = AI_SHAPE_2D_INIT(1, 1), 
  .dilation = AI_SHAPE_2D_INIT(1, 1), 
  .filter_pad = AI_SHAPE_INIT(4, 0, 0, 0, 0), 
)

AI_TENSOR_CHAIN_OBJ_DECLARE(
  add_4_chain, AI_STATIC_CONST, 4,
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 2, &conv2d_17_output, &conv2d_15_output),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &add_4_output),
  AI_TENSOR_LIST_OBJ_EMPTY,
  AI_TENSOR_LIST_OBJ_EMPTY
)

AI_LAYER_OBJ_DECLARE(
  add_4_layer, 101,
  ELTWISE_TYPE,
  eltwise, forward_eltwise,
  &AI_NET_OBJ_INSTANCE, &conv2d_18_layer, AI_STATIC,
  .tensors = &add_4_chain, 
  .operation = ai_sum, 
)


AI_STATIC_CONST ai_float conv2d_18_nl_params_data[] = { 0, 6.0 };
AI_ARRAY_OBJ_DECLARE(
    conv2d_18_nl_params, AI_ARRAY_FORMAT_FLOAT,
    conv2d_18_nl_params_data, conv2d_18_nl_params_data, 2, AI_STATIC_CONST)
AI_TENSOR_CHAIN_OBJ_DECLARE(
  conv2d_18_chain, AI_STATIC_CONST, 4,
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &add_4_output),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &conv2d_18_output),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 3, &conv2d_18_weights, &conv2d_18_bias, NULL),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &conv2d_18_scratch0)
)

AI_LAYER_OBJ_DECLARE(
  conv2d_18_layer, 102,
  OPTIMIZED_CONV2D_TYPE,
  conv2d_nl_pool, forward_conv2d_nl_pool,
  &AI_NET_OBJ_INSTANCE, &conv2d_19_layer, AI_STATIC,
  .tensors = &conv2d_18_chain, 
  .groups = 1, 
  .nl_params = &conv2d_18_nl_params, 
  .nl_func = nl_func_clip_array_f32, 
  .filter_stride = AI_SHAPE_2D_INIT(1, 1), 
  .dilation = AI_SHAPE_2D_INIT(1, 1), 
  .filter_pad = AI_SHAPE_INIT(4, 0, 0, 0, 0), 
  .pool_size = AI_SHAPE_2D_INIT(2, 2), 
  .pool_stride = AI_SHAPE_2D_INIT(2, 2), 
  .pool_pad = AI_SHAPE_INIT(4, 0, 0, 0, 0), 
  .pool_func = pool_func_ap_array_f32, 
)


AI_STATIC_CONST ai_float conv2d_19_nl_params_data[] = { 0, 6.0 };
AI_ARRAY_OBJ_DECLARE(
    conv2d_19_nl_params, AI_ARRAY_FORMAT_FLOAT,
    conv2d_19_nl_params_data, conv2d_19_nl_params_data, 2, AI_STATIC_CONST)
AI_TENSOR_CHAIN_OBJ_DECLARE(
  conv2d_19_chain, AI_STATIC_CONST, 4,
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &conv2d_18_output),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &conv2d_19_output),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 3, &conv2d_19_weights, &conv2d_19_bias, NULL),
  AI_TENSOR_LIST_OBJ_EMPTY
)

AI_LAYER_OBJ_DECLARE(
  conv2d_19_layer, 107,
  CONV2D_TYPE,
  conv2d, forward_conv2d,
  &AI_NET_OBJ_INSTANCE, &conv2d_20_layer, AI_STATIC,
  .tensors = &conv2d_19_chain, 
  .groups = 1, 
  .nl_params = &conv2d_19_nl_params, 
  .nl_func = nl_func_clip_array_f32, 
  .filter_stride = AI_SHAPE_2D_INIT(1, 1), 
  .dilation = AI_SHAPE_2D_INIT(1, 1), 
  .filter_pad = AI_SHAPE_INIT(4, 0, 0, 0, 0), 
)

AI_TENSOR_CHAIN_OBJ_DECLARE(
  conv2d_20_chain, AI_STATIC_CONST, 4,
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &conv2d_19_output),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &conv2d_20_output),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 3, &conv2d_20_weights, &conv2d_20_bias, NULL),
  AI_TENSOR_LIST_OBJ_EMPTY
)

AI_LAYER_OBJ_DECLARE(
  conv2d_20_layer, 109,
  CONV2D_TYPE,
  conv2d, forward_conv2d,
  &AI_NET_OBJ_INSTANCE, &conv2d_20_layer, AI_STATIC,
  .tensors = &conv2d_20_chain, 
  .groups = 1, 
  .nl_func = nl_func_sm_channel_f32, 
  .filter_stride = AI_SHAPE_2D_INIT(1, 1), 
  .dilation = AI_SHAPE_2D_INIT(1, 1), 
  .filter_pad = AI_SHAPE_INIT(4, 0, 0, 0, 0), 
)


AI_NETWORK_OBJ_DECLARE(
  AI_NET_OBJ_INSTANCE, AI_STATIC,
  AI_BUFFER_OBJ_INIT(AI_BUFFER_FORMAT_U8,
                     1, 1, 220896, 1,
                     NULL),
  AI_BUFFER_OBJ_INIT(AI_BUFFER_FORMAT_U8,
                     1, 1, 22680, 1,
                     NULL),
  AI_TENSOR_LIST_IO_OBJ_INIT(AI_FLAG_NONE, AI_NETWORK_HANDC5_IN_NUM, &input_1_output),
  AI_TENSOR_LIST_IO_OBJ_INIT(AI_FLAG_NONE, AI_NETWORK_HANDC5_OUT_NUM, &conv2d_20_output),
  &conv2d_1_layer, 0, NULL)



AI_DECLARE_STATIC
ai_bool network_handc5_configure_activations(
  ai_network* net_ctx, const ai_buffer* activation_buffer)
{
  AI_ASSERT(net_ctx &&  activation_buffer && activation_buffer->data)

  ai_ptr activations = AI_PTR(AI_PTR_ALIGN(activation_buffer->data, 4));
  AI_ASSERT(activations)
  AI_UNUSED(net_ctx)

  {
    /* Updating activations (byte) offsets */
    conv2d_18_scratch0_array.data = AI_PTR(activations + 960);
    conv2d_18_scratch0_array.data_start = AI_PTR(activations + 960);
    input_1_output_array.data = AI_PTR(NULL);
    input_1_output_array.data_start = AI_PTR(NULL);
    conv2d_1_output_array.data = AI_PTR(activations + 744);
    conv2d_1_output_array.data_start = AI_PTR(activations + 744);
    conv2d_2_output_array.data = AI_PTR(activations + 0);
    conv2d_2_output_array.data_start = AI_PTR(activations + 0);
    depthwise_conv2d_1_output_array.data = AI_PTR(activations + 18000);
    depthwise_conv2d_1_output_array.data_start = AI_PTR(activations + 18000);
    global_average_pooling2d_1_output_array.data = AI_PTR(activations + 0);
    global_average_pooling2d_1_output_array.data_start = AI_PTR(activations + 0);
    dense_1_output_array.data = AI_PTR(activations + 24);
    dense_1_output_array.data_start = AI_PTR(activations + 24);
    dense_1_nl_output_array.data = AI_PTR(activations + 0);
    dense_1_nl_output_array.data_start = AI_PTR(activations + 0);
    dense_2_output_array.data = AI_PTR(activations + 24);
    dense_2_output_array.data_start = AI_PTR(activations + 24);
    dense_2_nl_output_array.data = AI_PTR(activations + 0);
    dense_2_nl_output_array.data_start = AI_PTR(activations + 0);
    multiply_1_output_array.data = AI_PTR(activations + 24);
    multiply_1_output_array.data_start = AI_PTR(activations + 24);
    conv2d_3_output_array.data = AI_PTR(activations + 18000);
    conv2d_3_output_array.data_start = AI_PTR(activations + 18000);
    conv2d_4_output_array.data = AI_PTR(activations + 0);
    conv2d_4_output_array.data_start = AI_PTR(activations + 0);
    depthwise_conv2d_2_output_array.data = AI_PTR(activations + 15600);
    depthwise_conv2d_2_output_array.data_start = AI_PTR(activations + 15600);
    conv2d_5_output_array.data = AI_PTR(activations + 0);
    conv2d_5_output_array.data_start = AI_PTR(activations + 0);
    conv2d_6_output_array.data = AI_PTR(activations + 1792);
    conv2d_6_output_array.data_start = AI_PTR(activations + 1792);
    depthwise_conv2d_3_output_array.data = AI_PTR(activations + 6272);
    depthwise_conv2d_3_output_array.data_start = AI_PTR(activations + 6272);
    conv2d_7_output_array.data = AI_PTR(activations + 1792);
    conv2d_7_output_array.data_start = AI_PTR(activations + 1792);
    add_1_output_array.data = AI_PTR(activations + 3584);
    add_1_output_array.data_start = AI_PTR(activations + 3584);
    conv2d_8_output_array.data = AI_PTR(activations + 5376);
    conv2d_8_output_array.data_start = AI_PTR(activations + 5376);
    depthwise_conv2d_4_output_array.data = AI_PTR(activations + 0);
    depthwise_conv2d_4_output_array.data_start = AI_PTR(activations + 0);
    global_average_pooling2d_2_output_array.data = AI_PTR(activations + 1280);
    global_average_pooling2d_2_output_array.data_start = AI_PTR(activations + 1280);
    dense_3_output_array.data = AI_PTR(activations + 1360);
    dense_3_output_array.data_start = AI_PTR(activations + 1360);
    dense_3_nl_output_array.data = AI_PTR(activations + 1280);
    dense_3_nl_output_array.data_start = AI_PTR(activations + 1280);
    dense_4_output_array.data = AI_PTR(activations + 1360);
    dense_4_output_array.data_start = AI_PTR(activations + 1360);
    dense_4_nl_output_array.data = AI_PTR(activations + 1280);
    dense_4_nl_output_array.data_start = AI_PTR(activations + 1280);
    multiply_2_output_array.data = AI_PTR(activations + 1360);
    multiply_2_output_array.data_start = AI_PTR(activations + 1360);
    conv2d_9_output_array.data = AI_PTR(activations + 0);
    conv2d_9_output_array.data_start = AI_PTR(activations + 0);
    conv2d_10_output_array.data = AI_PTR(activations + 768);
    conv2d_10_output_array.data_start = AI_PTR(activations + 768);
    depthwise_conv2d_5_output_array.data = AI_PTR(activations + 4608);
    depthwise_conv2d_5_output_array.data_start = AI_PTR(activations + 4608);
    global_average_pooling2d_3_output_array.data = AI_PTR(activations + 768);
    global_average_pooling2d_3_output_array.data_start = AI_PTR(activations + 768);
    dense_5_output_array.data = AI_PTR(activations + 1008);
    dense_5_output_array.data_start = AI_PTR(activations + 1008);
    dense_5_nl_output_array.data = AI_PTR(activations + 768);
    dense_5_nl_output_array.data_start = AI_PTR(activations + 768);
    dense_6_output_array.data = AI_PTR(activations + 1008);
    dense_6_output_array.data_start = AI_PTR(activations + 1008);
    dense_6_nl_output_array.data = AI_PTR(activations + 768);
    dense_6_nl_output_array.data_start = AI_PTR(activations + 768);
    multiply_3_output_array.data = AI_PTR(activations + 8448);
    multiply_3_output_array.data_start = AI_PTR(activations + 8448);
    conv2d_11_output_array.data = AI_PTR(activations + 768);
    conv2d_11_output_array.data_start = AI_PTR(activations + 768);
    add_2_output_array.data = AI_PTR(activations + 1536);
    add_2_output_array.data_start = AI_PTR(activations + 1536);
    conv2d_12_output_array.data = AI_PTR(activations + 2304);
    conv2d_12_output_array.data_start = AI_PTR(activations + 2304);
    depthwise_conv2d_6_output_array.data = AI_PTR(activations + 6144);
    depthwise_conv2d_6_output_array.data_start = AI_PTR(activations + 6144);
    global_average_pooling2d_4_output_array.data = AI_PTR(activations + 0);
    global_average_pooling2d_4_output_array.data_start = AI_PTR(activations + 0);
    dense_7_output_array.data = AI_PTR(activations + 240);
    dense_7_output_array.data_start = AI_PTR(activations + 240);
    dense_7_nl_output_array.data = AI_PTR(activations + 0);
    dense_7_nl_output_array.data_start = AI_PTR(activations + 0);
    dense_8_output_array.data = AI_PTR(activations + 240);
    dense_8_output_array.data_start = AI_PTR(activations + 240);
    dense_8_nl_output_array.data = AI_PTR(activations + 0);
    dense_8_nl_output_array.data_start = AI_PTR(activations + 0);
    multiply_4_output_array.data = AI_PTR(activations + 2304);
    multiply_4_output_array.data_start = AI_PTR(activations + 2304);
    conv2d_13_output_array.data = AI_PTR(activations + 0);
    conv2d_13_output_array.data_start = AI_PTR(activations + 0);
    add_3_output_array.data = AI_PTR(activations + 768);
    add_3_output_array.data_start = AI_PTR(activations + 768);
    conv2d_14_output_array.data = AI_PTR(activations + 1536);
    conv2d_14_output_array.data_start = AI_PTR(activations + 1536);
    depthwise_conv2d_7_output_array.data = AI_PTR(activations + 0);
    depthwise_conv2d_7_output_array.data_start = AI_PTR(activations + 0);
    global_average_pooling2d_5_output_array.data = AI_PTR(activations + 960);
    global_average_pooling2d_5_output_array.data_start = AI_PTR(activations + 960);
    dense_9_output_array.data = AI_PTR(activations + 1200);
    dense_9_output_array.data_start = AI_PTR(activations + 1200);
    dense_9_nl_output_array.data = AI_PTR(activations + 960);
    dense_9_nl_output_array.data_start = AI_PTR(activations + 960);
    dense_10_output_array.data = AI_PTR(activations + 1200);
    dense_10_output_array.data_start = AI_PTR(activations + 1200);
    dense_10_nl_output_array.data = AI_PTR(activations + 960);
    dense_10_nl_output_array.data_start = AI_PTR(activations + 960);
    multiply_5_output_array.data = AI_PTR(activations + 1200);
    multiply_5_output_array.data_start = AI_PTR(activations + 1200);
    conv2d_15_output_array.data = AI_PTR(activations + 0);
    conv2d_15_output_array.data_start = AI_PTR(activations + 0);
    conv2d_16_output_array.data = AI_PTR(activations + 320);
    conv2d_16_output_array.data_start = AI_PTR(activations + 320);
    depthwise_conv2d_8_output_array.data = AI_PTR(activations + 1280);
    depthwise_conv2d_8_output_array.data_start = AI_PTR(activations + 1280);
    global_average_pooling2d_6_output_array.data = AI_PTR(activations + 320);
    global_average_pooling2d_6_output_array.data_start = AI_PTR(activations + 320);
    dense_11_output_array.data = AI_PTR(activations + 560);
    dense_11_output_array.data_start = AI_PTR(activations + 560);
    dense_11_nl_output_array.data = AI_PTR(activations + 320);
    dense_11_nl_output_array.data_start = AI_PTR(activations + 320);
    dense_12_output_array.data = AI_PTR(activations + 560);
    dense_12_output_array.data_start = AI_PTR(activations + 560);
    dense_12_nl_output_array.data = AI_PTR(activations + 320);
    dense_12_nl_output_array.data_start = AI_PTR(activations + 320);
    multiply_6_output_array.data = AI_PTR(activations + 2240);
    multiply_6_output_array.data_start = AI_PTR(activations + 2240);
    conv2d_17_output_array.data = AI_PTR(activations + 320);
    conv2d_17_output_array.data_start = AI_PTR(activations + 320);
    add_4_output_array.data = AI_PTR(activations + 640);
    add_4_output_array.data_start = AI_PTR(activations + 640);
    conv2d_18_output_array.data = AI_PTR(activations + 0);
    conv2d_18_output_array.data_start = AI_PTR(activations + 0);
    conv2d_19_output_array.data = AI_PTR(activations + 240);
    conv2d_19_output_array.data_start = AI_PTR(activations + 240);
    conv2d_20_output_array.data = AI_PTR(NULL);
    conv2d_20_output_array.data_start = AI_PTR(NULL);
    
  }
  return true;
}



AI_DECLARE_STATIC
ai_bool network_handc5_configure_weights(
  ai_network* net_ctx, const ai_buffer* weights_buffer)
{
  AI_ASSERT(net_ctx &&  weights_buffer && weights_buffer->data)

  ai_ptr weights = AI_PTR(weights_buffer->data);
  AI_ASSERT(weights)
  AI_UNUSED(net_ctx)

  {
    /* Updating weights (byte) offsets */
    
    conv2d_20_bias_array.format |= AI_FMT_FLAG_CONST;
    conv2d_20_bias_array.data = AI_PTR(weights + 220880);
    conv2d_20_bias_array.data_start = AI_PTR(weights + 220880);
    conv2d_20_weights_array.format |= AI_FMT_FLAG_CONST;
    conv2d_20_weights_array.data = AI_PTR(weights + 218960);
    conv2d_20_weights_array.data_start = AI_PTR(weights + 218960);
    conv2d_19_bias_array.format |= AI_FMT_FLAG_CONST;
    conv2d_19_bias_array.data = AI_PTR(weights + 218480);
    conv2d_19_bias_array.data_start = AI_PTR(weights + 218480);
    conv2d_19_weights_array.format |= AI_FMT_FLAG_CONST;
    conv2d_19_weights_array.data = AI_PTR(weights + 189680);
    conv2d_19_weights_array.data_start = AI_PTR(weights + 189680);
    conv2d_18_bias_array.format |= AI_FMT_FLAG_CONST;
    conv2d_18_bias_array.data = AI_PTR(weights + 189440);
    conv2d_18_bias_array.data_start = AI_PTR(weights + 189440);
    conv2d_18_weights_array.format |= AI_FMT_FLAG_CONST;
    conv2d_18_weights_array.data = AI_PTR(weights + 184640);
    conv2d_18_weights_array.data_start = AI_PTR(weights + 184640);
    conv2d_17_bias_array.format |= AI_FMT_FLAG_CONST;
    conv2d_17_bias_array.data = AI_PTR(weights + 184560);
    conv2d_17_bias_array.data_start = AI_PTR(weights + 184560);
    conv2d_17_weights_array.format |= AI_FMT_FLAG_CONST;
    conv2d_17_weights_array.data = AI_PTR(weights + 179760);
    conv2d_17_weights_array.data_start = AI_PTR(weights + 179760);
    dense_12_bias_array.format |= AI_FMT_FLAG_CONST;
    dense_12_bias_array.data = AI_PTR(weights + 179520);
    dense_12_bias_array.data_start = AI_PTR(weights + 179520);
    dense_12_weights_array.format |= AI_FMT_FLAG_CONST;
    dense_12_weights_array.data = AI_PTR(weights + 165120);
    dense_12_weights_array.data_start = AI_PTR(weights + 165120);
    dense_11_bias_array.format |= AI_FMT_FLAG_CONST;
    dense_11_bias_array.data = AI_PTR(weights + 164880);
    dense_11_bias_array.data_start = AI_PTR(weights + 164880);
    dense_11_weights_array.format |= AI_FMT_FLAG_CONST;
    dense_11_weights_array.data = AI_PTR(weights + 150480);
    dense_11_weights_array.data_start = AI_PTR(weights + 150480);
    depthwise_conv2d_8_bias_array.format |= AI_FMT_FLAG_CONST;
    depthwise_conv2d_8_bias_array.data = AI_PTR(weights + 150240);
    depthwise_conv2d_8_bias_array.data_start = AI_PTR(weights + 150240);
    depthwise_conv2d_8_weights_array.format |= AI_FMT_FLAG_CONST;
    depthwise_conv2d_8_weights_array.data = AI_PTR(weights + 144240);
    depthwise_conv2d_8_weights_array.data_start = AI_PTR(weights + 144240);
    conv2d_16_bias_array.format |= AI_FMT_FLAG_CONST;
    conv2d_16_bias_array.data = AI_PTR(weights + 144000);
    conv2d_16_bias_array.data_start = AI_PTR(weights + 144000);
    conv2d_16_weights_array.format |= AI_FMT_FLAG_CONST;
    conv2d_16_weights_array.data = AI_PTR(weights + 139200);
    conv2d_16_weights_array.data_start = AI_PTR(weights + 139200);
    conv2d_15_bias_array.format |= AI_FMT_FLAG_CONST;
    conv2d_15_bias_array.data = AI_PTR(weights + 139120);
    conv2d_15_bias_array.data_start = AI_PTR(weights + 139120);
    conv2d_15_weights_array.format |= AI_FMT_FLAG_CONST;
    conv2d_15_weights_array.data = AI_PTR(weights + 134320);
    conv2d_15_weights_array.data_start = AI_PTR(weights + 134320);
    dense_10_bias_array.format |= AI_FMT_FLAG_CONST;
    dense_10_bias_array.data = AI_PTR(weights + 134080);
    dense_10_bias_array.data_start = AI_PTR(weights + 134080);
    dense_10_weights_array.format |= AI_FMT_FLAG_CONST;
    dense_10_weights_array.data = AI_PTR(weights + 119680);
    dense_10_weights_array.data_start = AI_PTR(weights + 119680);
    dense_9_bias_array.format |= AI_FMT_FLAG_CONST;
    dense_9_bias_array.data = AI_PTR(weights + 119440);
    dense_9_bias_array.data_start = AI_PTR(weights + 119440);
    dense_9_weights_array.format |= AI_FMT_FLAG_CONST;
    dense_9_weights_array.data = AI_PTR(weights + 105040);
    dense_9_weights_array.data_start = AI_PTR(weights + 105040);
    depthwise_conv2d_7_bias_array.format |= AI_FMT_FLAG_CONST;
    depthwise_conv2d_7_bias_array.data = AI_PTR(weights + 104800);
    depthwise_conv2d_7_bias_array.data_start = AI_PTR(weights + 104800);
    depthwise_conv2d_7_weights_array.format |= AI_FMT_FLAG_CONST;
    depthwise_conv2d_7_weights_array.data = AI_PTR(weights + 98800);
    depthwise_conv2d_7_weights_array.data_start = AI_PTR(weights + 98800);
    conv2d_14_bias_array.format |= AI_FMT_FLAG_CONST;
    conv2d_14_bias_array.data = AI_PTR(weights + 98560);
    conv2d_14_bias_array.data_start = AI_PTR(weights + 98560);
    conv2d_14_weights_array.format |= AI_FMT_FLAG_CONST;
    conv2d_14_weights_array.data = AI_PTR(weights + 95680);
    conv2d_14_weights_array.data_start = AI_PTR(weights + 95680);
    conv2d_13_bias_array.format |= AI_FMT_FLAG_CONST;
    conv2d_13_bias_array.data = AI_PTR(weights + 95632);
    conv2d_13_bias_array.data_start = AI_PTR(weights + 95632);
    conv2d_13_weights_array.format |= AI_FMT_FLAG_CONST;
    conv2d_13_weights_array.data = AI_PTR(weights + 92752);
    conv2d_13_weights_array.data_start = AI_PTR(weights + 92752);
    dense_8_bias_array.format |= AI_FMT_FLAG_CONST;
    dense_8_bias_array.data = AI_PTR(weights + 92512);
    dense_8_bias_array.data_start = AI_PTR(weights + 92512);
    dense_8_weights_array.format |= AI_FMT_FLAG_CONST;
    dense_8_weights_array.data = AI_PTR(weights + 78112);
    dense_8_weights_array.data_start = AI_PTR(weights + 78112);
    dense_7_bias_array.format |= AI_FMT_FLAG_CONST;
    dense_7_bias_array.data = AI_PTR(weights + 77872);
    dense_7_bias_array.data_start = AI_PTR(weights + 77872);
    dense_7_weights_array.format |= AI_FMT_FLAG_CONST;
    dense_7_weights_array.data = AI_PTR(weights + 63472);
    dense_7_weights_array.data_start = AI_PTR(weights + 63472);
    depthwise_conv2d_6_bias_array.format |= AI_FMT_FLAG_CONST;
    depthwise_conv2d_6_bias_array.data = AI_PTR(weights + 63232);
    depthwise_conv2d_6_bias_array.data_start = AI_PTR(weights + 63232);
    depthwise_conv2d_6_weights_array.format |= AI_FMT_FLAG_CONST;
    depthwise_conv2d_6_weights_array.data = AI_PTR(weights + 57232);
    depthwise_conv2d_6_weights_array.data_start = AI_PTR(weights + 57232);
    conv2d_12_bias_array.format |= AI_FMT_FLAG_CONST;
    conv2d_12_bias_array.data = AI_PTR(weights + 56992);
    conv2d_12_bias_array.data_start = AI_PTR(weights + 56992);
    conv2d_12_weights_array.format |= AI_FMT_FLAG_CONST;
    conv2d_12_weights_array.data = AI_PTR(weights + 54112);
    conv2d_12_weights_array.data_start = AI_PTR(weights + 54112);
    conv2d_11_bias_array.format |= AI_FMT_FLAG_CONST;
    conv2d_11_bias_array.data = AI_PTR(weights + 54064);
    conv2d_11_bias_array.data_start = AI_PTR(weights + 54064);
    conv2d_11_weights_array.format |= AI_FMT_FLAG_CONST;
    conv2d_11_weights_array.data = AI_PTR(weights + 51184);
    conv2d_11_weights_array.data_start = AI_PTR(weights + 51184);
    dense_6_bias_array.format |= AI_FMT_FLAG_CONST;
    dense_6_bias_array.data = AI_PTR(weights + 50944);
    dense_6_bias_array.data_start = AI_PTR(weights + 50944);
    dense_6_weights_array.format |= AI_FMT_FLAG_CONST;
    dense_6_weights_array.data = AI_PTR(weights + 36544);
    dense_6_weights_array.data_start = AI_PTR(weights + 36544);
    dense_5_bias_array.format |= AI_FMT_FLAG_CONST;
    dense_5_bias_array.data = AI_PTR(weights + 36304);
    dense_5_bias_array.data_start = AI_PTR(weights + 36304);
    dense_5_weights_array.format |= AI_FMT_FLAG_CONST;
    dense_5_weights_array.data = AI_PTR(weights + 21904);
    dense_5_weights_array.data_start = AI_PTR(weights + 21904);
    depthwise_conv2d_5_bias_array.format |= AI_FMT_FLAG_CONST;
    depthwise_conv2d_5_bias_array.data = AI_PTR(weights + 21664);
    depthwise_conv2d_5_bias_array.data_start = AI_PTR(weights + 21664);
    depthwise_conv2d_5_weights_array.format |= AI_FMT_FLAG_CONST;
    depthwise_conv2d_5_weights_array.data = AI_PTR(weights + 15664);
    depthwise_conv2d_5_weights_array.data_start = AI_PTR(weights + 15664);
    conv2d_10_bias_array.format |= AI_FMT_FLAG_CONST;
    conv2d_10_bias_array.data = AI_PTR(weights + 15424);
    conv2d_10_bias_array.data_start = AI_PTR(weights + 15424);
    conv2d_10_weights_array.format |= AI_FMT_FLAG_CONST;
    conv2d_10_weights_array.data = AI_PTR(weights + 12544);
    conv2d_10_weights_array.data_start = AI_PTR(weights + 12544);
    conv2d_9_bias_array.format |= AI_FMT_FLAG_CONST;
    conv2d_9_bias_array.data = AI_PTR(weights + 12496);
    conv2d_9_bias_array.data_start = AI_PTR(weights + 12496);
    conv2d_9_weights_array.format |= AI_FMT_FLAG_CONST;
    conv2d_9_weights_array.data = AI_PTR(weights + 11536);
    conv2d_9_weights_array.data_start = AI_PTR(weights + 11536);
    dense_4_bias_array.format |= AI_FMT_FLAG_CONST;
    dense_4_bias_array.data = AI_PTR(weights + 11456);
    dense_4_bias_array.data_start = AI_PTR(weights + 11456);
    dense_4_weights_array.format |= AI_FMT_FLAG_CONST;
    dense_4_weights_array.data = AI_PTR(weights + 9856);
    dense_4_weights_array.data_start = AI_PTR(weights + 9856);
    dense_3_bias_array.format |= AI_FMT_FLAG_CONST;
    dense_3_bias_array.data = AI_PTR(weights + 9776);
    dense_3_bias_array.data_start = AI_PTR(weights + 9776);
    dense_3_weights_array.format |= AI_FMT_FLAG_CONST;
    dense_3_weights_array.data = AI_PTR(weights + 8176);
    dense_3_weights_array.data_start = AI_PTR(weights + 8176);
    depthwise_conv2d_4_bias_array.format |= AI_FMT_FLAG_CONST;
    depthwise_conv2d_4_bias_array.data = AI_PTR(weights + 8096);
    depthwise_conv2d_4_bias_array.data_start = AI_PTR(weights + 8096);
    depthwise_conv2d_4_weights_array.format |= AI_FMT_FLAG_CONST;
    depthwise_conv2d_4_weights_array.data = AI_PTR(weights + 6096);
    depthwise_conv2d_4_weights_array.data_start = AI_PTR(weights + 6096);
    conv2d_8_bias_array.format |= AI_FMT_FLAG_CONST;
    conv2d_8_bias_array.data = AI_PTR(weights + 6016);
    conv2d_8_bias_array.data_start = AI_PTR(weights + 6016);
    conv2d_8_weights_array.format |= AI_FMT_FLAG_CONST;
    conv2d_8_weights_array.data = AI_PTR(weights + 5376);
    conv2d_8_weights_array.data_start = AI_PTR(weights + 5376);
    conv2d_7_bias_array.format |= AI_FMT_FLAG_CONST;
    conv2d_7_bias_array.data = AI_PTR(weights + 5344);
    conv2d_7_bias_array.data_start = AI_PTR(weights + 5344);
    conv2d_7_weights_array.format |= AI_FMT_FLAG_CONST;
    conv2d_7_weights_array.data = AI_PTR(weights + 4704);
    conv2d_7_weights_array.data_start = AI_PTR(weights + 4704);
    depthwise_conv2d_3_bias_array.format |= AI_FMT_FLAG_CONST;
    depthwise_conv2d_3_bias_array.data = AI_PTR(weights + 4624);
    depthwise_conv2d_3_bias_array.data_start = AI_PTR(weights + 4624);
    depthwise_conv2d_3_weights_array.format |= AI_FMT_FLAG_CONST;
    depthwise_conv2d_3_weights_array.data = AI_PTR(weights + 3904);
    depthwise_conv2d_3_weights_array.data_start = AI_PTR(weights + 3904);
    conv2d_6_bias_array.format |= AI_FMT_FLAG_CONST;
    conv2d_6_bias_array.data = AI_PTR(weights + 3824);
    conv2d_6_bias_array.data_start = AI_PTR(weights + 3824);
    conv2d_6_weights_array.format |= AI_FMT_FLAG_CONST;
    conv2d_6_weights_array.data = AI_PTR(weights + 3184);
    conv2d_6_weights_array.data_start = AI_PTR(weights + 3184);
    conv2d_5_bias_array.format |= AI_FMT_FLAG_CONST;
    conv2d_5_bias_array.data = AI_PTR(weights + 3152);
    conv2d_5_bias_array.data_start = AI_PTR(weights + 3152);
    conv2d_5_weights_array.format |= AI_FMT_FLAG_CONST;
    conv2d_5_weights_array.data = AI_PTR(weights + 2512);
    conv2d_5_weights_array.data_start = AI_PTR(weights + 2512);
    depthwise_conv2d_2_bias_array.format |= AI_FMT_FLAG_CONST;
    depthwise_conv2d_2_bias_array.data = AI_PTR(weights + 2432);
    depthwise_conv2d_2_bias_array.data_start = AI_PTR(weights + 2432);
    depthwise_conv2d_2_weights_array.format |= AI_FMT_FLAG_CONST;
    depthwise_conv2d_2_weights_array.data = AI_PTR(weights + 1712);
    depthwise_conv2d_2_weights_array.data_start = AI_PTR(weights + 1712);
    conv2d_4_bias_array.format |= AI_FMT_FLAG_CONST;
    conv2d_4_bias_array.data = AI_PTR(weights + 1632);
    conv2d_4_bias_array.data_start = AI_PTR(weights + 1632);
    conv2d_4_weights_array.format |= AI_FMT_FLAG_CONST;
    conv2d_4_weights_array.data = AI_PTR(weights + 1152);
    conv2d_4_weights_array.data_start = AI_PTR(weights + 1152);
    conv2d_3_bias_array.format |= AI_FMT_FLAG_CONST;
    conv2d_3_bias_array.data = AI_PTR(weights + 1128);
    conv2d_3_bias_array.data_start = AI_PTR(weights + 1128);
    conv2d_3_weights_array.format |= AI_FMT_FLAG_CONST;
    conv2d_3_weights_array.data = AI_PTR(weights + 984);
    conv2d_3_weights_array.data_start = AI_PTR(weights + 984);
    dense_2_bias_array.format |= AI_FMT_FLAG_CONST;
    dense_2_bias_array.data = AI_PTR(weights + 960);
    dense_2_bias_array.data_start = AI_PTR(weights + 960);
    dense_2_weights_array.format |= AI_FMT_FLAG_CONST;
    dense_2_weights_array.data = AI_PTR(weights + 816);
    dense_2_weights_array.data_start = AI_PTR(weights + 816);
    dense_1_bias_array.format |= AI_FMT_FLAG_CONST;
    dense_1_bias_array.data = AI_PTR(weights + 792);
    dense_1_bias_array.data_start = AI_PTR(weights + 792);
    dense_1_weights_array.format |= AI_FMT_FLAG_CONST;
    dense_1_weights_array.data = AI_PTR(weights + 648);
    dense_1_weights_array.data_start = AI_PTR(weights + 648);
    depthwise_conv2d_1_bias_array.format |= AI_FMT_FLAG_CONST;
    depthwise_conv2d_1_bias_array.data = AI_PTR(weights + 624);
    depthwise_conv2d_1_bias_array.data_start = AI_PTR(weights + 624);
    depthwise_conv2d_1_weights_array.format |= AI_FMT_FLAG_CONST;
    depthwise_conv2d_1_weights_array.data = AI_PTR(weights + 408);
    depthwise_conv2d_1_weights_array.data_start = AI_PTR(weights + 408);
    conv2d_2_bias_array.format |= AI_FMT_FLAG_CONST;
    conv2d_2_bias_array.data = AI_PTR(weights + 384);
    conv2d_2_bias_array.data_start = AI_PTR(weights + 384);
    conv2d_2_weights_array.format |= AI_FMT_FLAG_CONST;
    conv2d_2_weights_array.data = AI_PTR(weights + 240);
    conv2d_2_weights_array.data_start = AI_PTR(weights + 240);
    conv2d_1_bias_array.format |= AI_FMT_FLAG_CONST;
    conv2d_1_bias_array.data = AI_PTR(weights + 216);
    conv2d_1_bias_array.data_start = AI_PTR(weights + 216);
    conv2d_1_weights_array.format |= AI_FMT_FLAG_CONST;
    conv2d_1_weights_array.data = AI_PTR(weights + 0);
    conv2d_1_weights_array.data_start = AI_PTR(weights + 0);
  }

  return true;
}


/**  PUBLIC APIs SECTION  *****************************************************/

AI_API_ENTRY
ai_bool ai_network_handc5_get_info(
  ai_handle network, ai_network_report* report)
{
  ai_network* net_ctx = AI_NETWORK_ACQUIRE_CTX(network);

  if ( report && net_ctx )
  {
    ai_network_report r = {
      .model_name        = AI_NETWORK_HANDC5_MODEL_NAME,
      .model_signature   = AI_NETWORK_HANDC5_MODEL_SIGNATURE,
      .model_datetime    = AI_TOOLS_DATE_TIME,
      
      .compile_datetime  = AI_TOOLS_COMPILE_TIME,
      
      .runtime_revision  = ai_platform_runtime_get_revision(),
      .runtime_version   = ai_platform_runtime_get_version(),

      .tool_revision     = AI_TOOLS_REVISION_ID,
      .tool_version      = {AI_TOOLS_VERSION_MAJOR, AI_TOOLS_VERSION_MINOR,
                            AI_TOOLS_VERSION_MICRO, 0x0},
      .tool_api_version  = {AI_TOOLS_API_VERSION_MAJOR, AI_TOOLS_API_VERSION_MINOR,
                            AI_TOOLS_API_VERSION_MICRO, 0x0},

      .api_version            = ai_platform_api_get_version(),
      .interface_api_version  = ai_platform_interface_api_get_version(),
      
      .n_macc            = 410898,
      .n_inputs          = 0,
      .inputs            = NULL,
      .n_outputs         = 0,
      .outputs           = NULL,
      .activations       = AI_STRUCT_INIT,
      .params            = AI_STRUCT_INIT,
      .n_nodes           = 0,
      .signature         = 0x0,
    };

    if ( !ai_platform_api_get_network_report(network, &r) ) return false;

    *report = r;
    return true;
  }

  return false;
}

AI_API_ENTRY
ai_error ai_network_handc5_get_error(ai_handle network)
{
  return ai_platform_network_get_error(network);
}

AI_API_ENTRY
ai_error ai_network_handc5_create(
  ai_handle* network, const ai_buffer* network_config)
{
  return ai_platform_network_create(
    network, network_config, 
    &AI_NET_OBJ_INSTANCE,
    AI_TOOLS_API_VERSION_MAJOR, AI_TOOLS_API_VERSION_MINOR, AI_TOOLS_API_VERSION_MICRO);
}

AI_API_ENTRY
ai_handle ai_network_handc5_destroy(ai_handle network)
{
  return ai_platform_network_destroy(network);
}

AI_API_ENTRY
ai_bool ai_network_handc5_init(
  ai_handle network, const ai_network_params* params)
{
  ai_network* net_ctx = ai_platform_network_init(network, params);
  if ( !net_ctx ) return false;

  ai_bool ok = true;
  ok &= network_handc5_configure_weights(net_ctx, &params->params);
  ok &= network_handc5_configure_activations(net_ctx, &params->activations);

  ok &= ai_platform_network_post_init(network);

  return ok;
}


AI_API_ENTRY
ai_i32 ai_network_handc5_run(
  ai_handle network, const ai_buffer* input, ai_buffer* output)
{
  return ai_platform_network_process(network, input, output);
}

AI_API_ENTRY
ai_i32 ai_network_handc5_forward(ai_handle network, const ai_buffer* input)
{
  return ai_platform_network_process(network, input, NULL);
}


#undef AI_NETWORK_HANDC5_MODEL_SIGNATURE
#undef AI_NET_OBJ_INSTANCE
#undef AI_TOOLS_VERSION_MAJOR
#undef AI_TOOLS_VERSION_MINOR
#undef AI_TOOLS_VERSION_MICRO
#undef AI_TOOLS_API_VERSION_MAJOR
#undef AI_TOOLS_API_VERSION_MINOR
#undef AI_TOOLS_API_VERSION_MICRO
#undef AI_TOOLS_DATE_TIME
#undef AI_TOOLS_COMPILE_TIME

