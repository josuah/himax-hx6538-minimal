/* Generated by Edge Impulse
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */
// Generated on: 20.05.2024 12:53:40

#include <stdio.h>
#include <stdlib.h>
#include "edge-impulse-sdk/tensorflow/lite/c/builtin_op_data.h"
#include "edge-impulse-sdk/tensorflow/lite/c/common.h"
#include "edge-impulse-sdk/tensorflow/lite/micro/micro_mutable_op_resolver.h"
#include "edge-impulse-sdk/porting/ei_classifier_porting.h"

#if EI_CLASSIFIER_PRINT_STATE
#if defined(__cplusplus) && EI_C_LINKAGE == 1
extern "C" {
    extern void ei_printf(const char *format, ...);
}
#else
extern void ei_printf(const char *format, ...);
#endif
#endif

#if defined __GNUC__
#define ALIGN(X) __attribute__((aligned(X)))
#elif defined _MSC_VER
#define ALIGN(X) __declspec(align(X))
#elif defined __TASKING__
#define ALIGN(X) __align(X)
#elif defined __ICCARM__
#define ALIGN(x) __attribute__((aligned(x)))
#endif

#ifndef EI_MAX_SCRATCH_BUFFER_COUNT
#ifndef CONFIG_IDF_TARGET_ESP32S3
#define EI_MAX_SCRATCH_BUFFER_COUNT 4
#else
#define EI_MAX_SCRATCH_BUFFER_COUNT 4
#endif // CONFIG_IDF_TARGET_ESP32S3
#endif // EI_MAX_SCRATCH_BUFFER_COUNT

#ifndef EI_MAX_OVERFLOW_BUFFER_COUNT
#define EI_MAX_OVERFLOW_BUFFER_COUNT 10
#endif // EI_MAX_OVERFLOW_BUFFER_COUNT

using namespace tflite;
using namespace tflite::ops;
using namespace tflite::ops::micro;

namespace {

#if defined(EI_CLASSIFIER_ALLOCATION_STATIC_HIMAX) || defined(EI_CLASSIFIER_ALLOCATION_STATIC_HIMAX_GNU)
constexpr int kTensorArenaSize = 1408;
#else
constexpr int kTensorArenaSize = 384;
#endif

#if defined(EI_CLASSIFIER_ALLOCATION_STATIC)
uint8_t tensor_arena[kTensorArenaSize] ALIGN(16);
#elif defined(EI_CLASSIFIER_ALLOCATION_STATIC_HIMAX)
#pragma Bss(".tensor_arena")
uint8_t tensor_arena[kTensorArenaSize] ALIGN(16);
#pragma Bss()
#elif defined(EI_CLASSIFIER_ALLOCATION_STATIC_HIMAX_GNU)
uint8_t tensor_arena[kTensorArenaSize] ALIGN(16) __attribute__((section(".tensor_arena")));
#else
#define EI_CLASSIFIER_ALLOCATION_HEAP 1
uint8_t* tensor_arena = NULL;
#endif

static uint8_t* tensor_boundary;
static uint8_t* current_location;

template <int SZ, class T> struct TfArray {
  int sz; T elem[SZ];
};

enum used_operators_e {
  OP_FULLY_CONNECTED, OP_SOFTMAX,  OP_LAST
};

struct TensorInfo_t { // subset of TfLiteTensor used for initialization from constant memory
  TfLiteAllocationType allocation_type;
  TfLiteType type;
  void* data;
  TfLiteIntArray* dims;
  size_t bytes;
  TfLiteQuantization quantization;
};

typedef struct {
  TfLiteTensor tensor;
  int16_t index;
} TfLiteTensorWithIndex;

typedef struct {
  TfLiteEvalTensor tensor;
  int16_t index;
} TfLiteEvalTensorWithIndex;

TfLiteContext ctx{};
static const int MAX_TFL_TENSOR_COUNT = 4;
static TfLiteTensorWithIndex tflTensors[MAX_TFL_TENSOR_COUNT];
static const int MAX_TFL_EVAL_COUNT = 4;
static TfLiteEvalTensorWithIndex tflEvalTensors[MAX_TFL_EVAL_COUNT];
TfLiteRegistration registrations[OP_LAST];

namespace g0 {
const TfArray<2, int> tensor_dimension0 = { 2, { 1,39 } };
const TfArray<1, float> quant0_scale = { 1, { 0.12946280837059021, } };
const TfArray<1, int> quant0_zero = { 1, { -85 } };
const TfLiteAffineQuantization quant0 = { (TfLiteFloatArray*)&quant0_scale, (TfLiteIntArray*)&quant0_zero, 0 };
const ALIGN(16) int32_t tensor_data1[6] = { -119, 409, -117, 66, -163, -110, };
const TfArray<1, int> tensor_dimension1 = { 1, { 6 } };
const TfArray<1, float> quant1_scale = { 1, { 0.00052103074267506599, } };
const TfArray<1, int> quant1_zero = { 1, { 0 } };
const TfLiteAffineQuantization quant1 = { (TfLiteFloatArray*)&quant1_scale, (TfLiteIntArray*)&quant1_zero, 0 };
const ALIGN(16) int8_t tensor_data2[6*10] = { 
  28, 66, -35, 65, -92, 54, 26, 54, -62, -22, 
  77, 8, -85, 32, 17, -29, -6, -60, 57, -31, 
  -76, 98, -67, -56, 80, 100, -59, -3, 1, 0, 
  24, -80, 37, -71, 81, -64, -83, 54, -89, 22, 
  56, -59, 6, -127, -34, 39, 48, 43, 91, 15, 
  -40, -11, -66, -3, -8, -73, 72, -27, -16, 105, 
};
const TfArray<2, int> tensor_dimension2 = { 2, { 6,10 } };
const TfArray<1, float> quant2_scale = { 1, { 0.0070633264258503914, } };
const TfArray<1, int> quant2_zero = { 1, { 0 } };
const TfLiteAffineQuantization quant2 = { (TfLiteFloatArray*)&quant2_scale, (TfLiteIntArray*)&quant2_zero, 0 };
const ALIGN(16) int32_t tensor_data3[10] = { 208, -6, -20, 287, 67, 57, -127, -22, 82, -292, };
const TfArray<1, int> tensor_dimension3 = { 1, { 10 } };
const TfArray<1, float> quant3_scale = { 1, { 0.000556748709641397, } };
const TfArray<1, int> quant3_zero = { 1, { 0 } };
const TfLiteAffineQuantization quant3 = { (TfLiteFloatArray*)&quant3_scale, (TfLiteIntArray*)&quant3_zero, 0 };
const ALIGN(16) int8_t tensor_data4[10*20] = { 
  42, -10, -66, 13, 44, 6, 65, -89, -20, -51, 10, 10, 44, 68, 43, -58, -2, -3, -67, -5, 
  -41, -82, 41, 36, 38, -20, -25, -53, -63, 39, -5, -24, 47, 22, 42, 68, -33, 20, -12, 17, 
  -15, -58, -5, 63, -31, -36, -23, 11, -26, -7, 18, 29, -15, -53, -10, -72, 50, -60, -63, -50, 
  46, 18, 74, 8, -65, 44, -51, -22, 4, -10, 80, 45, -57, -42, 17, -40, -8, -22, 25, 16, 
  -19, 54, -59, -19, -46, 45, -9, 11, -47, 52, 63, 45, -1, 27, 35, -50, -37, 65, 68, 47, 
  -59, 24, -41, 16, -14, -68, -15, 44, -68, -14, 72, 54, -22, 35, -21, -24, -23, 40, -5, 30, 
  57, 8, -13, -52, 76, 46, -1, 0, -15, -19, 34, 17, -64, -18, -3, -15, 58, -15, -26, -30, 
  0, 127, 13, 31, -11, 50, 52, -105, 90, 26, -56, 67, 45, 8, -61, 23, 88, 5, -6, 51, 
  45, 16, 15, 23, 73, 18, -67, -24, -65, -5, 62, -6, 7, -66, -26, 59, -42, 47, -24, -32, 
  39, -80, 20, -5, 87, 59, -24, 37, 53, -53, -45, 6, -79, -30, 37, -15, 9, 46, -13, 42, 
};
const TfArray<2, int> tensor_dimension4 = { 2, { 10,20 } };
const TfArray<1, float> quant4_scale = { 1, { 0.0063338028267025948, } };
const TfArray<1, int> quant4_zero = { 1, { 0 } };
const TfLiteAffineQuantization quant4 = { (TfLiteFloatArray*)&quant4_scale, (TfLiteIntArray*)&quant4_zero, 0 };
const ALIGN(16) int32_t tensor_data5[20] = { 214, 212, 19, -56, -50, -44, -54, -237, -143, 97, 251, 74, 196, -71, 50, 8, 55, -57, 92, 44, };
const TfArray<1, int> tensor_dimension5 = { 1, { 20 } };
const TfArray<1, float> quant5_scale = { 1, { 0.00067777856020256877, } };
const TfArray<1, int> quant5_zero = { 1, { 0 } };
const TfLiteAffineQuantization quant5 = { (TfLiteFloatArray*)&quant5_scale, (TfLiteIntArray*)&quant5_zero, 0 };
const ALIGN(16) int8_t tensor_data6[20*39] = { 
  32, -5, -15, 79, 38, 64, 51, -7, 44, -23, 20, 47, -31, -25, -18, 41, 27, 38, -21, -25, 57, -11, 48, 13, -57, 31, 12, -25, -45, -27, 28, 59, 76, -7, -15, 67, 1, 79, 27, 
  -127, 18, 23, 33, 3, 1, 10, 22, 27, 41, -67, -23, -52, 47, 83, -74, 71, 46, 92, 5, 33, 30, 82, 15, 42, 44, 5, 32, 2, -14, 16, 64, 48, 63, 49, -8, -37, 2, 18, 
  68, 14, -30, -43, 20, -11, 61, 80, 88, 29, -14, 40, -13, 49, -29, 17, 21, 34, 4, 28, 71, -46, 43, -33, 7, -25, 12, 40, -41, -41, -24, 14, 1, -72, -41, -51, -84, -39, -5, 
  -56, 31, -14, 28, -12, 26, -14, 49, 19, -59, 42, 42, 26, 28, 36, 0, -54, -37, 8, 25, -54, 10, -52, -38, 22, 15, -51, 43, 43, 32, -33, 17, 42, -35, 28, -1, 50, -66, 2, 
  67, 36, -11, -32, -29, -44, 39, -35, -49, 9, -33, 1, -34, -47, -30, 19, 33, 41, -1, -10, -25, 39, -34, -34, -47, -24, 82, 71, -97, -45, 36, -4, 91, -9, 12, 0, 12, -24, 44, 
  -22, 45, 6, -8, -21, -44, 10, 64, -19, 18, -51, -17, -55, 25, 7, -4, -61, 12, -14, 10, -42, -44, -1, -25, 49, -11, 11, -8, -33, 53, -16, -12, 41, -42, 9, -61, 36, -39, 20, 
  7, -1, -34, -35, -26, -50, -11, 51, -12, 13, 7, 5, 10, 47, 28, -22, -23, -12, 71, 59, -19, 21, -18, 49, -15, 3, 53, -46, 44, 5, 25, -49, 8, 0, -40, 43, -46, 35, -34, 
  93, -23, -30, 30, 42, 27, 23, 21, -17, 55, 2, 86, 24, 4, -36, -57, -34, -29, -48, 57, 37, -48, -42, -11, -11, 64, -37, 2, -86, -33, -5, 51, 24, -54, -33, -70, -24, 7, 38, 
  50, 104, 19, -10, -22, 37, -16, -47, -9, -13, 6, 40, 42, 9, 34, -53, 44, 15, 92, 46, 72, 47, 63, 33, 45, 50, 1, -2, 41, -69, 23, -53, -10, -7, -61, -30, -63, 42, -43, 
  -72, 35, -47, 13, -12, 17, -71, -16, -45, -74, -64, -57, -74, 16, -16, 54, 11, 54, 36, -45, -19, 31, 8, 49, 53, 34, -1, -2, 19, 39, -28, -17, -40, 1, 47, 20, 43, 45, -17, 
  113, -39, 58, -28, -26, 0, 66, 40, 20, 41, -31, 29, 21, -47, 3, -1, 14, -8, 13, -5, -51, 26, -62, 44, 49, -23, 18, -37, 63, 78, -16, -40, -34, 3, 78, 69, -4, -37, -46, 
  -37, 35, 53, -2, -23, -51, 61, -17, 61, -8, 51, 40, 21, 97, -10, 20, 2, 15, 90, 39, 23, 15, 84, 25, 35, 39, 25, 15, 76, 32, 3, -23, -11, -38, 3, -27, -56, -1, -5, 
  -83, -82, -12, 2, 49, -53, -35, -54, -50, 26, -20, -62, 2, 41, -17, 37, 27, -10, 61, -11, 36, 42, -30, 41, 34, -14, 43, -40, 63, 13, 7, 41, -38, 80, 68, 70, 83, -5, -15, 
  -5, -64, -18, -13, 44, 47, -60, 50, -15, 9, 6, 10, 45, 13, -20, -45, -68, -15, -50, -27, -46, -45, 3, -9, -13, -42, -51, 57, -14, -6, 6, -74, -63, -74, -23, -1, 13, -17, 37, 
  98, -81, -39, 18, 12, -12, 9, 50, 61, 53, 95, -25, -15, 70, -66, -8, -46, 20, 37, -51, 63, 71, -10, 65, -30, -13, -29, -72, 69, -25, 37, -29, -5, 59, 14, -20, -31, 31, 63, 
  -4, 46, -47, -57, -57, 5, -19, -2, 10, 8, -16, 11, -47, 27, 23, 54, 8, 23, 21, 35, -55, 52, -49, 26, 16, -1, -16, -23, -10, -33, 17, -20, -20, 32, -3, 12, -2, -57, 15, 
  15, -5, -44, 7, 59, 1, -14, -39, 26, -49, -47, -14, 20, 55, -3, 0, -34, -13, 38, -1, -41, -32, 27, -27, 22, -18, -30, 50, -50, 46, 17, 0, 37, -47, -33, -47, 2, -69, -45, 
  14, 13, 48, 37, 44, -34, -41, 32, 4, -26, -39, 43, -35, 19, -30, -6, -49, 13, 42, -44, 18, -39, -58, 45, -45, -59, 9, 9, 5, -41, -8, -21, 35, 41, 28, 82, 82, -6, 68, 
  -47, -34, 51, -57, 19, -1, -43, -8, -3, -23, -58, -27, -59, -29, -63, -13, 10, -45, -28, 30, -23, 58, -25, 55, -19, -49, -14, -36, -36, -34, 32, -46, -59, -74, 40, 29, -2, -60, -66, 
  -4, 47, 16, -22, -1, -32, 6, -18, 35, 32, -59, 36, 28, -10, 25, -12, 7, -35, -55, -7, -46, -76, 25, -71, -29, -22, -44, -33, 31, 53, -41, -4, 38, -51, -9, 6, -71, -5, -16, 
};
const TfArray<2, int> tensor_dimension6 = { 2, { 20,39 } };
const TfArray<1, float> quant6_scale = { 1, { 0.0052353148348629475, } };
const TfArray<1, int> quant6_zero = { 1, { 0 } };
const TfLiteAffineQuantization quant6 = { (TfLiteFloatArray*)&quant6_scale, (TfLiteIntArray*)&quant6_zero, 0 };
const TfArray<2, int> tensor_dimension7 = { 2, { 1,20 } };
const TfArray<1, float> quant7_scale = { 1, { 0.087901175022125244, } };
const TfArray<1, int> quant7_zero = { 1, { -128 } };
const TfLiteAffineQuantization quant7 = { (TfLiteFloatArray*)&quant7_scale, (TfLiteIntArray*)&quant7_zero, 0 };
const TfArray<2, int> tensor_dimension8 = { 2, { 1,10 } };
const TfArray<1, float> quant8_scale = { 1, { 0.07376563549041748, } };
const TfArray<1, int> quant8_zero = { 1, { -128 } };
const TfLiteAffineQuantization quant8 = { (TfLiteFloatArray*)&quant8_scale, (TfLiteIntArray*)&quant8_zero, 0 };
const TfArray<2, int> tensor_dimension9 = { 2, { 1,6 } };
const TfArray<1, float> quant9_scale = { 1, { 0.12478320300579071, } };
const TfArray<1, int> quant9_zero = { 1, { -18 } };
const TfLiteAffineQuantization quant9 = { (TfLiteFloatArray*)&quant9_scale, (TfLiteIntArray*)&quant9_zero, 0 };
const TfArray<2, int> tensor_dimension10 = { 2, { 1,6 } };
const TfArray<1, float> quant10_scale = { 1, { 0.00390625, } };
const TfArray<1, int> quant10_zero = { 1, { -128 } };
const TfLiteAffineQuantization quant10 = { (TfLiteFloatArray*)&quant10_scale, (TfLiteIntArray*)&quant10_zero, 0 };
const TfLiteFullyConnectedParams opdata0 = { kTfLiteActRelu, kTfLiteFullyConnectedWeightsFormatDefault, false, false };
const TfArray<3, int> inputs0 = { 3, { 0,6,5 } };
const TfArray<1, int> outputs0 = { 1, { 7 } };
const TfLiteFullyConnectedParams opdata1 = { kTfLiteActRelu, kTfLiteFullyConnectedWeightsFormatDefault, false, false };
const TfArray<3, int> inputs1 = { 3, { 7,4,3 } };
const TfArray<1, int> outputs1 = { 1, { 8 } };
const TfLiteFullyConnectedParams opdata2 = { kTfLiteActNone, kTfLiteFullyConnectedWeightsFormatDefault, false, false };
const TfArray<3, int> inputs2 = { 3, { 8,2,1 } };
const TfArray<1, int> outputs2 = { 1, { 9 } };
const TfLiteSoftmaxParams opdata3 = { 1 };
const TfArray<1, int> inputs3 = { 1, { 9 } };
const TfArray<1, int> outputs3 = { 1, { 10 } };
};

TensorInfo_t tensorData[] = {
{ kTfLiteArenaRw, kTfLiteInt8, (int32_t*)(tensor_arena + 0), (TfLiteIntArray*)&g0::tensor_dimension0, 39, {kTfLiteAffineQuantization, const_cast<void*>(static_cast<const void*>(&g0::quant0))}, },
{ kTfLiteMmapRo, kTfLiteInt32, (int32_t*)g0::tensor_data1, (TfLiteIntArray*)&g0::tensor_dimension1, 24, {kTfLiteAffineQuantization, const_cast<void*>(static_cast<const void*>(&g0::quant1))}, },
{ kTfLiteMmapRo, kTfLiteInt8, (int32_t*)g0::tensor_data2, (TfLiteIntArray*)&g0::tensor_dimension2, 60, {kTfLiteAffineQuantization, const_cast<void*>(static_cast<const void*>(&g0::quant2))}, },
{ kTfLiteMmapRo, kTfLiteInt32, (int32_t*)g0::tensor_data3, (TfLiteIntArray*)&g0::tensor_dimension3, 40, {kTfLiteAffineQuantization, const_cast<void*>(static_cast<const void*>(&g0::quant3))}, },
{ kTfLiteMmapRo, kTfLiteInt8, (int32_t*)g0::tensor_data4, (TfLiteIntArray*)&g0::tensor_dimension4, 200, {kTfLiteAffineQuantization, const_cast<void*>(static_cast<const void*>(&g0::quant4))}, },
{ kTfLiteMmapRo, kTfLiteInt32, (int32_t*)g0::tensor_data5, (TfLiteIntArray*)&g0::tensor_dimension5, 80, {kTfLiteAffineQuantization, const_cast<void*>(static_cast<const void*>(&g0::quant5))}, },
{ kTfLiteMmapRo, kTfLiteInt8, (int32_t*)g0::tensor_data6, (TfLiteIntArray*)&g0::tensor_dimension6, 780, {kTfLiteAffineQuantization, const_cast<void*>(static_cast<const void*>(&g0::quant6))}, },
{ kTfLiteArenaRw, kTfLiteInt8, (int32_t*)(tensor_arena + 48), (TfLiteIntArray*)&g0::tensor_dimension7, 20, {kTfLiteAffineQuantization, const_cast<void*>(static_cast<const void*>(&g0::quant7))}, },
{ kTfLiteArenaRw, kTfLiteInt8, (int32_t*)(tensor_arena + 0), (TfLiteIntArray*)&g0::tensor_dimension8, 10, {kTfLiteAffineQuantization, const_cast<void*>(static_cast<const void*>(&g0::quant8))}, },
{ kTfLiteArenaRw, kTfLiteInt8, (int32_t*)(tensor_arena + 16), (TfLiteIntArray*)&g0::tensor_dimension9, 6, {kTfLiteAffineQuantization, const_cast<void*>(static_cast<const void*>(&g0::quant9))}, },
{ kTfLiteArenaRw, kTfLiteInt8, (int32_t*)(tensor_arena + 0), (TfLiteIntArray*)&g0::tensor_dimension10, 6, {kTfLiteAffineQuantization, const_cast<void*>(static_cast<const void*>(&g0::quant10))}, },
};

#ifndef TF_LITE_STATIC_MEMORY
TfLiteNode tflNodes[4] = {
{ (TfLiteIntArray*)&g0::inputs0, (TfLiteIntArray*)&g0::outputs0, (TfLiteIntArray*)&g0::inputs0, nullptr, nullptr, const_cast<void*>(static_cast<const void*>(&g0::opdata0)), nullptr, 0, },
{ (TfLiteIntArray*)&g0::inputs1, (TfLiteIntArray*)&g0::outputs1, (TfLiteIntArray*)&g0::inputs1, nullptr, nullptr, const_cast<void*>(static_cast<const void*>(&g0::opdata1)), nullptr, 0, },
{ (TfLiteIntArray*)&g0::inputs2, (TfLiteIntArray*)&g0::outputs2, (TfLiteIntArray*)&g0::inputs2, nullptr, nullptr, const_cast<void*>(static_cast<const void*>(&g0::opdata2)), nullptr, 0, },
{ (TfLiteIntArray*)&g0::inputs3, (TfLiteIntArray*)&g0::outputs3, (TfLiteIntArray*)&g0::inputs3, nullptr, nullptr, const_cast<void*>(static_cast<const void*>(&g0::opdata3)), nullptr, 0, },
};
#else
TfLiteNode tflNodes[4] = {
{ (TfLiteIntArray*)&g0::inputs0, (TfLiteIntArray*)&g0::outputs0, (TfLiteIntArray*)&g0::inputs0, nullptr, const_cast<void*>(static_cast<const void*>(&g0::opdata0)), nullptr, 0, },
{ (TfLiteIntArray*)&g0::inputs1, (TfLiteIntArray*)&g0::outputs1, (TfLiteIntArray*)&g0::inputs1, nullptr, const_cast<void*>(static_cast<const void*>(&g0::opdata1)), nullptr, 0, },
{ (TfLiteIntArray*)&g0::inputs2, (TfLiteIntArray*)&g0::outputs2, (TfLiteIntArray*)&g0::inputs2, nullptr, const_cast<void*>(static_cast<const void*>(&g0::opdata2)), nullptr, 0, },
{ (TfLiteIntArray*)&g0::inputs3, (TfLiteIntArray*)&g0::outputs3, (TfLiteIntArray*)&g0::inputs3, nullptr, const_cast<void*>(static_cast<const void*>(&g0::opdata3)), nullptr, 0, },
};
#endif

used_operators_e used_ops[] =
{OP_FULLY_CONNECTED, OP_FULLY_CONNECTED, OP_FULLY_CONNECTED, OP_SOFTMAX, };


// Indices into tflTensors and tflNodes for subgraphs
const size_t tflTensors_subgraph_index[] = {0, 11, };
const size_t tflNodes_subgraph_index[] = {0, 4, };

// Input/output tensors
static const int in_tensor_indices[] = {
  0, 
};

static const int out_tensor_indices[] = {
  10, 
};


size_t current_subgraph_index = 0;

static void init_tflite_tensor(size_t i, TfLiteTensor *tensor) {
  tensor->type = tensorData[i].type;
  tensor->is_variable = false;

#if defined(EI_CLASSIFIER_ALLOCATION_HEAP)
  tensor->allocation_type = tensorData[i].allocation_type;
#else
  tensor->allocation_type = (tensor_arena <= tensorData[i].data && tensorData[i].data < tensor_arena + kTensorArenaSize) ? kTfLiteArenaRw : kTfLiteMmapRo;
#endif
  tensor->bytes = tensorData[i].bytes;
  tensor->dims = tensorData[i].dims;

#if defined(EI_CLASSIFIER_ALLOCATION_HEAP)
  if(tensor->allocation_type == kTfLiteArenaRw){
    uint8_t* start = (uint8_t*) ((uintptr_t)tensorData[i].data + (uintptr_t) tensor_arena);

    tensor->data.data =  start;
  }
  else {
      tensor->data.data = tensorData[i].data;
  }
#else
  tensor->data.data = tensorData[i].data;
#endif // EI_CLASSIFIER_ALLOCATION_HEAP
  tensor->quantization = tensorData[i].quantization;
  if (tensor->quantization.type == kTfLiteAffineQuantization) {
    TfLiteAffineQuantization const* quant = ((TfLiteAffineQuantization const*)(tensorData[i].quantization.params));
    tensor->params.scale = quant->scale->data[0];
    tensor->params.zero_point = quant->zero_point->data[0];
  }

}

static void init_tflite_eval_tensor(int i, TfLiteEvalTensor *tensor) {

  tensor->type = tensorData[i].type;

  tensor->dims = tensorData[i].dims;

#if defined(EI_CLASSIFIER_ALLOCATION_HEAP)
  auto allocation_type = tensorData[i].allocation_type;
  if(allocation_type == kTfLiteArenaRw) {
    uint8_t* start = (uint8_t*) ((uintptr_t)tensorData[i].data + (uintptr_t) tensor_arena);

    tensor->data.data =  start;
  }
  else {
    tensor->data.data = tensorData[i].data;
  }
#else
  tensor->data.data = tensorData[i].data;
#endif // EI_CLASSIFIER_ALLOCATION_HEAP
}

static void* overflow_buffers[EI_MAX_OVERFLOW_BUFFER_COUNT];
static size_t overflow_buffers_ix = 0;
static void * AllocatePersistentBufferImpl(struct TfLiteContext* ctx,
                                       size_t bytes) {
  void *ptr;
  uint32_t align_bytes = (bytes % 16) ? 16 - (bytes % 16) : 0;

  if (current_location - (bytes + align_bytes) < tensor_boundary) {
    if (overflow_buffers_ix > EI_MAX_OVERFLOW_BUFFER_COUNT - 1) {
      ei_printf("ERR: Failed to allocate persistent buffer of size %d, does not fit in tensor arena and reached EI_MAX_OVERFLOW_BUFFER_COUNT\n",
        (int)bytes);
      return NULL;
    }

    // OK, this will look super weird, but.... we have CMSIS-NN buffers which
    // we cannot calculate beforehand easily.
    ptr = ei_calloc(bytes, 1);
    if (ptr == NULL) {
      ei_printf("ERR: Failed to allocate persistent buffer of size %d\n", (int)bytes);
      return NULL;
    }
    overflow_buffers[overflow_buffers_ix++] = ptr;
    return ptr;
  }

  current_location -= bytes;

  // align to the left aligned boundary of 16 bytes
  current_location -= 15; // for alignment
  current_location += 16 - ((uintptr_t)(current_location) & 15);

  ptr = current_location;
  memset(ptr, 0, bytes);

  return ptr;
}

typedef struct {
  size_t bytes;
  void *ptr;
} scratch_buffer_t;

static scratch_buffer_t scratch_buffers[EI_MAX_SCRATCH_BUFFER_COUNT];
static size_t scratch_buffers_ix = 0;

static TfLiteStatus RequestScratchBufferInArenaImpl(struct TfLiteContext* ctx, size_t bytes,
                                                int* buffer_idx) {
  if (scratch_buffers_ix > EI_MAX_SCRATCH_BUFFER_COUNT - 1) {
    ei_printf("ERR: Failed to allocate scratch buffer of size %d, reached EI_MAX_SCRATCH_BUFFER_COUNT\n",
      (int)bytes);
    return kTfLiteError;
  }

  scratch_buffer_t b;
  b.bytes = bytes;

  b.ptr = AllocatePersistentBufferImpl(ctx, b.bytes);
  if (!b.ptr) {
    ei_printf("ERR: Failed to allocate scratch buffer of size %d\n",
      (int)bytes);
    return kTfLiteError;
  }

  scratch_buffers[scratch_buffers_ix] = b;
  *buffer_idx = scratch_buffers_ix;

  scratch_buffers_ix++;

  return kTfLiteOk;
}

static void* GetScratchBufferImpl(struct TfLiteContext* ctx, int buffer_idx) {
  if (buffer_idx > (int)scratch_buffers_ix) {
    return NULL;
  }
  return scratch_buffers[buffer_idx].ptr;
}

static const uint16_t TENSOR_IX_UNUSED = 0x7FFF;

static void ResetTensors() {
  for (size_t ix = 0; ix < MAX_TFL_TENSOR_COUNT; ix++) {
    tflTensors[ix].index = TENSOR_IX_UNUSED;
  }
  for (size_t ix = 0; ix < MAX_TFL_EVAL_COUNT; ix++) {
    tflEvalTensors[ix].index = TENSOR_IX_UNUSED;
  }
}

static TfLiteTensor* GetTensorImpl(const struct TfLiteContext* context,
                               int tensor_idx) {

  tensor_idx = tflTensors_subgraph_index[current_subgraph_index] + tensor_idx;

  for (size_t ix = 0; ix < MAX_TFL_TENSOR_COUNT; ix++) {
    // already used? OK!
    if (tflTensors[ix].index == tensor_idx) {
      return &tflTensors[ix].tensor;
    }
    // passed all the ones we've used, so end of the list?
    if (tflTensors[ix].index == TENSOR_IX_UNUSED) {
      // init the tensor
      init_tflite_tensor(tensor_idx, &tflTensors[ix].tensor);
      tflTensors[ix].index = tensor_idx;
      return &tflTensors[ix].tensor;
    }
  }

  ei_printf("ERR: GetTensor called beyond MAX_TFL_TENSOR_COUNT (%d)\n", MAX_TFL_TENSOR_COUNT);
  return nullptr;
}

static TfLiteEvalTensor* GetEvalTensorImpl(const struct TfLiteContext* context,
                                       int tensor_idx) {

  tensor_idx = tflTensors_subgraph_index[current_subgraph_index] + tensor_idx;

  for (size_t ix = 0; ix < MAX_TFL_EVAL_COUNT; ix++) {
    // already used? OK!
    if (tflEvalTensors[ix].index == tensor_idx) {
      return &tflEvalTensors[ix].tensor;
    }
    // passed all the ones we've used, so end of the list?
    if (tflEvalTensors[ix].index == TENSOR_IX_UNUSED) {
      // init the tensor
      init_tflite_eval_tensor(tensor_idx, &tflEvalTensors[ix].tensor);
      tflEvalTensors[ix].index = tensor_idx;
      return &tflEvalTensors[ix].tensor;
    }
  }

  ei_printf("ERR: GetTensor called beyond MAX_TFL_EVAL_COUNT (%d)\n", (int)MAX_TFL_EVAL_COUNT);
  return nullptr;
}

class EonMicroContext : public MicroContext {
 public:
 
  EonMicroContext(): MicroContext(nullptr, nullptr, nullptr) { }

  void* AllocatePersistentBuffer(size_t bytes) {
    return AllocatePersistentBufferImpl(nullptr, bytes);
  }

  TfLiteStatus RequestScratchBufferInArena(size_t bytes,
                                           int* buffer_index) {
  return RequestScratchBufferInArenaImpl(nullptr, bytes, buffer_index);
  }

  void* GetScratchBuffer(int buffer_index) {
    return GetScratchBufferImpl(nullptr, buffer_index);
  }
 
  TfLiteTensor* AllocateTempTfLiteTensor(int tensor_index) {
    return GetTensorImpl(nullptr, tensor_index);
  }

  void DeallocateTempTfLiteTensor(TfLiteTensor* tensor) {
    return;
  }

  bool IsAllTempTfLiteTensorDeallocated() {
    return true;
  }

  TfLiteEvalTensor* GetEvalTensor(int tensor_index) {
    return GetEvalTensorImpl(nullptr, tensor_index);
  }

};


} // namespace

TfLiteStatus tflite_learn_22_init( void*(*alloc_fnc)(size_t,size_t) ) {
#ifdef EI_CLASSIFIER_ALLOCATION_HEAP
  tensor_arena = (uint8_t*) alloc_fnc(16, kTensorArenaSize);
  if (!tensor_arena) {
    ei_printf("ERR: failed to allocate tensor arena\n");
    return kTfLiteError;
  }
#else
  memset(tensor_arena, 0, kTensorArenaSize);
#endif
  tensor_boundary = tensor_arena;
  current_location = tensor_arena + kTensorArenaSize;

  EonMicroContext micro_context_;
  
  // Set microcontext as the context ptr
  ctx.impl_ = static_cast<void*>(&micro_context_);
  // Setup tflitecontext functions
  ctx.AllocatePersistentBuffer = &AllocatePersistentBufferImpl;
  ctx.RequestScratchBufferInArena = &RequestScratchBufferInArenaImpl;
  ctx.GetScratchBuffer = &GetScratchBufferImpl;
  ctx.GetTensor = &GetTensorImpl;
  ctx.GetEvalTensor = &GetEvalTensorImpl;
  ctx.ReportError = &MicroContextReportOpError;

  ctx.tensors_size = 11;
  for (size_t i = 0; i < 11; ++i) {
    TfLiteTensor tensor;
    init_tflite_tensor(i, &tensor);
    if (tensor.allocation_type == kTfLiteArenaRw) {
      auto data_end_ptr = (uint8_t*)tensor.data.data + tensorData[i].bytes;
      if (data_end_ptr > tensor_boundary) {
        tensor_boundary = data_end_ptr;
      }
    }
  }

  if (tensor_boundary > current_location /* end of arena size */) {
    ei_printf("ERR: tensor arena is too small, does not fit model - even without scratch buffers\n");
    return kTfLiteError;
  }

  registrations[OP_FULLY_CONNECTED] = Register_FULLY_CONNECTED();
  registrations[OP_SOFTMAX] = Register_SOFTMAX();

  for (size_t g = 0; g < 1; ++g) {
    current_subgraph_index = g;
    for(size_t i = tflNodes_subgraph_index[g]; i < tflNodes_subgraph_index[g+1]; ++i) {
      if (registrations[used_ops[i]].init) {
        tflNodes[i].user_data = registrations[used_ops[i]].init(&ctx, (const char*)tflNodes[i].builtin_data, 0);
      }
    }
  }
  current_subgraph_index = 0;

  for(size_t g = 0; g < 1; ++g) {
    current_subgraph_index = g;
    for(size_t i = tflNodes_subgraph_index[g]; i < tflNodes_subgraph_index[g+1]; ++i) {
      if (registrations[used_ops[i]].prepare) {
        ResetTensors();
        TfLiteStatus status = registrations[used_ops[i]].prepare(&ctx, &tflNodes[i]);
        if (status != kTfLiteOk) {
          return status;
        }
      }
    }
  }
  current_subgraph_index = 0;

  return kTfLiteOk;
}

TfLiteStatus tflite_learn_22_input(int index, TfLiteTensor *tensor) {
  init_tflite_tensor(in_tensor_indices[index], tensor);
  return kTfLiteOk;
}

TfLiteStatus tflite_learn_22_output(int index, TfLiteTensor *tensor) {
  init_tflite_tensor(out_tensor_indices[index], tensor);
  return kTfLiteOk;
}

TfLiteStatus tflite_learn_22_invoke() {
  for (size_t i = 0; i < 4; ++i) {
    ResetTensors();

    TfLiteStatus status = registrations[used_ops[i]].invoke(&ctx, &tflNodes[i]);

#if EI_CLASSIFIER_PRINT_STATE
    ei_printf("layer %lu\n", i);
    ei_printf("    inputs:\n");
    for (size_t ix = 0; ix < tflNodes[i].inputs->size; ix++) {
      auto d = tensorData[tflNodes[i].inputs->data[ix]];

      size_t data_ptr = (size_t)d.data;

      if (d.allocation_type == kTfLiteArenaRw) {
        data_ptr = (size_t)tensor_arena + data_ptr;
      }

      if (d.type == TfLiteType::kTfLiteInt8) {
        int8_t* data = (int8_t*)data_ptr;
        ei_printf("        %lu (%zu bytes, ptr=%p, alloc_type=%d, type=%d): ", ix, d.bytes, data, (int)d.allocation_type, (int)d.type);
        for (size_t jx = 0; jx < d.bytes; jx++) {
          ei_printf("%d ", data[jx]);
        }
      }
      else {
        float* data = (float*)data_ptr;
        ei_printf("        %lu (%zu bytes, ptr=%p, alloc_type=%d, type=%d): ", ix, d.bytes, data, (int)d.allocation_type, (int)d.type);
        for (size_t jx = 0; jx < d.bytes / 4; jx++) {
          ei_printf("%f ", data[jx]);
        }
      }
      ei_printf("\n");
    }
    ei_printf("\n");

    ei_printf("    outputs:\n");
    for (size_t ix = 0; ix < tflNodes[i].outputs->size; ix++) {
      auto d = tensorData[tflNodes[i].outputs->data[ix]];

      size_t data_ptr = (size_t)d.data;

      if (d.allocation_type == kTfLiteArenaRw) {
        data_ptr = (size_t)tensor_arena + data_ptr;
      }

      if (d.type == TfLiteType::kTfLiteInt8) {
        int8_t* data = (int8_t*)data_ptr;
        ei_printf("        %lu (%zu bytes, ptr=%p, alloc_type=%d, type=%d): ", ix, d.bytes, data, (int)d.allocation_type, (int)d.type);
        for (size_t jx = 0; jx < d.bytes; jx++) {
          ei_printf("%d ", data[jx]);
        }
      }
      else {
        float* data = (float*)data_ptr;
        ei_printf("        %lu (%zu bytes, ptr=%p, alloc_type=%d, type=%d): ", ix, d.bytes, data, (int)d.allocation_type, (int)d.type);
        for (size_t jx = 0; jx < d.bytes / 4; jx++) {
          ei_printf("%f ", data[jx]);
        }
      }
      ei_printf("\n");
    }
    ei_printf("\n");
#endif // EI_CLASSIFIER_PRINT_STATE

    if (status != kTfLiteOk) {
      return status;
    }
  }
  return kTfLiteOk;
}

TfLiteStatus tflite_learn_22_reset( void (*free_fnc)(void* ptr) ) {
#ifdef EI_CLASSIFIER_ALLOCATION_HEAP
  free_fnc(tensor_arena);
#endif

  // scratch buffers are allocated within the arena, so just reset the counter so memory can be reused
  scratch_buffers_ix = 0;

  // overflow buffers are on the heap, so free them first
  for (size_t ix = 0; ix < overflow_buffers_ix; ix++) {
    ei_free(overflow_buffers[ix]);
  }
  overflow_buffers_ix = 0;
  return kTfLiteOk;
}
