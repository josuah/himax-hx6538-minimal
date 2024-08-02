/*
 * Copyright (c) 2024 EdgeImpulse Inc.
 *
 * Generated by Edge Impulse and licensed under the applicable Edge Impulse
 * Terms of Service. Community and Professional Terms of Service
 * (https://docs.edgeimpulse.com/page/terms-of-service) or Enterprise Terms of
 * Service (https://docs.edgeimpulse.com/page/enterprise-terms-of-service),
 * according to your product plan subscription (the “License”).
 *
 * This software, documentation and other associated files (collectively referred
 * to as the “Software”) is a single SDK variation generated by the Edge Impulse
 * platform and requires an active paid Edge Impulse subscription to use this
 * Software for any purpose.
 *
 * You may NOT use this Software unless you have an active Edge Impulse subscription
 * that meets the eligibility requirements for the applicable License, subject to
 * your full and continued compliance with the terms and conditions of the License,
 * including without limitation any usage restrictions under the applicable License.
 *
 * If you do not have an active Edge Impulse product plan subscription, or if use
 * of this Software exceeds the usage limitations of your Edge Impulse product plan
 * subscription, you are not permitted to use this Software and must immediately
 * delete and erase all copies of this Software within your control or possession.
 * Edge Impulse reserves all rights and remedies available to enforce its rights.
 *
 * Unless required by applicable law or agreed to in writing, the Software is
 * distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND,
 * either express or implied. See the License for the specific language governing
 * permissions, disclaimers and limitations under the License.
 */

#ifndef _EI_CLASSIFIER_MODEL_VARIABLES_H_
#define _EI_CLASSIFIER_MODEL_VARIABLES_H_

#include <stdint.h>
#include "model_metadata.h"

#include "tflite-model/tflite_learn_5_compiled.h"
#include "edge-impulse-sdk/classifier/ei_model_types.h"
#include "edge-impulse-sdk/classifier/inferencing_engines/engines.h"

const char* ei_classifier_inferencing_categories[] = { "helloworld", "noise", "unknown" };

uint8_t ei_dsp_config_25_axes[] = { 0 };
const uint32_t ei_dsp_config_25_axes_size = 1;
ei_dsp_config_mfcc_t ei_dsp_config_25 = {
    25, // uint32_t blockId
    3, // int implementationVersion
    1, // int length of axes
    NULL, // named axes
    0, // size of the named axes array
    13, // int num_cepstral
    0.02f, // float frame_length
    0.02f, // float frame_stride
    32, // int num_filters
    256, // int fft_length
    101, // int win_size
    300, // int low_frequency
    0, // int high_frequency
    0.98f, // float pre_cof
    1 // int pre_shift
};

const size_t ei_dsp_blocks_size = 1;
ei_model_dsp_t ei_dsp_blocks[ei_dsp_blocks_size] = {
    { // DSP block 25
        25,
        624, // output size
        &extract_mfcc_features, // DSP function pointer
        (void*)&ei_dsp_config_25, // pointer to config struct
        ei_dsp_config_25_axes, // array of offsets into the input stream, one for each axis
        ei_dsp_config_25_axes_size, // number of axes
        1, // version
        nullptr, // factory function
    }
};
const ei_config_tflite_eon_graph_t ei_config_tflite_graph_5 = {
    .implementation_version = 1,
    .model_init = &tflite_learn_5_init,
    .model_invoke = &tflite_learn_5_invoke,
    .model_reset = &tflite_learn_5_reset,
    .model_input = &tflite_learn_5_input,
    .model_output = &tflite_learn_5_output,
};

const ei_learning_block_config_tflite_graph_t ei_learning_block_config_5 = {
    .implementation_version = 1,
    .classification_mode = EI_CLASSIFIER_CLASSIFICATION_MODE_CLASSIFICATION,
    .block_id = 5,
    .object_detection = 0,
    .object_detection_last_layer = EI_CLASSIFIER_LAST_LAYER_UNKNOWN,
    .output_data_tensor = 0,
    .output_labels_tensor = 1,
    .output_score_tensor = 2,
    .threshold = 0,
    .quantized = 1,
    .compiled = 1,
    .graph_config = (void*)&ei_config_tflite_graph_5
};

const size_t ei_learning_blocks_size = 1;
const uint32_t ei_learning_block_5_inputs[1] = { 25 };
const uint32_t ei_learning_block_5_inputs_size = 1;
const ei_learning_block_t ei_learning_blocks[ei_learning_blocks_size] = {
    {
        5,
        false,
        &run_nn_inference,
        (void*)&ei_learning_block_config_5,
        EI_CLASSIFIER_IMAGE_SCALING_NONE,
        ei_learning_block_5_inputs,
        ei_learning_block_5_inputs_size,
        3
    },
};

const ei_model_performance_calibration_t ei_calibration = {
    1, /* integer version number */
    false, /* has configured performance calibration */
    (int32_t)(EI_CLASSIFIER_RAW_SAMPLE_COUNT / ((EI_CLASSIFIER_FREQUENCY > 0) ? EI_CLASSIFIER_FREQUENCY : 1)) * 1000, /* Model window */
    0.8f, /* Default threshold */
    (int32_t)(EI_CLASSIFIER_RAW_SAMPLE_COUNT / ((EI_CLASSIFIER_FREQUENCY > 0) ? EI_CLASSIFIER_FREQUENCY : 1)) * 500, /* Half of model window */
    0   /* Don't use flags */
};
const ei_object_detection_nms_config_t ei_object_detection_nms = {
    0.0f, /* NMS confidence threshold */
    0.2f  /* NMS IOU threshold */
};

const ei_impulse_t impulse_14225_0 = {
    .project_id = 14225,
    .project_owner = "Edge Impulse Inc.",
    .project_name = "Tutorial: Responding to your voice",
    .deploy_version = 91,

    .nn_input_frame_size = 624,
    .raw_sample_count = 15488,
    .raw_samples_per_frame = 1,
    .dsp_input_frame_size = 15488 * 1,
    .input_width = 0,
    .input_height = 0,
    .input_frames = 0,
    .interval_ms = 0.0625,
    .frequency = 16000,
    .dsp_blocks_size = ei_dsp_blocks_size,
    .dsp_blocks = ei_dsp_blocks,
    
    .object_detection_count = 0,
    .fomo_output_size = 0,
    
    .tflite_output_features_count = 3,
    .learning_blocks_size = ei_learning_blocks_size,
    .learning_blocks = ei_learning_blocks,

    .inferencing_engine = EI_CLASSIFIER_TFLITE,

    .sensor = EI_CLASSIFIER_SENSOR_MICROPHONE,
    .fusion_string = "audio",
    .slice_size = (15488/4),
    .slices_per_model_window = 4,

    .has_anomaly = EI_ANOMALY_TYPE_UNKNOWN,
    .label_count = 3,
    .calibration = ei_calibration,
    .categories = ei_classifier_inferencing_categories,
    .object_detection_nms = ei_object_detection_nms
};

ei_impulse_handle_t impulse_handle_14225_0 = ei_impulse_handle_t( &impulse_14225_0 );
ei_impulse_handle_t& ei_default_impulse = impulse_handle_14225_0;

#endif // _EI_CLASSIFIER_MODEL_METADATA_H_
