/* Edge Impulse ingestion SDK
 * Copyright (c) 2022 EdgeImpulse Inc.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 */

/* Includes ---------------------------------------------------------------- */
#include <CO2-estimation_inferencing.h>

enum sensor_status {
    NOT_USED = -1,
    NOT_INIT,
    INIT,
    SAMPLED
};

typedef struct{
    const char *name;
    int8_t status;
} eiSensors;

/* Constant defines -------------------------------------------------------- */
 
/** Number sensor axes used */
#define N_SENSORS     4

/* Forward declarations ------------------------------------------------------- */
static bool ei_connect_fusion_list(const char *input_list);

/* Private variables ------------------------------------------------------- */
static const bool debug_nn = false; 
static int8_t fusion_sensors[N_SENSORS];
static int fusion_ix = 0;

/** * Array de sensores ATUALIZADO com os nomes do seu modelo.
 */
eiSensors sensors[] =
{
    "intake_pressure", NOT_USED,   
    "intake_temperature", NOT_USED,
    "rpm", NOT_USED,               
    "speed", NOT_USED,             
};

/**
* @brief      Arduino setup function
*/
void setup()
{
    /* Init serial */
    Serial.begin(115200);
    while (!Serial);
    Serial.println("Edge Impulse UART Data Input Inference\r\n");

    /* Connect used sensors */
    if(ei_connect_fusion_list(EI_CLASSIFIER_FUSION_AXES_STRING) == false) {
        ei_printf("ERR: Errors in sensor list detected\r\n");
        return;
    }

    ei_printf("Modelo pronto para inferencia.\r\n");
    // Esta linha agora vai imprimir os seus sensores corretamente:
    ei_printf("Eixos esperados: %s\r\n", EI_CLASSIFIER_FUSION_AXES_STRING);
    ei_printf("Total de eixos: %d\r\n", fusion_ix);
    ei_printf("Total de valores esperados por inferencia: %d\r\n", EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE);
    ei_printf("\r\nEnvie %d valores numericos (float) separados por virgula (,) e terminados com uma nova linha (\\n).\r\n",
        EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE);
    ei_printf("Exemplo: val1,val2,val3,...,val%d\\n\r\n", 
        EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE);
}

/**
* @brief      Get data and run inferencing
*/
void loop()
{
    if (EI_CLASSIFIER_RAW_SAMPLES_PER_FRAME != fusion_ix) {
        ei_printf("ERR: Sensors don't match the sensors required in the model\r\n"
        "Following sensors are required: %s\r\n", EI_CLASSIFIER_FUSION_AXES_STRING);
        delay(1000);
        return;
    }

    ei_printf("Aguardando dados na porta serial...\r\n");

    while (Serial.available() == 0) {
        delay(5); 
    }

    float buffer[EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE] = { 0 };
    int buffer_ix = 0;

    String all_data = Serial.readStringUntil('\n');
    all_data.trim(); 

    int last_index = 0;
    int current_index = all_data.indexOf(',');

    while (current_index != -1 && buffer_ix < EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE) {
        String value_str = all_data.substring(last_index, current_index);
        buffer[buffer_ix++] = value_str.toFloat();
        last_index = current_index + 1;
        current_index = all_data.indexOf(',', last_index);
    }
    
    if (buffer_ix < EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE) {
        String value_str = all_data.substring(last_index);
        buffer[buffer_ix++] = value_str.toFloat();
    }

    if (buffer_ix != EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE) {
        ei_printf("ERR: Recebidos %d valores, mas o modelo espera %d. Descartando buffer.\r\n", 
            buffer_ix, EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE);
        return; 
    }

    ei_printf("Recebidos %d valores. Executando classificador...\r\n", buffer_ix);

    signal_t signal;
    int err = numpy::signal_from_buffer(buffer, EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE, &signal);
    if (err != 0) {
        ei_printf("ERR: numpy::signal_from_buffer falhou (%d)\r\n", err);
        return;
    }

    ei_impulse_result_t result = { 0 };
    err = run_classifier(&signal, &result, debug_nn);
    if (err != EI_IMPULSE_OK) {
        ei_printf("ERR: run_classifier falhou (%d)\r\n", err);
        return;
    }

    print_inference_result(result);
}

/**
 * @brief Go through sensor list to find matching axis name
 */
static int8_t ei_find_axis(char *axis_name)
{
    int ix;
    for(ix = 0; ix < N_SENSORS; ix++) {
        if(strstr(axis_name, sensors[ix].name)) {
            return ix;
        }
    }
    return -1;
}

/**
 * @brief Check if requested input list is valid sensor fusion, create sensor buffer
 */
static bool ei_connect_fusion_list(const char *input_list)
{
    char *buff;
    bool is_fusion = false;

    char *input_string = (char *)ei_malloc(strlen(input_list) + 1);
    if (input_string == NULL) {
        return false;
    }
    memset(input_string, 0, strlen(input_list) + 1);
    strncpy(input_string, input_list, strlen(input_list));

    memset(fusion_sensors, 0, N_SENSORS);
    fusion_ix = 0;

    buff = strtok(input_string, "+");

    while (buff != NULL) { 
        int8_t found_axis = 0;

        is_fusion = false;
        found_axis = ei_find_axis(buff);

        if(found_axis >= 0) {
            if(fusion_ix < N_SENSORS) {
                fusion_sensors[fusion_ix++] = found_axis;
                sensors[found_axis].status = NOT_INIT;
            }
            is_fusion = true;
        }

        buff = strtok(NULL, "+ ");
    }

    ei_free(input_string);

    return is_fusion;
}


void print_inference_result(ei_impulse_result_t result) {

    ei_printf("Timing: DSP %d ms, inference %d ms, anomaly %d ms\r\n",
            result.timing.dsp,
            result.timing.classification,
            result.timing.anomaly);

    ei_printf("Predictions:\r\n");
    for (uint16_t i = 0; i < EI_CLASSIFIER_LABEL_COUNT; i++) {
        ei_printf("  %s: ", ei_classifier_inferencing_categories[i]);
        ei_printf("%.5f\r\n", result.classification[i].value);
    }

#if EI_CLASSIFIER_HAS_ANOMALY == 1
    ei_printf("Anomaly prediction: %.3f\r\n", result.anomaly);
#endif
    ei_printf("\r\n"); 
}

#if !defined(EI_CLASSIFIER_SENSOR) || (EI_CLASSIFIER_SENSOR != EI_CLASSIFIER_SENSOR_FUSION && EI_CLASSIFIER_SENSOR != EI_CLASSIFIER_SENSOR_ACCELEROMETER)
#error "Invalid model for current sensor"
#endif