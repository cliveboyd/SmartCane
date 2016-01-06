/* Copyright (c) <2015> <Shun Bai (wanyancan at gmail)>

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in
all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
THE SOFTWARE.
*/
 

#include "sensorADC.h"
#include "app_error.h"															// for APP_ERROR_CHECK
#include "nrf_adc.h"
#include "nrf_soc.h"															// for NRF_APP_PRIORITY_HIGH

#define ADCDATASIZE 10															// Note: max256, 8192 bytes onchip ram limits
static const float ADCREFVALUE = 1.2; 
static float adcData[ADCDATASIZE];												// 4*SIZE bytes

unsigned int currStoreIndex;
unsigned int maxSeqSamples;

nrf_adc_config_input_t BattVoltageMeasureInput;

didGetADCSequenceBlock_callback_t blockFinishCallback;


void sensorADC_config(void) {
/**
*	@brief ADC initialization.
*/	
	nrf_adc_config_t nrf_adc_config = NRF_ADC_CONFIG_DEFAULT;					// 10bit, 1/3 scaling, internal VBG

	nrf_adc_config.scaling = NRF_ADC_CONFIG_SCALING_INPUT_ONE_THIRD;
	nrf_adc_config.reference = NRF_ADC_CONFIG_REF_EXT_REF0;
	
    nrf_adc_configure((nrf_adc_config_t *)&nrf_adc_config);						// Initialize and configure ADC
	
	BattVoltageMeasureInput = NRF_ADC_CONFIG_INPUT_6;							// Battery Voltage Input Scalled by 150k/(150k+680k) Voltage Divider

	blockFinishCallback = NULL;   												// initially no callback
	
//    nrf_adc_input_select(NRF_ADC_CONFIG_INPUT_2);  							// after select, it will be enabled
//    nrf_adc_int_enable(ADC_INTENSET_END_Enabled << ADC_INTENSET_END_Pos);
//    NVIC_SetPriority(ADC_IRQn, NRF_APP_PRIORITY_HIGH);
//    NVIC_EnableIRQ(ADC_IRQn);
}

/**
*	@brief ADC single point measurement
*/
float sensorADC_singleMeasure(void) {
	return (float)nrf_adc_convert_single(BattVoltageMeasureInput);
}

void sensorADC_startSequenceMeasure(unsigned int numSamples, didGetADCSequenceBlock_callback_t blockCallback) {
		maxSeqSamples = numSamples;
		for (int i=0; i<ADCDATASIZE; i++) {  									// clear buffer
			adcData[i] = 0;
		}
		
		blockFinishCallback = blockCallback;  									// set to new callback for each sequence measure 
		
		nrf_adc_input_select(NRF_ADC_CONFIG_INPUT_2);							// after select, it will be enabled
		nrf_adc_int_enable(ADC_INTENSET_END_Enabled << ADC_INTENSET_END_Pos);
		NVIC_SetPriority(ADC_IRQn, NRF_APP_PRIORITY_HIGH);
		NVIC_EnableIRQ(ADC_IRQn);
		currStoreIndex = 0;														// starting from position 0 to store the data
		nrf_adc_start();
}


void ADC_IRQHandler(void) {
/**
* 	@brief ADC interrupt handler.
*/	
	nrf_adc_conversion_event_clean();

	int32_t adc_sample = nrf_adc_result_get();

	adcData[(currStoreIndex)%ADCDATASIZE] = (adc_sample*ADCREFVALUE/1023.0)+currStoreIndex;  // cycle back to start to filling
	currStoreIndex++;

	unsigned int newstoreIndex = currStoreIndex%ADCDATASIZE;

	if (newstoreIndex==0 || currStoreIndex >= maxSeqSamples) {
		if (blockFinishCallback!=NULL) {
			blockFinishCallback(adcData, (currStoreIndex)%ADCDATASIZE);
		}
	}
		
	if (currStoreIndex < maxSeqSamples) {													// trigger next ADC conversion
		nrf_adc_start();
	} else {
		nrf_adc_stop();
	}
}
