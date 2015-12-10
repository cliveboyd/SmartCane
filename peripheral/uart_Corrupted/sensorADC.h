#ifndef _SENSORADC_H_
#define _SENSORADC_H_

void sensorADC_config(void);
float sensorADC_singleMeasure(void);
extern void ADC_IRQHandler(void);
typedef void (*didGetADCSequenceBlock_callback_t) (float *, unsigned int len);
void sensorADC_startSequenceMeasure(unsigned int numSamples, didGetADCSequenceBlock_callback_t blockCallback);

#endif // _SENSORADC_H_
