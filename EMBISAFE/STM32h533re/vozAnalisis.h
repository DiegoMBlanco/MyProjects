/*
 * VozAnalisis.h
 *
 *  Created on: Nov 25, 2025
 *      Author: alang
 */


#ifndef SRC_VOZANALISIS_H_
#define SRC_VOZANALISIS_H_

#include <stdint.h>

#define ADC_VREF 3.3f
#define FS 16000.0f
#define FRAME 512
#define HOP (FRAME/2) //256
#define LEN_SIGNAL 20000
#define MAX_FRAMES ((LEN_SIGNAL-FRAME)/HOP+2) //122
#define PI 3.14159f

#define MUESTRAS 10
#define FRAMES 122

int procesar(volatile float *array, int length,
             float *zcrBuff,
             float *centroidBuff,
             float *bandwithBuff,
             float *rolloffBuff);

void distancia_relativa(volatile float *a,
                        volatile float *b,
                        volatile float *out);

int definirPersona(volatile float *adrian,volatile float *alan,volatile float *diego,float *numero);

void iniciar();
void ventanaHann();
void frecuenciaBins();

float sum_f32(const float *x, int n);

float ZCR_f(const float *x, uint32_t N);
float specCentroid(const float *mag, const float *frecuencia, int nBins);
float specBandwidht(const float *mag, const float *frecuencia,
                    int nBins, float centroid);
float rolloff(const float *mag, const float *frecuencia,
              int nBins, float percent);

#endif /* SRC_VOZANALISIS_H_ */
