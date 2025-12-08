/*
 * VozAnalisis.c
 *  Created on: Nov 25, 2025
 *      Author: alang
 */

#include "VozAnalisis.h"

#include "stdbool.h"
#include "stm32h5xx.h"
#include "stdio.h"
#include "arm_math.h"
#include "math.h"
#include <string.h>
#include <stdint.h>

#define ADC_VREF 3.3f
#define FS      16000.0f
#define FRAME   512
#define HOP     (FRAME/2)
#define LEN_SIGNAL 20000
#define MAX_FRAMES ((LEN_SIGNAL - FRAME) / HOP + 2) //122
#define PI 3.14159f

/* Estado FFT */
static arm_rfft_fast_instance_f32 fft_inst;

/* Buffers */
static float hann[FRAME];
static float frecuenciaBin[FRAME/2];

static float spectrum[FRAME];
static float frame_buff[FRAME];
static float mag[FRAME/2];

static float zcrBuff[MAX_FRAMES];
static float centroidBuff[MAX_FRAMES];
static float bandwithBuff[MAX_FRAMES];
static float rolloffBuff[MAX_FRAMES];

//INICIO

void iniciar() {

    arm_rfft_fast_init_f32(&fft_inst, FRAME);
    ventanaHann();
    frecuenciaBins();
}

void ventanaHann() {
    for (int i = 0; i < FRAME; i++) {
        hann[i] = 0.5f - 0.5f * cosf((2.0f * PI * i) / (FRAME - 1));
    }
}

void frecuenciaBins() {
    for (int i = 0; i < FRAME/2; i++) {
        frecuenciaBin[i] = (float)i * FS / (float)FRAME;
    }
}

//SUMATORIA
float sum_f32(const float *x, int n) {
    float s = 0.0f;
    for (int i = 0; i < n; i++) {
        s += x[i];
    }
    return s;
}

//CARACTERISTICAS

float ZCR_f(const float *x, uint32_t N) {
    int cont = 0;
    for (uint32_t i = 1; i < N; i++) {
        if ((x[i] > 0.0f && x[i - 1] < 0.0f) ||
            (x[i] < 0.0f && x[i - 1] > 0.0f)) {
            cont++;
        }
    }
    return (float)cont / (float)(N - 1);
}

float specCentroid(const float *mag, const float *frecuencia, int nBins) {

    float num = 0.0f, den = 0.0f;

    arm_dot_prod_f32(frecuencia, mag, nBins, &num);
    den = sum_f32(mag, nBins);

    if (den < 1e-12f) return 0.0f;

    return num / den;
}

float specBandwidht(const float *mag, const float *frecuencia,
                    int nBins, float centroid) {

    float num = 0.0f, den = 0.0f;

    for (int i = 0; i < nBins; i++) {
        float diff = frecuencia[i] - centroid;
        num += mag[i] * diff * diff;
        den += mag[i];
    }

    if (den < 1e-12f) return 0.0f;

    float var = num / den;
    if (var < 0.0f) var = 0.0f;

    return sqrtf(var);
}

float rolloff(const float *mag, const float *frecuencia,
              int nBins, float percent) {

    float total = sum_f32(mag, nBins);
    if (total < 1e-12f) return 0.0f;

    float threshold = percent * total;
    float acc = 0.0f;

    for (int i = 0; i < nBins; i++) {
        acc += mag[i];
        if (acc >= threshold) {
            return frecuencia[i];
        }
    }

    return frecuencia[nBins - 1];
}


/*Esta funcion se va a llamra 3 veces. Cada vez va a entrar con un arreglos, con el vector caracteristico
 * de cada uno*/
void distancia_relativa(volatile float *a,
                        volatile float *b,
                        volatile float *out){
	float temp=0;
    for (int i = 0; i < 4; i++)
    {
        if (a[i] != 0.0f)
        {
        	temp=fabsf(((b[i] - a[i]) / a[i]) * 100.0f);
            out[i] = temp ;
        }
        else
        {
            out[i] = 0.0f;
        }
    }
}

int definirPersona(volatile float *adrian, volatile float *alan, volatile float *diego, float *numero){

    float sumaAdrian = 0, sumaAlan = 0, sumaDiego = 0, min = 10000.0f;
    int flag = 0;

    for(int i = 0; i < 4; i++){
        sumaAdrian += adrian[i];
        sumaAlan   += alan[i];
        sumaDiego  += diego[i];
    }

    //se define quien es el mas cercano
    if(sumaAdrian < min){
        min = sumaAdrian;
        flag = 1;
    }
    if(sumaAlan < min){
        min = sumaAlan;
        flag = 2;
    }
    if(sumaDiego < min){
        min = sumaDiego;
        flag = 3;
    }

    *numero = min;

    //Umbral de aceptacion
    if(min > 100){
        return 0;
    } else {
        /* Adrian=1
         * Alan=2
         * Diego=3
         */
        return flag;
    }


}




//PROCESAR LA SEÑAL

int procesar(volatile float *array, int length,
             float *zcrPromedio,
             float *centroidPromedio,
             float *bandwithPromedio,
             float *rolloffPromedio) {

    int frameCont = 0;

    for (int i = 0; i + FRAME <= length; i += HOP) {

    	//REMOVER DC
		float mean = 0.0f;
		for (int j = 0; j < FRAME; j++) {
			mean += array[i + j];
		}
		mean /= (float)FRAME;

		//CALCULAR centered Y MAXABS
		float maxAbs = 0.0f;
		for (int j = 0; j < FRAME; j++) {
			float centered = array[i + j] - mean;
			float a = fabsf(centered);
			if (a > maxAbs) maxAbs = a;
			frame_buff[j] = centered; // temporal; luego dividimos
		}

		if (maxAbs < 1e-9f) maxAbs = 1.0f; // evitar división por cero*/

		//NORMALIZAR POR EL PICO DEL FRAME
		for (int j = 0; j < FRAME; j++) {
			frame_buff[j] = frame_buff[j] / maxAbs; // ahora en [-1, 1]
		}

		//ACR
		float zcr = ZCR_f(frame_buff, FRAME);

        //VENTANA
        arm_mult_f32(frame_buff, hann, frame_buff, FRAME);

        //FFT
        arm_rfft_fast_f32(&fft_inst, frame_buff, spectrum, 0); //Output spectrum

        //MAGNITUD
        arm_cmplx_mag_f32(spectrum, mag, FRAME/2); //Output mag

        //FEATURES

        float centroid = specCentroid(mag, frecuenciaBin, FRAME/2);
        float bw = specBandwidht(mag, frecuenciaBin, FRAME/2, centroid);
        float roll = rolloff(mag, frecuenciaBin, FRAME/2, 0.85f);

        //GUARDAR

        if (frameCont < MAX_FRAMES) {
            zcrBuff[frameCont] = zcr;
            centroidBuff[frameCont] = centroid;
            bandwithBuff[frameCont] = bw;
            rolloffBuff[frameCont] = roll;
        }

        frameCont++;
        if (frameCont >= MAX_FRAMES) break;
    }

    //Calcular Promedio

    arm_mean_f32(zcrBuff, MAX_FRAMES, zcrPromedio);
    arm_mean_f32(centroidBuff, MAX_FRAMES, centroidPromedio);
    arm_mean_f32(bandwithBuff, MAX_FRAMES, bandwithPromedio);
    arm_mean_f32(rolloffBuff, MAX_FRAMES, rolloffPromedio);


    return frameCont;
}
