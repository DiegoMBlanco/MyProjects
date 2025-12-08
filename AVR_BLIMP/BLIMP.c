#define F_CPU 8000000UL
#include <avr/io.h>
#include "avr/interrupt.h"
#include <util/delay.h>
#include <math.h>

int valor;

float FactCor = 1.235;
float Emma = 664;
uint8_t ppm_scaled ;
int Stephanie;


ISR(USART1_UDRE_vect) {
	UDR1 = ppm_scaled;                 // Enviar el dato
	UCSR1B &= ~(1 << UDRIE1);          // Desactiva la interrupción (ya enviamos)
}

void start_UART()
{
	UCSR1B = 0b10111000; //Habilita UDRIEn y Tx, RX y la interrupción por RX
	UCSR1C = 0b00000110; // Asincrono con trama de 8 bits
	UBRR1 = 51; //9600 Baudios
}


void ADCStart() {
	// Referencia interna 2.56V (REFS1=1, REFS0=1) y canal ADC4 (MUX[3:0]=0100)
	ADMUX = (1 << REFS1) | (1 << REFS0) | (1 << MUX2);  // 0b11000100

	// Habilita ADC y prescaler de 128 (para 62.5 kHz si F_CPU = 8 MHz)
	ADCSRA = (1 << ADEN) | (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0);  // 0b10000111

	// ADCSRB default (modo libre)
	ADCSRB = 0x00;

	// Deshabilita el buffer digital del pin ADC4 (PF4)
	DIDR0 = (1 << ADC4D);
}

uint16_t ADCREAD() {
	ADCSRA |= (1 << ADSC); // Inicia conversión
	while (!(ADCSRA & (1 << ADIF))); // Espera a que termine
	ADCSRA |= (1 << ADIF); // Limpia bandera
	return ADC;
}

float res() {
	uint16_t val = ADCREAD();
	return ((1023.0 / (float)val) - 1.0) * 1000.0;
}

float rzero() {
	return res() * pow((Emma / 116.6020682), (1.0 / 2.769034857));
}

uint16_t ppm() {
	return (uint16_t)(116.6020682 * pow((res() / rzero()), -2.769034857));
}


void reset()
{
	
	DDRD = 0;
	DDRC = 0;
	DDRB = 0;
	
	TCCR1A = 0;
	TCCR1B = 0;
	
	TCCR3A = 0;
	TCCR3B = 0;
	
	TCCR0A = 0;
	TCCR0B = 0;
	
	OCR1A = 0;
	OCR1B = 0;
	OCR3A = 0;
	OCR0B = 0;
	
	ICR1 = 0; //TOP
	
}


void dobleServo(){
	
	DDRB = 0b01100000;	//PB6 - PB5
	
	TCCR1A = 0b10100010; //Comparacion para A y B / Modo Phase Correct
	TCCR1B = 0b00010011; //clk 64 Phase Correct
	
	ICR1 = 1249; //TOP
	
	OCR1A = 125; //10% - 90 grados
	OCR1B = 125; //10% -
	_delay_ms(150);
	
	OCR1A = 62; //5% -90 grados
	OCR1B = 62; //5%
	_delay_ms(150);
	
	reset();

}


void ciclo_Timer3A(){
	
	DDRC = 0b01000000;	//PC - PC6
	
	TCCR3A = 0b10000010; //Comparacion para A / Modo Phase Correct
	TCCR3B = 0b00010011; //clk 64 Phase Correct
	
	ICR3 = 1249;
	
	OCR3A = 62; //5% -90 grados
	_delay_ms(150);
	
	OCR3A = 125; //10% 90 grados
	_delay_ms(150);
	
	reset();

}


//Funcion para mover por el Timer0B

void ciclo_Timer0B(){
	
	DDRD = 0b00000001;	//PD - PD0
	
	TCCR0A = 0b00100001; //Comparacion para B / Modo Phase Correct
	TCCR0B = 0b00001101; //clk 1024 Phase Correct
	
	OCR0A = 77;
	
	OCR0B = 4; //5%
	_delay_ms(150); //-90 grados
	
	OCR0B = 8; //10% //90 grados
	_delay_ms(150);
	
	reset();
}

ISR (USART1_RX_vect){
	valor = UDR1;
}


int main(void)
{
	DDRF |= 0b10000000;
	ADCStart();
	start_UART();
	uint16_t ppm_env = ppm();
	ppm_scaled = ppm_env / 4;
	
	sei();
	
	while(1)
	{
		uint16_t ppm_env = ppm();
		ppm_scaled = ppm_env / 4;
		UCSR1B |= (1 << UDRIE1);
		_delay_ms(500);
		if(valor == 'S'){
			Stephanie=valor;
			dobleServo();
			if(Stephanie!=valor){
				if(valor=='N'){
					reset();
				}
			}
			}else if (valor == 'A'){
			Stephanie=valor;
			ciclo_Timer3A();
			if(Stephanie!=valor){
				if(valor=='N'){
					reset();
				}
			}
			}else if (valor == 'D'){
			Stephanie=valor;
			ciclo_Timer0B();
			if(Stephanie!=valor){
				if(valor=='N'){
					reset();
				}
			}
		}
		else if(valor == 'F'){//toma foto
			
			PORTF = PORTF & (0b01111111);
			
			_delay_ms(200);
			
			PORTF |= 0b10000000;
			
			
		}
		else if(valor=='N'){
			reset();
		}
		else{
			reset();
		}
	}
	return 0;
}
