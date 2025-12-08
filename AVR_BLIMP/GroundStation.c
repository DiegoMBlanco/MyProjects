#define LCD_CLEARDISPLAY 0x01
#define LCD_RETURNHOME 0x02
#define LCD_ENTRYMODESET 0x04
#define LCD_DISPLAYCONTROL 0x08
#define LCD_CURSORSHIFT 0x10
#define LCD_FUNCTIONSET 0x20
#define LCD_SETCGRAMADDR 0x40
#define LCD_SETDDRAMADDR 0x80

#define LCD_DATA_PORT PORTC
#define LCD_DATA_DDR  DDRC

#define LCD_CTRL_PORT PORTB
#define LCD_CTRL_DDR  DDRB
#define RS PB6
#define E  PB7

#define F_CPU 8000000UL
#include <avr/io.h>
#include <util/delay.h>
#include "avr/interrupt.h"
#include <stdio.h>

uint16_t joystickX;
uint16_t joystickY;
uint16_t valor_sensor;

void lcd_pulse_enable(void){
	LCD_CTRL_PORT &= ~(1 << E);
	_delay_us(1);
	LCD_CTRL_PORT |= (1 << E);
	_delay_us(1);
	LCD_CTRL_PORT &= ~(1 << E);
	_delay_us(100);
}

void lcd_send_command(uint8_t cmd){
	LCD_CTRL_PORT &= ~(1 << RS); // RS = 0 para comando
	LCD_DATA_PORT = cmd; //Mandamos el comando al puerto
	lcd_pulse_enable();
}

void lcd_send_data(uint8_t data){
	LCD_CTRL_PORT |= (1 << RS); //RS = 1 para datos
	LCD_DATA_PORT = data; //Mandamos el dato al puerto
	lcd_pulse_enable();
}

void lcd_clear(void){
	lcd_send_command(LCD_CLEARDISPLAY);
	_delay_ms(2); //Tiempo requerido para clear
}

void lcd_print(const char *str){
	while (*str){
		lcd_send_data(*str++);
	}
}

void lcd_start(void){
	LCD_DATA_DDR = 0xFF;
	LCD_CTRL_DDR |= (1 << RS) | (1 << E);
	
	_delay_ms(50); //Espera power ON
	lcd_send_command(0x38); //Modo de 8 bits, 2 líneas 5x8 pixeles
	lcd_send_command(0x0C); //Display ON, Cursor OFF, Blink OFF
	lcd_send_command(0x06); //Entry mode: Increment cursor
	lcd_clear();
}

// Cadena de caracteres a enviar
char datos[] = {0x41, 'E', 'I', 'O', 0x0A};

// Inicialización del USART
void usart_init(void) {
	UCSRB |= (1 << TXEN) | (1 << RXEN) | (1 << RXCIE);       // Habilita transmisión
	UCSRC |= (1 << URSEL) | (1 << UCSZ1) | (1 << UCSZ0); // 8 bits, modo asincrónico
	UBRRL = 0x33;              // Baud rate = 9600 con F_CPU = 8 MHz
}

// Función para enviar un carácter por USART
void usart_send(char c) {
	while (!(UCSRA & (1 << UDRE))); // Espera a que el buffer esté vacío
	UDR = c;                        // Carga el carácter al registro de transmisión
}

void ADCStartch0(){
	ADMUX = 0b11000000; //Inicializamos el
	ADCSRA = 0b10000111;
}

void ADCStartch1(){
	ADMUX = 0b11000001;
	ADCSRA = 0b10000111;
}

uint16_t ADCREAD(){
	ADCSRA |= (1 << ADSC); //Iniciamos conversión
	while (!(ADCSRA & (1 << ADIF))); //Esperamos bandera
	ADCSRA |= (1 << ADIF); //Apagamos bandera
	return ADC;
}

void joystick(void){
	// Leer eje X (canal 0)
	ADCStartch0();
	joystickX = ADCREAD();

	// Leer eje Y (canal 1)
	ADCStartch1();
	joystickY = ADCREAD();
}

void setupINT(void){
	GICR |= (1 << INT1)|(1 << INT0)|(1 << INT2);
	MCUCR = 0b00001010;
}

ISR(INT2_vect){
	_delay_ms(10);
	usart_send('Z');
	_delay_ms(200);
}

ISR(INT0_vect){
	_delay_ms(10);
	usart_send('X');
	_delay_ms(200);
}

ISR(INT1_vect){
	_delay_ms(10);
	usart_send('F');
	_delay_ms(200);
}

ISR(USART_RXC_vect){
	uint8_t valor_buffer = UDR;
	valor_sensor = valor_buffer * 4;
	char temp[16];
	lcd_clear();
	sprintf(temp, "CO2: %u ppm", valor_sensor);
	lcd_print(temp);
	_delay_ms(500);
}

int main(void) {
	DDRD |= (1 << PD1); // Configura TX como salida
	DDRC = 0xFF;
	lcd_start();
	usart_init();      // Inicializa USART
	setupINT();
	sei();
	while (1) {
		joystick(); // Actualiza joystickX y joystickY
		if(joystickY == 0x03FF){
			usart_send('D');
			} else if(joystickY < 100){
			usart_send('A');
			}else if(joystickX == 0x03FF){
			usart_send('W');
			}else if(joystickX < 100){
			usart_send('S');
			}else{
				usart_send('N');
			_delay_ms(1);
		}
		
		_delay_ms(200); // Retardo para no saturar
	}
}
