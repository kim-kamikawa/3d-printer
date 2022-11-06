// Kim Lopez
// Embedded Controller Hardware Design
// ECE - 40001
// Final Project - 3D Printer
// 08-31-2018
  
#define F_CPU                   16000000
#define BAUD                    9600

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <math.h>

#define BALANCE_RESISTOR 		100500.0
#define MAX_ADC 				1023.0
#define BETA 					3950.0
#define KELVIN_CONSTANT			273.15
#define ROOM_TEMP 				(25 + KELVIN_CONSTANT)
#define RESISTANCE_AT_ROOM_TEMP 100000

#define HOTEND_PWM_PIN          PB3

#define X_DIR_PIN               PD2   
#define X_STEP_PIN              PD3
#define X_HOME_PIN              PD4  
#define X_STEPS_PER_MM          100
 
#define Y_DIR_PIN               PD5 
#define Y_STEP_PIN              PD6
#define Y_HOME_PIN              PD7  
#define Y_STEPS_PER_MM          100

#define Z_DIR_PIN               PB0  
#define Z_STEP_PIN              PB1
#define Z_HOME_PIN              PB2   
#define Z_STEPS_PER_MM          400

#define E_DIR_PIN               PB4
#define E_STEP_PIN              PB5

#define PULSE_DURATION			200

struct Gcode {
	char letter;
	uint8_t number;
	uint8_t hasX;
	double x;
	uint8_t hasY;
	double y;
	uint8_t hasZ;
	double z;
	uint8_t hasE;
};

struct Position {
	double x;
	double y;
	double z;
};

volatile double hotEndTemperature = 0;
double hotEndSetting = 0;
int hotEndCounter = 0;

/*** SERIAL ***/

void initUSART() {
	// set baud rate
	unsigned int ubrr = F_CPU/16/BAUD-1;
	UBRR0H = (uint8_t)(ubrr>>8);
	UBRR0L = (uint8_t) ubrr;
	// enable receiver and transmitter
	UCSR0B |= (1<<RXEN0) | (1<<TXEN0);
	// 8-bit, no parity, 1 stop bit
	UCSR0C |= (1<<UCSZ01) | (1<<UCSZ00);   
}

uint8_t receiveUSART(void) {
	// wait for data to receive
	while (!(UCSR0A & (1<<RXC0)))
		;
	// return received byte
	return UDR0;
}

void transmitUSART(uint8_t data) {
	// wait for empty transmit buffer
	while (!(UCSR0A & (1<<UDRE0)))
		;
	// put data into buffer
	UDR0 = data;
}

/*** ADC ***/

void initADC() {
	/* initialize one second timer */
	// set CTC mode to match OCR1A (mode 4)
	TCCR1A = 0x0;
	TCCR1B = (1 << WGM12);
	// set prescaler to 1024 
	// => 16MHz/1024 = 15625Hz
	TCCR1B |= (1<<CS12) | (1<<CS10);
	// set compare match
	// => 15625 ticks = 1 second
	OCR1A = 15625;
	// initialize counter
	TCNT1 = 0;
	// enable Timer/Counter1 Compare Match A Interrupt
	TIMSK1 |= (1 << OCIE1A);

	/* initialize ADC */
	// set channel to pin A0 (PC0)
	// set voltage reference to AVcc
	ADMUX = (1<<REFS0);
	// set prescaler to 128
	// => 16MHz/128 = 125kHz
	// => ADC requires 50kHz-200kHz 
	ADCSRA |= (1<<ADPS2) | (1<<ADPS1) | (1<<ADPS0);
	// enable ADC interrupt
	ADCSRA |= (1<<ADIE);
	// turn off digital circuitry on A0
	DIDR0 |= (1<<ADC0D);
	// enable ADC
	ADCSRA |= (1<<ADEN);
	
	// enable global interrupt
	sei();
}

ISR(TIMER1_COMPA_vect) {
    // start ADC conversion
	ADCSRA |= (1<<ADSC);
}

ISR(ADC_vect) {
	// determine temperature of thermistor
	// https://www.allaboutcircuits.com/projects/measuring-temperature-with-an-ntc-thermistor/
	double rThermistor = BALANCE_RESISTOR * ((MAX_ADC / ADC) - 1);
	double tKelvin = (BETA * ROOM_TEMP) / (BETA + (ROOM_TEMP * log(rThermistor / RESISTANCE_AT_ROOM_TEMP)));
	hotEndTemperature = tKelvin - KELVIN_CONSTANT;
}

/*** HOT END ***/

void initHotEnd() {	
	/* initialize hot end PWM */
	// set pin 11 (PB3 & OC2A) as output
	DDRB |= (1<<HOTEND_PWM_PIN);
	// set fast PWM, top = 0xFF
	TCCR2A = (1<<WGM21) | (1<<WGM20);
	// set OC2A on compare match, clear OC2A at bottom
	TCCR2A |= (1<<COM2A1) | (1<<COM2A0);
	// set prescaler to 128
	// => 16MHz/128/256 = PWM frequency
	TCCR2B = (1<<CS22) | (1<<CS20);
	// set compare match
	// => 255 = 0% duty cycle (compare match ignored)
	// => 0 = 100% duty cycle
	OCR2A = 255;
	// initialize counter
	TCNT2 = 0;
}

/*** HOME SWITCHES and STEPPER MOTORS **/

void initHomeSwitches() {
	// home switches: set for input and pull-up resistor
	PORTD |= (1<<X_HOME_PIN) | (1<<Y_HOME_PIN);
	PORTB |= (1<<Z_HOME_PIN);
}

void initSteppers() {
	// set all steppers for output
	DDRD |= (1<<X_DIR_PIN) | (1<<X_STEP_PIN) | (1<<Y_DIR_PIN) | (1<<Y_STEP_PIN);
	DDRB |= (1<<Z_DIR_PIN) | (1<<Z_STEP_PIN) | (1<<E_DIR_PIN) | (1<<E_STEP_PIN);
	// set direction of stepper to extrude
	PORTB |= (1<<E_DIR_PIN);
}

/*** PARSING ***/

uint8_t getNumber(double *value) {
	
	uint8_t data;
	int num = 0;
	uint8_t multiplier = 1;
	double decimal = 0.0;
	uint8_t place = 0;
	
	// get integer part
	do {
		data = receiveUSART();
		switch (data) {
			case '+':
				break;
			case '-':
				multiplier = -1;
				break;
			case '0': case '1': case '2': case '3': case '4':
			case '5': case '6': case '7': case '8': case '9':
			    num = (num*10) + (data-'0');
				break;
			default:
				break;
		}
	} while (!((data == ' ') || (data == '.')));
	num *= multiplier;
	// get decimal part
	if (data == '.') {
		do {
			data = receiveUSART();
			switch (data) {
				case '0': case '1': case '2': case '3': case '4':
				case '5': case '6': case '7': case '8': case '9':
					decimal = (decimal*10.0) + (data-'0');
					place++;
					break;
				default:
					break;
			}
		} while (data != ' ');
		for (uint8_t x=0; x<place; x++)
			decimal *= 0.1;
	}
	
	*value = num + decimal;
	return data;
}

struct Gcode getCommand() {
	uint8_t data;
	struct Gcode command = {' ', 0, 0, 0.0, 0, 0.0, 0, 0.0, 0};
	
	// ignore line numbers
	do {
		
		data = receiveUSART(); 
	} while (data != ' ');
	
	// get letter
	data = receiveUSART();
		
	switch (data) {
		case 'G':
			data = receiveUSART();
			command.letter = 'G';			
			// get number
			switch (data) {
				case '0':
				case '1':
					command.number = 0;
					// G0: rapid linear move & G1: linear move
					data = receiveUSART();
					do {
						data = receiveUSART();
						
						// get parameters
						switch (data) {
							case 'X':
								// x coordinate in mm
								command.hasX = 1;
								data = getNumber(&(command.x));
								break;
							case 'Y':
								// y coordinate in mm
								command.hasY = 1;
								data = getNumber(&(command.y));
								break;
							case 'Z':
								// z coordinate in mm
								command.hasZ = 1;
								data = getNumber(&(command.z));
								break;
							case 'E':
								// e coordinate in mm
								command.hasE = 1;
								do {
									data = receiveUSART();
								} while (data != ' ');
								break;
							case 'F':
								// feedrate in mm per minute
								do {
									data = receiveUSART();
								} while (data != ' ');
								break;
							default:
								break;
						}
					} while (data != '*');
					break;
				case '2':
					data = receiveUSART();
					switch (data) {
						case '8':
							command.number = 28;
							// G28: move to origin
							data = receiveUSART();
							do {
								data = receiveUSART();
						
								// get parameters
								switch (data) {
									case 'X':
										// x coordinate in mm
										command.hasX = 1;
										data = getNumber(&(command.x));
										break;
									case 'Y':
										// y coordinate in mm
										command.hasY = 1;
										data = getNumber(&(command.y));
										break;
									case 'Z':
										// z coordinate in mm
										command.hasZ = 1;
										data = getNumber(&(command.z));
										break;
									default:
										break;
								}
							} while (data != '*');
							break;
						default:
							break;
					}
					break;
				default:
					break;
			}
			break; // 'G'
		case 'M':
			data = receiveUSART();
			command.letter = 'M';			
			// get number
			switch (data) {
				case '1':
					data = receiveUSART();
					switch (data) {
						case '0':
							data = receiveUSART();
							switch (data) {
								case '5':
									command.number = 105;
									// M105: get extruder temperature
									break;
								case '9':
									command.number = 109;
									// M109: set extruder temperature and wait
									data = receiveUSART();
									do {
										data = receiveUSART();
										switch (data) {
											case 'T':
												// temperature sensor number
												do {
													data = receiveUSART();
												} while (data != ' ');
												break;
											case 'S':
												// temperature value
												data = getNumber(&(hotEndSetting));
												break;
											default:
												break;
										}
									} while (data != '*');
									break;
								default:
									break;
							}
							break;
						default:
							break;
					}
					break;
				default:
					break;
			}
			break;
		default:
			break;
	}

	// ignore checksum
	do {
		data = receiveUSART(); 
	} while (!((data == '\r') || (data == '\n')));
	
	return command;
}
	
void sendOk() {
	// acknowledge receipt of command
	transmitUSART('o');
	transmitUSART('k');
	transmitUSART('\n');
}

/*** GCODE COMMANDS ***/

struct Position goLinear(struct Gcode gcode, struct Position currentPosition) {
	
	// G0: rapid linear move & G1: linear move
	int xSteps=0, ySteps=0, zSteps=0;
	struct Position position = currentPosition;

	if (gcode.hasX) {
		// compute steps
		xSteps = round((gcode.x - currentPosition.x) * X_STEPS_PER_MM);
		// set direction
		if (xSteps>0) {
			// go right
			PORTD |= (1<<X_DIR_PIN);
		}
		else {
			// go left
			PORTD &= ~(1<<X_DIR_PIN);
			xSteps *= -1;
		}
		// set ending position
		position.x = gcode.x;
	}
	if (gcode.hasY) {
		// compute steps
		ySteps = round((gcode.y - currentPosition.y) * Y_STEPS_PER_MM);
		// set direction
		if (ySteps>0) {
			// go front
			PORTD |= (1<<Y_DIR_PIN);
		}
		else {
			// go back
			PORTD &= ~(1<<Y_DIR_PIN);
			ySteps *= -1;
		}
		// set ending position
		position.y = gcode.y;
	}
	if (gcode.hasZ) {
		// compute steps
		zSteps = round((gcode.z - currentPosition.z) * Z_STEPS_PER_MM);
		// set direction
		if (zSteps>0) {
			// go up
			PORTB &= ~(1<<Z_DIR_PIN);
		}
		else {
			// go down
			PORTB |= (1<<Z_DIR_PIN);
			zSteps *= -1;
		}
		// set ending position
		position.z = gcode.z;
	}

	while (xSteps || ySteps || zSteps) {
		if (xSteps) {
			// send pulse to X-axis stepper
			PORTD |= (1<<X_STEP_PIN);
			xSteps--;
		}
		if (ySteps) {
			// send pulse to Y-axis stepper
			PORTD |= (1<<Y_STEP_PIN);
			ySteps--;
		}
		if (zSteps) {
			// send pulse to Z-axis steppers
			PORTB |= (1<<Z_STEP_PIN);
			zSteps--;
		}
		
		if (gcode.hasE) {
			// send pulse to extruder stepper
			// one extruder pulse for every 30 movement pulses
			if (hotEndCounter == 0) {
				PORTB |= (1<<E_STEP_PIN);
				hotEndCounter++;
			}
			else {
				if (hotEndCounter == 29)
					hotEndCounter = 0;
				else
					hotEndCounter++;
			}
		}
		
		// pulse duration
		_delay_us(PULSE_DURATION);
		
		// stop pulse
		PORTD &= ~(1<<X_STEP_PIN);
		PORTD &= ~(1<<Y_STEP_PIN);
		PORTB &= ~(1<<Z_STEP_PIN);	
		PORTB &= ~(1<<E_STEP_PIN);		
		
		// stop pulse duration
		_delay_us(PULSE_DURATION);
	} 
	
	return position;
}

struct Position goHome(struct Gcode gcode, struct Position currentPosition) {
	
	// G28: move to origin
	struct Position position = currentPosition;
	
	if (gcode.hasX) {
		// move X-axis stepper home
		PORTD &= ~(1<<X_DIR_PIN);
		while (bit_is_set(PIND, X_HOME_PIN)) {
			PORTD |= (1<<X_STEP_PIN);
			_delay_us(PULSE_DURATION);
			PORTD &= ~(1<<X_STEP_PIN);
			_delay_us(PULSE_DURATION);
		}
		// move X-axis stepper to zero
		PORTD |= (1<<X_DIR_PIN);
		for (int x=0; x<33*100; x++) {
			PORTD |= (1<<X_STEP_PIN);
			_delay_us(PULSE_DURATION);
			PORTD &= ~(1<<X_STEP_PIN);
			_delay_us(PULSE_DURATION);
		}
		position.x = 0;
	}
	if (gcode.hasY) {
		// move Y-axis stepper home
		PORTD &= ~(1<<Y_DIR_PIN);
		while (bit_is_set(PIND, Y_HOME_PIN)) {
			PORTD |= (1<<Y_STEP_PIN);
			_delay_us(PULSE_DURATION);
			PORTD &= ~(1<<Y_STEP_PIN);
			_delay_us(PULSE_DURATION);
		} 
		// move Y-axis stepper to zero
		PORTD |= (1<<Y_DIR_PIN);
		for (int x=0; x<4*100; x++) {
			PORTD |= (1<<Y_STEP_PIN);
			_delay_us(PULSE_DURATION);
			PORTD &= ~(1<<Y_STEP_PIN);
			_delay_us(PULSE_DURATION);
		}
		position.y = 0;
	}
	if (gcode.hasZ) {
		// move Z-axis stepper home
		PORTB |= (1<<Z_DIR_PIN);
		while (bit_is_set(PINB, Z_HOME_PIN)) {
			PORTB |= (1<<Z_STEP_PIN);
			_delay_us(PULSE_DURATION);
			PORTB &= ~(1<<Z_STEP_PIN);
			_delay_us(PULSE_DURATION);
		}
		// home is Z-axis origin
		position.z = 0;
	}

	return position;
}

void sendHotEndTemperature() {
	// M105: get extruder temperature
	transmitUSART('T');
	transmitUSART(':');
	transmitUSART('0' + ((int)hotEndTemperature / 100)); 
	transmitUSART('0' + (((int)hotEndTemperature / 10) % 10));                   
	transmitUSART('0' + ((int)hotEndTemperature % 10));                             
	transmitUSART('\n');
}

void waitHotEndTemperature(struct Gcode gcode) {
	// M109: set extruder temperature and wait
	while (hotEndTemperature < hotEndSetting)
		// send hot end temperature to host
		sendHotEndTemperature();
}

/*** MAIN LOOP ***/

int main(void) {
	
	struct Gcode gcode;
	struct Position currentPosition = {0.0, 0.0, 0.0};

	// initialization
	initUSART();
	initADC();
	initHotEnd();
	initHomeSwitches();
	initSteppers();

	// main loop
	while (1) {

		gcode = getCommand();
		
		// maintain hot end temperature
		if (hotEndTemperature > (hotEndSetting))
			// PWM off
			OCR2A = 255;
			// PWM on
		else
			OCR2A = 64;

		switch (gcode.letter) {
			case 'G':
				switch (gcode.number) {
					case 0:
						// linear move
						currentPosition = goLinear(gcode, currentPosition);
						break;
					case 28:
						// home
						currentPosition = goHome(gcode, currentPosition);
						break;
					default:
						break;
				}
				break;
			case 'M':
				switch (gcode.number) {
					case 105:
						// send hot end temperature to host
						sendHotEndTemperature();
						break;
					case 109:
						// wait for hot end to meet temperature
						waitHotEndTemperature(gcode);
					default:
						break;
				}
				break;
			default:
				break;
		}
		// send acknowledgment to host
		sendOk();
	}                                                
	return 0;
}
