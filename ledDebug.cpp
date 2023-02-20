#include "ledDebug.h"
#include <avr/io.h>
#include <avr/interrupt.h>
#include "Arduino.h"

#define NUM_OF_SEGMENTS 8
uint8_t segments[NUM_OF_SEGMENTS] = { SEG_A, SEG_B, SEG_C, SEG_D, SEG_E, SEG_F,
SEG_G, SEG_DP };

#define NUM_OF_DIGITS 4
uint8_t digits[NUM_OF_DIGITS] = { DIGIT_1, DIGIT_2, DIGIT_3, DIGIT_4 };
uint8_t digitValues[NUM_OF_DIGITS] = { 0, 0, 0, 0 };

int16_t currentValue = 0;
//int16_t resetValue = 9999;

uint8_t zero[NUM_OF_SEGMENTS] = { HIGH, HIGH, HIGH, HIGH, HIGH, HIGH, LOW, LOW };
uint8_t one[NUM_OF_SEGMENTS] = { LOW, HIGH, HIGH, LOW, LOW, LOW, LOW, LOW };
uint8_t two[NUM_OF_SEGMENTS] = { HIGH, HIGH, LOW, HIGH, HIGH, LOW, HIGH, LOW };
uint8_t three[NUM_OF_SEGMENTS] = { HIGH, HIGH, HIGH, HIGH, LOW, LOW, HIGH, LOW };
uint8_t four[NUM_OF_SEGMENTS] = { LOW, HIGH, HIGH, LOW, LOW, HIGH, HIGH, LOW };
uint8_t five[NUM_OF_SEGMENTS] = { HIGH, LOW, HIGH, HIGH, LOW, HIGH, HIGH, LOW };
uint8_t six[NUM_OF_SEGMENTS] = { HIGH, LOW, HIGH, HIGH, HIGH, HIGH, HIGH, LOW };
uint8_t seven[NUM_OF_SEGMENTS] = { HIGH, HIGH, HIGH, LOW, LOW, LOW, LOW, LOW };
uint8_t eight[NUM_OF_SEGMENTS] =
		{ HIGH, HIGH, HIGH, HIGH, HIGH, HIGH, HIGH, LOW };
uint8_t nine[NUM_OF_SEGMENTS] = { HIGH, HIGH, HIGH, HIGH, LOW, HIGH, HIGH, LOW };

#define NOP __asm__ __volatile__ ("nop")

void clearSegments() {
	for (uint8_t segment = 0; segment < NUM_OF_SEGMENTS; segment++) {
		digitalWrite(segments[segment], LOW);
	}
}

void clearDigits() {
	for (uint8_t digit = 0; digit < NUM_OF_DIGITS; digit++) {
//		digitalWrite(digits[digit], HIGH);

		if (digit == 0) PORTF |= (1 << PF1);
		if (digit == 1) PORTF |= (1 << PF2);
		if (digit == 2) PORTF |= (1 << PF3);
		if (digit == 3) PORTF |= (1 << PF4);
	}
}

void clearLEDs() {
	clearSegments();
	clearDigits();
}

uint8_t* getDisplayValue(uint8_t value) {
	uint8_t *displayValue = NULL;
	switch (value) {
	case 0:
		displayValue = zero;
		break;
	case 1:
		displayValue = one;
		break;
	case 2:
		displayValue = two;
		break;
	case 3:
		displayValue = three;
		break;
	case 4:
		displayValue = four;
		break;
	case 5:
		displayValue = five;
		break;
	case 6:
		displayValue = six;
		break;
	case 7:
		displayValue = seven;
		break;
	case 8:
		displayValue = eight;
		break;
	case 9:
		displayValue = nine;
		break;
	default:
		break;
	}
	return displayValue;
}

void displayNumber() {
	for (uint8_t digit = 0; digit < NUM_OF_DIGITS; digit++) {
		clearLEDs();
//		digitalWrite(digits[digit], LOW);

		if (digit == 0) PORTF &= ~(1 << PF1);
		if (digit == 1) PORTF &= ~(1 << PF2);
		if (digit == 2) PORTF &= ~(1 << PF3);
		if (digit == 3) PORTF &= ~(1 << PF4);

		for (uint8_t segment = 0; segment < NUM_OF_SEGMENTS; segment++) {
			digitalWrite(segments[segment],
					getDisplayValue(digitValues[digit])[segment]);
		}
		NOP;
	}
}

void printValue(int16_t value) {
	digitValues[0] = (int8_t) (value / 1000);
	digitValues[1] = (int8_t) ((value % 1000) / 100);
	digitValues[2] = (int8_t) ((value % 100) / 10);
	digitValues[3] = (int8_t) (value % 10);

	displayNumber();
}

void updateCurrentValue() {
//	if (currentValue == 0) {
//		currentValue = 8888;
//	} else {
//		currentValue = 0;
//	}
	currentValue ++;
	if (currentValue > 9999)
		currentValue = 0;
}

ISR(TIMER1_COMPA_vect) {
	PORTB ^= (1 << PORTB7);	// Reverse LED value 0 - 128

	updateCurrentValue();
//	Serial.println(currentValue);
}

void setup() {
	Serial.begin(9600);

//	Serial.print(DDRB, BIN); Serial.print("->"); Serial.println(PB7);
	DDRB = (1 << PORTB7);	// Pull-up	//	pinMode(LED_BUILTIN, OUTPUT);
//	Serial.print(DDRB, BIN); Serial.print("<-"); Serial.println(PB7);

	TCCR1B = 0;					// Stop timer
	TCCR1B |= (1 << WGM12);		// CTC mode
	OCR1A = 62500 / 2;			// Timer1 compare match A
	TIMSK1 |= (1 << OCIE1A);	// déclencher l'interruption sur le match A
	TCCR1B |= (1 << CS12); 		// démarrer le timer à 1/256 de la fréquence d'horloge
	sei(); 						// activer les interruptions

	// Individual segments
//	pinMode(SEG_A, OUTPUT);
	DDRE |= (1 << PORTE3);
//	pinMode(SEG_B, OUTPUT);
	DDRH |= (1 << PORTH3);
//	pinMode(SEG_C, OUTPUT);
	DDRH |= (1 << PORTH4);
//	pinMode(SEG_D, OUTPUT);
	DDRH |= (1 << PORTH5);
//	pinMode(SEG_E, OUTPUT);
	DDRH |= (1 << PORTH6);
//	pinMode(SEG_F, OUTPUT);
	DDRB |= (1 << PORTB4);
//	pinMode(SEG_G, OUTPUT);
	DDRB |= (1 << PORTB5);
//	pinMode(SEG_DP, OUTPUT);
	DDRB |= (1 << PORTB6);

	// Individual digits
	DDRF |= (1 << PORTF1) | (1 << PORTF2) | (1 << PORTF3) | (1 << PORTF4);
}

void loop() {
	printValue(currentValue);
}

int main(void) {
	setup();
	for (;;) {
		loop();
	}
	return 0;
}
