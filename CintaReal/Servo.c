/**
 * @file    Servo.c
 * @brief   Implementacion del driver PWM por software para servomotores SG90.
 *
 * @details Usa Timer2 en modo CTC (prescaler 64, OCR2A = 124) para generar
 *          una interrupcion cada 0.5 ms. Un contador ciclico de 40 ticks
 *          produce el periodo de 20 ms (50 Hz) requerido por los servos SG90.
 *
 *          En cada ciclo:
 *          - tick 0: se activan las salidas de los servos habilitados.
 *          - tick N (= ancho de pulso): se desactiva la salida correspondiente.
 *
 *          Mapeo de pines (segun consigna):
 *          - Servo 0 (SERVO 1): PD7
 *          - Servo 1 (SERVO 2): PB3
 *          - Servo 2 (SERVO 3): PB4
 *
 * @author  Luciano
 * @date    Mayo 2026
 */

#include <avr/io.h>
#include <avr/interrupt.h>
#include "Servo.h"

/** @brief Ancho de pulso actual de cada servo en ticks de 0.5 ms (0 = deshabilitado). */
static volatile uint8_t servo_pulse[SERVO_COUNT];

/** @brief Ancho de pulso configurado para la posicion extendida de cada servo. */
static uint8_t servo_pulse_max[SERVO_COUNT];

/** @brief Contador ciclico del periodo PWM (0..SERVO_PERIOD-1). */
static volatile uint8_t servo_counter;

/**
 * @brief ISR de Timer2 Compare Match A: genera la senal PWM de los servos.
 *
 * @details Se ejecuta cada 0.5 ms. Incrementa el contador ciclico y
 *          controla los pines de salida segun el ancho de pulso configurado.
 */
ISR(TIMER2_COMPA_vect) {
    servo_counter++;
    if (servo_counter >= SERVO_PERIOD) {
        servo_counter = 0;
    }

    if (servo_counter == 0) {
        if (servo_pulse[0]) PORTD |=  (1 << PD7);
        if (servo_pulse[1]) PORTB |=  (1 << PB3);
        if (servo_pulse[2]) PORTB |=  (1 << PB4);
    }

    if (servo_pulse[0] && servo_counter == servo_pulse[0]) PORTD &= ~(1 << PD7);
    if (servo_pulse[1] && servo_counter == servo_pulse[1]) PORTB &= ~(1 << PB3);
    if (servo_pulse[2] && servo_counter == servo_pulse[2]) PORTB &= ~(1 << PB4);
}

void Servo_Init(void) {
    DDRD |= (1 << PD7);
    DDRB |= (1 << PB3) | (1 << PB4);

    for (uint8_t i = 0; i < SERVO_COUNT; i++) {
        servo_pulse[i]     = SERVO_PULSE_MAX;
        servo_pulse_max[i] = SERVO_PULSE_MAX;
    }
    servo_counter = 0;

    TCCR2A = (1 << WGM21);     /* CTC mode (top = OCR2A) */
    OCR2A  = 124;              /* 16 MHz / 64 / 125 = 2000 Hz = 0.5 ms */
    TIMSK2 = (1 << OCIE2A);   /* Habilitar interrupcion Compare Match A */
    TCCR2B = (1 << CS22);     /* Prescaler 64: CS22=1, CS21=0, CS20=0 */
}

void Servo_SetPosition(uint8_t servoNum, uint8_t extended) {
    if (servoNum >= SERVO_COUNT) return;
    servo_pulse[servoNum] = extended ? SERVO_PULSE_MIN : servo_pulse_max[servoNum];
}

void Servo_SetPulse(uint8_t servoNum, uint8_t pulseTicks) {
    if (servoNum >= SERVO_COUNT) return;
    if (pulseTicks < 1 || pulseTicks > 5) return;
    servo_pulse_max[servoNum] = pulseTicks;
}
