/**
 * @file    Servo.h
 * @brief   Driver PWM por software para servomotores SG90.
 *
 * @details Genera senales PWM de 50 Hz (periodo 20 ms) usando Timer2
 *          en modo CTC con resolucion de 0.5 ms por tick.
 *
 *          Mapeo de pines (segun consigna):
 *          | Servo | Consigna | ATmega328P |
 *          |-------|----------|------------|
 *          |   0   | SERVO 1  |    PD7     |
 *          |   1   | SERVO 2  |    PB3     |
 *          |   2   | SERVO 3  |    PB4     |
 *
 *          Rango de pulso SG90:
 *          - 1.0 ms (2 ticks) = posicion retraida (0 grados)
 *          - 2.0 ms (4 ticks) = posicion extendida (180 grados)
 *
 *          Timer2 configuracion:
 *          - Modo CTC, prescaler 64, OCR2A = 124
 *          - Frecuencia de interrupcion: 16 MHz / 64 / 125 = 2000 Hz (0.5 ms)
 *          - 40 ticks = 20 ms = 50 Hz
 *
 * @author  Luciano
 * @date    Mayo 2026
 */

#ifndef SERVO_H_
#define SERVO_H_

#include <stdint.h>

/** @brief Cantidad de servomotores controlados. */
#define SERVO_COUNT      3

/** @brief Ancho de pulso para posicion retraida (ticks de 0.5 ms). */
//#define SERVO_PULSE_MIN  2    /* 1.0 ms */
/** @brief Ancho de pulso para posicion extendida (ticks de 0.5 ms). */
//#define SERVO_PULSE_MAX  4    /* 2.0 ms */
/* Si el servo gira en sentido inverso, comentar las 2 lineas anteriores
   y descomentar las siguientes: */
#define SERVO_PULSE_MIN  2.5
#define SERVO_PULSE_MAX  4

/** @brief Periodo completo de la senal PWM: 20 ms = 40 ticks de 0.5 ms. */
#define SERVO_PERIOD     40

/**
 * @brief Inicializa el modulo de servos.
 *
 * @details Configura los pines PD7, PB3 y PB4 como salidas,
 *          posiciona todos los servos en la posicion retraida
 *          e inicia Timer2 para la generacion de PWM.
 */
void Servo_Init(void);

/**
 * @brief Cambia la posicion de un servomotor.
 *
 * @param servoNum Numero de servo (0, 1 o 2).
 * @param extended 1 = posicion extendida (2 ms), 0 = posicion retraida (1 ms).
 */
void Servo_SetPosition(uint8_t servoNum, uint8_t extended);

/**
 * @brief Ajusta el ancho de pulso de extension de un servo.
 *
 * @details Permite modificar en runtime el angulo de patada.
 *          El pulso de retraccion (SERVO_PULSE_MIN) permanece fijo.
 *
 * @param servoNum   Numero de servo (0, 1 o 2).
 * @param pulseTicks Ancho de pulso en ticks de 0.5 ms (2..4 tipico).
 */
void Servo_SetPulse(uint8_t servoNum, uint8_t pulseTicks);

#endif /* SERVO_H_ */
