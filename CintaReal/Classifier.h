/**
 * @file    Classifier.h
 * @brief   Clasificador de cajas para cinta transportadora (modo open-loop).
 *
 * @details Mantiene una lista de cajas en transito. Cada caja tiene asignado
 *          un servo destino y un timer de arribo (timePushBox). El timer se
 *          decrementa cada 2 ms en Classifier_Process(); al llegar a cero se
 *          dispara el servo correspondiente.
 *
 *          Flujo de uso:
 *          @code
 *          Classifier_Init(&clas, boxTypes, onTrigger);
 *          Classifier_NewBox(&clas, tipo);       // al medir una caja
 *          Classifier_Process(&clas);            // cada 2 ms
 *          @endcode
 *
 * @author  Luciano
 * @date    Mayo 2026
 */

#ifndef CLASSIFIER_H_
#define CLASSIFIER_H_

#include <stdint.h>

/** @brief Cantidad maxima de cajas en transito que puede rastrear el clasificador. */
#define CLASSIFIER_MAX_BOXES    32

/** @brief Numero de sensores IR / salidas en la cinta. */
#define CLASSIFIER_NUM_SENSORS   3

#define TIME_ARRIVAL_SERVO0   1000   // 2 segundos = 1000 ticks
#define TIME_ARRIVAL_SERVO1   2000   // 4 segundos = 2000 ticks
#define TIME_ARRIVAL_SERVO2   3000   // 6 segundos = 3000 ticks

/**
 * @brief Callback que intenta disparar el actuador de la salida @p outNum.
 *
 * @param outNum Numero de salida (0, 1 o 2).
 * @return       1 si el disparo fue exitoso, 0 si el brazo estaba ocupado.
 */
typedef uint8_t (*classifier_trigger_cb)(uint8_t outNum);

/**
 * @brief Representa una caja en transito en la cinta.
 */
typedef struct {
    uint8_t		type;   /**< Tipo de caja (altura en cm: 6, 8 o 10).               */
    uint8_t		servo;  /**< A que servo va esta caja: (0, 1 o 2).                   */
	uint16_t	timePushBox;
	
} classifier_box_t;

/**
 * @brief Contexto del clasificador.
 */
typedef struct {
    classifier_box_t      list[CLASSIFIER_MAX_BOXES]; /**< Lista de cajas en transito (orden de llegada). */
    uint8_t               count;                      /**< Cantidad de cajas actualmente en la lista.     */
    uint8_t               boxType[CLASSIFIER_NUM_SENSORS]; /**< Tipo de caja asignado a cada salida.      */
    classifier_trigger_cb onTrigger;                  /**< Callback para disparar actuadores.             */
	uint16_t			  timeArrival[3];
	
} classifier_t;

/**
 * @brief Inicializa el clasificador con las asignaciones de salida.
 *
 * @details Resetea la lista de cajas en transito y guarda los tipos
 *          asignados a cada salida y el callback de disparo.
 *
 * @param clas      Puntero al clasificador a inicializar.
 * @param boxType   Arreglo de CLASSIFIER_NUM_SENSORS elementos con el tipo
 *                  de caja correspondiente a cada salida (ej: {6, 8, 10}).
 * @param onTrigger Callback que sera invocado cuando se deba disparar un actuador.
 */
void Classifier_Init(classifier_t *clas, uint8_t *boxType, classifier_trigger_cb onTrigger);

/**
 * @brief Vacia la lista de cajas en transito sin modificar la configuracion.
 * @param clas Puntero al clasificador.
 */
void Classifier_Reset(classifier_t *clas);

/**
 * @brief Registra una nueva caja medida por el HC-SR04.
 *
 * @details Busca en boxType[] a que servo corresponde el tipo de caja,
 *          carga el timer de arribo y agrega la caja a la lista.
 *          Si la lista esta llena o el tipo no coincide, se descarta.
 *
 * @param clas Puntero al clasificador.
 * @param type Tipo de caja (altura en cm: 6, 8 o 10).
 */
void Classifier_NewBox(classifier_t *clas, uint8_t type);

/**
 * @brief Decrementa los timers de las cajas en transito y dispara los servos.
 *
 * @details Debe llamarse cada 2 ms desde el tick del Timer0. Recorre la lista
 *          de cajas, decrementa timePushBox y al llegar a 0 invoca onTrigger()
 *          con el servo correspondiente.
 *
 * @param clas Puntero al clasificador.
 */
void Classifier_Process(classifier_t *clas);

#endif /* CLASSIFIER_H_ */
