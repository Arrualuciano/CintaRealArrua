/**
 * @file    Classifier.c
 * @brief   Implementacion del clasificador de cajas (modo open-loop).
 *
 * @details Cada caja medida recibe un servo destino y un timer de arribo.
 *          Classifier_Process() decrementa los timers cada 2 ms y dispara
 *          el servo cuando el timer llega a cero.
 *
 *          Invariantes de la lista:
 *          - Las cajas estan ordenadas por orden de llegada (la mas antigua en list[0]).
 *          - Una caja se elimina cuando su timer expira y el servo es disparado.
 *
 * @author  Luciano
 * @date    Mayo 2026
 */

#include <stdint.h>
#include "Classifier.h"

/**
 * @brief Elimina la caja en la posicion @p idx desplazando las siguientes.
 *
 * @details Operacion O(n). Mantiene el orden de llegada de las cajas restantes.
 *
 * @param clas Puntero al clasificador.
 * @param idx  Indice (0-based) de la caja a eliminar.
 */
static void removeAt(classifier_t *clas, uint8_t idx) {
    if (idx >= clas->count) return;
    for (uint8_t i = idx; i + 1 < clas->count; i++) {
        clas->list[i] = clas->list[i + 1];
    }
    clas->count--;
}

/* -------------------------------------------------------------------------
 * Implementacion de las funciones publicas
 * ---------------------------------------------------------------------- */

/**
 * @brief Inicializa el clasificador con las asignaciones de salida.
 *
 * @details Resetea la lista de cajas en transito, guarda los tipos
 *          asignados a cada salida, el callback de disparo, y configura
 *          los tiempos de arribo por defecto para cada servo.
 *
 * @param clas      Puntero al clasificador a inicializar.
 * @param boxType   Arreglo de CLASSIFIER_NUM_SENSORS elementos con el tipo
 *                  de caja correspondiente a cada salida (ej: {6, 8, 10}).
 * @param onTrigger Callback que sera invocado cuando se deba disparar un actuador.
 */
void Classifier_Init(classifier_t *clas, uint8_t *boxType, classifier_trigger_cb onTrigger) {
    for (uint8_t i = 0; i < CLASSIFIER_NUM_SENSORS; i++) {
        clas->boxType[i] = boxType[i];
    }
    clas->count     = 0;
    clas->onTrigger = onTrigger;
    clas->timeArrival[0] = TIME_ARRIVAL_SERVO0;
    clas->timeArrival[1] = TIME_ARRIVAL_SERVO1;
    clas->timeArrival[2] = TIME_ARRIVAL_SERVO2;
}

/**
 * @brief Vacia la lista de cajas en transito sin modificar la configuracion.
 * @param clas Puntero al clasificador.
 */
void Classifier_Reset(classifier_t *clas) {
    clas->count = 0;
}

/**
 * @brief Registra una nueva caja medida por el HC-SR04.
 *
 * @details Busca en boxType[] a que servo corresponde el tipo de caja,
 *          carga el timer de arribo configurado y agrega la caja a la lista.
 *          Si la lista esta llena o el tipo no coincide con ninguna salida,
 *          la caja se descarta silenciosamente.
 *
 * @param clas Puntero al clasificador.
 * @param type Tipo de caja (altura en cm: 6, 8 o 10).
 */
void Classifier_NewBox(classifier_t *clas, uint8_t type) {
    if (clas->count >= CLASSIFIER_MAX_BOXES) return;   /* Lista llena: descartar */

    /* Buscar a que servo va esta caja */
    for (uint8_t i = 0; i < CLASSIFIER_NUM_SENSORS; i++) {
        if (clas->boxType[i] == type) {
            /* Encontro el servo: guardar datos y salir */
            clas->list[clas->count].type        = type;
            clas->list[clas->count].servo       = i;
            clas->list[clas->count].timePushBox  = clas->timeArrival[i];
            clas->count++;
            return;
        }
    }
}

/**
 * @brief Decrementa los timers de las cajas en transito y dispara los servos.
 *
 * @details Debe llamarse cada 2 ms desde el tick del Timer0. Recorre la lista
 *          de cajas, decrementa timePushBox y al llegar a 0 invoca onTrigger()
 *          con el servo correspondiente. La caja disparada se elimina de la lista.
 *
 * @param clas Puntero al clasificador.
 */
void Classifier_Process(classifier_t *clas) {

    for (uint8_t i = 0; i < clas->count; i++) {
        clas->list[i].timePushBox--;
        if (clas->list[i].timePushBox == 0) {
            clas->onTrigger(clas->list[i].servo);
            removeAt(clas, i);
            i--;   /* Compensar el desplazamiento de la lista */
        }
    }
}
