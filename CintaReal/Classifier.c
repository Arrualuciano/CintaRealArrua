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

void Classifier_Reset(classifier_t *clas) {
    clas->count = 0;
}

void Classifier_NewBox(classifier_t *clas, uint8_t type) {
    if (clas->count >= CLASSIFIER_MAX_BOXES) return;   /* Lista llena: descartar */

	// Buscar a qu� servo va esta caja
	for (uint8_t i = 0; i < CLASSIFIER_NUM_SENSORS; i++) {
		if (clas->boxType[i] == type){
			// Encontr� el servo: guardar datos y salir
			clas->list[clas->count].type = type;
			clas->list[clas->count].servo = i;
			clas->list[clas->count].timePushBox = clas->timeArrival[i];
			clas->count++;
			return;
		}
	}
}

	
void Classifier_Process(classifier_t *clas){
	
	for(uint8_t i=0; i<clas->count; i++){

		clas->list[i].timePushBox--;
		if (clas->list[i].timePushBox == 0){
			clas->onTrigger(clas->list[i].servo);
			removeAt(clas, i);
			i--;
		}
	}
}


