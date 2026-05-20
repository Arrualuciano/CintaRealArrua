/**
 * @file    Protocol.h
 * @brief   Definiciones del protocolo de comunicacion entre MCU y HMI Qt.
 *
 * @details Define los identificadores de comandos (Qt -> MCU) y eventos
 *          (MCU -> Qt) utilizados sobre el protocolo UNER.
 *
 *          Comandos (Qt -> MCU):
 *          - CMD_ALIVE (0xF0):         heartbeat bidireccional.
 *          - CMD_START (0x50):         iniciar clasificacion.
 *          - CMD_STOP  (0x51):         detener clasificacion.
 *          - CMD_RESET (0x53):         resetear sistema.
 *          - CMD_SET_CONFIG (0x60):    configurar tipos de salida [tipo0, tipo1, tipo2].
 *          - CMD_SET_TIMES  (0x61):    configurar tiempo de arribo [servo, timeHi, timeLo].
 *          - CMD_SET_THRESHOLD (0x62): configurar umbrales de distancia [grande, mediana, pequena].
 *          - CMD_TEST_SERVO (0x63):    test manual de un servo [servoNum].
 *          - CMD_SET_SERVO_ANGLE (0x64): ajustar pulso de servo [servoNum, pulseTicks].
 *          - CMD_CALIB_MODE (0x65):    entrar/salir del modo calibracion [1=entrar, 0=salir].
 *
 *          Eventos (MCU -> Qt):
 *          - EVT_ALIVE (0xF0):        respuesta heartbeat.
 *          - EVT_NEW_BOX (0x70):      nueva caja detectada [type, servo, timeHi, timeLo].
 *          - EVT_IR_SENSOR (0x71):    cambio de estado IR [sensorNum, state].
 *          - EVT_SERVO_FIRED (0x72):  servo disparado [servoNum].
 *          - EVT_DISTANCE (0x74):     distancia medida [distHi, distLo].
 *          - EVT_CALIB_TIME (0x75):   tiempo de calibracion [sensorNum, timeHi, timeLo].
 *
 * @author  Luciano
 * @date    Mayo 2026
 */

#ifndef PROTOCOL_H_
#define PROTOCOL_H_

#include <stdint.h>

/* =========================================================================
 * Comandos Qt -> MCU
 * ====================================================================== */
#define CMD_ALIVE            0xF0
#define CMD_START            0x50
#define CMD_STOP             0x51
#define CMD_RESET            0x53
#define CMD_SET_CONFIG       0x60   /* [tipo0, tipo1, tipo2]           */
#define CMD_SET_TIMES        0x61   /* [servo, timeHi, timeLo]         */
#define CMD_SET_THRESHOLD    0x62   /* [grande, mediana, pequena]      */
#define CMD_TEST_SERVO       0x63   /* [servoNum]                      */
#define CMD_SET_SERVO_ANGLE  0x64   /* [servoNum, pulseTicks]          */
#define CMD_CALIB_MODE       0x65   /* [1=entrar, 0=salir]             */

/* =========================================================================
 * Eventos MCU -> Qt
 * ====================================================================== */
#define EVT_ALIVE            0xF0
#define EVT_NEW_BOX          0x70   /* [type, servo, timeHi, timeLo]   */
#define EVT_IR_SENSOR        0x71   /* [sensorNum, state]              */
#define EVT_SERVO_FIRED      0x72   /* [servoNum]                      */
#define EVT_DISTANCE         0x74   /* [distHi, distLo]                */
#define EVT_CALIB_TIME       0x75   /* [sensorNum, timeHi, timeLo]     */

#endif /* PROTOCOL_H_ */
