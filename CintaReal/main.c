/**
 * @file    main.c
 * @brief   Aplicacion principal del clasificador de cajas en cinta transportadora real.
 *
 * @details Integra los modulos Comm, Actuator, Classifier, HCSR04 y Servo
 *          sobre un ATmega328P a 16 MHz.
 *
 *          Arquitectura de tiempo:
 *          - Timer0 (prescaler 256): tick de 2 ms para FSMs y clasificador.
 *          - Timer1 (prescaler 8, free-running): base de tiempo en us para HC-SR04.
 *          - Timer2 (prescaler 64, CTC): PWM de 50 Hz para servomotores SG90.
 *
 *          Pinout (segun consigna):
 *          - PB1: TRIGGER HC-SR04 (salida)
 *          - PB2: ECHO HC-SR04 (entrada)
 *          - PB3: SERVO 2 (salida)
 *          - PB4: SERVO 3 (salida)
 *          - PB5: LED heartbeat (salida)
 *          - PD5: IR sensor S0 - trigger zona medicion (entrada, pin 5)
 *          - PD2: IR sensor S1 - salida 0 (entrada, pin 2)
 *          - PD3: IR sensor S2 - salida 1 (entrada, pin 3)
 *          - PD4: IR sensor S3 - salida 2 (entrada, pin 4)
 *          - PC0: control de cinta transportadora (salida, activo-bajo)
 *          - PD7: SERVO 1 (salida)
 *          - USART0: comunicacion serie con HMI Qt (115200 baud, 8N1)
 *
 *          Modos de operacion:
 *          - Normal (open-loop): S0 -> HC-SR04 -> Classifier -> Actuator.
 *          - Calibracion: actuadores deshabilitados, mide tiempos S0->S1/S2/S3.
 *
 * @author  Luciano
 * @date    Mayo 2026
 */

#define F_CPU  16000000u

#include <avr/io.h>
#include <avr/interrupt.h>
#include "HCSR04.h"
#include "Comm.h"
#include "Actuator.h"
#include "Classifier.h"
#include "Servo.h"
#include "Protocol.h"

/* =========================================================================
 * Umbrales de distancia (en cm)
 * ====================================================================== */

/** Altura de montaje del sensor: 20 cm sobre la cinta.
 *  Grande (10cm) -> dist 10cm | Mediana (8cm) -> dist 12cm | Pequena (6cm) -> dist 14cm
 *  Los umbrales se configuran como puntos medios entre clases. */
static uint8_t threshGrande  = 12;
static uint8_t threshMediana = 14;
static uint8_t threshPequena = 17;

/* =========================================================================
 * Prototipos de funciones locales
 * ====================================================================== */
static void     InitTMR0(void);
static void     InitTMR1(void);
static void     On2ms(void);

static void     hw_trigger_write(uint8_t state);
static uint8_t  hw_echo_read(void);
static uint32_t hw_get_us(void);
static void     hw_Act_Set(uint8_t armNum, uint8_t state);

static void    OnCommCmd(uint8_t cmdId, uint8_t *payload, uint8_t nBytes);
static void    OnNewBox(uint8_t boxType);
static uint8_t OnActuatorTrigger(uint8_t outNum);

static void    SendEvent(uint8_t evtId, uint8_t *data, uint8_t nBytes);
static void    ProcessIRSensors(void);

/* =========================================================================
 * Variables globales
 * ====================================================================== */

static uint8_t time100ms;
static uint8_t s0Prev = 0;

static HCSR04_Setting_t sensor;
static Comm_Protocol    comm;
static Actuator_t       act0, act1, act2;
static classifier_t     classifier;
static Actuator_t      *actuators[3];

/** Estado de la cinta: 0 = detenida, 1 = clasificando. */
static uint8_t beltRunning = 0;

/** Modo calibracion: 0 = normal, 1 = calibrando. */
static uint8_t calibMode = 0;

/** Contador de ticks para medir tiempos de calibracion (desde S0). */
static uint16_t calibTickCounter = 0;

/** Flags para evitar envio duplicado del tiempo de cada sensor en calibracion. */
static uint8_t calibSent[3] = {0, 0, 0};

/** Estado previo de los sensores IR S1, S2, S3 (para deteccion de flanco). */
static uint8_t irPrev[3] = {0, 0, 0};

/** Debounce de S0 en ticks de 2 ms: evita detecciones multiples por rebote. */
#define S0_DEBOUNCE_TICKS  250   /* 500 ms */
static uint16_t s0Debounce = 0;

/** Contador heartbeat para envio de alive al HMI. */
static uint8_t aliveCounter = 0;

/* =========================================================================
 * ISRs
 * ====================================================================== */

ISR(TIMER0_COMPA_vect) {
    OCR0A += 125;
    GPIOR0 |= _BV(GPIOR00);
}

ISR(USART_RX_vect) {
    Comm_PutRxByte(&comm, UDR0);
}

/* =========================================================================
 * Inicializacion de perifericos
 * ====================================================================== */

/** Timer0: tick de 2 ms (prescaler 256, CTC libre). */
static void InitTMR0(void) {
    TCCR0A = 0;
    TIFR0  = TIFR0;
    OCR0A  = 124;
    TIMSK0 = (1 << OCIE0A);
    TCCR0B = (1 << CS02);
}

/** Timer1: free-running a 2 MHz (prescaler 8) para HC-SR04. */
static void InitTMR1(void) {
    TCCR1A = 0;
    TCCR1B = (1 << CS11);
}

/* =========================================================================
 * HAL
 * ====================================================================== */

/** Controla el pin TRIGGER del HC-SR04 (PB1). */
static void hw_trigger_write(uint8_t state) {
    if (state) PORTB |=  (1 << PB1);
    else       PORTB &= ~(1 << PB1);
}

/** Lee el pin ECHO del HC-SR04 (PB2). */
static uint8_t hw_echo_read(void) {
    return (PINB & (1 << PB2)) ? 1 : 0;
}

/** Retorna el tiempo actual en microsegundos (TCNT1 / 2). */
static uint32_t hw_get_us(void) {
    return TCNT1 / 2;
}

/**
 * @brief Controla un actuador: mueve el servo fisico.
 * @param armNum Numero de servo (0, 1 o 2).
 * @param state  1 = extender, 0 = retraer.
 */
static void hw_Act_Set(uint8_t armNum, uint8_t state) {
    Servo_SetPosition(armNum, state);
}

/* =========================================================================
 * Comunicacion con HMI
 * ====================================================================== */

/**
 * @brief Envia un evento al HMI Qt via protocolo UNER.
 * @param evtId   Identificador del evento.
 * @param data    Puntero a los datos del evento (puede ser NULL).
 * @param nBytes  Cantidad de bytes de datos.
 */
static void SendEvent(uint8_t evtId, uint8_t *data, uint8_t nBytes) {
    Comm_Send(&comm, evtId, data, nBytes);
}

/* =========================================================================
 * Callbacks de la aplicacion
 * ====================================================================== */

/**
 * @brief Despacha comandos recibidos via protocolo UNER desde el HMI.
 *
 * @details Soporta todos los comandos definidos en Protocol.h.
 */
static void OnCommCmd(uint8_t cmdId, uint8_t *payload, uint8_t nBytes) {
    switch (cmdId) {

        case CMD_ALIVE:
            SendEvent(EVT_ALIVE, 0, 0);
            break;

        case CMD_START:
            beltRunning = 1;
            calibMode   = 0;
            PORTC &= ~(1 << PC0);
            break;

        case CMD_STOP:
            beltRunning = 0;
            PORTC |= (1 << PC0);
            break;

        case CMD_RESET:
            beltRunning = 0;
            calibMode   = 0;
            PORTC |= (1 << PC0);
            Classifier_Reset(&classifier);
            break;

        case CMD_SET_CONFIG:
            /* payload: [tipo0, tipo1, tipo2] */
            if (nBytes >= 3) {
                classifier.boxType[0] = payload[0];
                classifier.boxType[1] = payload[1];
                classifier.boxType[2] = payload[2];
            }
            break;

        case CMD_SET_TIMES:
            /* payload: [servo, timeHi, timeLo] */
            if (nBytes >= 3) {
                uint8_t servo = payload[0];
                uint16_t ticks = ((uint16_t)payload[1] << 8) | payload[2];
                if (servo < 3) {
                    classifier.timeArrival[servo] = ticks;
                }
            }
            break;

        case CMD_SET_THRESHOLD:
            /* payload: [grande, mediana, pequena] */
            if (nBytes >= 3) {
                threshGrande  = payload[0];
                threshMediana = payload[1];
                threshPequena = payload[2];
            }
            break;

        case CMD_TEST_SERVO:
            /* payload: [servoNum] - disparo manual de un servo */
            if (nBytes >= 1) {
                uint8_t servo = payload[0];
                if (servo < 3) {
                    Actuator_Trigger(actuators[servo]);
                }
            }
            break;

        case CMD_SET_SERVO_ANGLE:
            /* payload: [servoNum, pulseTicks] */
            if (nBytes >= 2) {
                Servo_SetPulse(payload[0], payload[1]);
            }
            break;

        case CMD_CALIB_MODE:
            /* payload: [1=entrar, 0=salir] */
            if (nBytes >= 1) {
                calibMode = payload[0];
                if (calibMode) {
                    beltRunning = 0;
                    calibTickCounter = 0;
                    calibSent[0] = 0;
                    calibSent[1] = 0;
                    calibSent[2] = 0;
                }
            }
            break;
    }
}

/**
 * @brief Registra una nueva caja y envia el evento al HMI.
 * @param boxType Tipo de caja en cm (6, 8 o 10).
 */
static void OnNewBox(uint8_t boxType) {
    Classifier_NewBox(&classifier, boxType);

    /* Buscar a que servo fue asignada para informar al HMI */
    for (uint8_t i = 0; i < 3; i++) {
        if (classifier.boxType[i] == boxType) {
            uint8_t evtData[4];
            evtData[0] = boxType;
            evtData[1] = i;
            evtData[2] = (uint8_t)(classifier.timeArrival[i] >> 8);
            evtData[3] = (uint8_t)(classifier.timeArrival[i] & 0xFF);
            SendEvent(EVT_NEW_BOX, evtData, 4);
            break;
        }
    }
}

/**
 * @brief Callback del clasificador: solicita el disparo del actuador y notifica al HMI.
 */
static uint8_t OnActuatorTrigger(uint8_t outNum) {
    if (outNum < 3) {
        uint8_t result = Actuator_Trigger(actuators[outNum]);
        if (result) {
            uint8_t evtData = outNum;
            SendEvent(EVT_SERVO_FIRED, &evtData, 1);
        }
        return result;
    }
    return 0;
}

/* =========================================================================
 * Lectura de sensores IR (S1, S2, S3)
 * ====================================================================== */

/**
 * @brief Lee los sensores IR S1 (PD2, pin 2), S2 (PD3, pin 3), S3 (PD4, pin 4) y envia eventos
 *        al HMI al detectar cambios de estado.
 *
 * @details En modo calibracion, al detectar flanco de subida de un sensor
 *          se envia el tiempo transcurrido desde S0 como EVT_CALIB_TIME.
 */
static void ProcessIRSensors(void) {
    uint8_t pins[3];
    pins[0] = (PIND & (1 << PD2)) ? 1 : 0;   /* S1: pin 2 */
    pins[1] = (PIND & (1 << PD3)) ? 1 : 0;   /* S2: pin 3 */
    pins[2] = (PIND & (1 << PD4)) ? 1 : 0;   /* S3: pin 4 */

    for (uint8_t i = 0; i < 3; i++) {
        if (pins[i] != irPrev[i]) {
            /* Enviar cambio de estado (sensor activo-bajo: invertir) */
            uint8_t evtData[2];
            evtData[0] = i + 1;
            evtData[1] = !pins[i];
            SendEvent(EVT_IR_SENSOR, evtData, 2);

            /* Modo calibracion: flanco de bajada (activo-bajo) = objeto detectado */
            if (calibMode && !pins[i] && !calibSent[i]) {
                uint8_t calibData[3];
                calibData[0] = i;
                calibData[1] = (uint8_t)(calibTickCounter >> 8);
                calibData[2] = (uint8_t)(calibTickCounter & 0xFF);
                SendEvent(EVT_CALIB_TIME, calibData, 3);
                calibSent[i] = 1;
            }

            irPrev[i] = pins[i];
        }
    }
}

/* =========================================================================
 * Tarea periodica
 * ====================================================================== */

/** Tarea de 2 ms: heartbeat LED, alive al HMI, contador de calibracion. */
static void On2ms(void) {
    GPIOR0 &= ~_BV(GPIOR00);

    /* Heartbeat LED cada 100 ms */
    if (--time100ms == 0) {
        time100ms = 50;
        PINB = (1 << PINB5);
    }

    /* Envio periodico de alive al HMI (~500 ms = 250 ticks) */
    if (++aliveCounter >= 250) {
        aliveCounter = 0;
        SendEvent(EVT_ALIVE, 0, 0);
    }

    /* Decrementar debounce de S0 */
    if (s0Debounce > 0) s0Debounce--;

    /* Incrementar contador de calibracion si esta activo */
    if (calibMode) {
        calibTickCounter++;
    }
}

/* =========================================================================
 * Punto de entrada
 * ====================================================================== */

int main(void) {
    cli();

    time100ms = 50;

    /* Configurar pines de I/O */
    DDRB  = (1 << PB5);         /* PB5: LED heartbeat (salida)   */
    DDRB |= (1 << PB1);         /* PB1: TRIGGER HC-SR04 (salida) */
    DDRB &= ~(1 << PB2);        /* PB2: ECHO HC-SR04 (entrada)   */
    DDRC |= (1 << PC0);         /* PC0 (A0): control cinta (salida) */
    PORTC |= (1 << PC0);        /* Cinta apagada al inicio (activo-bajo) */
    DDRD &= ~((1 << PD2) | (1 << PD3) | (1 << PD4) | (1 << PD5));
    /* Pull-ups internos en los pines IR para evitar lecturas flotantes */
    PORTD |= (1 << PD2) | (1 << PD3) | (1 << PD4) | (1 << PD5);

    /* Tipos de salida por defecto: salida 0 = 6cm, salida 1 = 8cm, salida 2 = 10cm */
    uint8_t defaultBoxType[3] = {6, 8, 10};

    actuators[0] = &act0;
    actuators[1] = &act1;
    actuators[2] = &act2;

    /* Inicializar perifericos y modulos */
    InitTMR0();
    InitTMR1();
    Comm_Init(&comm, OnCommCmd);
    HCSR04_Init(&sensor, hw_trigger_write, hw_echo_read, hw_get_us);
    Servo_Init();
    Actuator_Init(&act0, 0, 0, hw_Act_Set);
    Actuator_Init(&act1, 1, 0, hw_Act_Set);
    Actuator_Init(&act2, 2, 0, hw_Act_Set);
    Classifier_Init(&classifier, defaultBoxType, OnActuatorTrigger);

    sei();

    /* ---- Super-loop ---- */
    while (1) {

        if (GPIOR0 & _BV(GPIOR00)) {
            On2ms();
            Actuator_Process(&act0);
            Actuator_Process(&act1);
            Actuator_Process(&act2);

            /* Clasificador activo solo en modo normal con cinta en marcha */
            if (beltRunning && !calibMode) {
                Classifier_Process(&classifier);
            }

            /* Lectura de sensores IR cada 2 ms */
            ProcessIRSensors();
        }

        Comm_Process(&comm);

        /* S0 (PD5, pin 5): detectar flanco para disparar HC-SR04 (con debounce) */
        {
            uint8_t s0Now = (PIND & (1 << PD5)) ? 1 : 0;

            if (s0Debounce == 0) {
                if (s0Now != s0Prev) {
                    /* Notificar cambio de S0 al HMI (sensor activo-bajo: invertir) */
                    uint8_t evtData[2] = {0, !s0Now};
                    SendEvent(EVT_IR_SENSOR, evtData, 2);

                    if (!s0Now && s0Prev) {
                        /* Flanco de bajada (activo-bajo): objeto detectado */
                        if (sensor.state == HCSR_IDLE) {
                            sensor.state = HCSR_TRIGGER;
                        }

                        /* Resetear contador de calibracion al detectar caja en S0 */
                        if (calibMode) {
                            calibTickCounter = 0;
                            calibSent[0] = 0;
                            calibSent[1] = 0;
                            calibSent[2] = 0;
                        }

                        /* Iniciar debounce: ignorar rebotes durante 500 ms */
                        s0Debounce = S0_DEBOUNCE_TICKS;
                    }
                }
            }
            s0Prev = s0Now;
        }

        HCSR04_Process(&sensor);

        if (sensor.state == HCSR_READY) {
            /* Enviar distancia medida al HMI */
            {
                uint8_t distData[2];
                distData[0] = (uint8_t)(sensor.distance >> 8);
                distData[1] = (uint8_t)(sensor.distance & 0xFF);
                SendEvent(EVT_DISTANCE, distData, 2);
            }

            /* Clasificar segun umbrales (solo en modo normal) */
            if (beltRunning && !calibMode) {
                if (sensor.distance < threshGrande) {
                    OnNewBox(10);
                } else if (sensor.distance < threshMediana) {
                    OnNewBox(8);
                } else if (sensor.distance < threshPequena) {
                    OnNewBox(6);
                }
            }
            sensor.state = HCSR_IDLE;
        }

        if (sensor.state == HCSR_ERROR) {
            sensor.state = HCSR_IDLE;
        }
    }
}
