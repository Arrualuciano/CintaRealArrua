/**
 * @file    HCSR04.c
 * @brief   Implementacion del driver para sensor ultrasonico HC-SR04.
 *
 * @details La FSM no bloqueante ejecuta la secuencia completa de medicion:
 *
 *          1. HCSR_TRIGGER:   activa el pin TRIGGER en alto y registra t_ref.
 *          2. HCSR_TRIG_WAIT: mantiene TRIGGER alto durante >= 10 us, luego
 *                             lo baja y pasa a esperar el eco.
 *          3. HCSR_ECHO_WH:   espera el flanco ascendente en el pin ECHO.
 *                             Si supera el timeout, pasa a HCSR_ERROR.
 *          4. HCSR_ECHO_WL:   mide la duracion del pulso ECHO hasta el flanco
 *                             descendente y calcula la distancia en cm.
 *          5. HCSR_READY:     la distancia esta disponible en @p distance.
 *
 *          Formula de conversion:
 *          @code
 *          distancia_cm = tiempo_echo_us / 58
 *          @endcode
 *          (velocidad del sonido: 343 m/s -> 29 us/cm ida, 58 us/cm ida+vuelta)
 *
 * @author  Luciano
 * @date    Abril 2026
 */

#include "HCSR04.h"

/**
 * @brief Inicializa el driver HC-SR04.
 *
 * @details Almacena los punteros a funcion para el acceso al hardware,
 *          configura el timeout por defecto (HCSR04_TIMEOUT_US) y deja
 *          el sensor en estado HCSR_IDLE con el pin TRIGGER en bajo.
 *
 * @param setting  Puntero a la instancia HCSR04_Setting_t a inicializar.
 * @param trigger  Funcion de escritura del pin TRIGGER.
 * @param echo     Funcion de lectura del pin ECHO.
 * @param timer    Funcion que retorna el tiempo actual en microsegundos.
 */
void HCSR04_Init(HCSR04_Setting_t *setting, hcsr_write_ptr trigger,
                 hcsr_read_ptr echo, hcsr_get_time_ptr timer) {

    /* Guardar punteros a funcion de hardware */
    setting->trigger_write = trigger;
    setting->echo_read     = echo;
    setting->get_us        = timer;

    /* Inicializar variables internas */
    setting->state    = HCSR_IDLE;
    setting->t_ref    = 0;
    setting->timeout  = HCSR04_TIMEOUT_US;
    setting->distance = 0;

    /* Asegurar que el pin TRIGGER comience en bajo */
    setting->trigger_write(0);
}

/**
 * @brief Ejecuta un paso de la FSM del sensor HC-SR04 (no bloqueante).
 *
 * @details Debe llamarse frecuentemente desde el super-loop. Para iniciar
 *          una nueva medicion, el usuario debe setear:
 *          @code
 *          setting->state = HCSR_TRIGGER;
 *          @endcode
 *
 *          Al finalizar, el estado sera:
 *          - HCSR_READY: medicion exitosa, distancia en @p setting->distance (cm).
 *          - HCSR_ERROR: timeout, el sensor no respondio.
 *
 *          El usuario debe resetear el estado a HCSR_IDLE despues de leer el
 *          resultado para permitir nuevas mediciones.
 *
 * @param setting Puntero a la instancia HCSR04_Setting_t.
 */
void HCSR04_Process(HCSR04_Setting_t *setting) {

    switch (setting->state) {

        case HCSR_IDLE:
            /* Sensor inactivo, esperando solicitud externa */
            break;

        case HCSR_TRIGGER:
            /* Iniciar pulso de trigger: poner TRIGGER en alto */
            setting->trigger_write(1);
            setting->t_ref = setting->get_us();
            setting->state = HCSR_TRIG_WAIT;
            break;

        case HCSR_TRIG_WAIT:
            /* Mantener TRIGGER alto al menos 10 us */
            if ((setting->get_us() - setting->t_ref) >= 10) {
                setting->trigger_write(0);
                setting->t_ref = setting->get_us();
                setting->state = HCSR_ECHO_WH;
            }
            break;

        case HCSR_ECHO_WH:
            /* Esperar flanco ascendente del pin ECHO */
            if (setting->echo_read() == 1) {
                setting->t_ref = setting->get_us();
                setting->state = HCSR_ECHO_WL;
            } else if ((setting->get_us() - setting->t_ref) >= setting->timeout) {
                setting->state = HCSR_ERROR;
            }
            break;

        case HCSR_ECHO_WL:
            /* Esperar flanco descendente del pin ECHO y calcular distancia */
            if (setting->echo_read() == 0) {
                uint32_t time_echo = setting->get_us() - setting->t_ref;
                setting->distance = (uint16_t)(time_echo / 58);
                setting->state = HCSR_READY;
            }
            break;

        case HCSR_READY:
            /* Medicion completa, esperando que el usuario lea el resultado */
            break;

        case HCSR_ERROR:
            /* Error de timeout, esperando reset del usuario */
            break;

        default:
            setting->state = HCSR_IDLE;
            break;
    }
}
