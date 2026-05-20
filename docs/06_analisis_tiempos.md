# Analisis de Tiempos del Sistema

## 1. Arquitectura de Temporizacion

El ATmega328P opera a **16 MHz** y utiliza tres timers independientes:

| Timer  | Prescaler | Modo        | Frecuencia     | Periodo     | Funcion                           |
|--------|-----------|-------------|----------------|-------------|-----------------------------------|
| Timer0 | 256       | CTC (OCR0A) | 500 Hz         | **2 ms**    | Tick del sistema (FSMs, Classifier)|
| Timer1 | 8         | Free-running| 2 MHz          | **0.5 us**  | Base de tiempo para HC-SR04       |
| Timer2 | 64        | CTC (OCR2A) | 2000 Hz        | **0.5 ms**  | Generacion PWM servos (50 Hz)     |

---

## 2. Medicion Ultrasonica (HC-SR04)

### 2.1. Secuencia temporal

| Fase           | Duracion         | Descripcion                                   |
|----------------|------------------|-----------------------------------------------|
| Trigger        | >= 10 us         | Pulso en PB1 para iniciar medicion            |
| Espera echo    | Variable         | Tiempo hasta que el sensor responde           |
| Pulso echo     | 116 - 23200 us   | Proporcional a la distancia (2 cm - 400 cm)   |
| Timeout        | 38000 us (38 ms) | Limite maximo si no hay respuesta             |

### 2.2. Calculo de distancia

```
distancia_cm = tiempo_echo_us / 58
```

Donde 58 us/cm proviene de: velocidad del sonido = 343 m/s -> 29.15 us/cm (ida) x 2 (ida y vuelta) = 58.3 us/cm.

### 2.3. Rango de medicion para cajas del sistema

| Tipo de caja | Altura | Distancia sensor-caja | Tiempo echo esperado |
|--------------|--------|-----------------------|----------------------|
| Grande       | 10 cm  | 10 cm                 | 580 us               |
| Mediana      | 8 cm   | 12 cm                 | 696 us               |
| Pequena      | 6 cm   | 14 cm                 | 812 us               |
| Sin caja     | -      | 20 cm (base)          | 1160 us              |

**Nota:** El sensor esta montado a 20 cm sobre la cinta. La distancia medida es 20 - altura_caja.

### 2.4. Resolucion del Timer1

- Frecuencia de Timer1: 16 MHz / 8 = **2 MHz**
- Resolucion: **0.5 us** por tick de TCNT1
- Resolucion en distancia: 0.5 us / 58 = **0.0086 cm** (despreciable)
- Error sistematico real: dominado por la variacion de la cinta (~1 cm)

### 2.5. Umbrales de clasificacion

Configurados para tolerar la irregularidad de la cinta transportadora:

| Condicion                  | Umbral | Clasificacion     |
|----------------------------|--------|--------------------|
| distancia < 12 cm         | 12     | Grande (10 cm)     |
| 12 <= distancia < 14 cm   | 14     | Mediana (8 cm)     |
| 14 <= distancia < 17 cm   | 17     | Pequena (6 cm)     |
| distancia >= 17 cm        | -      | No clasificable    |

---

## 3. Tiempos de Respuesta de los Servomotores SG90

### 3.1. Generacion PWM

| Parametro        | Valor           | Descripcion                        |
|------------------|-----------------|------------------------------------|
| Periodo PWM      | 20 ms (50 Hz)   | Estandar para servos RC            |
| Resolucion       | 0.5 ms          | 1 tick de Timer2                   |
| Pulso reposo     | 2.0 ms (4 ticks)| Posicion vertical (aspa retraida)  |
| Pulso patear     | 1.0 ms (2 ticks)| Posicion extendida (empuja caja)   |

### 3.2. Ciclo completo del actuador

| Fase          | Duracion  | Ticks (2ms) | Estado del servo         |
|---------------|-----------|-------------|--------------------------|
| Waiting       | Variable  | Configurable| Reposo (esperando caja)  |
| Extending     | 150 ms    | 75 ticks    | Pateando (pulso 1.0 ms)  |
| Retracting    | 150 ms    | 75 ticks    | Volviendo (pulso 2.0 ms) |
| **Total activo** | **300 ms** | **150 ticks** | Extension + retraccion |

### 3.3. Velocidad del SG90

- Velocidad tipica del SG90: **0.1 s / 60 grados** (a 4.8V)
- Rango de movimiento utilizado: ~90 grados (de 1.0 ms a 2.0 ms)
- Tiempo mecanico estimado: **0.15 s = 150 ms** (coincide con ACT_PHASE_TICKS)

---

## 4. Tiempos de Arribo (Open-Loop)

### 4.1. Configuracion por defecto

| Servo   | Sensor destino | Tiempo de arribo | Ticks (2 ms)  |
|---------|----------------|------------------|---------------|
| Servo 1 | S1 (PD2)      | **2.0 s**        | 1000 ticks    |
| Servo 2 | S2 (PD3)      | **4.0 s**        | 2000 ticks    |
| Servo 3 | S3 (PD4)      | **6.0 s**        | 3000 ticks    |

### 4.2. Precision del timer de arribo

- Base de tiempo: Timer0 a 2 ms
- Error acumulado por tick: 16 MHz / 256 / 125 = exactamente 500 Hz -> **0% error**
- Resolucion: 2 ms (para un tiempo de 2 s, la resolucion es 0.1%)

### 4.3. Modo calibracion

El modo calibracion permite medir los tiempos reales S0 -> S1/S2/S3 para ajustar los valores. El contador `calibTickCounter` se incrementa cada 2 ms y se reporta como EVT_CALIB_TIME al detectar el flanco de cada sensor IR.

---

## 5. Comunicacion Serie (UART)

| Parametro        | Valor                          |
|------------------|--------------------------------|
| Baud rate        | 115200 bps                     |
| Formato          | 8N1 (8 datos, sin paridad, 1 stop) |
| U2X              | Habilitado (doble velocidad)   |
| UBRR             | 16                             |
| Error de baud    | +2.1%                          |
| Tiempo por byte  | 86.8 us (10 bits / 115200)     |

### 5.1. Tiempo de transmision de tramas UNER

| Tipo de trama            | Bytes | Tiempo de TX   |
|--------------------------|-------|----------------|
| Minima (solo CMDID)      | 8     | 694 us         |
| EVT_NEW_BOX (4 datos)    | 12    | 1.04 ms        |
| EVT_DISTANCE (2 datos)   | 10    | 868 us         |
| Maxima (32 payload)       | 40    | 3.47 ms        |

### 5.2. Heartbeat

- Periodo: **500 ms** (250 ticks del Timer0)
- Direccion: MCU -> Qt (EVT_ALIVE) periodico + Qt -> MCU (CMD_ALIVE) periodico

---

## 6. Debounce del Sensor S0

| Parametro            | Valor          |
|----------------------|----------------|
| S0_DEBOUNCE_TICKS    | 250 ticks      |
| Tiempo de debounce   | **500 ms**     |
| Frecuencia maxima de deteccion | 2 cajas/s |

Esto evita que el rebote mecanico del sensor S0 genere multiples mediciones para una misma caja.

---

## 7. Resumen de Tiempos Criticos

| Evento                          | Tiempo          | Fuente        |
|---------------------------------|-----------------|---------------|
| Tick del sistema                | 2 ms            | Timer0        |
| Medicion HC-SR04 (caso tipico)  | ~1 ms           | Timer1        |
| Medicion HC-SR04 (timeout)      | 38 ms           | Timer1        |
| Extension del servo             | 150 ms          | Timer0        |
| Retraccion del servo            | 150 ms          | Timer0        |
| Debounce S0                     | 500 ms          | Timer0        |
| Heartbeat                       | 500 ms          | Timer0        |
| Arribo Servo 1 (defecto)        | 2000 ms         | Classifier    |
| Arribo Servo 2 (defecto)        | 4000 ms         | Classifier    |
| Arribo Servo 3 (defecto)        | 6000 ms         | Classifier    |
| Trama UART tipica               | ~1 ms           | UART 115200   |
