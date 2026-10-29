#ifndef SERVO_PARAMS_H
#define SERVO_PARAMS_H

#include <cstdint>

// ============================================================
//  Parámetros de los servos RDS51150 (270 grados)
//  VALORES NOMINALES - Pendientes de calibrar
//
//  Rango PWM: 500-2500 us para 0-270 grados
//  Frecuencia: 50 Hz
// ============================================================

#define PWM_FREQ_HZ      50

// ============================================================
//  Valores nominales del servo RDS51150
//  (CALIBRAR DESPUÉS con el proyecto de calibración)
// ============================================================
#define US_PER_DEG       7.4074f      // us por grado (nominal)
#define US0              500.0f       // us en 0 grados (nominal)
#define US_MIN           500          // ancho de pulso mínimo [us]
#define US_MAX           2500         // ancho de pulso máximo [us]

// ============================================================
//  Estructura de parámetros por servo
// ============================================================
struct ServoParams {
    uint8_t pin;        // GPIO de la ESP32 que genera el PWM
    float angleHome;    // Ángulo de reposo (grados)
    float angleMin;     // Límite inferior seguro (grados)
    float angleMax;     // Límite superior seguro (grados)
    const char* name;   // Nombre descriptivo
};

#define NUM_SERVOS 6

// ============================================================
//  Orden del mensaje /servo_commands (6 valores, en grados):
//
//    data[0..2] -> pierna IZQUIERDA: Hip Roll, Hip Pitch, Knee
//    data[3..5] -> pierna DERECHA:   Hip Roll, Hip Pitch, Knee
//
//  Un valor NaN significa "no cambiar este servo": el firmware
//  mantiene su última posición objetivo.
//
//  Cableado (GPIO de la ESP32 directo a la señal de cada servo):
//    S1 -> GPIO 18   IZQ Hip Roll   (cad roll)
//    S2 -> GPIO 19   IZQ Knee       (rod pitch)
//    S3 -> GPIO 23   IZQ Hip Pitch  (cad pitch)
//    S4 -> GPIO 25   DER Knee       (rod pitch)
//    S5 -> GPIO 26   DER Hip Roll   (cad roll)
//    S6 -> GPIO 27   DER Hip Pitch  (cad pitch)
//
//  Límites y reposo: los mismos para ambas piernas (copiados del
//  esquema anterior con PCA9685). Pendientes de calibrar.
// ============================================================

static const ServoParams SERVO_PARAMS[NUM_SERVOS] = {
    // pin, angleHome, angleMin, angleMax, name
    { 18, 180.0f, 165.0f, 270.0f, "IZQ Hip Roll"  },
    { 23, 135.0f,  45.0f, 225.0f, "IZQ Hip Pitch" },
    { 19, 135.0f,  45.0f, 225.0f, "IZQ Knee"      },
    { 26, 180.0f, 165.0f, 270.0f, "DER Hip Roll"  },
    { 27, 135.0f,  45.0f, 225.0f, "DER Hip Pitch" },
    { 25, 135.0f,  45.0f, 225.0f, "DER Knee"      },
};

#endif
