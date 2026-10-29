#ifndef HELPERS_H
#define HELPERS_H

#include "servo_params.h"

// ============================================================
//  clampValue() - Limita un valor entre min y max
// ============================================================
template <typename T>
T clampValue(T value, T minVal, T maxVal) {
    if (value < minVal) return minVal;
    if (value > maxVal) return maxVal;
    return value;
}

// ============================================================
//  angleToUs() - Convierte ángulo (grados) a microsegundos PWM
//
//  Usa los valores nominales (o calibrados cuando los tengas)
//  para el servo específico.
// ============================================================
inline float angleToUs(uint8_t servoId, float angleDeg) {
    if (servoId >= NUM_SERVOS) return 1500.0f;  // Neutro por defecto

    const ServoParams& p = SERVO_PARAMS[servoId];

    // Clamp al rango seguro del servo
    float angle = clampValue(angleDeg, p.angleMin, p.angleMax);

    // Conversión lineal: us = US0 + grados * US_PER_DEG
    return US0 + angle * US_PER_DEG;
}

#endif
