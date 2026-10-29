#ifndef PINS_H
#define PINS_H

#include <Arduino.h>

// ============================================================
//  Pines del ESP32 para el robot bípedo
//
//  Las señales PWM de los servos van directo a los GPIO definidos
//  en servo_params.h (campo "pin" de SERVO_PARAMS).
// ============================================================

// LED indicador
#define LED_PIN   2    // LED onboard del ESP32

#endif
