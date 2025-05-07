#ifndef TIPOS_H
#define TIPOS_H
//
#include <Arduino.h>
// tipos
typedef struct struct_message
{
    short estado;
    bool  swPulsado;
    bool  limiteTension;
    float vFilamento;
    float iFilamento;
    float temperatura;
    short potTension;
    short potCorriente;
} struct_message;
//
#endif
