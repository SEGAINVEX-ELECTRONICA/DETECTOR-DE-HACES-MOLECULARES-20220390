/**
 * UNIVERSIDAD AUTÓNOMA DE MADRID
 *  Proyecto: 20220390
 *  Unidad Electronica para cañón de Electrones Detector
 *  de  Haces Moléculares: Anodo
 *
 *  Autor: Patricio Coronado Collado
 *  Versión: V.1 marzo 2024
 *  Aplicación para SAMD21 ecosistema Arduino Seeeduino XIAO.
 *  Descripción: Don un rotary encoder, un display y un DAC
 *  MCP4525 se controla la tensión de salida de una fuente de
 *  alta tensión F02CT a través de un módulo
 * I3A4W005A150V-001-R G01. Se actualiza el set point con
 *  el rotary encoder y se lee la tensión Y CORRIENTE de salida
 *  con el ADC del SAM D21 que monta el SEEEDUINO XIAO
 *
 *  Ultima actualizacion 12-07-2024
 */
#include <Arduino.h>
#include <U8g2lib.h>   //Display
#include <Wire.h>      //I2C para el display y el DAC
#include <TimerTCC0.h> //Timer para periodo de muestreo
#include "WDTZero.h"   //Watchdog
// #include <FlashStorage.h> //Para emular EEPROM
#include "SegaSCPI.h"   //Para depurar con SCPI
#include "scpi_anodo.h" //Funciones scpi locales
#include "ZeroConfigureADC.h"
/*************************************************************
  Macros
**************************************************************/
#define DEP(x)      \
    if (depuracion) \
    Serial.println(x)
/**************************************************************
 * Pines
 * ************************************************************/
#define CLK 2    // clk del rotary encoder
#define DT 3     // DT del rotary encoder
#define BUZZER 6 //
#define PIN_TEST 0
#define ADC_I 8 // Entradas analógicas
#define ADC_V 9
#define HV_ON_OFF 10 // Activa y desactiva la fuente de HV
#define SWITCH_ROTARY 7
/**************************************************************
 * Constantes y variables globales
 **************************************************************/
#define WATCHDOG // para que compile la parte del watchdog
// Direcciones de los DACs MCP4725
#define ADD_DAC 0x60 // dirección i2c del DAC A0=0
// Constantes de buzzer
#define FREQ_LIM 500  // hz frecuencia
#define FREQ_PRE 1000 // hz frecuencia
#define MS_LIM 30     // mseg tiempo de duración
#define MS_PRE 10     // mseg tiempo de duración
// Constantes para el convertidor ADC
#define TS_20ms 20000  // Periodo de muestreo us
#define M_V 0.054786456 // pendiente y offset de la recta (D,Vo)
#define OFFSET_V -1.5713
// TO DO calcular la recta de calibración de corriente
//#define M_I 0.0067851459 // pendiente y offset de la recta (D,Vo)
//#define OFFSET_I 0.005458886
#define M_I 0.006501826 // pendiente y offset de la recta (D,Vo)
#define OFFSET_I 0.055092853
#define N_MUESTRAS 16 // Número de muestras del ADC a promediar
uint16_t adcV = 0;
uint16_t _adcV = 0; // Para reservar el valor digital del ADC entre loops
uint16_t adcI = 0;
uint16_t _adcI = 0;   // Para reservar el valor digital del ADC entre loops
bool leerADC = false; // Flag para indicar que hay que leer el ADC
uint16_t lecturaTension = 0;
int8_t muestrasLeidas = 0; // Contador de muestras acumuladas
float corriente = 0;       // Valores reales de tensión y corriente
float tension = 0;
uint16_t adcCorriente; //Para depuración por SCPI
uint16_t adcTension;
// Display
#define DISPLAY_FONT_2_LINEAS display.setFont(u8x8_font_courB18_2x3_f) // Font para solo 2 lineas LINEA1 y LINEA5
#define LINEA1 1
#define LINEA2 5
// Constantes y variables del rotary encoder
#define LIM_MAX 3750 // Valores límite del rotary encoder. Máximo valor del ADC con 200V a la salida (empirico)
#define LIM_MIN 250
#define DELTA 10          // Incremento del encoder en cada paso
int16_t rotary = LIM_MIN; // Estado del rotary encoder
bool intEncoder = true;
bool depuracion = false;
bool depuracionRegulador = false;
bool clkRotary = true; // Valor del pin clk del rotary
bool dtRotary = true;  // Valor del pin dt del rotary
bool rotaryLimite = false;
bool intSwitch = false; // Valor del pin sw del rotary
bool hvOff = true;      // Flag de estado on/off de la fuente de HV. Por defecto off
// Para mostar en el display
char msg[32];
// para temporizar en el loop
unsigned long antes = 0;
// Parámetros del regulador
//  Variables del regulador
int16_t ek_1 = 0;  // error anterior, con signo
uint16_t rk_1 = 0; // salida anterior
int16_t ek = 0;    // Error actual, con signo
uint16_t rk = 0;   // Salida actual
//
/**************************************************************
 *  Objetos
 **************************************************************/
extern SegaSCPI segaScpi;                                // Comunicación con el PC
U8X8_SH1106_128X64_NONAME_HW_I2C display(U8X8_PIN_NONE); // Display
#ifdef WATCHDOG
WDTZero WatchDog; // Objeto Watchdog
#endif
/**************************************************************
 * Prototipos de funciones
 ***************************************************************/
inline void display_presentacion(void);      // Presentación inicial UAM / SEGAINVEX
void int_timerTS(void);                // Callback del timer para leer muestras del ADC
inline bool cambio_encoder(int16_t *); // Rutina para actualizar el estado del rotary activo
void int_encoder(void);                // Callback del rotary
inline void actualiza_DAC(uint16_t, uint8_t, bool);
inline void beep(uint32_t, uint32_t);
void int_switch(void);
inline void fuente_hv_on_off(void);
inline bool calcula_tension_real(void);   // Pasa de digital a tensión real
inline bool calcula_corriente_real(void); // Pasa de digital a corriente real
inline bool ajusta_cadena(char, char *, uint8_t);
inline void calcula_y_muestra_variables(void);
inline bool cambio_en_encoder(int16_t *);
inline void lee_encoder(void);
inline void pin_on_off(bool);
inline void regulador(uint16_t, uint16_t);
/**************************************************************
 * setup
 ***************************************************************/
void setup()
{
    //  Pines
    pinMode(HV_ON_OFF, OUTPUT);
    pin_on_off(false); // desactiva la fuente de HV
    pinMode(CLK, INPUT);
    pinMode(DT, INPUT);
    pinMode(SWITCH_ROTARY, INPUT_PULLUP);
    pinMode(LED_BUILTIN, OUTPUT);
    digitalWrite(LED_BUILTIN, LOW);
    pinMode(PIN_LED_RXL, OUTPUT);
    digitalWrite(PIN_LED_RXL, LOW);
    pinMode(PIN_LED_TXL, OUTPUT);
    digitalWrite(PIN_LED_TXL, LOW);
    pinMode(PIN_TEST, OUTPUT);
    digitalWrite(PIN_TEST, HIGH);
    // Configuración ADC // https://blog.thea.codes/getting-the-most-out-of-the-samd21-adc/
    while (ADC->STATUS.bit.SYNCBUSY == 1)
        ;                                                   // Sincroniza
    ADC->INPUTCTRL.bit.GAIN = ADC_INPUTCTRL_GAIN_1X_Val;    // Ganancia 1
    ADC->REFCTRL.bit.REFSEL = ADC_REFCTRL_REFSEL_AREFB_Val; // Pin de referencia A1
    while (ADC->STATUS.bit.SYNCBUSY == 1)
        ;                   // Sincroniza
    configure_ADC(3, 7, 10); // CLK_ADC 1.5MHz, muestras a promediaren cada lectura =64,tiempo de muestreo 2us
    // Serial
    Serial.begin(115200);     // Serial
    analogReadResolution(12); // La resolución de los adcs a 12 bits
    // Display
    display.begin();                     // Display. Implica wire.begin()
    Wire.setTimeout(5 /*milisegundos*/); // Tieme out del I2C, para salir de cuelgues por el i2c
    display.setBusClock(400000);
    display_presentacion(); // Muestra la presentación inicial
    // Timer e interrupciones
    attachInterrupt(CLK, int_encoder, FALLING);          // Interrupción del encoder
    attachInterrupt(SWITCH_ROTARY, int_switch, FALLING); // Interrupción del swuitch del rotary encoder
// Watchdog
//  WDT_HARDCYCLE62m  0x0430    // WDT HARD only : 64 clockcycles @ 1024hz = 62.5ms
//  WDT_HARDCYCLE250m 0x0450    // WDT HARD only : 256 clockcycles @ 1024hz = 250ms
//  WDT_HARDCYCLE1S   0x0470    // WDT HARD only : 1024 clockcycles @ 1024hz = 1 seg
//  WDT_HARDCYCLE2S   0x0480    // WDT HARD only : 2048 clockcycles @ 1024hz = 2 seconds
//  WDT_HARDCYCLE4S   0x0490    // WDT HARD cycle 4 Seconds
#ifdef WATCHDOG
    WatchDog.setup(WDT_HARDCYCLE1S); // Activa el watchdog WDT_SOFTCYCLE8S
#endif
    TimerTcc0.initialize(TS_20ms /*0,2 seg*/); // preferiblemente 20ms
    TimerTcc0.attachInterrupt(int_timerTS);    // Interrupción del timer para periodo de muesteo
    // actualiza_DAC(rotary, ADD_DAC,1);   // El DAC y eeprom a cero
    fuente_hv_on_off(); // Enciende la fuente de HV
}
/**************************************************************
 * loop
 ***************************************************************/
void loop()
{
// Refesca el watchdog ..............................................
#ifdef WATCHDOG
    WatchDog.clear();
#endif
    // Atiende al PC....................................................
    if (Serial.available())
    {
        segaScpi.scpi(&Serial);
    }
    // Gestiona la interrupción del rotary encoder.......................
    if (intEncoder)
        lee_encoder();
    // Promedia y muestas las lectuas del ADC............................
    if (muestrasLeidas >= N_MUESTRAS)
    {
        calcula_y_muestra_variables();
    }
    // Transcurrido un periodo de muestreo Ts lee el ADC.......................
    if (leerADC)
    {
        lecturaTension = analogRead(ADC_V); // Lee ADCs cada Ts (idealmente 20ms)
        adcV += lecturaTension;             // Acumula muestras de tensión hasta N_MUESTRAS
        regulador(rotary, lecturaTension);  // Ejecuta el regulador para que funcione en lazo cerrado
        adcI += analogRead(ADC_I);         // Acumula muestras de corriente hasta N_MUESTRAS
        muestrasLeidas++;                  // Contador de muestas. Muestra 1,2,3...N_MUESTRAS
        leerADC = false;                   // Baja el flag que indica que hay que leer muestras
    }
    // Servicio a la interrupción del switch del rotary.....................
    if (intSwitch) // Si se pulsa el wsitch del rotary se apaga/enciende la fuente de alta tensión (HV)
    {
        // Antirrebote. Lee el pin y si sigue a 0 ejecuta el on off de la fuente.
        if (!digitalRead(SWITCH_ROTARY))
            fuente_hv_on_off();
        intSwitch = false;
    }
// Código temporizado a LAPSO milisegundos...............................
// Para temporizar en el loop
#define LAPSO 1000 // 1 segundos de temporzación en el loop
    ulong ms = millis();
    if (ms >= antes + LAPSO)
    {
        // TODO código temporizado aquí
        //.................................................
        if(depuracionRegulador)
        {
            Serial.println("rotary=" + String(rotary));
            Serial.println("ADCV=" + String(lecturaTension));
            Serial.println("ek=" + String(ek));
            Serial.println("ek_1=" + String(ek_1));
            Serial.println("rk=" + String(rk));
            Serial.println("rk_1=" + String(rk_1));
            Serial.println("................\r\n");
        }
        //.................................................
        antes = millis();
    }
}
/*************************************************************
 *  Función que ejecuta el regulador
 * Los parámetros de entrada son la lectura del ADC fk (feedback)
 * y el valor del rotary encoder, la consigna uk
 *************************************************************/
inline void regulador(uint16_t uk, uint16_t fk)
{
    digitalWrite(PIN_TEST, LOW);
    if (hvOff)
        return;   // Si la fuente está desactivada no ejecuta el regulador
    ek = uk - fk; // Calcula el error con signo, consigna menos feedback
    // Limita el error
    if (ek > 500)
        ek = 500;
    if (ek < -500)
        ek = -500;
    // parámetros del regulador
    #define b1 0.7089
    #define b0 1.1417
    rk = rk_1 + ek * b0 - ek_1 * b1;
    // Limita el valor de rk a su rango de 12 bits sin signo
    if (rk > 4095)
        rk = 4095;
    if (rk <= LIM_MIN)
        rk = LIM_MIN;
    //  Salida al DAC
    if (rk != rk_1)
        actualiza_DAC(rk, ADD_DAC, 0); // Si la salida del regulador ha cambiado actualiza el DAC
    rk_1 = rk;                         // Actualiza la salida anterior con el valor de la actual para la próxima iteración
    ek_1 = ek;                         // Actualiza el error anterior con el valor del error actual para la próxima iteración
    digitalWrite(PIN_TEST, HIGH);
}
/*************************************************************
 * Calcula la media de las N_MUESTRAS leidas, pasa los valores
 * a real y muestra en el display
 * ***********************************************************/
inline void calcula_y_muestra_variables(void)
{
    TimerTcc0.stop();                                     // Detiene el timer de muestreo
    digitalWrite(PIN_LED_RXL, !digitalRead(PIN_LED_RXL)); // Led para ver la latencia
    if (calcula_tension_real() & !hvOff) //Si la fuente está en off no muestra la tensión
    {                                                 // Solo calcula y refresca el display si el valor de tensión ha cambiado
        adcTension=tension; //Para depuración TODO borrar
        snprintf(msg, sizeof msg, "%3.1fV", tension); //
        ajusta_cadena('V', msg, 8);                   // Formatea el array para que no se vean caracteres dobles
        if (tension > 200.0f)
        {
            display.inverse();
            display.drawString(0, LINEA1, msg); // Muestra en el display
            display.noInverse();
        }
        else
            display.drawString(0, LINEA1, msg); // Muestra en el display
        DEP("ADC_V= " + String(adcV));
        DEP("V= " + String(tension));
    }
    if (calcula_corriente_real())
    {   adcCorriente=corriente; //Para depuración TODO borrar                                                 // Solo calcula y refresca el display si el valor de corriente ha cambiado
        snprintf(msg, sizeof msg, "%2.1fmA", corriente); // Pone en msg la linea a mostar
        ajusta_cadena('A', msg, 8);                      // Formatea el array para que no se vean caracteres dobles
        if (corriente > 15.0f)
        {
            display.inverse();
            display.drawString(0, LINEA2, msg); // Muestra en el display
            display.noInverse();
        }
        else
            display.drawString(0, LINEA2, msg); // Muestra en el display
    }
    // Resetea variables para empezar un nuevo ciclo de capturas
    adcV = 0;           // Resetea el acumulador de muetras de tensión
    adcI = 0;           // Resetea el acumulador de muetras de corriente
    muestrasLeidas = 0; // Pone a cero el contdor de muestras leidas
    TimerTcc0.start();  // Arranca el timer de muestreo
}
/*************************************************************
  Funciones del display
**************************************************************/
/*************************************************************
  Cartel inicial
**************************************************************/
inline void display_presentacion(void) // Cartel de presentación inicial
{
    display.clear();
    // Cambia la resolución
    display.setFont(u8x8_font_courB18_2x3_f); // Display con un font más grande
    display.inverse();
    display.drawString(0, LINEA1, " ANODO  "); // Cartel de la fila superior
    display.noInverse();
    delay(2000); // Presentación inicial del display
    display.clear();
}
/**************************************************************
 *      FUNCIONES DEL ROTARY ENCODER
 *  callback de la interrupción "int_encoder"
 ***************************************************************/
/**************************************************************
 * Si ha habido un cambio en el encoder el flag "intEncoder"
 * está lebantado y hay que ejecutar esta función. actualiza el DAC
 * con el nuevo valor. Hace un "tono en el buzzer distinguiendo
 * si ha habido un cambio en el rotary o ha alcanzado un límite.
 *  baje el flag "intEncoder" y habilita interrupciones de pines
 * ************************************************************/
inline void lee_encoder(void)
{
    digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN)); // Led para ver la latencia
    if (cambio_en_encoder(&rotary))
    {
        // actualiza_DAC(rotary, ADD_DAC, 0); // Actualiza el DAC, absoluto
        // Se actualiza en el regulador
    }
    if (rotaryLimite) // Si el encoder está tocando límites emite un beep más agudo
    {
        beep(FREQ_LIM, MS_LIM);
        rotaryLimite = false;
    }
    else // Si el encoder no está tocando límites emite un beep más grave
    {
        beep(FREQ_PRE, MS_PRE); // Si no ha llegado a límite emite un beep de pulso de rotary encoder bueno PRE
    }
    intEncoder = false;       // Baja el flag de la interrupción del encoder
    NVIC_EnableIRQ(EIC_IRQn); // Habilita la interrupción de los pines para que funcione el encoder
    DEP("rotary=" + String(rotary));
}
/**************************************************************
   Función que actualiza el valor del rotary encoder y devuelve
   true si este ha cambiado, Aplica los valores límites.
   Como cerca de 0 las fuentes de HV no son lineales se evita
   tramos de valores en torno a 0
**************************************************************/
inline bool cambio_en_encoder(int16_t *rotary)
{
    if (clkRotary)
        return false;          // Si CLK no está a nivel bajo sale sin hacer nada
    int16_t _rotary = *rotary; // guarda el valor de entrada
    if (!dtRotary)             // si DT es 0 decrementa
    {
        *rotary -= DELTA;
        if (*rotary < LIM_MIN) // Ajusta al límite inferior
        {
            *rotary = LIM_MIN;
            rotaryLimite = true;
        }
    }
    else // if(dtRotary) Si DT es 1 incrementa
    {
        *rotary += DELTA;
        if (*rotary > LIM_MAX) // Ajusta al límite superior
        {
            *rotary = LIM_MAX;
            rotaryLimite = true;
        }
    }
    return _rotary != *rotary; // Si el valor de entrada es diferente de el de salida sale con true
}
/**************************************************************
 * Rutina de la interrupción del rotary encoder
 * Levanta el flag "intEncoder" que se lee en el loop
 * Lee DT y CLK del rotary encoder
 * Desactiva la interrupción de pines (no es imprescindible)
 * ***********************************************************/
void int_encoder(void)
{
    intEncoder = true;            // Flag qu deja pendiente la atención a la interrupción en el loop
    NVIC_DisableIRQ(EIC_IRQn);    // Inhabilita esta interrupción que se habilitará cuando se limpie el flag intEncoder
    clkRotary = digitalRead(CLK); // lee el pin CK del rotary
    dtRotary = digitalRead(DT);   // Lee el pin DP para detectar la dirección
}
/**************************************************************
 * Callback de la interrupción del temporizador del periodo de
 * muestreo
 * ***********************************************************/
void int_timerTS(void) { leerADC = true; } // Levanta el flag para que se lean muestras en el loop
/**************************************************************
   Actualiza el valor del DAC MCP4725 12 bits por I2C
   ver datasheet para detalles
***************************************************************/
inline void actualiza_DAC(uint16_t dato, uint8_t direccion, bool eprom)
{
    // Dirección del MCP4725 de Adafruit A0=GND 0X62
    Wire.beginTransmission(direccion); // dirección del MCP4725 de Adafruit A0=GND
    if (eprom)
        Wire.write(0x60); // Escribe el dato en el DAC y en EPROM
    else
        Wire.write(0x40);         // Escribe el dato en el DAC
    Wire.write(dato / 16);        // 8 bites altos (D11.D10.D9.D8.D7.D6.D5.D4)
    Wire.write((dato % 16) << 4); // 4 bites bajos (D3.D2.D1.D0)
    Wire.endTransmission();
}
/**************************************************************
 *  entrega una señal cuadrada por el pin BUZZ de con
 *  frecuencia FREQ hz y duración MS milisegundos, para
 *  un buzzer pasivo
 ***************************************************************/
inline void beep(uint32_t frecuencia, uint32_t milisegundos)
{
    tone(BUZZER, frecuencia, milisegundos);
}
/*************************************************************
 * Funciones para atender al switch del rotary encoder
 * callback de la interrupción "int_switch" que levanta
 * el flag "intSwitch" y función que ejecuta el "loop"
 * cuando ve el flag levantado "fuente_hv_on_off"
 **************************************************************/
/**************************************************************
 * callback de la interrupción del switch del ratary encoder.
   Sube el flag para que se de servicio a la interrupción
 * ***********************************************************/
void int_switch(void) { intSwitch = true; }
/**************************************************************
 * función que da servicio a la interrupción de switch del
 * rotary encoder. Apaga enciende la fuente de HV
 * ***********************************************************/
inline void fuente_hv_on_off(void)
{
    if (hvOff) // Si la fuente está en off la pone en on
    {
        rotary = LIM_MIN;                   // Pone a mínimo el rotary
        ek_1 = 0;                           // error anterior, con signo
        rk_1 = 0;                           // salida anterior
        ek = 0;                             // Error actual, con signo
        rk = 0;                             // Salida actual
        actualiza_DAC(LIM_MIN, ADD_DAC, 0); ////Salida del DAC al mínimo
        pin_on_off(true);
        hvOff = false; // Flag que informa de que la fuente está activada
    }
    else // Si la fuente etá en on la pone en off
    {
        rotary = LIM_MIN;                   // Pone a mínimo el rotary
        actualiza_DAC(LIM_MIN, ADD_DAC, 0); // Salida del DAC al mínimo
        pin_on_off(false);
        display.inverse();
        display.drawString(0, LINEA1, "  OFF   "); // Cartel de la fila superior
        display.noInverse();
        hvOff = true; // Flag que informa de que la fuente está apagada
    }
}
/*****************************************************************
 * Actua sobre el pin on/off de la fuente HV
 ******************************************************************/
inline void pin_on_off(bool estado)
{
    digitalWrite(HV_ON_OFF, estado);
}
/***************************************************************
 * convierte el dato digital de tensión en tensión real
 ****************************************************************/
inline bool calcula_tension_real(void)
{
    adcV = adcV / N_MUESTRAS; // Promedia muetras de tensión
    if (adcV == _adcV)
    {
        return false;
    }                                       // Si el valor del ADC es diferente del anterior lo actualiza. Si no, sale
    _adcV = adcV;                           // Guarda el valor para compararlo con el nuevo valor en la próxima iteración del loop
    tension = float(adcV) * M_V + OFFSET_V; // Curva de calibración para pasar de binario a float
    return true;
}
/***************************************************************
 * convierte el dato digital de corriente en corriente real
 ****************************************************************/
inline bool calcula_corriente_real(void)
{
    adcI = adcI / N_MUESTRAS; // Promedia muestras de corriente
    DEP("ADC_I= " + String(adcI));
    if (adcI == _adcI)
        return false;                         // Si el valor del ADC es diferente del anterior lo actualiza. Si no, sale
    _adcI = adcI;                             // Guarda el valor para compararlo con el nuevo valor en la próxima iteración del loop
    corriente = float(adcI * M_I + OFFSET_I); // Curva de calibración para pasar de binario a float
    return true;
}
/**************************************************************
 * recorre "cadena"  buscando el caracter car, lo deja y
 * borra todo lo demás hasta finCadena
 * ***********************************************************/
inline bool ajusta_cadena(char car, char *cadena, uint8_t finCadena)
{
    uint8_t i;
    for (i = 0; cadena[i] != car; i++) // Busca "car" en la cadena
    {
        if (i >= finCadena) // Si no encuetra el caracter no hace nada...
            return false;   //...sale con false
    }
    // Si encuentra el caracter...
    for (i += 1; i < finCadena; i++) // ...apunta al siguiente caracter,...
    {
        cadena[i] = ' '; // ...borra todo desde el hasta el final de la cadena...
    }
    cadena[finCadena] = '\0'; // Cierra la cadena..
    return true;              //..y sale
}
/*********************** FIN *************************************/
