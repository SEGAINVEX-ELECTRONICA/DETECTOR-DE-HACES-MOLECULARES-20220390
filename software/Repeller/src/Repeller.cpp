/**
 * UNIVERSIDAD AUTÓNOMA DE MADRID
 *  Proyecto: 20220390
 *  Unidad Electronica para cañón de Electrones Detector
 *  de  Haces Moléculares: Relepller
 *
 *  Autor: Patricio Coronado Collado
 *  Versión: 1.0 febrero 2024
 *  Aplicación para SAMD21 ecosistema Arduino Seeeduino XIAO.
 *  Descripción:
 *
 *  Ultima actualizacion 15-10-2024
 */
#include <Arduino.h>
#include <U8g2lib.h>      //Display
#include <Wire.h>         //I2C para el display y el DAC
#include <TimerTCC0.h>    //Timer para periodo de muestreo
#include "WDTZero.h"      //Watchdog
#include <FlashStorage.h> //Para emular EEPROM
#include "SegaSCPI.h"     //Para depurar con SCPI
#include "ZeroConfigureADC.h"
/*************************************************************
  Macros
**************************************************************/
#define DEP(x) if(depuracion) Serial.println(x)
 /**************************************************************
 * Pines
 * ************************************************************/
#define CLK 2
#define DT 3
#define BUZZER 6
#define PIN_ADC 8
#define POLARIDAD 9
#define PIN_TEST 10
#define SWITCH_ENCODER // Para que compile la parte dedicada al switch del rotary encoder
#ifdef SWITCH_ENCODER  // Switch del rotary opcional
    #define SW 7
#endif
/**************************************************************
 * Constantes y variables globales
 **************************************************************/
#define WATCHDOG // para que compile la parte del watchdog
// Constantes para tone()
#define FREQ_LIM 500  // hz frecuencia
#define FREQ_PRE 1000 // hz frecuencia
#define MS_LIM 30     // mseg tiempo de duración
#define MS_PRE 10     // mseg tiempo de duración
// Direcciones de los DACs MCP4725
#define ADD_DAC 0x60  // A0=0
// Periodo de muestreo y número de muestas
#define TS_20ms 20000 // us
#define N_MUESTRAS 16
// Coeficientes para ajustar la recta de respuesta del ADC
#define PENDIENTE_P 0.0267
#define T_INDEPENDIENTE_P 0.2279
#define PENDIENTE_N 0.028628
#define T_INDEPENDIENTE_N 0.148
// Display
#define LINEA1 1
#define LINEA2 5
#define POS false
#define NEG true
#ifdef SWITCH_ENCODER    // Switch del rotary opcional
//Constantes del Rotary encoder
#define LIM_MAX 3660 // Valores límite del rotary encoder
#define LIM_MIN -3660
#define DELTA 15 // Incremento en cada pulso
#define OFFSET_P 240 //Valor positivo más pequeño
#define OFFSET_N -240 //Valor negativo más grande
    bool intSwitch = false; // Valor del pin sw del rotary
#endif
// Rotary encoder
int16_t rotary = OFFSET_P;  // Estado del rotary encoder
bool signoRepeller = 0; // Polaridad de salida de Repeller
uint16_t adc = 0;            // Valor digital del ADC
uint16_t _adc =0;//Para guardar el ADC entre iteraciones
int8_t muestrasLeidas = 0;  // Muestras acumuladas
// Flags
bool leerADC = false; // Para indicar que hay que leer el ADC
bool intEncoder = true;
bool depuracion = false;
bool ckRotary = true;
bool dtRotary = true;
bool rotaryLimite = false;
//
char msg[32]; // Para mostar en el display
unsigned long antes=0; //Para temporizar en el loop{}
/**************************************************************
 *  Objetos
 **************************************************************/
extern SegaSCPI segaScpi;   // Comunicación con el PC
U8X8_SH1106_128X64_NONAME_HW_I2C display(U8X8_PIN_NONE); // Display
#ifdef WATCHDOG
WDTZero WatchDog; // Objeto Watchdog
#endif
/**************************************************************
 * prototipos de funciones
 ***************************************************************/
inline void display_presentacion(void);// Presentación inicial 
void int_timerTS(void);                // Callback del timer para leer muestras del ADC
inline bool cambio_en_encoder(int16_t *); // Rutina para actualizar el estado del rotary activo
void int_encoder(void);                // Callback del rotary
inline void muestra_tension(float, bool);
inline void actualiza_DAC(uint16_t, uint8_t, bool);
inline void actualiza_polaridad(void);
inline void beep(uint32_t, uint32_t);
void int_switch(void);
inline void switch_encoder(void);
inline void lee_encoder(void);
inline void calcula_tension(void);
inline void lee_adc(void);
inline bool ajusta_cadena(char, char *, uint8_t);
/**************************************************************
 * setup
 ***************************************************************/
void setup()
{
    /*-------------------SOBRE INTERRUPCIONES DEL SAMD21-------------------
      https://developerhelp.microchip.com/xwiki/bin/view/products/mcu-mpu/32bit-mcu/sam/samd21-mcu-overview/samd21-processor-overview/samd21-nvic-overview/
      NVIC_SetPriority(XXX_IRQn, priority);  priority={0,1,2,3} o más alta.

        Establece la prioridad para una interrupción *
        void NVIC_SetPriority(IRQn_t IRQn, uint32_t prioridad);
        Habilitar una interrupción específica del dispositivo *
        void NVIC_EnableIRQ (IRQn_Type IRQn);
        Deshabilitar una interrupción específica del dispositivo
        void NVIC_DisableIRQ (IRQn_Type IRQn)
        void NVIC_DisableIRQ (IRQn_Type IRQn)
    ----------------------------------------------------------------------*/
    // Experimental no he visto que tenga efecto
    //NVIC_SetPriority(EIC_IRQn, 4);  // Interrupciones de pines. Mínima prioridad
    //NVIC_SetPriority(TCC0_IRQn, 2); // Prioridad media de interrupción Timer0
    //NVIC_SetPriority(WDT_IRQn, 4);  // Interrupción del watchdog baja
    // En el SEEEDUINO XIAO el wire está montado sobre  SERCOM2 SERCOM2_IRQn
    //NVIC_SetPriority(SERCOM2_IRQn, 0); // Interrupción del wire más alta
    // NVIC_DisableIRQ(SERCOM2_IRQn);
    //
    // Configuración ADC // https://blog.thea.codes/getting-the-most-out-of-the-samd21-adc/
    while (ADC->STATUS.bit.SYNCBUSY == 1);//Sincroniza
    ADC->INPUTCTRL.bit.GAIN = ADC_INPUTCTRL_GAIN_1X_Val;  //Ganancia 1
    ADC->REFCTRL.bit.REFSEL = ADC_REFCTRL_REFSEL_AREFB_Val; //Pin de referencia A1
    while (ADC->STATUS.bit.SYNCBUSY == 1); //Sincroniza
    configure_ADC(3,6,1); // CLK_ADC 1.5MHz, muestras a promediaren cada lectura =64,tiempo de muestreo 2us
    analogReadResolution(12);//La resolución de los adcs a 12 bits
    //Pines
    pinMode(CLK, INPUT_PULLUP);
    pinMode(DT, INPUT_PULLUP);
    pinMode(LED_BUILTIN, OUTPUT);
    pinMode(PIN_LED_RXL, OUTPUT);
    pinMode(PIN_LED_TXL, OUTPUT);
    pinMode(POLARIDAD, OUTPUT);
    pinMode(PIN_TEST, OUTPUT);
    digitalWrite(PIN_TEST, HIGH);
#ifdef SWITCH_ENCODER // Switch del rotary opcional
    pinMode(SW, INPUT_PULLUP);
#endif
    // Display
    display.begin(); // Display
    display.setBusClock(400000);
    display_presentacion(); // Muestra la presentación inicial
    // Recursos
    Serial.begin(115200);                // Serial
    Wire.setTimeout(5 /*milisegundos*/); // Tieme out del I2C, para salir de cuelgues por el i2c
    attachInterrupt(CLK, int_encoder, FALLING);
    TimerTcc0.initialize(TS_20ms /*0,02 seg*/); // Periodo de muestreo preferiblemente 20ms
    TimerTcc0.attachInterrupt(int_timerTS);
#ifdef SWITCH_ENCODER                          // Switch del rotary opcional
    attachInterrupt(SW, int_switch, FALLING); // Interrupción del swuitch del rotary encoder
#endif
// Más Comandos del timer TimerTcc0.start();TimerTcc0.restart();TimerTcc0.stop();
#ifdef WATCHDOG
    // WDT_HARDCYCLE62m  0x0430    // WDT HARD only : 64 clockcycles @ 1024hz = 62.5ms
    // WDT_HARDCYCLE250m 0x0450    // WDT HARD only : 256 clockcycles @ 1024hz = 250ms
    // WDT_HARDCYCLE1S   0x0470    // WDT HARD only : 1024 clockcycles @ 1024hz = 1 seg
    // WDT_HARDCYCLE2S   0x0480    // WDT HARD only : 2048 clockcycles @ 1024hz = 2 seconds
    // WDT_HARDCYCLE4S   0x0490    // WDT HARD cycle 4 Seconds
    WatchDog.setup(WDT_HARDCYCLE1S); // Activa el watchdog WDT_SOFTCYCLE8S
#endif
    switch_encoder();//Inicializa el rotary encoder a su estado por defecto
    muestra_tension(0.0, false); // Pantalla inicial del encoder
}
/**************************************************************
 * loop
 ***************************************************************/
void loop()
{
#ifdef WATCHDOG
    WatchDog.clear(); // Refesca el watchdog
#endif
//Gestión de la interrupción del dato del rotary encoder    
    if (intEncoder)
    {
        lee_encoder();// Rutina de atención al rotary encoder
    }
 // Atiende al PC
    if (Serial.available())
    {
        segaScpi.scpi(&Serial);
    }
 // Switch del rotary. Pone a cero la salida
#ifdef SWITCH_ENCODER
    if (intSwitch)
    {
        switch_encoder(); //Pone el rotary en coder en estado por defecto
    } 
#endif
 // Promedia y muestas las lectuas del ADC
    if (muestrasLeidas >= N_MUESTRAS)
    {
        calcula_tension();
    }
 // Si a transcurrido un TS (periodo de muestreo) lee el ADC
    if (leerADC)
    {
        lee_adc();
    }

  // Loop temporizado--------------------------------------
  if (millis() - antes > 200) //1/3 seg
  {
    //snprintf(msg, sizeof msg, "%d :) ", rotary); //
    //NVIC_DisableIRQ(EIC_IRQn); //Inhabilita al encoder para lanzar una interrupción
    //display.drawString(0, LINEA2, msg); //
    //NVIC_EnableIRQ(EIC_IRQn); //Inhabilita al encoder para lanzar una interrupción
 // Fin loop temporizado--------------------------------------
    antes = millis();
  }
}
/**************************************************************
 * Lee el adc e incrementa el contador de muestras
 ***************************************************************/
inline void lee_adc(void)
{
    //digitalWrite(PIN_TEST,LOW);
    adc = adc + analogRead(PIN_ADC);// Acumula lecturas del ADC menos offset
    //digitalWrite(PIN_TEST,HIGH);
    muestrasLeidas++; //Resetea el contador de muestras y el flag de la interrupción
    leerADC = false;
}
/*************************************************************
     Rutina que responde cuando se acumulan las muestras a 
     promediar
**************************************************************/
inline void calcula_tension(void)
{
        TimerTcc0.stop(); // Detiene el timer de muestreo
        float adcFloat;
        adc = (adc >> 4); //Calcula la media (divide entre 16) y le resta el offset
        if (adc != _adc) // Si el valor del ADC es diferente del anterior lo actualiza
        {
            _adc = adc;//Guarda una copia para la siguiente iteración
            //Calcula la tensión con los coeficientes correspondientes según el signo
            if(!signoRepeller) adcFloat = float(adc*PENDIENTE_P+T_INDEPENDIENTE_P);
            else adcFloat = float(adc*PENDIENTE_N+T_INDEPENDIENTE_N);
            digitalWrite(PIN_TEST,LOW);
            NVIC_DisableIRQ(EIC_IRQn); //Inhabilita al encoder para lanzar una interrupción
            muestra_tension(adcFloat, signoRepeller); //
            NVIC_EnableIRQ(EIC_IRQn); //Habilita al encoder para lanzar una interrupción
            digitalWrite(PIN_TEST,HIGH);
            DEP("ADC= "+String(adc));
        }
        adc = 0;         // Resetea las variables para empezar un nuevo ciclo de medidas
        muestrasLeidas = 0; // Pone a cero el contdor de medidas
        TimerTcc0.start();    // Arranca el timer de muestreo
        digitalWrite(PIN_LED_RXL, !digitalRead(PIN_LED_RXL)); // Led para ver la latencia
 }
/**************************************************************
 * Rutina de la interrupción del rotary encoder
 * attachInterrupt(CLK_ROTARY_ENCODER, encoder, FALLING);
 * ***********************************************************/
void int_encoder(void)
{
    //digitalWrite(PIN_TEST,LOW);
    intEncoder = true;          // Flag qu deja pendiente la atención a la interrupción en el loop
    NVIC_DisableIRQ(EIC_IRQn);  // Inhabilita esta interrupción que se habilitará cuando se limpie el flag intEncoder
    ckRotary = digitalRead(CLK);//lee el pin CK del rotary
    dtRotary = digitalRead(DT); // Lee el pin DP para detectar la dirección
    //digitalWrite(PIN_TEST,HIGH);
}
/*************************************************************
 *  Gestión del encoder
 * // Respuesta al flag de interrupción del encoder
 * **********************************************************/
inline void lee_encoder(void)
{
    if (cambio_en_encoder(&rotary)) // Comprueba si ha cambiado del encoder
    { // Solo si rotary encoder ha cambiado actualiza la polaridad
        if (rotary >= 0)
        {
            signoRepeller = POS;
            actualiza_DAC(rotary, ADD_DAC, 0); // Actualiza el DAC, absoluto
        }
        else // rotary < 0
        {
            uint16_t rotPositivo = -1 * rotary;
            actualiza_DAC(rotPositivo, ADD_DAC, 0); // Actualiza el DAC, absoluto
            signoRepeller = NEG;
       }
       actualiza_polaridad();
        DEP("rotary= "+String(rotary));
    }
    if (rotaryLimite)
    {
        beep(FREQ_LIM, MS_LIM);
        rotaryLimite = false;
    } // Si el encoder está tocando límites emite un beep
    else
    {
        beep(FREQ_PRE, MS_PRE); // Si no ha llegado a límite emite un beep de pulso de rotary encoder bueno PRE
    }
    intEncoder = false;         // Baja el flag de la interrupción del encoder
    NVIC_EnableIRQ(EIC_IRQn);   // Habilita la interrupción de los pines para que funcione el encoder
}
/*************************************************************
 *  función que actualiza el valor del rotary encoder
 *  Los pines del encoder se leen en la interrupción
 * ***********************************************************/
inline bool cambio_en_encoder(int16_t *rotary)
{
    if (ckRotary) return false;   // Si CK no está a nivel bajo sale sin hacer nada
    int16_t _rotary = *rotary;    // guarda el valor de entrada
    if(!dtRotary)//si DT es 0 decrementa
    {
        *rotary-=DELTA;
        if(*rotary<OFFSET_P && *rotary>0 ) //Si etá en el tramo prohibido  0<rotary<240 ajusta a 240
        {
            *rotary=OFFSET_N;
        } 
        else if(*rotary<LIM_MIN) //Ajusta al límite inferior
        {
            *rotary=LIM_MIN;
            rotaryLimite = true;
        }
    }
    else //if(dtRotary) Si DT es 1 incrementa
    {
        *rotary+=DELTA;
        if(*rotary>OFFSET_N && *rotary<0) //Si etá en el tramo prohibido  0>rotary>-240 ajusta a -240
        {
            *rotary=OFFSET_P;
        } 
        else if(*rotary>LIM_MAX) //Ajusta al límite superior
        {
            *rotary=LIM_MAX;
            rotaryLimite = true;
        }
    }
    return _rotary != *rotary;
}
/*************************************************************
  Funciones para utilizar el display
**************************************************************/
/*************************************************************
  Cartel inicial
**************************************************************/
inline void display_presentacion(void) // Cartel de presentación inicial
{
    display.clear();
    // Cambia la resolución
    //display.setFont(u8x8_font_courB18_2x3_f); // Display con un font más grande
    display.setFont(u8x8_font_inr21_2x4_f); // Display con un font más grande
    display.inverse();
    display.drawString(0, LINEA1, "REPELLER"); // Cartel de la fila superior
    display.noInverse();
    delay(2000); // Presentación inicial del display
    display.clear();
}
/**************************************************************
 * Callback de la interrupción del temporizador del periodo de
 * muestreo y función que lee los ADCs
 * lectura de los ADCs (4mS)
 * ***********************************************************/
void int_timerTS(void) { leerADC = true; }
/**************************************************************
 * Muestra el estado en el display
 * ***********************************************************/
inline void muestra_tension(float tension, bool signo)
{
    static float copiaTension;
    if (signo) tension *= (-1);  // Si el relé está en negativo...
    if (tension == copiaTension) return; //No actua sobre el display para poner el mismo valor que ya tiene
    snprintf(msg, sizeof msg, "%+3.1f V", tension); //
    ajusta_cadena('V', msg, 8);                   // Formatea el array para que no se vean caracteres dobles
    /*
    uint8_t i;
    for (i = 0; msg[i] != 'V'; i++)
        ; // Evita que se vea una V doble
    for (i += 1; i < 8; i++)
    msg[i] = ' '; // borra todo desde V al final de la cadena
    msg[++i] = 'V';
    msg[8] = '\0';                      // Cierra bien la cadena
    */
    display.drawString(0, LINEA1, msg); //
    copiaTension=tension; //Guarda la tensión actual para la próxima iteración
}
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
 * Actualiza la polaridad cambiando el estado del relé
 ***************************************************************/
inline void actualiza_polaridad(void)
{
    digitalWrite(POLARIDAD, signoRepeller);
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
/**************************************************************
 * callback de la interrupción del switch del ratary encoder.
   Sube el flag para que se de servicio a la interrupción
 * ***********************************************************/
#ifdef SWITCH_ENCODER // Switch del rotary opcional
    void int_switch(void) { intSwitch = true; }
#endif
/**************************************************************
 * función que da servicio a la interrupción de switch del
 * rotary encoder
 * ***********************************************************/
#ifdef SWITCH_ENCODER // Switch del rotary opcional
inline void switch_encoder(void)
{
    rotary = OFFSET_P;                   // Pone a valor por defecto el rotary
    actualiza_DAC(rotary, ADD_DAC, 0); // Actualiza el DAC a valor por defecto
    signoRepeller = POS;      // Polaridad a positiva
    actualiza_polaridad();
    intSwitch = false;
    beep(FREQ_PRE, MS_PRE); // Emite un beep de aprobación
}
#endif
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
/*********************** FIN ***********************************/
