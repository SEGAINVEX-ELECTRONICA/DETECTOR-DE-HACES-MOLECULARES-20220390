/**
 * UNIVERSIDAD AUTÓNOMA DE MADRID
 *  Proyecto: 20220390
 *  Electronica para cañón de Electrones Detector
 *  de  Haces Moléculares: Deplection Plates
 *
 *  Autor: Patricio Coronado Collado
 *  Versión: 1.0 marzo 2024
 *  Aplicación para SAMD21 ecosistema Arduino Seeeduino XIAO.
 *  Descripción:
 *
 *  Ultima actualizacion 28-06-2024
 */
#include <Arduino.h>
#include <U8g2lib.h>          //Display
#include <Wire.h>             //I2C para el display y el DAC
#include "WDTZero.h"          //Watchdog
#include <TimerTCC0.h>        //Timer para periodo de muestreo
#include "SegaSCPI.h"         //Para depurar con SCPI
#include "ZeroConfigureADC.h"
/*************************************************************
  Macros
**************************************************************/
#define DEP(x) if(depuracion) Serial.println(x)
/**************************************************************
 *  defines opcionales
 * ************************************************************/
#define WATCHDOG //Resetea el sistema si se cuelga
/**************************************************************
 * Pines
 * ************************************************************/
#define CLK 6
#define DT 3
#define SW 0
#define BUZZER 7
#define PIN_TEST 8
#define ADC_A 9
#define ADC_B 10
/**************************************************************
 * Constantes y variables globales
 **************************************************************/
//Direcciones y constantes de los DACs MCP4725
#define DAC_A 0x60 //A0=0
#define DAC_B 0x61 //A0=1
#define EN_EEPROM 1
#define NO_EN_EEPROM 0
//Parámetros de conversión de entero a real
  #define PENDIENTE 0.02466F
  #define OFFSET -0.06946317
// Constantes para tone()
#define FREQ_LIM 500  // hz frecuencia
#define FREQ_PRE 1000 // hz frecuencia
#define MS_LIM 30     // mseg tiempo de duración
#define MS_PRE 10     // mseg tiempo de duración
#define LSB 0.024416//0.0008058*30.3 //Voltios
//ADC. Periodo de muestreo y muestras a promediar
#define TS_20ms 20000 //us
#define N_MUESTRAS 16
// bytes para actuar sobre los relés que cambian la polaridad de las fuentes a través del expansor
#define POL_DPB_POS 0b00000000
#define POL_DPB_NEG 0b00000001
#define POL_DPA_POS 0b00000000
#define POL_DPA_NEG 0b00000010
//Display
#define DISPLAY_FONT_2_LINEAS display.setFont(u8x8_font_courB18_2x3_f) //Font para solo 2 lineas LINEA1 y LINEA5
#define LINEA1 1
#define LINEA2 5
#define MUESTRA_DPA true
#define MUESTRA_DPB false
//Frecuencias del I2C 
#define I2C_VEL_1 100000
#define I2C_VEL_3 300000
//
//Constantes del Rotary encoder
#define LIM_MAX 3650 // Valores límite del rotary encoder
#define LIM_MIN -3650
#define DELTA 15 // Incremento en cada pulso
#define OFFSET_P 200 //Valor positivo más pequeño
#define OFFSET_N -200 //Valor negativo más grande
//Valres de los rotary encoders
int16_t rotaryA = OFFSET_P+DELTA;    // Estado del rotaryA valor con signo
int16_t rotaryB = OFFSET_P+DELTA ;    // Estado del rotaryB valor con signo
int16_t *rotaryActivo=&rotaryA; //Rotary  A activo por defecto
bool rotaryLimite = false; //flag para anotar límites de los rotary
//Para cambiar los relés de signo a través del expansor
uint8_t polaridadDpA=POL_DPA_POS; //Polaridad deplection plate A positivo por defecto
uint8_t polaridadDpB=POL_DPA_POS; //Polaridad deplection plate B positivo por defecto
// variables ADC
uint16_t adcA=0;//Valor binario de los adcs
uint16_t adcB=0;
uint16_t _adcA=0;//Para guardar el valor entre iteraciones
uint16_t _adcB=0;
float dPlateBreal; //Valres reales de los deplection plates
float dPlateAreal;
int8_t muestrasLeidas=0;//Contador de muestras del ADC a promediar
//Flags
bool leerADCs=false;//Para indicar que hay que leer el ADC
bool intEncoder=true;//Para anotar una interrupción del rotary
bool DplateActivoA=true; //true deplection plate A activa por defecto
bool DplateActivoB=false; //true deplection plate A activa por defecto
bool depuracion=true;//Comentar en la versión final
bool clkRotary=true;//Valor del pin clk del rotary
bool dtRotary=true;//Valor del pin dt del rotary
bool rotaryPulsado=false;//Valor del pin sw del rotary
bool suenaBeep=false; //True para hacer sonar un "beep"
bool refrescarDisplay=false; //Para mostar valres al cambiar de DP activo
/**************************************************************
 *  Objetos
 **************************************************************/
extern SegaSCPI segaScpi; //Comunicación con el PC
U8X8_SH1106_128X64_NONAME_HW_I2C display(U8X8_PIN_NONE); // Display
#ifdef WATCHDOG
  WDTZero WatchDog; // Objeto Watchdog
#endif
 /**************************************************************
 * prototipos de funciones
 ***************************************************************/
inline void display_presentacion(void);
void int_timerTS(void); //Callback del timer para leer muestras del ADC
inline bool cambio_en_encoder(int16_t*); //Rutina para actualizar el estado del rotary activo
void int_encoder(void); //Callback del rotary
void int_switch(void); //Callback del timer del switch del rotary
inline void muestra_tensiones();
inline void actualiza_DAC(uint16_t, uint8_t, bool);
inline void actualiza_reles(void);
inline void lee_adcs(void);
inline void beep(uint32_t, uint32_t);//Emite un tono
void lee_encoder(void);
inline void calcula_tensiones(void);
inline bool ajusta_cadena(char,char*,uint8_t);
/**************************************************************
 * setup
 ***************************************************************/
void setup()
{
  //NVIC_SetPriority(EIC_IRQn,4); // Interrupciones de pines. Mínima prioridad
  //NVIC_SetPriority(TCC0_IRQn, 2); // Baja prioridad de interrupción Timer0
  //NVIC_SetPriority(WDT_IRQn,4); //Interrupción del watchdog
  // Configuración ADC // https://blog.thea.codes/getting-the-most-out-of-the-samd21-adc/
  while (ADC->STATUS.bit.SYNCBUSY == 1);//Sincroniza
  ADC->INPUTCTRL.bit.GAIN = ADC_INPUTCTRL_GAIN_1X_Val;  //Ganancia 1
  ADC->REFCTRL.bit.REFSEL = ADC_REFCTRL_REFSEL_AREFB_Val; //Pin de referencia A1
  while (ADC->STATUS.bit.SYNCBUSY == 1); //Sincroniza
  configure_ADC(3,6,1); // CLK_ADC 1.5MHz, muestras a promediaren cada lectura =64,tiempo de muestreo 2us
  analogReadResolution(12);//La resolución de los adcs a 12 bits
  // Pines
  pinMode(CLK,INPUT_PULLUP);
  pinMode(DT,INPUT_PULLUP);
  pinMode(SW,INPUT_PULLUP);
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(PIN_LED_RXL,OUTPUT);
  pinMode(PIN_LED_TXL,OUTPUT);
  pinMode(BUZZER,OUTPUT);
  pinMode(PIN_TEST,OUTPUT);
  digitalWrite(PIN_TEST,HIGH);
  //Display
  display.begin();// Display
  display.setBusClock(I2C_VEL_1);//ADS1115 puede llegar a 400KHz
  Wire.setTimeout(5/*milisegundos*/); // Time-out del I2C, para salir de cuelgues por el i2c
  actualiza_reles(); //Configura la polaridad de salida de los DPs
  actualiza_DAC(rotaryA,DAC_A,NO_EN_EEPROM);//Valor absoluto y dirección
  actualiza_DAC(rotaryB,DAC_B,NO_EN_EEPROM);//Valor absoluto y dirección
  display_presentacion(); //Muestra la presentación inicial
  DISPLAY_FONT_2_LINEAS;  //Display con un font más grande
  muestra_tensiones();//Actualiza el display con las tensiónes de la DPB leidas
  // Recursos
  Serial.begin(115200);//Serial
  attachInterrupt(CLK, int_encoder, FALLING);
  attachInterrupt(SW, int_switch, FALLING);
  TimerTcc0.initialize(TS_20ms); // preferiblemente 20ms
  TimerTcc0.attachInterrupt(int_timerTS);
  //Más Comandos del timer TimerTcc0.start();TimerTcc0.restart();TimerTcc0.stop();
  //Watchdog
  #ifdef WATCHDOG
  //WDT_HARDCYCLE62m  0x0430    // WDT HARD only : 64 clockcycles @ 1024hz = 62.5ms
  //WDT_HARDCYCLE250m 0x0450    // WDT HARD only : 256 clockcycles @ 1024hz = 250ms
  //WDT_HARDCYCLE1S   0x0470    // WDT HARD only : 1024 clockcycles @ 1024hz = 1 seg
  //WDT_HARDCYCLE2S   0x0480    // WDT HARD only : 2048 clockcycles @ 1024hz = 2 seconds
  //WDT_HARDCYCLE4S   0x0490    // WDT HARD cycle 4 Seconds
    WatchDog.setup(WDT_HARDCYCLE1S);// Activa el watchdog WDT_SOFTCYCLE8S
  #endif
  // Empieza el espectáculo
}
/**************************************************************
 * loop
 ***************************************************************/
void loop() {
  #ifdef WATCHDOG
    WatchDog.clear(); // Refesca el watchdog
   #endif
   //Atiende al PC
  if (Serial.available()){segaScpi.scpi(&Serial);}//Atiende al PC
  // Gestiona la interrupción del rotary encoder
  if(intEncoder) lee_encoder();
  //Si ha transcurrido un tiempo de muestreo se leen los adcs
  if(leerADCs) lee_adcs();
  // Calcula promedios de lecturas de los adcs y muestra en display
  if(muestrasLeidas >= N_MUESTRAS) calcula_tensiones();
}
/***************************************************************
 *
 * ************************************************************/
void lee_encoder(void)
{
    digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN)); // Led para ver la latencia
    if(DplateActivoA)//Si el deplection plate A es el activo..
    {
      if(cambio_en_encoder(&rotaryA))// comprueba si ha cambiado el valor del rotary
      {
        uint8_t polDpA=polaridadDpA;//Guarda la polariad de entrada
        if (rotaryA >= 0)
        {
            polaridadDpA=POL_DPA_POS; //Salida positiva
            actualiza_DAC(rotaryA, DAC_A, 0); // Actualiza el DAC, absoluto
        }
        else // rotary < 0
        {
            uint16_t rotPositivo = -1 * rotaryA;//Salida negativa
            polaridadDpA=POL_DPA_NEG;
            actualiza_DAC(rotPositivo, DAC_A, 0); // Actualiza el DAC, absoluto
        }
        if (polDpA!=polaridadDpA)actualiza_reles();//Si ha cambiado la polaridad actualiza los relés
        DEP("rotary A="+String(rotaryA));
      }
    }
    if(DplateActivoB)//Si el deplection plate B es el activo..
    { //Si el deplection plate activo es el B actualiza su polaridad y tensión de salida...
      if(cambio_en_encoder(&rotaryB))
      {
        uint8_t polDpB=polaridadDpB;//Guarda la polariad de entrada
        if (rotaryB >= 0)
        {
            polaridadDpB=POL_DPB_POS;
            actualiza_DAC(rotaryB, DAC_B, 0); // Actualiza el DAC, absoluto
        }
        else // rotary < 0
        {
            uint16_t rotPositivo = -1 * rotaryB;
            polaridadDpB=POL_DPB_NEG;
            actualiza_DAC(rotPositivo, DAC_B, 0); // Actualiza el DAC, absoluto
        }
        if (polDpB!=polaridadDpB)actualiza_reles();//Si ha cambiado la polaridad actualiza los relés
        DEP("rotary B="+String(rotaryB));
      }
    }
    if (rotaryLimite)// Si el encoder está tocando límites emite un beep más agudo
    {
        beep(FREQ_LIM, MS_LIM);
        rotaryLimite = false;
    }
    else // Si el encoder no está tocando límites emite un beep más grave
    {
        beep(FREQ_PRE, MS_PRE); // Si no ha llegado a límite emite un beep de pulso de rotary encoder bueno PRE
    }
    intEncoder = false;         // Baja el flag de la interrupción del encoder
    NVIC_EnableIRQ(EIC_IRQn);   // Habilita la interrupción de los pines para que funcione el encoder
}
/**************************************************************
 * Rutina de la interrupción del rotary encoder
 * attachInterrupt(CLK_ROTARY_ENCODER, encoder, FALLING);
 * ***********************************************************/
void int_encoder(void)
{
    intEncoder = true;          // Flag qu deja pendiente la atención a la interrupción en el loop
    NVIC_DisableIRQ(EIC_IRQn);  // Inhabilita esta interrupción que se habilitará cuando se limpie el flag intEncoder
    clkRotary = digitalRead(CLK);//lee el pin CK del rotary
    dtRotary = digitalRead(DT); // Lee el pin DP para detectar la dirección
}
/**************************************************************
   función que revela cambio en un rotary físico y actualiza
    el valor incrementandolo o decrementandolo DELTA.
      Los pines del encoder se leen en la interrupción. El argumento
   de entrada apunta al valor del rotary activo.
   Como cerca de 0 las fuentes de HV no son lineales se evita
   tramos de valores en torno a 0
**************************************************************/
inline bool cambio_en_encoder(int16_t *rotary)
{
  if (clkRotary) return false;   // Si CLK no está a nivel bajo sale sin hacer nada
    int16_t _rotary = *rotary;    // guarda el valor de entrada
    if(!dtRotary)//si DT es 0 decrementa
    {
        *rotary-=DELTA;
        if(*rotary<OFFSET_P && *rotary>0 ) //Si está en el tramo prohibido  0<rotary<240 ajusta a 240
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
        if(*rotary>OFFSET_N && *rotary<0) //Si etá en el tramo prohibido  0>rotary>OFFSET_N ajusta a OFFSET_N
        {
            *rotary=OFFSET_P;
        }
        else if(*rotary>LIM_MAX) //Ajusta al límite superior
        {
            *rotary=LIM_MAX;
            rotaryLimite = true;
        }
    }
    return _rotary != *rotary; //Si el valor de entrada es diferente de el de salida sale con true
}
/**************************************************************
 * Callback de la interrupción del temporizador del periodo de
 * muestreo y función que lee los ADCs
 * lectura de los ADCs (4mS)
 * ***********************************************************/
void int_timerTS(void) { leerADCs = true;}
/**************************************************************
 * Muestra el estado en el display
 * si DPA=true muestra el DPA y si es false el DPB
 * si DplateActivoA=true muestra DPA en inverso y
 * viceversa
 * ***********************************************************/
inline void muestra_tensiones()
{
  char msg[16]; //Para mostar en el display
  char signo;
    //Deflection plate A
    if(rotaryA<0) signo='-';
    else          signo='+';
    snprintf(msg, sizeof msg, "%c%3.1f%s", signo,dPlateAreal,"VA"); //
    ajusta_cadena('A',msg,8);// Formatea el array para que no se vean caracteres dobles
    if(DplateActivoA)
    {
      display.inverse();
      display.drawString(0, LINEA1, msg); //
      display.noInverse();
    }
    else    display.drawString(0, LINEA1, msg); //
    DEP("DPA="+String(dPlateAreal));
    //Deflection plate B
    if(rotaryB<0) signo='-';
    else          signo='+';
    snprintf(msg, sizeof msg, "%c%3.1f%s", signo,dPlateBreal,"VB"); //
    ajusta_cadena('B',msg,8);//Caracter a buscar, cadena y última posición de la cadena +1 para cerrar
    if(DplateActivoB)
    { //Si d. plate B activa...
      display.inverse();
      display.drawString(0, LINEA2, msg); //
      display.noInverse();
    }
    else    display.drawString(0, LINEA2, msg); //
    DEP("DPB="+String(dPlateBreal));
}
/**************************************************************
 * callback de la interrupción del switch del ratary encoder
 * Cambia el deplection plate activo
 * ***********************************************************/
void int_switch(void)
{
  if(DplateActivoA)
  {
    DplateActivoA=false;
    DplateActivoB=true;
  }
  else
  {
    DplateActivoB=false;
    DplateActivoA=true;
  }
  rotaryPulsado=true; //Para que actualice el encoder
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
 ver datasheet para detalles
 Actualiza el valor del expansor PCF8574 por I2C (A3,2,1)=000
 velocidad máxima I2C 100KHz
***************************************************************/
inline void actualiza_reles(void){
  //https://www.nxp.com/docs/en/data-sheet/PCF8574_PCF8574A.pdf
  //https://www.luisllamas.es/mas-pines-digitales-con-arduino-y-el-expansor-es-i2c-pcf8574/
  Wire.beginTransmission(0x20);//dirección del MCP4725 
	Wire.write(polaridadDpA | polaridadDpB); // 8 bites
  Wire.endTransmission();
}
/**************************************************************
 *  entrega una señal cuadrada por el pin BUZZER de con
 *  frecuencia FREQ hz y duración MS milisegundos, para
 *  un buzzer pasivo
 ***************************************************************/
inline void beep(uint32_t frecuencia, uint32_t milisegundos)
{
    tone(BUZZER, frecuencia, milisegundos);
}
/*************************************************************
  Cartel inicial
**************************************************************/
inline void display_presentacion(void) // Cartel de presentación inicial
{
    display.clear();
    // Cambia la resolución
    display.setFont(u8x8_font_courB18_2x3_f); // Display con un font más grande
    display.inverse();
    display.drawString(0, LINEA1, "DEPL.PLT"); // Cartel de la fila superior
    display.noInverse();
    delay(2000); // Presentación inicial del display
    display.clear();
}
/**************************************************************
 * Conversión AD de  las entradas que miran las tensiones de los DPs
 ***************************************************************/
inline void lee_adcs(void)
{
    digitalWrite(PIN_TEST,LOW);
    adcA = adcA + analogRead(ADC_A);
    adcB = adcB + analogRead(ADC_B);
    digitalWrite(PIN_TEST,HIGH);
    muestrasLeidas++; //Actualiza el contador de muestras y el flag de la interrupción
    leerADCs = false;
}
/**************************************************************
 * Calcula los valores reales de las tensiones  y las muestra
 * ***********************************************************/
void calcula_tensiones(void)
{
    TimerTcc0.stop(); // Detiene el timer de muestreo
    digitalWrite(PIN_LED_RXL, !digitalRead(PIN_LED_RXL)); // Led para ver la latencia
    adcA = (adcA >> 4);//Promedia sobre las muestras leidas
    adcB = (adcB >> 4);
    if ((_adcA != adcA) || (_adcB != adcB) || rotaryPulsado)//Solo se refresca el display si ha cambiado algún valor o se ha pulsador el rotary
    {
        if(rotaryPulsado) rotaryPulsado=false;// Si se ha entrado por rotary pulsado se baja el flag
        _adcA=adcA;//Guarda los valores para la siguiente iteración
        _adcB=adcB;
        dPlateBreal=(float)adcB*PENDIENTE+OFFSET;//Si el rotary es negativo calcula la tensión real como negativa
        dPlateAreal=(float)adcA*PENDIENTE+OFFSET;//Si el rotary es negativo calcula la tensión real como negativa
        muestra_tensiones(); // Actualiza el display con las tensiónes actuales
    }
    adcA = 0; // Resetea las variables para empezar un nuevo ciclo de medidas
    adcB = 0;
    muestrasLeidas = 0; // Pone a cero el contdor de medidas
    TimerTcc0.start();  // Arranca el timer de muestreo
}
/**************************************************************
 * recorre "cadena"  buscando el caracter car, lo deja y
 * borra todo lo demás hasta finCadena
 * ***********************************************************/
bool ajusta_cadena(char car, char *cadena, uint8_t finCadena)
{
    uint8_t i;
    for (i = 0; cadena[i] != car; i++)//Busca "car" en la cadena
    {
        if (i >= finCadena) //Si no encuetra el caracter sale con false
            return false;
    }
    //Si encuentra el caracter...
    for (i += 1; i < finCadena; i++) // ...apunta al siguiente caracter y
    {
        cadena[i] = ' ';//'_'; // ...borra todo desde el hasta el final de la cadena...
    }
    cadena[finCadena] = '\0'; //Cierra la cadena..
    return true;//..y sale
}
/*********************** FIN **********************************/
