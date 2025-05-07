/***************************************************************
  Proyecto: Electrónica para detector de haces moleculares
  OT:20220390
  Set point del filamento
  Recibe los set point de corriente y tensión del set point
  de filamento y envía los valores de tensión y corriente
  al microcontrolador del set point
  //
  UNIVERSIDAD AUTÓNOMA DE MADRID 
  (SEGAINVEX-ELECTRÓNICA)
  PAtricio Coronado Collado: Julio 2023
  Ultima actualización 8 de marzo de 2024
***************************************************************/
//Objetos opcionales
//#define DISPLAY_FILAMENTO //Comentar cuando no se ponga el display
//#define SENSOR_BME280 //Comentar si no se pone sensor
//
#include <Arduino.h>
#include "scpi.h"
#include <espnow.h>
#include <ESP8266WiFi.h>
#include "tipos.h"
#include <Wire.h>
#include <Ticker.h>
#ifdef DISPLAY_FILAMENTO
  #include "U8g2lib.h" //Display
#endif  
#include "Adafruit_ADS1015.h" //ADC ADS1115
#ifdef SENSOR_BME280 
  #include "PacoAdafruit_BME280.h"
#endif
//*****************************
//I2C device found at address 0x3C  Display
//I2C device found at address 0x48  ADC
//I2C device found at address 0x60  DAC
//*****************************
//*****************************
// * funciones
//*****************************
void activa_filamento(bool);
inline void escucha_scpi(void);
void OnDataSent(uint8_t *mac_addr, uint8_t sendStatus);
void OnDataRecv(uint8_t *mac, uint8_t *incomingData, uint8_t len);
#ifdef DISPLAY_FILAMENTO
  inline void display_encoders(void);
  inline void display_ini(unsigned int);//inicializa display con frecuencia i2c
  inline void display_medidas(void);
#endif  
void ini_adc(void);
inline void lee_adc(void);
inline void envia_medidas(void);
inline void calcula_regulador(void);
inline void actualiza_DAC_MCP4725( uint16_t);
inline void serial_datos(void);
void muestrea_adc(void); //callback de ticker "muestreo"
#ifdef SENSOR_BME280
  void lee_sensor_temperatura(void);
#endif
void suena_beep(void);
void set_flag_1(void);
void int_pulsador(void); //Callback del timer del switch del rotary
//*****************************
//constantes y variables globales
//*****************************
#define MAX_FREQ_I2C 400000
//Recta de tensión 
#define ROTARY2DAC 36 //Para convertir el rotary encoder de tensión a palabra digital del DAC (ver cuaderno 10 pg 58)
//#define ROTARY2DAC 37.8 //Para convertir el rotary encoder de tensión a palabra digital del DAC (ver cuaderno 10 pg 58)
#define CONVERSION_ROTARY_AMPERIOS 10.0 //Cada unidad de % del rotary encoder es 0.1A 
uint16_t DACmax=1840; //Maxima salida del DAC a 5V
#define SI 1
#define NO 0
#ifdef SENSOR_BME280
  float temperatura=-1.0;
#endif
bool filamentoActivado=true;
#ifdef DISPLAY_FILAMENTO
  bool displayActivo=false;
#endif
bool limiteTension=false;
bool reguladorActivo=true;
uint16_t adcI; //Valor digital de la lectura de la corriente
//Variables para acumular medidas y calcular medias
uint32_t acumuladorAdcI=0;
uint16_t contadorCorriente=0;
uint32_t acumuladorAdcV=0;
uint16_t contadorTension=0;
uint16_t adcV; //Valor digital de la lectura de la tensión
int iADC=0;
bool leerAdc=false;
bool swPulsado=false;
bool statusBME280;
bool leerMuestas=false;
bool depuracion=false;
short buclesTemp=0;
short buclesTemp2=0;
short buclesTemp3=0;
uint16_t DatoDAC=0;//Dato para el DAC
long adc1 = 0;
long adc2 = 0;
float corrienteFilamento=0.0;
float tensionFilamento=0.0;
float setPointCorriente=0;
float setPointTension=5.0;
float u_k=0.0;//Salida actual
float u_k_1=0.0;//salida anterior 
float error_k=0.0;//Error actual
float error_k_1=0.0;//Error anterior
bool estadoLed=false;
int enviosFallidos=0;
unsigned long antes=0;
//espnow
struct_message datosSalientes;
struct_message datosEntrantes;//Para copiar los datos recibidos
// 48:55:19:E0:27:40
uint8_t broadcastAddress[] = {0x24, 0xD7, 0xEB, 0xCB, 0x6A, 0x7C}; //Mac del setpoint. Par espnow
const char* identificacion = "FILAMENTO"; 
bool spnowActivo=false;
bool datosRecibidos=false;
bool datosEnviados=false;
#define MAX_ENCODER 1000
#define MAX_DAC 4095
#define MILISEGUNDOS_LOOP 20
#define PERIODO_MUESTREO 20 //Periodo de muestreo en ms
//Pines
#define PIN_TEST D6
#define PIN_TEST2 D0
#define ON_OFF D5 //Pin para activar desactivar el módulo de potencia
//
#define TM_WDOG 200 //200 milisegundos watchdog
#define BME_WIRE_ADD 0x76 //Dirección wire del sensor de temperatura
#define T_SENSOR 5.5 //Tiempo entre lecturas del sensor de temperatura
//*****************************
//  objetos
//*****************************
#ifdef DISPLAY_FILAMENTO//Este objeto funciona bien con el display 2,4' y el de 1,3'
  U8X8_SH1106_128X64_NONAME_HW_I2C display(U8X8_PIN_NONE/*,15,14*/); // Display //argumentos (reset,clk,data);
#endif
//Convertidor ADC
#define LSB_ADS 0.0001250038//125.0038F / 1000000.0 // LSB del AD1115 con VFS=4.096V y 15 bits (el bit 16 es el de signo)
//El divisor de tensión para leer la tensión del filamento son (7K,3K) el ADC ve 3V por cada 10V de salida hay que multiplicar por 3.3333
#define PEND_V 0.00041993  //LSB_ADS*3.3006 //corregido empiricamente.   V_fil=D*PEND_V, D= palabra digital 15 bits
#define OFFSET_V -0.00241024
//El sensor de corriente (RS=20moh, LT6105)tiene una ganancia de 20e-3*15=0.3V/A. La corriente es LSB_ADS/0.3 = LSB_ADS*3.33333=
#define PEND_I 0.0006333 //LSB_ADS*3.35593 //corregido empiricamente.   I_fil=D*PEND_I
#define OFFSET_I -0.0361333
Adafruit_ADS1115 ads; // Convertidor ADC I2C: ADS1115 I2C address 0x48. 15 bits más 1 de signo
//Timers
Ticker muestreo; //Timer para muestrear las señales I y V
#ifdef SENSOR_BME280
  Ticker sensorTemperatura;//Timer par leer el sensor de temperatura
  Adafruit_BME280 bme; // Instancia el sensor de temperatura
#endif  
//*****************************
// * setup
//*****************************
void setup() 
{
  //Pin ON_OFF
  pinMode(ON_OFF,OUTPUT);
  digitalWrite(ON_OFF,LOW);//El módulo de potencia inhabilitado
  //Inicializa objetos
  ////Serial1.begin(115200,SERIAL_8N1);
  Serial.begin(115200);
  Wire.begin();
  Wire.setClock(MAX_FREQ_I2C);//I2C a máxima frecuencia
  #ifdef DISPLAY_FILAMENTO
      display_ini(MAX_FREQ_I2C);//Inicializa display, Wire a maximo 400KHz
  #endif
  pinMode(LED_BUILTIN,OUTPUT);
  digitalWrite(LED_BUILTIN,HIGH);
  pinMode(PIN_TEST,OUTPUT);
  digitalWrite(PIN_TEST,HIGH);
  pinMode(PIN_TEST2,OUTPUT);
  digitalWrite(PIN_TEST2,HIGH);
  //Sensor de temperatura
  #ifdef SENSOR_BME280
			statusBME280 = bme.begin(BME_WIRE_ADD,&Wire);  
	#endif
  //espnow
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
  // Init ESP-NOW
  if (esp_now_init() != 0) spnowActivo=false;
  else spnowActivo=true;
  if(spnowActivo)
  {
  // Once ESPNow is successfully Init, we will register for Send CB to
  // get the status of Trasnmitted packet
    esp_now_set_self_role(ESP_NOW_ROLE_COMBO);
    esp_now_register_send_cb(OnDataSent);
    // Register peer
    esp_now_add_peer(broadcastAddress, ESP_NOW_ROLE_COMBO, 1, NULL, 0);
    // Register for a callback function that will be called when data is received
    esp_now_register_recv_cb(OnDataRecv);
    //
  }
   actualiza_DAC_MCP4725(0);//El DAC a 0
  ini_adc();
  muestreo.attach_ms(PERIODO_MUESTREO,muestrea_adc);//Inicia el muestreo del ADCdatosEnviados
  #ifdef SENSOR_BME280
    sensorTemperatura.attach(T_SENSOR,lee_sensor_temperatura);
  #endif 
  delay(200);//Espera 200ms antes de empezar a funcionar
  ESP.wdtEnable(TM_WDOG);
  digitalWrite(ON_OFF,HIGH);//El módulo de potencia habilitado
}
//*****************************
// * loop
//*****************************
void loop()
{
  ESP.wdtFeed();//reinicia el timer del watchdog
  //scpi....................................................
  escucha_scpi();
  //Si ha pasado un TS desde la última lectura del ADC tiene lo lee de nuevo.....................
  if(leerAdc)
  {
    //digitalWrite(PIN_TEST,LOW);//D6
    lee_adc(); //Si leo dentro de la interrupción det ticker me da problemas, solo lee un canal ¿?
    //digitalWrite(PIN_TEST,HIGH);
    if(filamentoActivado)
    {
      //digitalWrite(PIN_TEST2,LOW);//D0
      calcula_regulador();
      actualiza_DAC_MCP4725(DatoDAC);//
      //digitalWrite(PIN_TEST2,HIGH);
    }
    leerAdc=false;
  }
  // Si ha recibido datos de espnow.........................
  if (datosRecibidos)
  {
    digitalWrite(LED_BUILTIN,estadoLed=!estadoLed);//Para ver la carencia de lecturas
    //lee el setpoint de corriente
    setPointCorriente=datosEntrantes.potCorriente/CONVERSION_ROTARY_AMPERIOS;//100% son 10A. Convierte % a amperios
    setPointTension=datosEntrantes.potTension;
    //Calcula la máxima tensión de salida solicitada por el rotary encoder de tensión (cuaderno 10 página 58)
    float ParteEntera;
    modf((float)setPointTension*ROTARY2DAC,&ParteEntera);//Extrae la parte entera tras multiplicar por el factor de conversión
    DACmax=(uint16_t) ParteEntera; //Convierte float a entero corto
    Serial.println(DACmax);
    //Gestiona la pulsación del switch del rotary encoder
    swPulsado=datosEntrantes.swPulsado;
    if(swPulsado && filamentoActivado) activa_filamento(NO);
    if(!swPulsado && !filamentoActivado) activa_filamento(SI);
    #ifdef DISPLAY_FILAMENTO
      if(displayActivo) display_encoders();
    #endif
    datosRecibidos = false;
  }
  //Si tiene datos nuevos que enviar.......................
  if (datosEnviados)
  {
    datosEnviados = false;
  }
    // Loop temporizado--------------------------------------
  if (millis() - antes > MILISEGUNDOS_LOOP)
  {
    buclesTemp++;
    if(buclesTemp>=10)
    {
      envia_medidas();
      #ifdef DISPLAY_FILAMENTO
        if(displayActivo) display_medidas();
      #endif

      buclesTemp=0;
    }
    //
    buclesTemp2++;
    #define N_BUCLES 50
    if(buclesTemp2>=N_BUCLES)//20ms*N_BUCLES
    {
       if(depuracion)
       {
          serial_datos();
          //Serial.println();
          //Serial.println("corriente");
          //Serial.println(adcI);
          //Serial.println(corrienteFilamento);
          //Serial.println();
          //Serial.println("tension");
          //Serial.println(adcV);
          //Serial.println(tensionFilamento);
       }
       buclesTemp2=0;
    }
    //-------------------------------------------------------
    antes = millis();
  }
}
//*****************************
// * scpi
//*****************************
inline void escucha_scpi(void)
{
  if(Serial.available()){scpi.scpi(&Serial);}
}
/*******************************
 * Envía medidas del display
********************************/
inline void envia_medidas(void)
{
  if (contadorCorriente!=0) acumuladorAdcI=acumuladorAdcI/contadorCorriente;
  float corriente = acumuladorAdcI*PEND_I+OFFSET_I;
  if(corriente < 0.0) corriente=0.0;
  //datosSalientes.iFilamento=corrienteFilamento;
  acumuladorAdcI=0.0;
  contadorCorriente=0;
  if (contadorTension!=0) acumuladorAdcV=acumuladorAdcV/contadorTension;
  float tension = acumuladorAdcV*PEND_V+OFFSET_V;
  if(tension < 0.0) tension=0.0;
  acumuladorAdcV=0.0;
  contadorTension=0;
  datosSalientes.iFilamento=corriente;
  datosSalientes.vFilamento=tension;
  datosSalientes.limiteTension=limiteTension; //Anota si se está limitando por tensión
  // Send message via ESP-NOW
  if (spnowActivo)
  {
    esp_now_send(broadcastAddress, (uint8_t *)&datosSalientes, sizeof(datosSalientes));
  }
}
//*****************************
// funciones del display
#ifdef DISPLAY_FILAMENTO
  //*****************************
  // inicializa display
  //*****************************
  inline void display_ini(unsigned int frecuenciaI2C)
  {
    display.begin();
    display.setBusClock(frecuenciaI2C);
    display.setFont(u8x8_font_7x14_1x2_f);
    display.drawUTF8(2, 0, identificacion);
    // Por defecto el display está inactivo
    display.drawUTF8(2, 2, "display off");
  }
   /********************************
    muestra el valor de los rotary
    encoders
  *******************************/
    inline void display_encoders(void)
    {
      char buffer[16];
      snprintf(buffer,sizeof buffer,"%d%s",datosEntrantes.potCorriente,"%  "); 
      display.drawString(9, 2, buffer);
      snprintf(buffer,sizeof buffer,"%d%s",datosEntrantes.potTension,"%  "); 
      display.drawString(9, 4, buffer);
    }
  //*********************************
  //  muestra variables del filamento
  //  en el display
  //*********************************
  inline void display_medidas(void)
  {
    if(!displayActivo) return;
    char buffer[16];
    snprintf(buffer, sizeof buffer, "  %.2fA ", corrienteFilamento);
    display.drawString(0, 2, buffer);
    snprintf(buffer, sizeof buffer, "  %.2fV ", tensionFilamento);
    display.drawString(0, 4, buffer);
    if (swPulsado)
    {
      snprintf(buffer, sizeof buffer, "fallos:%d :(", enviosFallidos);
      display.drawString(0, 6, buffer);
    }
    else
    {
      snprintf(buffer, sizeof buffer, "fallos:%d :)", enviosFallidos);
      display.drawString(0, 6, buffer);
    }
    #ifdef SENSOR_BME280
      snprintf(buffer,sizeof buffer,"%.1fC",temperatura);//Muestra la temperatura
      display.drawString(10, 0, buffer);
    #endif
  }
#endif
/********************************
    funcion scpi;
    activa o desactiva el display
    Esta función está definida
    siempre ya que es una función
    scpi y está declarada en scpi.h
  *******************************/
  void display_on_off(void)
  {
    #ifdef DISPLAY_FILAMENTO 
      scpi.actualizaVarBooleana(&displayActivo);
      display.clear();
      display.drawUTF8(2, 0, identificacion);
      if (displayActivo) return; //Si ha activado el display sale
      display.drawUTF8(2, 2, "display off");//Si ha desactivado el display lo muestra 
    #endif
    return;
  }
// fin funciones del display
//******************************
// Callback when data is sent
//******************************
void OnDataSent(uint8_t *mac_addr, uint8_t sendStatus) {
  datosEnviados=true;
  if (sendStatus == 0){ }
  else{
    enviosFallidos++;
  }
}
//********************************
// Callback when data is received
//********************************
void OnDataRecv(uint8_t * mac, uint8_t *incomingData, uint8_t len) {
  memcpy(&datosEntrantes, incomingData, sizeof(datosEntrantes));
  datosRecibidos=true;
}
//********************************
// inicaliza adc
//********************************
inline void ini_adc(void)
{
    ads.begin(); // Inicializa conversor ADS1115
    //ads.setGain(GAIN_TWO);
    ads.setGain(GAIN_ONE); //VFS=4.096V LSB=4.096V/(2^15-1)
  /*
  Según la ganancia que programemos tendremos un VFS y un LSB
  teniendo en cuenta que son 15 bits de resolución ya que el
  primer bit es el de signo. LSB=VFS/2^15

  GAIN_TWOTHIRDS    = ADS1015_REG_CONFIG_PGA_6_144V,VFS=6,144V 15 BITS
  GAIN_ONE          = ADS1015_REG_CONFIG_PGA_4_096V,VFS=4,096V 15 BITS
  GAIN_TWO          = ADS1015_REG_CONFIG_PGA_2_048V,VFS=2,048V, LSB=62.50191F/ 1000000.0
  GAIN_FOUR         = ADS1015_REG_CONFIG_PGA_1_024V, VFS=1,024V 15 BITS
  GAIN_EIGHT        = ADS1015_REG_CONFIG_PGA_0_512V,VFS=0,512V 15 BITS
  GAIN_SIXTEEN      = ADS1015_REG_CONFIG_PGA_0_256VVFS=0,256V 15 BITS
  */
}
//*********************************
// Lectura del adc
//*********************************
 inline void lee_adc(void)
{
    //Tensión del filamento
    adcV = ads.readADC_SingleEnded(2);
    acumuladorAdcV=adcV+acumuladorAdcV;//Acumula para calcular la media antes de enviar al setup
    contadorTension++;//Muestras acumuladas
    tensionFilamento = adcV*PEND_V+OFFSET_V;//Muestras acumuladas
    if (tensionFilamento < 0.0) tensionFilamento=0.0;//Evita valres negativos
    //Corriente del filamento
    adcI= ads.readADC_SingleEnded(0);
    acumuladorAdcI=adcI+acumuladorAdcI;//Acumula para calcular la media antes de enviar al setup
    contadorCorriente++;//Muestras acumuladas
    corrienteFilamento = adcI* PEND_I+OFFSET_I;//Valor de corriente actual
    if (corrienteFilamento < 0.0) corrienteFilamento=0.0;//Evita valres negativos

}
//*********************************
// callback de ticker "muestreo"
// Activa la lectura del adc
//
//*********************************
void muestrea_adc(void)
{
    leerAdc=true;//Flag para que lea el ADC en el loop
}
//************************************
// ACTUALIZA EL VALOR DEL DAC
//************************************
inline void actualiza_DAC_MCP4725( uint16_t DatoDAC)
{
  Wire.beginTransmission(0X60);//dirección del MCP4725 de Adafruit A0=GND
	Wire.write(0x40);//Escribe el dato en el DAC
	Wire.write(DatoDAC / 16); // 8 bites altos (D11.D10.D9.D8.D7.D6.D5.D4)
	Wire.write((DatoDAC % 16) << 4);// 4 bites bajos (D3.D2.D1.D0)
	Wire.endTransmission();
} 
//********************************
// envía todas las variables por
// el puerto serie
//********************************
void inline serial_datos(void)
{
    char buffer[64];
    snprintf(buffer,sizeof buffer,"SPcorriente=%.4fAmperios, Vmax=%.4fVoltios\n",setPointCorriente,setPointTension/10);
    Serial.println(buffer);
    snprintf(buffer,sizeof buffer,"Corriente=%.2f Amperios\nVoltaje=%.2f Voltios\n",corrienteFilamento,tensionFilamento); 
    Serial.println(buffer);
    snprintf(buffer,sizeof buffer,"DAC_MAX=%d,DAC=%d",DACmax,DatoDAC);
    Serial.println(buffer);
    snprintf(buffer,sizeof buffer,"error=%.4f,uk=%.4f,D=%d \n",error_k,u_k,DatoDAC);
    Serial.println(buffer);
    snprintf(buffer,sizeof buffer,"Encoder1=%d\nEncoder2=%d\n",datosEntrantes.potCorriente,datosEntrantes.potTension); 
    Serial.println(buffer);
    snprintf(buffer,sizeof buffer,"Envios fallidos=%d\n",enviosFallidos); 
    Serial.println(buffer);
    if(limiteTension)Serial.println("limitando tension");
    Serial.println("---------------------------");
}
#ifdef SENSOR_BME280
//********************************
// Lee el sensor de temperatura
//********************************
  void lee_sensor_temperatura(void)
  {
    if(!statusBME280) 
    {
      Serial.println("Sensor no disponible");
      return;
    }
    temperatura=bme.readTemperature();
    datosSalientes.temperatura=temperatura;
    if(depuracion)Serial.printf("Temp=%5.1fC\r\n",temperatura);
  }
#endif
//********************************
//  Calcula la salida binaria
//  del regulador
//********************************
inline void calcula_regulador(void)
{
  if (reguladorActivo)
  {
    // Calcula el error actual
    error_k = setPointCorriente - corrienteFilamento;
    if(error_k>1.0) error_k=1.0; //Limita el error
    if(error_k<-1.0) error_k=-1.0;
    //Calcula la salida del regulador
    //......................................................
    //#define b0 0.02
    //#define b1 0.0
    //u_k = u_k_1 + b0 * error_k; // Calcula la salida
    //......................................................
    //Regulador calculado PI robusto con sisotool
    #define b_0 0.2299
    #define b_1 -0.1616
    u_k=u_k_1 + b_0*error_k + b_1*error_k_1;
    //......................................................
    // antiwindup
    if (u_k > 3.0) u_k = 3.0;
    if (u_k < 0.0) u_k = 0.0;
    // Actualiza variables del reguldor para el siguiente ciclo
    u_k_1 = u_k;
    error_k_1=error_k;
    //Calcula la salida del DAC
    float DAC = (u_k / 3.0) * 4095.0;
    DatoDAC = (uint16_t)DAC;
    //Evita sobrepasamientos
    if (DatoDAC >= DACmax) //Anti windup de la Tensión de salida, si está al límite lo anota en un flag
    {
      DatoDAC = DACmax; //Limita la salida del ACD al máximo determinado por el rotary encoder de tensión
      limiteTension=true;
    }
    else limiteTension=false;
    //
    if (DatoDAC < 0) DatoDAC = 0;
  }
  else //Regulador inactivo (en lazo abierto)
  {
    DatoDAC = MAX_DAC*datosEntrantes.potTension/100; //potTension es un tanto por ciento
  }
}
//********************************
//  activa (1) desactiva (0) el filamento 
//********************************
void activa_filamento(bool activar)
{
  if(activar)
  {
    //Resetea variables del regulador
    u_k_1=0;
    error_k_1=0;
    error_k=0;
    digitalWrite(ON_OFF,HIGH);//El módulo de potencia habilitado
    filamentoActivado=true;
  }
  else
  {
     DatoDAC=0;
     actualiza_DAC_MCP4725(DatoDAC);//
     digitalWrite(ON_OFF,LOW);//El módulo de potencia inhabilitado
     filamentoActivado=false;     
  }
}
//********************************
// Funciones para depuración con scpi
//********************************
void suena_beep(void){return;}//hace sonar un beep
void set_flag_1(void){return;}//Cambia el flag1
//************* fin ****************