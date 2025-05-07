/**********************************************************  
  Proyecto: Electrónica para detector de haces moleculares
  OT:20220390
    Dos microcontroladores esp8266; uno a tensión de
    red lee los set point de tensión y corriente del
    filamento y lo envía al microcontrolador del filamento
    que a su vez devuelve los valres medidos de tensión
    y corriente del filamento
***********************************************************/
/*******************************************
MAPA DE PINES PARA EL WEMOS D1 MINI ESP8266
SS =15
MOSI=13
MISO=12
SCK=14
A0=17 ¿?
PIN_WIRE_SDA=4
PIN_WIRE_SCL=5
LED_BUILTIN=2
D0   = 16
D1   = 5
D2   = 4
D3   = 0
TXD1=D4   = 2
D5   = 14
D6   = 12
D7   = 13
TXD2=D8  = 15
RXD=D9   = 3
TXD=D10  = 1
**********************************************************/
//*********************************************************
//
// Seleccionar uno u otro dispositivo.
// Cambiar tambien el upload_port en platformio.ini
//
//#define FILAMENTO //Si se comenta se compila setpoint
//*********************************************************
#ifdef FILAMENTO
  #include "filamento.h"
#else
  #include "setpoint.h"
#endif
//MAC de filamento 08:3A:8D:CF:85:8A
//MAC del setpoint 48:55:19:E0:27:40 antigua
//MAC del setpoint 24:D7:EB:CB:6A:7C actual
// Comandos para compilar y cargar
//C:\Users\Patricio\.platformio\penv\Scripts\platformio.exe run --target upload --upload-port COM30
//C:\Users\PC.5014885\.platformio\penv\Scripts\platformio.exe run --target upload --upload-port COM33
//****************** fin ***********************************
