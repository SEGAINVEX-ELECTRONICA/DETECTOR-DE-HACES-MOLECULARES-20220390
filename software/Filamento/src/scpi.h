#ifndef SCPI_LOCAL_H
#define SCPI_LOCAL_H
/************************************************************************
 *  SCPI declaraciones, funciones propias y menús
 ************************************************************************/
#include <Arduino.h>
#include <Wire.h>
#include "SegaSCPI.h"
#include <ESP8266WiFi.h>      //Para que reconozca tipos
extern ESP8266WiFiClass WiFi; // Objeto en el programa principal
extern const char * identificacion;
extern bool depuracion;
extern bool reguladorActivo;
/***********************************************************************
 *  Prototipos de funciones scpi
 ************************************************************************/
void clsSCPI(void);
void opcSCPI(void);
void idnSCPI(void);
void errorSCPI(void);
void reset(void);
void help(void);
void envia_mac(void);
void busca_i2c(void);
void modo_depuracion(void);
void cambia_regulador(void);
extern void display_on_off(void);
extern void suena_beep(void);
extern void set_flag_1(void);
/***********************************************************************
 *  Prototipos de funciones comunes
 ************************************************************************/
//String NombreDelSistema = "Clima salon"; // Puesto para depuración. Se puede quitar.
// Lista de errores:
String Errores[] =
    {
        // Errores del sistema SCPI 0...6 ya están definidas
        // Errores personalizados por el usuario
        "7 ",
        "8 ",
        "9 ",
        "10 ",
};
/************************************************************************
    Menú scpi
*************************************************************************/
MENU_SCPI // menú de  comandos
    {
        // SCPI_SUBMENU(NIVEL1,N1)	//COMANDOS DEL PC
        // Comandos que ejecutan funciones definidas en la librería Segainvex_SCPI_Serial
        
        SCPI_COMANDO(SETFLAG1,SF1,set_flag_1)   //cambia flag1 
        SCPI_COMANDO(DISPLAY,DSY,display_on_off)   //apaga el display
        SCPI_COMANDO(REGULADOR,REG,cambia_regulador)   //modifica el regulador
        SCPI_COMANDO(BUZZER,BUZ,suena_beep)   //hace sonar el buzzer
        SCPI_COMANDO(DEP,DEP,modo_depuracion)   //activa el flag de depuración
        SCPI_COMANDO(I2C,I2C,busca_i2c)   //
        SCPI_COMANDO(MAC, MAC, envia_mac)   //
        SCPI_COMANDO(RESET, RST, reset)     // Resetea el ESP32
        SCPI_COMANDO(ERROR, ERR, errorSCPI) // Envía el ultimo error
        SCPI_COMANDO(HELP, HLP, help)       // Lee los comandos disponibles
        SCPI_COMANDO(*IDN, *IDN, idnSCPI)   // Identifica el instrumento
        SCPI_COMANDO(*OPC, *OPC, opcSCPI)   // Devuelve un 1 al PC
        SCPI_COMANDO(*CLS, *CLS, clsSCPI)   // Borra la pila de errores
    };
/************************************************************************
                      Nivel principal
*************************************************************************/
tipoNivel Raiz[] = SCPI_RAIZ // Declaración OBLIGATORIA del nivel Raiz. Siempre igual
    /***********************************************************************
     *                       Objeto scpi
     ************************************************************************/
    
    SegaSCPI scpi(Raiz, identificacion, Errores);
#define print_scpi scpi.PuertoActual->println
/************************************************************************
        Definición de funciones scpi comunes a todos los sistemas
*************************************************************************/
/************************************************************************
    Función del Comando: ERROR ó ERR
    Envia por el puerto el último error registrado por SEGAINVEX-SCPI
*************************************************************************/
void errorSCPI(void) { scpi.errorscpi(0); }
/*************************************************************************
  Función del Comando: *IDN"
   Envia por el puerto una cadena que identifica al sistema
   Se ejecuta en 1ms
 *************************************************************************/
void idnSCPI(void)
{
  scpi.enviarNombreDelSistema();
}
/************************************************************************
 Función del Comando:*OPC
 Envia por el puerto el carácter uno
 Se ejecuta en 1ms
 *************************************************************************/
void opcSCPI(void) { print_scpi("1"); }
/*************************************************************************
    Comando: CLS
    Limpia la pila de errores de SEGAINVEX-SCPI
  Se ejecuta en 430us
 *************************************************************************/
void clsSCPI(void) { scpi.errorscpi(-1); }
/*************************************************************************/

// void reset(void);
// void help(void);

/*************************************************************************
  Reset RESET,RST
**************************************************************************/
void reset()
{
  print_scpi("se va a resetear el sistema");
  delay(500);
  ESP.restart();
}
/************************************************************************
  envia los comandos propios del sistema.
  HELP,HLP
 ************************************************************************/
void help()
{
  print_scpi("HELP\n el sistema debuelve los comandos\ndisponibles");
  print_scpi("RESET,RST\n el sistema se resetea");
  print_scpi("DEP 1/0\n el sistema entra/sale a modo depuracion");
  print_scpi("I2C\n devuelve la mac del ESP8266\n");
  print_scpi("DEP\n modifica el flag de depuracion\n");
  print_scpi("REG 1/0\n regulador en lazo cerrado/abierto\n");
  print_scpi("MAC\n devuelve la mac del ESP8266\n");
  print_scpi("DSY 1/0\n activa desactiva display\n");
  print_scpi("ERR\n devuelve el ultimo error del sistema\n");
  print_scpi("*IDN\n devuelve el nombre del sistema\n");
}
/************************************************************************
  envia mac del dispositivo
  MAC,MAC
 ************************************************************************/

void envia_mac() 
{
   print_scpi(WiFi.macAddress()); 
}
/************************************************************************
  Activa el flag depuracion
  DEP,DEP
 ************************************************************************/
void modo_depuracion(void)
{
  scpi.actualizaVarBooleana(&depuracion);
}
/************************************************************************
  Busca dispositivos i2c
  I2C,I2C
 ************************************************************************/
void busca_i2c(void)
{
  byte error, address;
  int nDevices;
  Wire.begin();
  Serial.println("Buscando dispositivos i2c...");
  nDevices = 0;
  for (address = 1; address < 127; address++)
  {
    // The i2c_scanner uses the return value of
    // the Write.endTransmisstion to see if
    // a device did acknowledge to the address.
    Wire.beginTransmission(address);
    error = Wire.endTransmission();

    if (error == 0)
    {
      Serial.print("Dispositivo i2c encontrado en 0x");
      if (address < 16)
        Serial.print("0");
      Serial.print(address, HEX);
      Serial.println("  !");

      nDevices++;
    }
    else if (error == 4)
    {
      Serial.print("error desconocido en 0x");
      if (address < 16)
        Serial.print("0");
      Serial.println(address, HEX);
    }
  }
  if (nDevices == 0)
    Serial.println("No se han encontrado dispositivos i2c\n");
  else
    Serial.println("...hecho\n");
}
/************************************************************************
  cambia una variable boolenan que condiciona el comportamiento del 
  regulador
 ************************************************************************/
void cambia_regulador(void)
{
    scpi.actualizaVarBooleana(&reguladorActivo);
}
/************************************************************************/

#endif // SCPI_LOCAL_H