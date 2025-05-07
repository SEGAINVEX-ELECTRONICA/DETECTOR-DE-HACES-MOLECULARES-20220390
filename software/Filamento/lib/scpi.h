#ifndef SCPI_LOCAL_H
#define SCPI_LOCAL_H
/************************************************************************
 *  SCPI declaraciones, funciones propias y menús
************************************************************************/
#include <Arduino.h>
#include "SegaSCPI.h"
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

/***********************************************************************
 *  Prototipos de funciones comunes
************************************************************************/
String NombreDelSistema = "Clima salon"; //Puesto para depuración. Se puede quitar.
//Lista de errores:
String Errores[]=
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
MENU_SCPI  //menú de  comandos
{
	//SCPI_SUBMENU(NIVEL1,N1)	//COMANDOS DEL PC
	//Comandos que ejecutan funciones definidas en la librería Segainvex_SCPI_Serial
  SCPI_COMANDO(MAC,MAC,envia_mac)// 
  SCPI_COMANDO(RESET,RST,reset)// Resetea el ESP32
  SCPI_COMANDO(ERROR,ERR,errorSCPI)// Envía el ultimo error
  SCPI_COMANDO(HELP,HLP,help)// Lee los comandos disponibles
  SCPI_COMANDO(*IDN,*IDN,idnSCPI)// Identifica el instrumento
	SCPI_COMANDO(*OPC,*OPC,opcSCPI)// Devuelve un 1 al PC
	SCPI_COMANDO(*CLS,*CLS,clsSCPI)// Borra la pila de errores
};
/************************************************************************
                      Nivel principal 
*************************************************************************/
tipoNivel Raiz[]= SCPI_RAIZ // Declaración OBLIGATORIA del nivel Raiz. Siempre igual
/***********************************************************************
*                       Objeto scpi
************************************************************************/
SegaSCPI scpi(Raiz,"Detector de haces moleculares: Filamento",Errores);
#define print_scpi scpi.PuertoActual->println
/************************************************************************
        Definición de funciones scpi comunes a todos los sistemas
*************************************************************************/
/************************************************************************
    Función del Comando: ERROR ó ERR
    Envia por el puerto el último error registrado por SEGAINVEX-SCPI
*************************************************************************/
void errorSCPI(void){scpi.errorscpi(0);}
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
void opcSCPI(void){	print_scpi("1");}
/*************************************************************************
    Comando: CLS
    Limpia la pila de errores de SEGAINVEX-SCPI
	Se ejecuta en 430us
 *************************************************************************/
void clsSCPI(void){scpi.errorscpi(-1);}
/*************************************************************************/

//void reset(void);
//void help(void);

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
  print_scpi("RESET,RST\n el sistema se resetea");
  print_scpi("HELP,HLP\n el sistema debuelve los comandos\ndisponibles");
}



/************************************************************************
  envia mac del dispositivo
  MAC,MAC
 ************************************************************************/

void envia_mac() {print_scpi(WiFi.macAddress());}

/************************************************************************/
#endif //SCPI_LOCAL_H