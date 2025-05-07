/*********************************************************************
 SegaSCPI
 Uso de la librería SegaSCPI para  comunicar
 un PC con Arduino a través del puerto serie "Serial" 
 Ultima actualizacion 12-06-2024
**********************************************************************/
#include <Arduino.h>
#include "SegaSCPI.h"
#include <Wire.h>
/*********************************************************************
 * Macros
 *********************************************************************/
#define scpiPrintln segaScpi.PuertoActual->println
#define scpiPrint segaScpi.PuertoActual->print
/**********************************************************************
Prototipos de funciones
***********************************************************************/
//Funciones scpi comunes a todos los sistemas
void errorSCPI(void);
void opcSCPI(void);
void idnSCPI(void);
void clsSCPI(void);
// Funciones particulares de nuestro sistema
void scanI2C(void);
void actualiza_dac(void);
void activa_watch_dog(void);
void modo_depuracion(void);
void help(void);
extern inline uint8_t actualiza_DAC(uint16_t, uint8_t,bool);
/**********************************************************************
 Menú principal que contiene comandos
**********************************************************************/
MENU_SCPI  //menú de  comandos y submenús OBLIGATORIO
{
	SCPI_COMANDO(DEPURACION, DEP, modo_depuracion) // entra 1 o sale de modo depuracion
    SCPI_COMANDO(WATCHDOG,WTD,activa_watch_dog)// Envía el ultimo error
	SCPI_COMANDO(BUSI2C,I2C,scanI2C)// Envía el ultimo error
	SCPI_COMANDO(ADAC,ADA,actualiza_dac)// Envía el ultimo error
	SCPI_COMANDO(ERROR,ERR,errorSCPI)// Envía el ultimo error
  	SCPI_COMANDO(*IDN,*IDN,idnSCPI)// Identifica el instrumento
	SCPI_COMANDO(*OPC,*OPC,opcSCPI)// Devuelve un 1 al PC
	SCPI_COMANDO(*CLS,*CLS,clsSCPI)// Borra la pila de errores
    SCPI_COMANDO(HELP,HLP,help)// Borra la pila de errores
};
//declaramos el nivel Raíz cuya dirección se pasa a la función begin de SegaSCPI
tipoNivel Raiz[]= SCPI_RAIZ //// Declaración OBLIGATORIA del nivel Raiz. Siempre igual
//Opcionalmente podemos definir la lista de errores (a partir de 7):
/**********************************************************************
 		errores del sistema definidos por el usuario
**********************************************************************/
String misErrores[]=
{//Los errores de 0 a 6 son de scpi
	"7 la variable 1 no se ha cambiado",
	"8 la variable 2 no se ha cambiado",
	"9 otro error",
	"10 y otro error",

};
//Ahora el código habitual de Arduino:
SegaSCPI segaScpi(Raiz,"Haces Moleculares:Repeller",misErrores);//Instanciamos el objeto SCPI	global
/************************************************************************
    Funciones scpi comunes a todos los sistemas
 *************************************************************************/
 /************************************************************************
    Función del Comando: ERROR ó ERR
    Envia por el puerto el último error registrado por SEGAINVEX-SCPI
 *************************************************************************/
void errorSCPI(void){segaScpi.errorscpi(0);}
/*************************************************************************
  Función del Comando: *IDN"
   Envia por el puerto una cadena que identifica al sistema
 *************************************************************************/
void idnSCPI(void){segaScpi.enviarNombreDelSistema();}
 /************************************************************************
  Función del Comando:*OPC
  Envia por el puerto el carácter uno
 *************************************************************************/
void opcSCPI(void){	segaScpi.PuertoActual->println("1");}
/*************************************************************************
    Comando: CLS
    Limpia la pila de errores de SEGAINVEX-SCPI
 *************************************************************************/
void clsSCPI(void){segaScpi.errorscpi(-1);}
/************************************************************************
    Funciones scpi particulares del sistema
 *************************************************************************/
/************************************************************************
 * función que informa de los comandos disponibles
***************************************************************************/
void help(void)
{
    scpiPrintln("I2C\n busca dispositivos i2c");
    scpiPrintln("WTD\n hace un delay para que actue el\n watchdog");
    scpiPrintln("HELP\n para ver los comandos disponibles");
    scpiPrintln("DEP <1/0>\n el sistema entra/sale a modo depuracion");
    scpiPrintln("ADA <int16_t>\n envia dato al DAC");
    scpiPrintln("ERR\n devuelve el ultimo error del sistema\n");
    scpiPrintln("*IDN\n devuelve el nombre del sistema\n");
}
/************************************************************************
 * funciòn para test: actualiza el valor del dac en add i2c 0x62
***************************************************************************/
void actualiza_dac(void)
{
	int valor;
	if(segaScpi.actualizaVarEntera(&valor,0xfff, 0))	
		{
			actualiza_DAC(valor,0x60,0);
			scpiPrintln("Ok");
		}
		else scpiPrintln("parámetro inválido");
}
/************************************************************************
  Activa el flag depuracion a 1 "dep 1" ó a cero "dep 0"
  DEP,DEP
 ************************************************************************/
void modo_depuracion(void)
{
    extern bool depuracion;
    segaScpi.actualizaVarBooleana(&depuracion);
    if (depuracion)
        scpiPrintln("en modo depuracion");
    else
        scpiPrintln("no en modo depuracion");
}
/************************************************************************
 * funciòn para test: busca dispositivos i2c
**************************************************************************/
void scanI2C(void)
{
	//Dejar sin comentar el que corresponda
	//#define miWire Wire1 
	#define miWire Wire
	//miWire.begin();
	byte error, address;
	int nDevices;
	scpiPrintln("Scaneando...\r\n");
	nDevices = 0;
	for(address = 1; address < 127; address++ )
	{
		miWire.beginTransmission(address);
		error = miWire.endTransmission();
		if (error == 0)
		{
			scpiPrint("I2C device found at address 0x");
			if (address<16)
			scpiPrintln("0");
			scpiPrint(address,HEX);
			scpiPrintln("  !\r\n");

			nDevices++;
		}
		else if (error==4)
		{
			scpiPrint("Unknown error at address 0x");
			if (address<16)	scpiPrint("0\r\n");
			scpiPrintln(address,HEX);
		}
	}
	if (nDevices == 0)
	scpiPrint("No I2C devices found\r\n");
	else
	scpiPrint("done\r\n");
}
/***********************************************************************
 * función para hacer actuar al watchdog
 * *********************************************************************/
void activa_watch_dog(void)
{
	delay(2000);//Para ver que salta el watchdog
}
/*******************************FIN***************************************/
