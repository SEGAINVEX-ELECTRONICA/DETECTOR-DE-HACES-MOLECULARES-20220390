#ifndef scpi_anodo_h
#define scpi_anodo_h
/*********************************************************************
 scpi.h
 Uso de la librería SegaSCPI para  comunicar
 un PC con Arduino a través del puerto serie "Serial"
**********************************************************************/
#include <Arduino.h>
#include "SegaSCPI.h"
#include <Wire.h>
/*********************************************************************
 * Macros
 *********************************************************************/
#define scpiPrintln segaScpi.PuertoActual->println
#define scpiPrint segaScpi.PuertoActual->print
/*********************************************************************
 * Constantes
 *********************************************************************/
extern uint16_t adcCorriente;
extern uint16_t adcTension;
extern bool depuracionRegulador;
/**********************************************************************
Prototipos de funciones
***********************************************************************/
// Funciones scpi comunes a todos los sistemas
void errorSCPI(void);
void opcSCPI(void);
void idnSCPI(void);
void clsSCPI(void);
// Funciones particulares de nuestro sistema
void modo_depuracion(void);
void i2cScan(void);
void help(void);
void watch_dog(void);
extern void on_off(void);
extern void pin_on_off(bool);
extern inline void actualiza_DAC(uint16_t, uint8_t,bool);
void step(void);
void adc_corriente(void);
void adc_tension(void);
void depuracion_regulador(void);
/**********************************************************************
 Menú principal que contiene comandos
**********************************************************************/
MENU_SCPI // menú de  comandos y submenús OBLIGATORIO
    {
        SCPI_COMANDO(DEPR, DER,depuracion_regulador)   //Envía datos del regulador periodicamente
        SCPI_COMANDO(ADCV, ADV,adc_tension)            //Envía la palabra digital de tensión
        SCPI_COMANDO(ADCI, ADI,adc_corriente)          //Envía la palabra digital de corriente
        SCPI_COMANDO(STEP, STP, step)                  // step para analizar la planta
        SCPI_COMANDO(ONOFF, ONF, on_off)               // activa desactiva la fuente HV 
        SCPI_COMANDO(DEPURACION, DEP, modo_depuracion) // entra 1 o sale de modo depuracion
        SCPI_COMANDO(WATCHDOG, WTD, watch_dog)         // Para que actue el watchdog
        SCPI_COMANDO(I2C, I2C, i2cScan)                // Borra la pila de errores
        SCPI_COMANDO(HELP, HLP, help)                  // Lee los comandos disponibles
        SCPI_COMANDO(ERROR, ERR, errorSCPI)            // Envía el ultimo error
        SCPI_COMANDO(*IDN, *IDN, idnSCPI)              // Identifica el instrumento
        SCPI_COMANDO(*OPC, *OPC, opcSCPI)              // Devuelve un 1 al PC
        SCPI_COMANDO(*CLS, *CLS, clsSCPI)              // Borra la pila de errores
    };
// declaramos el nivel Raíz cuya dirección se pasa a la función begin de SegaSCPI
tipoNivel Raiz[] = SCPI_RAIZ //// Declaración OBLIGATORIA del nivel Raiz. Siempre igual
    // Opcionalmente podemos definir la lista de errores (a partir de 7):
    /**********************************************************************
            errores del sistema definidos por el usuario
    **********************************************************************/
    String misErrores[] =
        {
            // Los errores de 0 a 6 son de scpi
            "7 la variable 1 no se ha cambiado",
            "8 la variable 2 no se ha cambiado",
            "9 otro error",
            "10 y otro error",
};
// Ahora el código habitual de Arduino:
SegaSCPI segaScpi(Raiz, "Haces Moleculares:Anodo", misErrores); // Instanciamos el objeto SCPI	global
/************************************************************************
    Funciones scpi comunes a todos los sistemas
 *************************************************************************/
/************************************************************************
   Función del Comando: ERROR ó ERR
   Envia por el puerto el último error registrado por SEGAINVEX-SCPI
*************************************************************************/
void errorSCPI(void) { segaScpi.errorscpi(0); }
/*************************************************************************
  Función del Comando: *IDN"
   Envia por el puerto una cadena que identifica al sistema
 *************************************************************************/
void idnSCPI(void) { segaScpi.enviarNombreDelSistema(); }
/************************************************************************
 Función del Comando:*OPC
 Envia por el puerto el carácter uno
*************************************************************************/
void opcSCPI(void) { segaScpi.PuertoActual->println("1"); }
/*************************************************************************
    Comando: CLS
    Limpia la pila de errores de SEGAINVEX-SCPI
 *************************************************************************/
void clsSCPI(void) { segaScpi.errorscpi(-1); }
/*************************************************************************/
// Funciones particulares del sistema
/*************************************************************************
  envia los comandos propios del sistema.
  HELP,HLP
 ************************************************************************/
void help()
{
    scpiPrintln("DER <1/0>\n envía, o no periodicamente al PC datos del regulador");
    scpiPrintln("ADV \n envía la palabra digital de tensión");
    scpiPrintln("ADI \n envía la palabra digital de corriente");
    scpiPrintln("STP <valor>\n actualiza el DAC con <valor>");
    scpiPrintln("ONF <1/0>\n activa/desactiva la fuente de HV");
    scpiPrintln("I2C\n busca dispositivos i2c");
    scpiPrintln("WTD\n hace un delay para que actue el watchdog");
    scpiPrintln("HELP\n para ver los comandos disponibles");
    scpiPrintln("DEP <1/0>\n el sistema entra/sale a modo depuracion");
    scpiPrintln("ERR\n devuelve el ultimo error del sistema");
    scpiPrintln("*IDN\n devuelve el nombre del sistema");
}
/*************************************************************************
    Comando: I2C
    Limpia la pila de errores de SEGAINVEX-SCPI
 *************************************************************************/
void i2cScan(void)
{
// Dejar sin comentar el que corresponda
// #define miWire Wire1
#define miWire Wire
    // miWire.begin();
    byte error, address;
    int nDevices;
    scpiPrintln("Scaneando...");
    nDevices = 0;
    for (address = 1; address < 127; address++)
    {
        miWire.beginTransmission(address);
        error = miWire.endTransmission();
        if (error == 0)
        {
            scpiPrint("componente I2C en la direccion 0x");
            if (address < 16)
                scpiPrint("0");
            scpiPrint(address, HEX);
            scpiPrint("  !\r\n");
            nDevices++;
        }
        else if (error == 4)
        {
            scpiPrintln("error desconocido en la direccion 0x");
            if (address < 16)
                scpiPrint("0\r\n");
            scpiPrintln(address, HEX);
        }
    }
    if (nDevices == 0)
        scpiPrintln("ningun componente I2C encontrado\r\n");
    else
        scpiPrintln("hecho!!\r\n");
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
    Hace un delay largo.
    Se utiliza para ver si el watcdog funciona
************************************************************************/
void watch_dog(void)
{
    scpiPrintln("retardo de 2 segundos");
    delay(2000); // Para ver que salta el watchdog
}
/************************************************************************
 * para modificar el pin on off que controla el módulo de potencia
 ***********************************************************************/
void on_off(void)
{
    static bool estadoPin=0;
    segaScpi.actualizaVarBooleana(&estadoPin);
    pin_on_off(estadoPin);
}
/************************************************************************
 *  Función que aplica un valor al DAC i2c MCP4725 en la ADD=0x60 
 * *********************************************************************/
void step(void)
{
    #define ADD_DAC 0x60 // dirección i2c del DAC A0=0
    uint16_t extern rotary;
    int rotaryLocal;
    if(segaScpi.actualizaVarEntera(&rotaryLocal,4095, 0)==1)
    {
        rotary=(uint16_t)rotaryLocal;
        actualiza_DAC(rotary, ADD_DAC, 0); // Actualiza el DAC
        scpiPrintln("step ejecutado");
    }
    else scpiPrintln("step no ejecutado");
}
/************************************************************************
 *  Función que envía al PC el valor del ADC de tension
 * *********************************************************************/
void adc_tension(void)
{
    scpiPrintln("ADC tension="+String(adcTension));
}
/************************************************************************
 *  Función que envía al PC el valor del ADC de corriente
 * *********************************************************************/
void adc_corriente(void)
{
    scpiPrintln("ADC corriente="+String(adcCorriente));
}
/************************************************************************
 *  Función que envía periodicamente al PC datos del regulador
 * *********************************************************************/
void depuracion_regulador(void)
{
    segaScpi.actualizaVarBooleana(&depuracionRegulador);
}
/*******************************FIN***************************************/
#endif // scpi_anodo_h