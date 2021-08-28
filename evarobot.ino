//Copyright (C) 2018
/*This program is free software; you can redistribute it and/or
modify it under the terms of the GNU General Public License
as published by the Free Software Foundation; either version 2
of the License, or (at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.*/

//Librería programada por Wayoda (contáctese a <e.fahle@wayoda.org>)
#include <LedControl.h>

//Librerías del control remoto
#include <boarddefs.h>
//Librería programada por Ken Shirriff
#include <IRremote.h>
#include <IRremoteInt.h>
#include <ir_Lego_PF_BitStreamEncoder.h>

//Librería AdaFruit para los motores
#include <AFMotor.h>

//Librería de servomotores
#include <Servo.h>

//Inicializando y estableciendo datos del control remoto
int receiver = 45; //<--- Pin 9 ('S' para el infrarrojo) [PWM]
IRrecv irrecv(receiver);
decode_results results;

//Entradas DIN, CS Y CLK con sus respectivos pines
int DIN = 38; //<--- Pin 38
int CS =  40; //<--- Pin 40
int CLK = 42; //<--- Pin 42

//Establecimiento de dirección de motores
AF_DCMotor motor_r(1);
AF_DCMotor motor_l(2);

//Adjuntamiento de servomotores
Servo arm_l; //<---- brazo izquierdo
Servo arm_r; //<---- brazo derecho
int ang;
int increasing = 1;

//Variables para sensor de ultrasonido HC-SR04
const short echo = 52;
const short trig = 50;
long duration, distance;
boolean state_DisMeasurement = false;

//Variables de tiempo [ADVERTENCIA: DEBEN SER SIEMPRE VARIABLES TIPO 'unsigned long']____________________________________________________
/*                                                                                                                                       |*/
  unsigned long global_previous_millis = 0;           /* El tiempo que hay que esperar. */  /*                                           |*/
  unsigned long global_current_millis;                /* desde acá indicamos el límite. */  /* Estas variables se asignan como globales  |*/
  unsigned long global_direct_number;                 /* Al igual que el "delay()", acá */  /* ya que hay funciones que no pueden tomar  |*/
                                                      /* lo asignamos en milisegundos.  */  /* variables provenientes como locales de    |*/
  unsigned long global_ultrasonic_previous_millis = 0;/* Esta variable se va a usar     */  /* "void loop()". De esta manera, al ser     |*/
  unsigned long global_ultrasonic_current_millis;     /* cuánto tiempo ha pasado        */  /* (como es el caso de "void printSurprise()"|*/
  unsigned long global_ultrasonic_direct_number;      /* desde "que se ejecutó          */  /* acceder a éstas.                          |*/
                                                      /* una orden". También sirve      */  /*                                           |*/
  boolean state_blinking = false;                     /* para "volcado" o "rollover".   */  /*                                           |*/
      /*↓↓↓ Disparadores ↓↓↓*/                                                              /*                                           |*/
  boolean state_animated_surp = false;                                                      /*                                           |*/
  boolean state_animated_blink = true;                                                      /*                                           |*/
  boolean state_animated_surp_cancel = false;                                               /*                                           |*/
  boolean state_animated_leave = false;                                                     /*                                           |*/
  boolean state_animated_push = false;                                                      /*                                           |*/
  boolean state_animated_push_cancel = false;                                               /*                                           |*/
  boolean state_suspension = false;
//_______________________________________________________________________________________________________________________________________|

//___________________Secuencia y posicionamiento de los leds____________________

/*
    *En ellas se guardan por variables de tipo 'Byte' el 
    *valor posicional de cada led encendido y apagado. Para 
    *poder programar correctamente y de manera más sencilla 
    *esta parte, se recomienda usar el software PixelToMatrix.exe
*/

//Secuencia Blinking. 
byte blink_0[8]=     {0x00,0x00,0x00,0x7E,0x7E,0x00,0x00,0x00}; // -
byte blink_1[8]=     {0x00,0x18,0x18,0x18,0x18,0x18,0x18,0x00}; // |
boolean state_blink;

//Secuencia printSuprise.
byte surp_0[8]=      {0x00,0x1C,0x22,0x41,0x41,0x41,0x22,0x1C};
byte surp_1[8]=      {0x00,0x38,0x44,0x82,0x82,0x82,0x44,0x38};
byte surp_2[8]=      {0x38,0x44,0x82,0x82,0x82,0x44,0x38,0x00};
byte surp_3[8]=      {0x1C,0x22,0x41,0x41,0x41,0x22,0x1C,0x00};

//Secuencia eva
byte eva_0[8]=       {0x00,0x00,0x38,0x7C,0x7E,0x3C,0x00,0x00};

//Secuencia leave
byte leave[8]=       {0xC3,0xE7,0x7E,0x3C,0x3C,0x7E,0xE7,0xC3};

//Secuencia de números principales
byte one[8]=         {0x18,0x38,0x38,0x18,0x18,0x18,0x18,0x18};
byte two[8]=         {0x3C,0x7E,0x66,0x0E,0x1C,0x38,0x7E,0x7E};
byte three[8]=       {0x3C,0x7E,0x66,0x0C,0x0C,0x66,0x7E,0x38};

//Secuencia de ojos diagonales
byte left_up[8]=     {0xC0,0xC0,0xC0,0xC0,0xC0,0xC0,0x00,0x00};
byte right_up[8]=    {0x03,0x03,0x03,0x03,0x03,0x03,0x00,0x00};
byte left_down[8]=   {0x00,0x00,0xC0,0xC0,0xC0,0xC0,0xC0,0xC0};
byte right_down[8]=  {0x00,0x00,0x03,0x03,0x03,0x03,0x03,0x03};

//Ojos direccionales
byte right[8]=       {0x00,0x03,0x03,0x03,0x03,0x03,0x03,0x00};
byte left[8]=        {0x00,0xC0,0xC0,0xC0,0xC0,0xC0,0xC0,0x00};
byte up[8]=          {0x18,0x18,0x18,0x18,0x18,0x18,0x00,0x00};
byte down[8]=        {0x00,0x00,0x18,0x18,0x18,0x18,0x18,0x18};

//Advertencia de denegación
byte reject[8]=      {0x18,0x18,0x18,0x18,0x18,0x00,0x18,0x18};

//Secuencia push (izquierda)
byte push_l_0[8]=    {0x00,0x70,0x0C,0x03,0x03,0x0C,0x70,0x00}; // < →
byte push_l_1[8]=    {0x70,0x0C,0x03,0x03,0x0C,0x70,0x00,0x00}; // < ↑
byte push_l_2[8]=    {0x00,0x70,0x0C,0x03,0x03,0x0C,0x70,0x00}; // < →
byte push_l_3[8]=    {0x00,0x00,0x70,0x0C,0x03,0x03,0x0C,0x70}; // < ↓

//Secuencia push (derecha)
byte push_r_0[8]=    {0x00,0x0E,0x30,0xC0,0xC0,0x30,0x0E,0x00}; // > ←
byte push_r_1[8]=    {0x00,0x00,0x0E,0x30,0xC0,0xC0,0x30,0x0E}; // > ↓
byte push_r_2[8]=    {0x00,0x0E,0x30,0xC0,0xC0,0x30,0x0E,0x00}; // > ←
byte push_r_3[8]=    {0x0E,0x30,0xC0,0xC0,0x30,0x0E,0x00,0x00}; // > ↑

//___________________Secuencia y posicionamiento de los leds____________________


//______________Instancia para el control del dispositivo MAX7219____________

/*
    *la variable 'lc' equivale a la instancia LedControl 
    *(la que nos ofrece la librería LedControl.h). 
    *LedControl tiene cuatro argumentos los cuales:
    *
    *LedControl(DIN,CLK,CS,Dispositivos)
    *            ^   ^  ^       ^
    *            |   |  |       |
    *  Entrada de Datos-Serial  |
    *                |  |       |
    *     Entrada de Reloj-Serial
    *                   |       |
    *       Entrada de Carga de Datos
    *                           |
    * Número de dispositivos a manipular (se estima un límite de hasta 8
*/
LedControl lc=LedControl(DIN,CLK,CS,2);

//______________Instancia para el control del dispositivo MAX7219____________

//___________________________Algunas variables______________________________

//Para el loop
int n = 10; //<-- n times of repeat
int i = 0;  //<-- 0 by default
//int val = Serial.read() - '0';

//__________________________________________________________________________


void setup(){
//___________________________Crear objetos serial_____________________________
 Serial.begin(9600);         /* Serial.begin(9600) establece una conexión
                              * serial.*/
                             /* Serial.begin(velocidad)
                              *                  ^
                              *                  |
                              *            Tasa de bits*/
//___________________________Crear objetos serial_____________________________

//____________Establecer sensor infrarrojo como entrada disponible___________
 irrecv.enableIRIn();        /* irrecv.enableIRIn Permite recibir luz 
                              * infrarroja codificada.*/
//____________Establecer sensor infrarrojo como entrada disponible___________

//______________________________Modo Ahorro_________________________________
 lc.shutdown(0,false);       /* lc.shutdown(addr,b) esta función determina
                              * si la matriz de leds entra en modo ahorro
                              * de energía, o no.*/
 lc.shutdown(1,false);       /* lc.shutdown(dirección,estado);
                              *                ^       ^
                              *                |       |
                              *  número de dispositivo (matriz)
                              *                        |
                              *                  estado (boolean)*/
//______________________________Modo Ahorro_________________________________

//__________________________________Brillo__________________________________
 lc.setIntensity(0,1);       /* lc.setIntesity(addr,brightness) esta función
                              * determina la luminosidad que el MAX7219 le
                              * proveerá a los leds (la matriz)*/
 lc.setIntensity(1,1);       /* lc.setIntensity(dirección,intensidad);
                              *                     ^         ^
                              *                     |         |
                              *       número de dispositivo (matriz)
                              *                               |
                              *                    intensidad de la luz (int)*/
//__________________________________Brillo__________________________________

//_____________________________Limpiar Pantalla______________________________
 lc.clearDisplay(0);         /* lc.clearDisplay(addr) esta función realiza
                              * una limpieza de la matriz, apagando todos los
                              * leds que se encuentren encendidos*/
 lc.clearDisplay(1);         /* lc.clearDisplay(dirección);
                              *                     ^
                              *                     |
                              *       número de dispositivo (matriz)           
                               */
//_____________________________Limpiar Pantalla______________________________

//___________________________Velocidad de motores____________________________
 motor_r.setSpeed(250);
 motor_l.setSpeed(250);
//___________________________Velocidad de motores____________________________

//______________________________Servo motores________________________________
 arm_l.attach(9);
 arm_r.attach(10);

}

//__________________________Direcciones de leds______________________________

//Cosas a tener en cuenta
/*
     *La función 'setRow' toma 3 
     *argumentos. La primera es  
     *la ya conocida dirección 
     *de nuestro dispositivo. La  
     *segunda es la fila que se  
     *va a actualizar y la tercera 
     *el valor de esta fila.
     *
     *        ___Sintaxis___ 
     *        
     *            dirección,fila,valor
     *                |      |     |
     *                V      V     V
     *   lc.setRow(Address,Row,Value);
*/

//Pinta los leds a la dirección de la primer matriz
void print_l(byte l[])
{
  int i = 0;
  for (i = 0; i < 8; i++)
  {
    lc.setRow(0,i,l[i]);
  }
}
//Pinta los leds a la dirección de la segunda matriz
void print_r(byte r[])
{
  int i = 0;
  for (i = 0; i < 8; i++)
  {
    lc.setRow(1,i,r[i]);
  }
}
//Pinta los leds por igual a ambas matrices
void print_lr(byte lr[])
{
  int i = 0;
  for (i = 0; i < 8; i++)
  {
    lc.setRow(0,i,lr[i]);
    lc.setRow(1,i,lr[i]);
  }
}
////__________________________Direcciones de leds______________________________

//___________________________Animaciones___________________________

//Sorpresa
void printSurprise()
{
  //Variables [de tiempo] locales
  unsigned long scope_interval = 400;
  
  //Variables compuestas de globales
    /*VACÍO*/
    
  //Ciclo condicional
  if (state_animated_surp == true)
  {
    
    if (global_direct_number >= scope_interval)
      {
        //Vuelve a hacer el cuenteo
        global_previous_millis = millis();
      }
      else
      {
        switch (global_direct_number)
          {
          
            case 0:
              print_lr(surp_0); //  _o
              Serial.print("SURP: ");
              Serial.println(global_direct_number);
            break;

            case 100:
              print_lr(surp_1); //  o_
              Serial.print("SURP: ");
              Serial.println(global_direct_number);
            break;

            case 200:
              print_lr(surp_2); //  O_
              Serial.print("SURP: ");
              Serial.println(global_direct_number);
            break;

            case 300:
              print_lr(surp_3); //  _O
              Serial.print("SURP: ");
              Serial.println(global_direct_number);
            break;

            case 400:
              Serial.println("This can not be read");
            break;
          }
      }
  }
}

//
void blinking()
{
  //Variables [de tiempo] locales
  unsigned long scope_interval = 6000;
  
  //Ciclo condicional
  if (state_animated_blink == true)
  {
    //Comparación    
    if (global_direct_number >= scope_interval)
    {
      //Vuelve a hacer el cuenteo
      global_previous_millis = millis();
    }
    else
    {
      switch (global_direct_number)
      {
        case 0:
          Serial.print("BLINK: ");
          Serial.println(global_direct_number);
          state_blinking = true;
          cond_blink();
        break;

        case 2000:
          Serial.print("BLINK: ");
          Serial.println(global_direct_number);
          state_blinking = !state_blinking;
          cond_blink();
        break;

        case 2100:
          Serial.print("BLINK: ");
          Serial.println(global_direct_number);
          state_blinking = !state_blinking;
          cond_blink();
        break;

        case 2300:
          Serial.print("BLINK: ");
          Serial.println(global_direct_number);
          state_blinking = !state_blinking;
          cond_blink();
        break;

        case 2400:
          Serial.print("BLINK: ");
          Serial.println(global_direct_number);
          state_blinking = !state_blinking;
          cond_blink();
        break;
      }
    }
  }
}

//Condicional de cambio de estado de blink
void cond_blink()
{
  //Ciclo condicional de cambio de estado
  if (state_blinking == true)
    {
      print_lr(blink_1);
    }
  else //if (state_blinking == false)
    {
      print_lr(blink_0);
    }
}

//Denegación
void cancel()
{
  //Variables [de tiempo] locales
  unsigned long scope_interval = 1000;

  //Ciclo condicional
  if (state_animated_surp_cancel == true)
  {
    if (global_direct_number >= scope_interval)
    {
      //Finalización del ciclo cancel()
      state_animated_surp_cancel = false;
      
      //Reactivación de printSurprise()
      state_animated_surp = true;
    }
    else
    {
      //Desactivación temporal de printSurprise() y cualquier otro estado animado
      state_animated_surp = false;
      state_animated_blink = false;
      
      switch (global_direct_number)
      {
        case 100:
          print_l(reject);
          lc.clearDisplay(1);
          Serial.print("CANCEL: ");
          Serial.println(global_direct_number);
        break;

        case 200:
          lc.clearDisplay(0);
          print_r(reject);
          Serial.print("CANCEL: ");
          Serial.println(global_direct_number);
        break;

        case 300:
          print_lr(reject);
          lc.clearDisplay(1);
          Serial.print("CANCEL: ");
          Serial.println(global_direct_number);
        break;

        case 400:
          lc.clearDisplay(0);
          print_r(reject);
          Serial.print("CANCEL: ");
          Serial.println(global_direct_number);
        break;

        case 500:
          print_lr(reject);
          lc.clearDisplay(1);
          Serial.print("CANCEL: ");
          Serial.println(global_direct_number);
        break;

        case 600:
          print_l(reject);
          lc.clearDisplay(1);
          Serial.print("CANCEL: ");
          Serial.println(global_direct_number);
        break;

        case 700:
          lc.clearDisplay(0);
          print_r(reject);
          Serial.print("CANCEL: ");
          Serial.println(global_direct_number);
        break;

        case 800:
          print_lr(reject);
          lc.clearDisplay(1);
          Serial.print("CANCEL: ");
          Serial.println(global_direct_number);
        break;

        case 900:
          lc.clearDisplay(0);
          print_r(reject);
          Serial.print("CANCEL: ");
          Serial.println(global_direct_number);
        break;

        case 1000:
          print_lr(reject);
          lc.clearDisplay(1);
          Serial.print("CANCEL: ");
          Serial.println(global_direct_number);
        break;
      }
    }
  }
}

//Denegación #2
void push_cancel()
{
  //Variables [de tiempo] locales
  unsigned long scope_interval = 1000;

  //Ciclo condicional
  if (state_animated_push_cancel == true)
  {
    if (global_direct_number >= scope_interval)
    {
      //Finalización del ciclo cancel()
      state_animated_push_cancel = false;
      
      //Reactivación de printPush()
      state_animated_push = true;
    }
    else
    {
      //Desactivación temporal de printSurprise() y cualquier otro estado animado
      state_animated_push = false;
      state_animated_blink = false;
      
      switch (global_direct_number)
      {
        case 100:
          print_l(reject);
          lc.clearDisplay(1);
          Serial.print("CANCEL: ");
          Serial.println(global_direct_number);
        break;

        case 200:
          lc.clearDisplay(0);
          print_r(reject);
          Serial.print("CANCEL: ");
          Serial.println(global_direct_number);
        break;

        case 300:
          print_lr(reject);
          lc.clearDisplay(1);
          Serial.print("CANCEL: ");
          Serial.println(global_direct_number);
        break;

        case 400:
          lc.clearDisplay(0);
          print_r(reject);
          Serial.print("CANCEL: ");
          Serial.println(global_direct_number);
        break;

        case 500:
          print_lr(reject);
          lc.clearDisplay(1);
          Serial.print("CANCEL: ");
          Serial.println(global_direct_number);
        break;

        case 600:
          print_l(reject);
          lc.clearDisplay(1);
          Serial.print("CANCEL: ");
          Serial.println(global_direct_number);
        break;

        case 700:
          lc.clearDisplay(0);
          print_r(reject);
          Serial.print("CANCEL: ");
          Serial.println(global_direct_number);
        break;

        case 800:
          print_lr(reject);
          lc.clearDisplay(1);
          Serial.print("CANCEL: ");
          Serial.println(global_direct_number);
        break;

        case 900:
          lc.clearDisplay(0);
          print_r(reject);
          Serial.print("CANCEL: ");
          Serial.println(global_direct_number);
        break;

        case 1000:
          print_lr(reject);
          lc.clearDisplay(1);
          Serial.print("CANCEL: ");
          Serial.println(global_direct_number);
        break;
      }
    }
  }
}

//Salida
void leaving()
{
  //Variables [de tiempo] locales
  unsigned long scope_interval = 1000;

  //Ciclo condicional
  if (state_animated_leave == true)
  {
    if (global_direct_number >= scope_interval)
    {
      //Finalización del ciclo cancel()
      state_animated_leave = false;
      state_animated_blink = true;
    }
    else
    {
      //Desactivación temporal de printSurprise() y cualquier otro estado animado
      state_animated_surp = false;
      state_animated_blink = false;
      
      switch (global_direct_number)
      {
        case 100:
          print_l(leave);
          lc.clearDisplay(1);
          Serial.print("LEAVE: ");
          Serial.println(global_direct_number);
        break;

        case 200:
          lc.clearDisplay(0);
          print_r(leave);
          Serial.print("LEAVE: ");
          Serial.println(global_direct_number);
        break;

        case 300:
          print_lr(leave);
          lc.clearDisplay(1);
          Serial.print("LEAVE: ");
          Serial.println(global_direct_number);
        break;

        case 400:
          lc.clearDisplay(0);
          print_r(leave);
          Serial.print("LEAVE: ");
          Serial.println(global_direct_number);
        break;

        case 500:
          print_lr(leave);
          lc.clearDisplay(1);
          Serial.print("LEAVE: ");
          Serial.println(global_direct_number);
        break;

        case 600:
          print_l(leave);
          lc.clearDisplay(1);
          Serial.print("LEAVE: ");
          Serial.println(global_direct_number);
        break;

        case 700:
          lc.clearDisplay(0);
          print_r(leave);
          Serial.print("LEAVE: ");
          Serial.println(global_direct_number);
        break;

        case 800:
          print_lr(leave);
          lc.clearDisplay(1);
          Serial.print("LEAVE: ");
          Serial.println(global_direct_number);
        break;

        case 900:
          lc.clearDisplay(0);
          lc.clearDisplay(1);
          Serial.print("LEAVE: ");
          Serial.println(global_direct_number);
        break;

        case 1000:
          print_lr(leave);
          lc.clearDisplay(1);
          Serial.print("LEAVE: ");
          Serial.println(global_direct_number);
        break;
      }
    }
  }
}

//Sorpresa
void printPush()
{
  //Variables [de tiempo] locales
  unsigned long scope_interval = 400;
  
  //Variables compuestas de globales
    /*VACÍO*/
    
  //Ciclo condicional
  if (state_animated_push == true)
  {
    
    if (global_direct_number >= scope_interval)
      {
        //Vuelve a hacer el cuenteo
        global_previous_millis = millis();
      }
      else
      {
        //Desactivación temporal de algunas funciones
        state_animated_blink = false;
        
        switch (global_direct_number)
          {
          
            case 0:
              print_l(push_l_0);
              print_r(push_r_0);
              Serial.print("PUSH: ");
              Serial.println(global_direct_number);
            break;

            case 100:
              print_l(push_l_1);
              print_r(push_r_1);
              Serial.print("PUSH: ");
              Serial.println(global_direct_number);
            break;

            case 200:
              print_l(push_l_2);
              print_r(push_r_2);
              Serial.print("PUSH: ");
              Serial.println(global_direct_number);
            break;

            case 300:
              print_l(push_l_3);
              print_r(push_r_3);
              Serial.print("PUSH: ");
              Serial.println(global_direct_number);
            break;

            case 400:
              Serial.println("This can not be read");
            break;
          }
      }
   }
}

//Medición de distancia
void DisMeasurement()
{
  //Variables locales
  short interval = 400;

  //Condicional
  if (global_ultrasonic_direct_number >= interval)
  {
    if (state_DisMeasurement == true)
    {
    
      digitalWrite(trig, LOW);
      delayMicroseconds(2);
      digitalWrite(trig, HIGH);   // Genera el pulso de triger por 10ms
      delayMicroseconds(10);
      digitalWrite(trig, LOW);
  
      duration = pulseIn(echo, HIGH);

      /* Normalmente se intenta obtener los metros
       * pero en este caso, al dividirse en dos,
       * terminaremos sacando los centímetros como
       * es debido
       *                                           
       *              |
       *              |
       *              v                           */
      distance = (duration/2) / 29;             //<--- '29' son los microsegundos
  
      if (distance >= 500 || distance <= 0)     // Si la distancia es mayor a 500cm o menor a 0cm
      {   
        Serial.println("---");                  // No mide nada
        motor_r.run(RELEASE);
        motor_l.run(RELEASE);
      }
      else 
      {
        Serial.print(distance);                 // Envia el valor de la distancia por el puerto serial
        Serial.println("cm");                   // Le coloca a la distancia la unidad de medida 'centímetros'
        digitalWrite(LED_BUILTIN, LOW);         // En bajo el pin 13
        motor_r.run(FORWARD);
        motor_l.run(FORWARD);
      }
  
      if (distance <= 10 && distance >= 1)
      {
        digitalWrite(LED_BUILTIN, HIGH);        // En alto el pin 13 si la distancia es menor a 10cm
        Serial.println("Alarma.......");        // Envia la palabra Alarma por el puerto serial
        state_animated_push = true;
        motor_r.run(FORWARD);
        motor_l.run(BACKWARD);
        
      }
      else
      {
        state_animated_push = false;
        print_lr(blink_1);
      }
      //delay(100);                             // espera 400ms para que se logre ver la distancia en la consola (desactivado)
      global_ultrasonic_previous_millis = millis();        //Tempo-Reset a toda la operación condicional
    }
  }
}

//Encendido/suspendido
void suspension()
{
  if (state_suspension == false)
  {
    lc.shutdown(0,false);
    lc.shutdown(1,false);
    boolean state_animated_blink = true;
    motor_r.setSpeed(150);
    motor_l.setSpeed(150);
  }
  else
  {
    lc.shutdown(0,true);
    lc.shutdown(1,true);
    motor_r.setSpeed(0);
    motor_l.setSpeed(0);
  }
}

//___________________________Animaciones___________________________

void loop()
{
   //Variables locales
    //Estas variables almacenaran los valores que indican constantemente
    global_current_millis = millis();
    global_direct_number = global_current_millis - global_previous_millis;

    global_ultrasonic_current_millis = millis();
    global_ultrasonic_direct_number = global_ultrasonic_current_millis - global_ultrasonic_previous_millis;

    //Funciones personalizadas listas para ejecutarse (más abajo se activan)
    blinking();
    printSurprise();
    printPush();
    cancel();
    push_cancel();
    leaving();
    DisMeasurement();
    
    //Serial.println(local_state);
    //Serial.println(global_cancel_direct_number);
    /*El arduino espera hasta recibir algo desde el infrarrojo
     * (alimentación USB y accionar a control remoto).
     * irrecv.decode(&results) se encargará de decodificar la
     * comunicación infrarroja codificada para poder ejecutarla
     */
    if (irrecv.decode(&results)){ 

    /*Lee la entrada de datos.
     *Por defecto devuelve una
     *respuesta del código ascii,
     *excepto en esta línea ['0']
     *Por el momento está desactivada
     *ya que se puede controlar la
     *matriz de leds directamente
     *desde el control remoto
     */
    //int val = Serial.read() - '0';


    switch (results.value)
    {
      //CASE 1
      case 0x00FFA25D:
        Serial.println("Tecla: 1");
        //Condicional surp
        if (state_animated_surp == true)
        {
          state_animated_surp_cancel = true;
          break;
        }
        //Condicional push
        else if (state_animated_push == true)
        {
          state_animated_push_cancel = true;
          break;
        }
        //Condicional of false
        else if (state_animated_surp_cancel == false || state_animated_push_cancel == false)
        {
          state_animated_blink = false;
//↓↓↓ Acá va el código que debe ejecutarse con la tecla 1 ↓↓↓
          print_lr(one);
          lc.setIntensity(0,1);
          lc.setIntensity(1,1);
          motor_r.setSpeed(90);
          motor_l.setSpeed(90);
        }
      break;

      //CASE 2
      case 0x00FF629D:
        Serial.println("Tecla: 2");
        //Condicional surp
        if (state_animated_surp == true)
        {
          state_animated_surp_cancel = true;
          break;
        }
        //Condicional push
        else if (state_animated_push == true)
        {
          state_animated_push_cancel = true;
          break;
        }
        //Condicional of false
        else if (state_animated_surp_cancel == false || state_animated_push_cancel == false)
        {
          state_animated_blink = false;
//↓↓↓ Acá va el código que debe ejecutarse con la tecla 2 ↓↓↓
          print_lr(two);
          lc.setIntensity(0,2);
          lc.setIntensity(1,2);
          motor_r.setSpeed(150);
          motor_l.setSpeed(150);
        }
      break;

      //CASE 3
      case 0x00FFE21D:
        Serial.println("Tecla: 3");
        //Condicional surp
        if (state_animated_surp == true)
        {
          state_animated_surp_cancel = true;
          break;
        }
        //Condicional push
        else if (state_animated_push == true)
        {
          state_animated_push_cancel = true;
          break;
        }
        //Condicional of false
        else if (state_animated_surp_cancel == false || state_animated_push_cancel == false)
        {
          state_animated_blink = false;
//↓↓↓ Acá va el código que debe ejecutarse con la tecla 3 ↓↓↓
          print_lr(three);
          lc.setIntensity(0,5);
          lc.setIntensity(1,5);
          motor_r.setSpeed(250);
          motor_l.setSpeed(250);
        }
      break;

      //CASE 4
      case 0x00FF22DD:
        Serial.println("Tecla: 4");
        //Condicional
        //Condicional surp
        if (state_animated_surp == true)
        {
          state_animated_surp_cancel = true;
          break;
        }
        //Condicional push
        else if (state_animated_push == true)
        {
          state_animated_push_cancel = true;
          break;
        }
        //Condicional of false
        else if (state_animated_surp_cancel == false || state_animated_push_cancel == false)
        {
          state_animated_blink = false;
//↓↓↓ Acá va el código que debe ejecutarse con la tecla 4 ↓↓↓
          print_lr(left_up);
          arm_l.write(90);
          arm_r.write(0);
          //HERE
        }
      break;

      //CASE 5
      case 0x00FF02FD:
        Serial.println("Tecla: 5");
        //Condicional
        if (state_animated_push == true)
        {
          state_animated_push_cancel = true;
          break;
        }
        //state_animated_surp = true;
        //state_animated_blink = false;
        print_lr(up);
        arm_l.write(90);
        arm_r.write(90);
      break;

      //CASE 6
      case 0x00FFC23D:
        Serial.println("Tecla: 6");
        //Condicional surp
        if (state_animated_surp == true)
        {
          state_animated_surp_cancel = true;
          break;
        }
        //Condicional push
        else if (state_animated_push == true)
        {
          state_animated_push_cancel = true;
          break;
        }
        //Condicional of false
        else if (state_animated_surp_cancel == false || state_animated_push_cancel == false)
        {
          state_animated_blink = false;
//↓↓↓ Acá va el código que debe ejecutarse con la tecla 6 ↓↓↓
          print_lr(right_up);
          arm_l.write(0);
          arm_r.write(90);
        }
      break;

      //CASE 7
      case 0x00FFE01F:
        Serial.println("Tecla: 7");
        //Condicional surp
        if (state_animated_surp == true)
        {
          state_animated_surp_cancel = true;
          break;
        }
        //Condicional push
        else if (state_animated_push == true)
        {
          state_animated_push_cancel = true;
          break;
        }
        //Condicional of false
        else if (state_animated_surp_cancel == false || state_animated_push_cancel == false)
        {
          state_animated_blink = false;
//↓↓↓ Acá va el código que debe ejecutarse con la tecla 7 ↓↓↓
          print_lr(left_down);
        }
      break;
                  
      //CASE 8
      case 0x00FFA857:
        Serial.println("Tecla: 8");
        //Condicional
        if (state_animated_surp == true)
        {
          state_animated_surp_cancel = true;
          break;
        }
//↓↓↓ Acá va el código que debe ejecutarse con la tecla 8 ↓↓↓
          print_lr(down);
      break;

      //CASE 9
      case 0x00FF906F:
        Serial.println("Tecla: 9");
        //Condicional surp
        if (state_animated_surp == true)
        {
          state_animated_surp_cancel = true;
          break;
        }
        //Condicional push
        else if (state_animated_push == true)
        {
          state_animated_push_cancel = true;
          break;
        }
        //Condicional of false
        else if (state_animated_surp_cancel == false || state_animated_push_cancel == false)
        {
          state_animated_blink = false;
//↓↓↓ Acá va el código que debe ejecutarse con la tecla 9 ↓↓↓
          print_lr(right_down);
        }
      break;

      //CASE 0
      case 0x00FF9867:
        Serial.println("Tecla: 0");
        //Cancelar procesos
        if (state_animated_surp == true || state_animated_surp_cancel == true || /*state_animated_blink == false ||*/ state_animated_push == true || state_animated_push_cancel == true)
        {
          state_animated_surp = false;
          state_animated_surp_cancel = false;
          state_animated_push = false;
          state_animated_push_cancel = false;
          state_animated_leave = true;
        }
      break;

      //CASE #
      case 0x0FFB04F:
        Serial.println("Tecla: #");
        //Condicional
        //Condicional surp
        if (state_animated_surp == true)
        {
          state_animated_surp_cancel = true;
          break;
        }
        //Condicional push
        else if (state_animated_push == true)
        {
          state_animated_push_cancel = true;
          break;
        }
        //Condicional of false
        else if (state_animated_surp_cancel == false || state_animated_push_cancel == false)
        {
          state_animated_blink = false;
//↓↓↓ Acá va el código que debe ejecutarse con la tecla # ↓↓↓
          state_DisMeasurement = !state_DisMeasurement;
          print_lr(blink_1);
        }
      break;

      //CASE *
      case 0x00FF6897:
        Serial.println("Tecla: *");
        //Condicional
        //Condicional surp
        if (state_animated_surp == true)
        {
          state_animated_surp_cancel = true;
          break;
        }
        //Condicional push
        else if (state_animated_push == true)
        {
          state_animated_push_cancel = true;
          break;
        }
        //Condicional of false
        else if (state_animated_surp_cancel == false || state_animated_push_cancel == false)
        {
          state_animated_blink = false;
//↓↓↓ Acá va el código que debe ejecutarse con la tecla * ↓↓↓
          state_suspension = !state_suspension;
          suspension();
        }
      break;

      //CASE ▲
      case 0x00FF18E7:
        Serial.println("Tecla: ▲");
        //Condicional
        //Condicional surp
        if (state_animated_surp == true)
        {
          state_animated_surp_cancel = true;
          break;
        }
        //Condicional push
        else if (state_animated_push == true)
        {
          state_animated_push_cancel = true;
          break;
        }
        //Condicional of false
        else if (state_animated_surp_cancel == false || state_animated_push_cancel == false)
        {
          state_animated_blink = false;
//↓↓↓ Acá va el código que debe ejecutarse con la tecla ▲ ↓↓↓
          print_lr(blink_1);
          motor_r.run(FORWARD);
          motor_l.run(FORWARD);
          
          /*if (increasing)
          {
            ang++;
          }
          else
          {
            ang--;
          }
          
          Serial.println(ang);

          if (ang<=1) 
          {
            ang = 1;
            increasing = 1;
            delay(200);
          }

          if (ang>=180) 
          {
            ang = 180;
            increasing = 0;
            delay(200);
          }

          servo_izq.write(ang);
          servo_der.write(ang);*/
        }
      break;

      //CASE ▼
      case 0x00FF4AB5:
        Serial.println("Tecla: ▼");
        //Condicional
        //Condicional surp
        if (state_animated_surp == true)
        {
          state_animated_surp_cancel = true;
          break;
        }
        //Condicional push
        else if (state_animated_push == true)
        {
          state_animated_push_cancel = true;
          break;
        }
        //Condicional of false
        else if (state_animated_surp_cancel == false || state_animated_push_cancel == false)
        {
          state_animated_blink = false;
//↓↓↓ Acá va el código que debe ejecutarse con la tecla ▼ ↓↓↓
          print_lr(blink_1);
          motor_r.run(BACKWARD);
          motor_l.run(BACKWARD);
        }
      break;

      //CASE ◄
      case 0x00FF10EF:
        Serial.println("Tecla: ◄");
        //Condicional
        //Condicional surp
        if (state_animated_surp == true)
        {
          state_animated_surp_cancel = true;
          break;
        }
        //Condicional push
        else if (state_animated_push == true)
        {
          state_animated_push_cancel = true;
          break;
        }
        //Condicional of false
        else if (state_animated_surp_cancel == false || state_animated_push_cancel == false)
        {
          state_animated_blink = false;
//↓↓↓ Acá va el código que debe ejecutarse con la tecla ◄ ↓↓↓
          print_lr(left);
          motor_r.run(BACKWARD);
          motor_l.run(FORWARD);
        }
      break;

      //CASE ►
      case 0x00FF5AA5:
        Serial.println("Tecla: ►");
        //Condicional
        //Condicional surp
        if (state_animated_surp == true)
        {
          state_animated_surp_cancel = true;
          break;
        }
        //Condicional push
        else if (state_animated_push == true)
        {
          state_animated_push_cancel = true;
          break;
        }
        //Condicional of false
        else if (state_animated_surp_cancel == false || state_animated_push_cancel == false)
        {
          state_animated_blink = false;
//↓↓↓ Acá va el código que debe ejecutarse con la tecla ► ↓↓↓
          print_lr(right);
          motor_r.run(FORWARD);
          motor_l.run(BACKWARD);
        }
      break;

      //CASE ok
      case 0x00FF38C7:
        Serial.println("Tecla: OK");
        //Condicional surp
        if (state_animated_surp == true)
        {
          state_animated_surp_cancel = true;
          break;
        }
        //Condicional push
        else if (state_animated_push == true)
        {
          state_animated_push_cancel = true;
          break;
        }
        //Condicional of false
        else if (state_animated_surp_cancel == false || state_animated_push_cancel == false)
        {

//↓↓↓ Acá va el código que debe ejecutarse con la tecla OK ↓↓↓
          state_animated_blink = true;
          motor_r.run(RELEASE);
          motor_l.run(RELEASE);
          
        }        
      break;            
    }
    Serial.flush();
    irrecv.resume();
    
  }
}
