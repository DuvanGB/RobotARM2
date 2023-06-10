 // interrup 20,21
#include <PID_v1.h>    // Librería PID de Brett Beauregard: https://playground.arduino.cc/Code/PIDLibrary

// ********************************************** I/O *****************************************************************************
// ******************************************** Motor 1
const byte    encA = 2;              // Entrada de la señal A del encoder.
const byte    PWMA = 4;              // Salida PWM al primer pin.
const byte    PWMB = 5;              // Salida PWM al segundo pin.
double        ki1 = 0.0154, kp1 = 0.35, kd1 = 0.129;
double        Sensi1 = 7.222222222;
double        compensacion1 = 45.4563;
volatile long contador1_1 = 0, contador1_2 = 0;           // Guarda los pulsos del encoder 1.
bool          flagM1_1, flagM1_2, flagM1_3;
// ******************************************** Motor 2
const byte    encB = 18;              // Entrada de la señal A del encoder.
const byte    encC = 19;              // Entrada de la señal B del encoder.
const byte    PWMC = 6;              // Salida PWM al primer pin.
const byte    PWMD = 7;              // Salida PWM al segundo pin.
double        ki2 = 3.2, kp2 = 2.1, kd2 = 0.1956;
double        Sensi2 = 0.421;
volatile long contador2 = 0;           // Guarda los pulsos del encoder 2.
boolean       Direccion2;
byte          Encoder_C1Last2;
byte          ant2 = 0, act2 = 0;       // Sólo se utiliza los dos primeros bits de estas variables y servirán para decodificar el encoder2 (ant2 = anterior, act2 = actual).
// ******************************************** Motor 3
const byte    encD = 3;              // Entrada de la señal A del encoder.
const byte    PWME = 8;              // Salida PWM al primer pin.
const byte    PWMF = 9;              // Salida PWM al segundo pin.
const byte    Iman = 10;             
double        ki3 = 2.154, kp3 = 2.971, kd3 = 0.10945;
double        Sensi3 = 1.7777777777777777777777777777778;
double        compensacion3 = 45.4563;
volatile long contador3_1 = 0, contador3_2 = 0;           // Guarda los pulsos del encoder 3.
bool          flagM3_1, flagM3_2, flagM3_3;
// ************************************************ Variables PID *****************************************************************
double        Setpoint = 0.0, Input = 0.0, Output = 0.0;  // Setpoint = Posición designada; Input = Posición del motor; Output = Tensión de salida para el motor.
double        kp = 0.0, ki = 0.0, kd = 0.0;               // Constante proporcional, integral y derivativa.
double        outMax = 0.0, outMin = 0.0;                 // Límites para no sobrepasar la resolución del PWM.
double        Grados = 0.0, Respuesta = 0.0;
double        Sensi = 0.0;
// **************************************************** Otras Variables ***********************************************************
volatile long contador = 0;           // Guarda los pulsos de los encoders.
byte          cmd = 0;                // Byte para la comunicación serie (cmd = comando).
byte          banderas = 0;           // Motor a mover.
unsigned int  tmp = 0;                // Tiempo de muestreo.
// ********************************************************************************************************************************

PID myPID(&Input, &Output, &Setpoint, 0.0, 0.0, 0.0, DIRECT); // Parámetros y configuración para invocar la librería.

void setup()                         
{
  Serial.begin(2000000);    
         
  pinMode(PWMA, OUTPUT);              // Declara las dos salidas PWM para el control del motor.
  digitalWrite(PWMA, LOW);            // Y ambas salidas las inicializa a cero.
  pinMode(PWMB, OUTPUT);              
  digitalWrite(PWMB, LOW);           
  pinMode(PWMC, OUTPUT);           
  digitalWrite(PWMC, LOW);            
  pinMode(PWMD, OUTPUT);            
  digitalWrite(PWMD, LOW);
  pinMode(PWME, OUTPUT);              
  digitalWrite(PWME, LOW);            
  pinMode(PWMF, OUTPUT);             
  digitalWrite(PWMF, LOW);
  
  TCCR0B = TCCR0B & B11111000 | 1;   // Configuración de la frecuencia del PWM para los pines 5 y 6.
  TCCR1B = TCCR1B & B11111000 | 1;   // Podemos variar la frecuencia del PWM con un número de 1 (32KHz) hasta 7 (32Hz). El número que pongamos es un divisor de frecuencia. Min.=7, Max.=1.
  
  attachInterrupt(digitalPinToInterrupt(encA), encoder1, RISING); // Flanco ascendente
  attachInterrupt(digitalPinToInterrupt(encB), encoder2, CHANGE); // En cualquier flanco ascendente o descendente
  attachInterrupt(digitalPinToInterrupt(encD), encoder3, RISING); // Flanco ascendente 
  
  outMax =  255.0;                    // Límite máximo del PWM.
  outMin = -outMax;                   // Límite mínimo del PWM.
  
  tmp = 1;                            // Tiempo de muestreo en milisegundos.
  
  // Constantes PID iniciales.  
  kp = kp2; // 2.701;                         
  ki = ki2; // 9.324;                          
  kd = kd2; // 0.1956;

  myPID.SetSampleTime(tmp);             // Envía a la librería el tiempo de muestreo.
  myPID.SetOutputLimits(outMin, outMax);// Límites máximo y mínimo; corresponde a Max.: 0=0V hasta 255=5V (PWMA), y Min.: 0=0V hasta -255=5V (PWMB). Ambos PWM se convertirán a la salida en valores absolutos, nunca negativos.
  myPID.SetTunings(kp, ki, kd);         // Constantes de sintonización.
  myPID.SetMode(AUTOMATIC);             // Habilita el control PID (por defecto).
  Setpoint = (double)contador;          // Para evitar que haga cosas raras al inciarse, igualamos los dos valores para que comience estando el motor parado.

  imprimir(3);                        // Muestra los datos de sintonización y el tiempo de muestreo por el terminal serie.
}

void loop()
{  
  // Recepción de datos para posicionar el motor, modificar las constantes PID o el tiempo de muestreo.
  if (Serial.available() > 0)           // Comprueba si ha recibido algún dato por el terminal serie.
  {
    cmd = 0;                            // Por seguridad se limpia cmd.
    cmd = Serial.read();                // cmd guarda el byte recibido.
    if (cmd > 31)
    {
      byte flags = 0;                                     // Borramos la bandera que decide lo que hay que imprimir.
   
      if (cmd >  'Z') cmd -= 32;                          // Si una letra entra en minúscula la covierte en mayúscula.
      
      // Decodificador para modificar las constantes PID.
      switch(cmd)                                                                            // p2.5 i0.5 d40 carga valores en kp, ki y kd.
      {                                                                                      // Tambien se puede uno por uno (Chicaneo :v).
        case 'P': kp  = Serial.parseFloat(); myPID.SetTunings(kp, ki, kd); flags = 1; break; // Carga las constantes y muestra los valores de las variables que hayan sido modificadas.
        case 'I': ki  = Serial.parseFloat(); myPID.SetTunings(kp, ki, kd); flags = 1; break;
        case 'D': kd  = Serial.parseFloat(); myPID.SetTunings(kp, ki, kd); flags = 1; break;
        case 'T': tmp = Serial.parseInt();   myPID.SetSampleTime(tmp);     flags = 1; break;
        case 'X': Grados = Serial.parseFloat();                            flags = 2; banderas = 1; break;  // Posicion en grados g180, se posiciona en 180.
        case 'Y': Grados = Serial.parseFloat();                            flags = 2; banderas = 2; break;  // Posicion en grados g180, se posiciona en 180.
        case 'Z': Grados = Serial.parseFloat();                            flags = 2; banderas = 3; break;  // Posicion en grados g180, se posiciona en 180.
        case 'K':                                                          flags = 3; break;  // K para ver los parametros anteriores del PID.
      }
      imprimir(flags); 
    }
  }
  Asignacion(banderas);
  Serial.print(Grados);
  Serial.print(" ");
  Serial.println(Respuesta);
}

void Asignacion(byte bandera)
{
  switch(bandera)                                                                            
      {                                                                                      
        case 1: contador = contador1_1 + contador1_2; Sensi = Sensi1; Motor1(); break; 
        case 2: contador = contador2; Sensi = Sensi2; Motor2(); break;
        case 3: contador = contador3_1 + contador3_2; Sensi = Sensi3; Motor3(); break;
      }
}

void encoder1()                        
{
  if(flagM1_1 == true)
  {
    contador1_1++;
  }
  if(flagM1_2 == true)
  {
    contador1_2--;
  }
  if(flagM1_3 == true)
  {
    contador1_1 = 0; contador1_2 = 0;
  }
}

void encoder2()                        
{
  int Lstate2 = digitalRead(encB);
  if ((Encoder_C1Last2 == LOW) && Lstate2 == HIGH)
  {
    int val2 = digitalRead(encC);
    if (val2 == LOW && Direccion2)
    {
      Direccion2 = false; //Reverse
    }
    else if (val2 == HIGH && !Direccion2)
    {
      Direccion2 = true;  //Forward
    }
  }
  Encoder_C1Last2 = Lstate2;
  if (!Direccion2)  contador2++;
  else  contador2--;
  ant2 = act2;                          // Guardamos el valor 'act2' en 'ant2' para convertirlo en pasado.
  act2 = PIND & 12;                    // Guardamos en 'act2' el valor que hay en ese instante en el encoder.
}

void encoder3()                        
{
  if(flagM3_1 == true)
  {
    contador3_1++;
  }
  if(flagM3_2 == true)
  {
    contador3_2--;
  }
  if(flagM3_3 == true)
  {
    contador3_1 = 0; contador3_2 = 0;
  }
}

void Motor1()
{
  double recta1 = 0;
  if(Grados < 0) // Angulo negativo
  {
    flagM1_1 = false; flagM1_2 = true; flagM1_3 = false;
    recta1 = Sensi*abs(Grados);
    if(abs(contador1_2) < (recta1 + abs(contador1_1)) - compensacion1)
    {
      digitalWrite(PWMA, LOW);
      analogWrite(PWMB, 80);
    }else
    {
      digitalWrite(PWMB, LOW);
      analogWrite(PWMA, 20);
      if(abs(contador1_2) > (recta1 + abs(contador1_1)) - compensacion1)
      {
        digitalWrite(PWMB, LOW);
        digitalWrite(PWMA, LOW);
      } 
    }
  }else if(Grados > 0) // Angulo positivo
  {
    flagM1_3 = false; flagM1_2 = false; flagM1_1 = true;
    recta1 = Sensi*abs(Grados);
    if(abs(contador1_1) < (recta1 + abs(contador1_2)) - compensacion1)
    {
      analogWrite(PWMA, 80);
      digitalWrite(PWMB, LOW);
    }else
    {
      digitalWrite(PWMA, LOW);
      digitalWrite(PWMB, LOW);
    } 
  }else if(Grados == 0) // Home
  {
    Home1();
  }
  Respuesta = contador/Sensi;
}

void Motor2()
{
  Setpoint = Grados*Sensi; //Es para ingresar valores de 0 a 360 grados como setpoint.
  Respuesta = contador/Sensi; //En este caso el contador que seria la respuestas en forma de pulsos.
 
  Input = (double)contador;           // Lectura del encoder óptico. El valor del contador se incrementa/decrementa a través de las interrupciones externas (pines 2 y 3).
  
  while(!myPID.Compute());            // Mientras no se cumpla el tiempo de muestreo, se queda en este bucle.

  // *********************************************** Control del Motor *************************************************
  if (((long)Setpoint - contador) == 0)// Cuando está en el punto designado, parar el motor.
  {
    digitalWrite(PWMC, LOW);          // Pone a 0 los dos pines del puente en H.
    digitalWrite(PWMD, LOW); 
  }
  else                                // En caso contrario hay que mirar si el motor va hacia delante o hacia atrás, con el signo de la variable Output.
  {
    if (Output > 0.0)                 // Mueve el motor hacia delante con el PWM correspondiente a su posición.
    {
      digitalWrite(PWMD, LOW);        // Pone a 0 el segundo pin del puente en H.
      analogWrite(PWMC, abs(Output)); // Por el primer pin sale la señal PWM.
    }
    else                              // Mueve el motor hacia  atrás   con el PWM correspondiente a su posición.
    {
      digitalWrite(PWMC, LOW);        // Pone a 0 el primer pin del puente en H.
      analogWrite(PWMD, abs(Output)); // Por el segundo pin sale la señal PWM.
    }
  }
}

void Motor3()
{
  int recta3 = 0;
  if(Grados < 0) // Angulo negativo
  {
    flagM3_1 = false; flagM3_2 = true; flagM3_3 = false;
    recta3 = Sensi*abs(Grados);
    if(abs(contador3_2) < (recta3 + abs(contador3_1)) - compensacion3)
    {
      digitalWrite(PWME, LOW);
      analogWrite(PWMF, 80);
    }else
    {
      digitalWrite(PWMF, LOW);
      analogWrite(PWME, 20);
      if(abs(contador3_2) > (recta3 + abs(contador3_1)) - compensacion3)
      {
        digitalWrite(PWME, LOW);
        digitalWrite(PWMF, LOW);
      }
    }
  }else if(Grados > 0) // Angulo positivo
  {
    flagM3_3 = false; flagM3_2 = false; flagM3_1 = true;
    recta3 = Sensi*abs(Grados);
    if(abs(contador3_1) < recta3 + abs(contador3_2))
    {
      analogWrite(PWME, 80);
      digitalWrite(PWMF, LOW);
    }else
    {
      digitalWrite(PWME, LOW);
      digitalWrite(PWMF, LOW);
    } 
  }else if(Grados == 0) // Home
  {
    Home3();
  }
  Respuesta = contador/Sensi;
}

void Home1()
{
  if(abs(contador1_1) > 0) // Positivo
  {
    flagM1_1 = false; flagM1_2 = true; flagM1_3 = false;
    if(abs(contador1_2) < abs(contador1_1) - compensacion1)
    {
      digitalWrite(PWMA, LOW);
      analogWrite(PWMB, 80);
      flagM1_1 = false;
    }else
    {
      digitalWrite(PWMA, LOW);
      digitalWrite(PWMB, LOW);
      flagM1_3 = true;
      flagM1_2 = false;
    }
  }
  else if(abs(contador1_2) > 0) // Negativo
  {
    flagM1_2 = false; flagM1_1 = true; flagM1_3 = false;
    if(abs(contador1_1) < abs(contador1_2) - compensacion1)
    {
      digitalWrite(PWMB, LOW);
      analogWrite(PWMA, 80);
      flagM1_2 = false;
    }else
    {
      digitalWrite(PWMA, LOW);
      digitalWrite(PWMB, LOW);
      flagM1_3 = true;
      flagM1_1 = false;
    }
  } 
}

void Home2()
{
  Grados = 0;
}

void Home3()
{
  if(abs(contador3_1) > 0) // Positivo
  {
    flagM3_1 = false; flagM3_2 = true; flagM3_3 = false;
    if(abs(contador3_2) < abs(contador3_1) - compensacion3)
    {
      digitalWrite(PWME, LOW);
      analogWrite(PWMF, 80);
      flagM3_1 = false;
    }else
    {
      digitalWrite(PWME, LOW);
      digitalWrite(PWMF, LOW);
      flagM3_3 = true;
      flagM3_2 = false;
    }
  }
  else if(abs(contador3_2) > 0) // Negativo
  {
    flagM3_2 = false; flagM3_1 = true; flagM3_3 = false;
    if(abs(contador3_1) < abs(contador3_2) - compensacion3)
    {
      digitalWrite(PWMF, LOW);
      analogWrite(PWME, 80);
      flagM3_2 = false;
    }else
    {
      digitalWrite(PWME, LOW);
      digitalWrite(PWMF, LOW);
      flagM3_3 = true;
      flagM3_1 = false;
    }
  } 
}

void imprimir(byte flag) // Imprime en el terminal serie los datos de las contantes PID, tiempo de muestreo y posición. En los demás casos sólo imprime la posición del motor.
{
  if ((flag == 1) || (flag == 3))
  {
    Serial.print("KP=");     Serial.print(kp);
    Serial.print(" KI=");    Serial.print(ki);
    Serial.print(" KD=");    Serial.print(kd);
    Serial.print(" Time=");  Serial.println(tmp);
  }
}
