
// Bibliotecas usadas
#include <PID_v1.h>
#include <PWM.h>
#include <SoftwareSerial.h>

SoftwareSerial bt_serial(13, 9); //rx e tx


//Definindo as variaveis que o arduino irá se conectar 
  double Setpoint1,Setpoint2, Setpoint3,Input1,Input2,Input3, Output1, Output2, Output3; //variaveis de armazenamento de valor
  int inputPin1 = 1; // entrada do LDR1 / QUARTO
  int inputPin2 = 2; // entrada do LDR2 / SALA
  int inputPin3 = 3; // entrada do LDR3 / SALA DE JANTAR
  int outputPin11 = 11; // saida para LED1 - QUARTO
  int outputPin12 = 12; // saida para LED2 - SALA
  int outputPin6 = 6; // saida para LED3 - SALA DE JANTAR
  int32_t frequencia=38000; // FRENQUENCIA DE CHAVEAMENTO DO MOSFET
  char caracter;

// parametros de controle
  float Kp=0.0144; //ganho proporcional 
  float Ki=0.27; //ganho integral
  float Kd=0; //ganho derivativo 

// realiza uma interação com as variaves de ajuste inicial
  PID myPID1(&Input1, &Output1, &Setpoint1, Kp, Ki, Kd, DIRECT);
  PID myPID2(&Input2, &Output2, &Setpoint2, Kp, Ki, Kd, DIRECT);
  PID myPID3(&Input3, &Output3, &Setpoint3, Kp, Ki, Kd, DIRECT);

  // registro de tempo
  unsigned long serialTime; //isso nos ajudará a saber quando conversar com o porta serial
  const int sampleRate = 1; //Variável que determina com que rapidez nosso loop PID será executado
  const long serialPing = 500; //Isso determina com que frequência faremos o ping de nosso loop
  unsigned long now = 0; //Essa variável é usada para acompanhar o tempo

 
 
void setup()
{
  InitTimersSafe(); //inicialize todos os temporizadores, exceto os temporizadores 0, para economizar tempo mantendo funções
  SetPinFrequencySafe(outputPin11, frequencia); //define a frequência 50kHz para o pino especificado
  SetPinFrequencySafe(outputPin12, frequencia); //define a frequência 50kHz para o pino especificado
 
  
  Input1 = analogRead(inputPin1); // Ler nível de luz no quarto
  Input2 = analogRead(inputPin2); // Ler nível de luz no sala
  Input3 = analogRead(inputPin3); // Ler nível de luz no sala de jantar
  
  Serial.begin(115200); //Iniciar uma sessão serial
  bt_serial.begin(115200);
  

  myPID1.SetMode(AUTOMATIC); //Liga o loop PID como controle automático
  myPID1.SetSampleTime(sampleRate); //Define a taxa de amostragem
   
    
  myPID2.SetMode(AUTOMATIC); //Liga o loop PID como controle automático
  myPID2.SetSampleTime(sampleRate); //Define a taxa de amostragem
  
    
  myPID3.SetMode(AUTOMATIC); //Liga o loop PID como controle automático
  myPID3.SetSampleTime(sampleRate); //Define a taxa de amostragem
   
}


void loop(){
  
  Input1 = analogRead(inputPin1); // Ler nível de luz no quarto
  Input2 = analogRead(inputPin2); // Ler nível de luz no sala
  Input3 = analogRead(inputPin3); // Ler nível de luz no sala de jantar
  
  if (bt_serial.available()>0){
 
   
    caracter = bt_serial.read();


  }

  if (caracter =='a') { //Controle - Quarto - Estudar  
        Setpoint1=871;
  
     
   }

   if (caracter == 'b') { //Controle - Quarto - Ver TV
 
       Setpoint1=790;
   
   
   }

   if (caracter == 'c') { //Controle - Quarto - Ver Filme
 
       Setpoint1=360;

   
   }

    if (caracter == 'd'){ //Controle - Quarto - Romance
    
       Setpoint1=480;

   }


    if (caracter == 'e') { //Controle - Quarto - Desligar  
 
       Setpoint1=0;
    }

    if (caracter== 'f') { //Controle - Sala  - Eventos 


      Setpoint2=930;
    
    }

    if (caracter== 'g') { //Controle - Sala  - Reunião 
      
      Setpoint2=890;    
   }

    if (caracter== 'h') { //Controle - Sala  - Ver TV 

      Setpoint2=830;
  
    }

    if (caracter== 'i') { //Controle - Sala  - Ver Filme  

     Setpoint2=370;
  
  }

    if (caracter== 'j') { //Sala Controle - Sala  - Romance

      Setpoint2=640;

    }
  
   
    if (caracter== 'l') { //Controle - Sala  - Desligar


      Setpoint2=0;
    
    }
    
    if (caracter== 'm') { //Controle - Sala de Jantar - Preparando da mesa

      Setpoint3=900;
   
    }

    if (caracter== 'n') { //Controle - Sala de Jantar - jantar

        Setpoint3=830;
    
   }

   if (caracter== 'o') { //Controle - Sala de Jantar - Desligar

        Setpoint3=0;
    
   }

   
    myPID1.Compute(); // Executa o loop PID
    pwmWrite(outputPin11, Output1); // Escreva a saída do loop PID para o pino LED do quarto
    now = millis(); // Acompanhe o tempo

    myPID2.Compute(); // Executa o loop PID
    pwmWrite(outputPin12, Output2); // Escreva a saída do loop PID para o  pino LED da sala
    now = millis(); // Acompanhe o tempo

    
    myPID3.Compute(); // Executa o loop PID
    analogWrite(outputPin6, Output3); // Escreva a saída do loop PID para o pino LED da sala de jantar
    now = millis(); // Acompanhe o tempo


  //envia e recebe dados para serial
  if(millis()>serialTime)
  {
    SerialSend();
    serialTime+=500;
  }  
}

/********************************************
// * Serial Communication functions / helpers/
// ********************************************/


union {                // This Data structure lets
  byte asBytes[24];    // us take the byte array
  float asFloat[6];    // sent from processing and
}                      // easily convert it to a
foo;                   // float array



// getting float values from processing into the arduino
// was no small task.  the way this program does it is
// as follows:
//  * a float takes up 4 bytes.  in processing, convert
//    the array of floats we want to send, into an array
//    of bytes.
//  * send the bytes to the arduino
//  * use a data structure known as a union to convert
//    the array of bytes back into an array of floats

//  the bytes coming from the arduino follow the following
//  format:
//  0: 0=Manual, 1=Auto, else = ? error ?
//  1: 0=Direct, 1=Reverse, else = ? error ?
//  2-5: float setpoint
//  6-9: float input
//  10-13: float output  
//  14-17: float P_Param
//  18-21: float I_Param
//  22-245: float D_Param
void SerialReceive()
{

  // read the bytes sent from Processing
  int index=0;
  byte Auto_Man = -1;
  byte Direct_Reverse = -1;
  while(Serial.available()&&index<26)
  {
    if(index==0) Auto_Man = Serial.read();
    else if(index==1) Direct_Reverse = Serial.read();
    else foo.asBytes[index-2] = Serial.read();
    index++;
  } 
  
  // if the information we got was in the correct format, 
  // read it into the system
  if(index==26  && (Auto_Man==0 || Auto_Man==1)&& (Direct_Reverse==0 || Direct_Reverse==1))
  {
    Setpoint1=double(foo.asFloat[0]);
    //Input1=double(foo.asFloat[1]);       // * the user has the ability to send the 
                                          //   value of "Input"  in most cases (as 
                                          //   in this one) this is not needed.
    if(Auto_Man==0)                       // * only change the output if we are in 
    {                                     //   manual mode.  otherwise we'll get an
      Output1=double(foo.asFloat[2]);      //   output blip, then the controller will 
    }                                     //   overwrite.
    
    double p, i, d;                       // * read in and set the controller tunings
    p = double(foo.asFloat[3]);           //
    i = double(foo.asFloat[4]);           //
    d = double(foo.asFloat[5]);           //
    myPID1.SetTunings(p, i, d);            //
    
    if(Auto_Man==0) myPID1.SetMode(MANUAL);// * set the controller mode
    else myPID1.SetMode(AUTOMATIC);             //
    
    if(Direct_Reverse==0) myPID1.SetControllerDirection(DIRECT);// * set the controller Direction
    else myPID1.SetControllerDirection(REVERSE);          //
  }
  Serial.flush();                         // * clear any random data from the serial buffer
}

// unlike our tiny microprocessor, the processing ap
// has no problem converting strings into floats, so
// we can just send strings.  much easier than getting
// floats from processing to here no?

void SerialSend()
{
  Serial.print("Controle PID1 -    QUARTO    - ");
  Serial.print(Setpoint1);   
  Serial.print(" ");
  Serial.print(Input1);   
  Serial.print(" ");
  Serial.print(Output1);   
  Serial.print(" ");
  Serial.print(myPID1.GetKp());   
  Serial.print(" ");
  Serial.print(myPID1.GetKi());   
  Serial.print(" ");
  Serial.print(myPID1.GetKd());   
  Serial.print(" ");
  if(myPID1.GetMode()==AUTOMATIC) Serial.print("Automatic");
  else Serial.print("Manual");  
  Serial.print(" ");
  if(myPID1.GetDirection()==DIRECT) Serial.println("Direct");
  else Serial.println("Reverse");

  Serial.print("Controle PID2 -     SALA     - ");
  Serial.print(Setpoint2);   
  Serial.print(" ");
  Serial.print(Input2);   
  Serial.print(" ");
  Serial.print(Output2);   
  Serial.print(" ");
  Serial.print(myPID2.GetKp());   
  Serial.print(" ");
  Serial.print(myPID2.GetKi());   
  Serial.print(" ");
  Serial.print(myPID2.GetKd());   
  Serial.print(" ");
  if(myPID2.GetMode()==AUTOMATIC) Serial.print("Automatic");
  else Serial.print("Manual");  
  Serial.print(" ");
  if(myPID2.GetDirection()==DIRECT) Serial.println("Direct");
  else Serial.println("Reverse");

  Serial.print("Controle PID3 - SALA JANTAR  - ");
  Serial.print(Setpoint3);   
  Serial.print(" ");
  Serial.print(Input3);   
  Serial.print(" ");
  Serial.print(Output3);   
  Serial.print(" ");
  Serial.print(myPID3.GetKp());   
  Serial.print(" ");
  Serial.print(myPID3.GetKi());   
  Serial.print(" ");
  Serial.print(myPID3.GetKd());   
  Serial.print(" ");
  if(myPID3.GetMode()==AUTOMATIC) Serial.print("Automatic");
  else Serial.print("Manual");  
  Serial.print(" ");
  if(myPID3.GetDirection()==DIRECT) Serial.println("Direct");
  else Serial.println("Reverse");
  
}
