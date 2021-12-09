

 //********************************************************

import java.nio.ByteBuffer;
import processing.serial.*;
import controlP5.*;

/***********************************************
 *Seção de especificação do usuário
 **********************************************/
int windowWidth = 900;      // defina o tamanho do 
int windowHeight = 600;     // formulário

float InScaleMin = 0;       // defina o eixo Y mínimo 
float InScaleMax = 1024;    // e máximo para as tendências
float OutScaleMin = 0;      // superior e inferior 
float OutScaleMax = 255;    


int windowSpan = 300000;    // número de mS no passado que você deseja exibir
int refreshRate = 100;      // com que frequência você deseja que o gráfico seja redesenhado;


float displayFactor = 60000; // exibe o tempo como minutos

String outputFileName = "c:\pid.txt"; // se você deseja enviar dados para um arquivo,
//  especifique o caminho aqui

/***********************************************
 * especificação do usuário final
 **********************************************/

int nextRefresh;
int arrayLength = windowSpan / refreshRate+1;
int[] InputData = new int[arrayLength];     // podemos não precisar deles desse tamanho,
int[] SetpointData = new int[arrayLength];  // mas esse é o pior caso
int[] OutputData = new int[arrayLength];


float inputTop = 25;
float inputHeight = (windowHeight-70)*2/3;
float outputTop = inputHeight+50;
float outputHeight = (windowHeight-70)*1/3;

float ioLeft = 150, ioWidth = windowWidth-ioLeft-50;
float ioRight = ioLeft+ioWidth;
float pointWidth= (ioWidth)/float(arrayLength-1);

int vertCount = 10;

int nPoints = 0;

float Input, Setpoint, Output;

boolean madeContact =false;
boolean justSent = true;

Serial myPort;

ControlP5 controlP5;
controlP5.Button AMButton, DRButton;
controlP5.Textlabel AMLabel, AMCurrent, InLabel, 
OutLabel, SPLabel, PLabel, 
ILabel, DLabel,DRLabel, DRCurrent;
controlP5.Textfield SPField, InField, OutField, 
PField, IField, DField;

PrintWriter output;
PFont AxisFont, TitleFont; 

void setup()
{
  frameRate(30);
  size(windowWidth , windowHeight);

  println(Serial.list());                                           // * Inicialize a
  myPort = new Serial(this, Serial.list()[0], 115200);                //   comunicação serial
  myPort.bufferUntil(10);                                           //   com o Arduino
  controlP5 = new ControlP5(this);                                  // * botões, rótulos e
  SPField= controlP5.addTextfield("Setpoint",10,100,60,20);         //   campos de texto que
  InField = controlP5.addTextfield("Input",10,150,60,20);           //  usaremos
  OutField = controlP5.addTextfield("Output",10,200,60,20);         
  PField = controlP5.addTextfield("Kp (Proportional)",10,275,60,20);         
  IField = controlP5.addTextfield("Ki (Integral)",10,325,60,20);          
  DField = controlP5.addTextfield("Kd (Derivative)",10,375,60,20);         
  AMButton = controlP5.addButton("Toggle_AM",0.0,10,50,60,20);    
  AMLabel = controlP5.addTextlabel("AM","Manual",12,72);           
  AMCurrent = controlP5.addTextlabel("AMCurrent","Manual",80,65);   
  controlP5.addButton("Send_To_Arduino",0.0,10,475,120,20);        
  SPLabel=controlP5.addTextlabel("SP","3",80,103);                 
  InLabel=controlP5.addTextlabel("In","1",80,153);                
  OutLabel=controlP5.addTextlabel("Out","2",80,203);                
  PLabel=controlP5.addTextlabel("P","4",80,278);                    
  ILabel=controlP5.addTextlabel("I","5",80,328);                    
  DLabel=controlP5.addTextlabel("D","6",80,378);                    
  DRButton = controlP5.addButton("Toggle_DR",0.0,10,425,60,20);      
  DRLabel = controlP5.addTextlabel("DR","Direct",12,447);            
  DRCurrent = controlP5.addTextlabel("DRCurrent","Direct",80,440);   

  AxisFont = loadFont("axis.vlw");
  TitleFont = loadFont("Titles.vlw");
 
  nextRefresh=millis();
  if (outputFileName!="") output = createWriter(outputFileName);
}

void draw()
{
  background(200);
  drawGraph();
  drawButtonArea();
}

void drawGraph()
{
  //draw Base, gridlines
  stroke(0);
  fill(230);
  rect(ioLeft, inputTop,ioWidth-1 , inputHeight);
  rect(ioLeft, outputTop, ioWidth-1, outputHeight);
  stroke(210);

  
// Títulos da Seção
  textFont(TitleFont);
  fill(255);
  text("PID Input / Setpoint",(int)ioLeft+10,(int)inputTop-5);
  text("PID Output",(int)ioLeft+10,(int)outputTop-5);


  
// Linhas de grade e títulos
  textFont(AxisFont);
  
  
// linhas de grade horizontais
  int interval = (int)inputHeight/5;
  for(int i=0;i<6;i++)
  {
    if(i>0&&i<5) line(ioLeft+1,inputTop+i*interval,ioRight-2,inputTop+i*interval);
    text(str((InScaleMax-InScaleMin)/5*(float)(5-i)+InScaleMin),ioRight+5,inputTop+i*interval+4);

  }
  interval = (int)outputHeight/5;
  for(int i=0;i<6;i++)
  {
    if(i>0&&i<5) line(ioLeft+1,outputTop+i*interval,ioRight-2,outputTop+i*interval);
    text(str((OutScaleMax-OutScaleMin)/5*(float)(5-i)+OutScaleMin),ioRight+5,outputTop+i*interval+4);
  }


  // linhas de grade verticais e TimeStamps
  int elapsedTime = millis();
  interval = (int)ioWidth/vertCount;
  int shift = elapsedTime*(int)ioWidth / windowSpan;
  shift %=interval;

  int iTimeInterval = windowSpan/vertCount;
  float firstDisplay = (float)(iTimeInterval*(elapsedTime/iTimeInterval))/displayFactor;
  float timeInterval = (float)(iTimeInterval)/displayFactor;
  for(int i=0;i<vertCount;i++)
  {
    int x = (int)ioRight-shift-2-i*interval;

    line(x,inputTop+1,x,inputTop+inputHeight-1);
    line(x,outputTop+1,x,outputTop+outputHeight-1);    

    float t = firstDisplay-(float)i*timeInterval;
    if(t>=0)  text(str(t),x,outputTop+outputHeight+10);
  }


  // adiciona os dados mais recentes às
  // matrizes de dados. Os valores precisam ser
  // manipulado para que eles representem corretamente o grafico .
  // O valores precisam ser redimensionados para se ajustarem aonde 
  // estão indo porque 0 é o canto superior esquerdo,
  // precisando inverter os valores. Isso é mais fácil do que fazer o usuário ler o gráfico de cabeça para baixo.
   
  if(millis() > nextRefresh && madeContact)
  {
    nextRefresh += refreshRate;

    for(int i=nPoints-1;i>0;i--)
    {
      InputData[i]=InputData[i-1];
      SetpointData[i]=SetpointData[i-1];
      OutputData[i]=OutputData[i-1];
    }
    if (nPoints < arrayLength) nPoints++;

    InputData[0] = int(inputHeight)-int(inputHeight*(Input-InScaleMin)/(InScaleMax-InScaleMin));
    SetpointData[0] =int( inputHeight)-int(inputHeight*(Setpoint-InScaleMin)/(InScaleMax-InScaleMin));
    OutputData[0] = int(outputHeight)-int(outputHeight*(Output-OutScaleMin)/(OutScaleMax-OutScaleMin));
  }
 // desenha linhas para a entrada, setpoint e saída
  strokeWeight(2);
  for(int i=0; i<nPoints-2; i++)
  {
    int X1 = int(ioRight-2-float(i)*pointWidth);
    int X2 = int(ioRight-2-float(i+1)*pointWidth);
    boolean y1Above, y1Below, y2Above, y2Below;

    
// DESENHE A ENTRADA
    boolean drawLine=true;
    stroke(255,0,0);
    int Y1 = InputData[i];
    int Y2 = InputData[i+1];

    y1Above = (Y1>inputHeight);                     // se ambos os pontos estiverem fora do 
    y1Below = (Y1<0);                               // mínimo ou máximo,
    y2Above = (Y2>inputHeight);                     // não desenhe a linha. 
    y2Below = (Y2<0);                               // Se apenas um ponto estiver fora, limite-o
    if(y1Above)                                     // ao limite e deixe o outro isolado.
    {                                               //
      if(y2Above) drawLine=false;                   //
      else if(y2Below) {                            //
        Y1 = (int)inputHeight;                      //
        Y2 = 0;                                     //
      }                                             //
      else Y1 = (int)inputHeight;                   //
    }                                               //
    else if(y1Below)                                //
    {                                               //
      if(y2Below) drawLine=false;                   //
      else if(y2Above) {                            //
        Y1 = 0;                                     //
        Y2 = (int)inputHeight;                      //
      }                                             //
      else Y1 = 0;                                  //
    }                                               //
    else                                            //
    {                                               //
      if(y2Below) Y2 = 0;                           //
      else if(y2Above) Y2 = (int)inputHeight;       //
    }                                               //

    if(drawLine)
    {
      line(X1,Y1+inputTop, X2, Y2+inputTop);
    }

    
// DESENHO DO SETPOINT
    drawLine=true;
    stroke(0,255,0);
    Y1 = SetpointData[i];
    Y2 = SetpointData[i+1];

    y1Above = (Y1>(int)inputHeight);                // se ambos os pontos estiverem fora do  
    y1Below = (Y1<0);                               // mínimo ou máximo,
    y2Above = (Y2>(int)inputHeight);                // não desenhe a linha.
    y2Below = (Y2<0);                               // Se apenas um ponto estiver fora, limite-o 
    if(y1Above)                                     // ao limite e deixe o outro isolado.
    {                                               //
      if(y2Above) drawLine=false;                   //
      else if(y2Below) {                            //
        Y1 = (int)(inputHeight);                    //
        Y2 = 0;                                     //
      }                                             //
      else Y1 = (int)(inputHeight);                 //
    }                                               //
    else if(y1Below)                                //
    {                                               //
      if(y2Below) drawLine=false;                   //
      else if(y2Above) {                            //
        Y1 = 0;                                     //
        Y2 = (int)(inputHeight);                    //
      }                                             //
      else Y1 = 0;                                  //
    }                                               //
    else                                            //
    {                                               //
      if(y2Below) Y2 = 0;                           //
      else if(y2Above) Y2 = (int)(inputHeight);     //
    }                                               //

    if(drawLine)
    {
      line(X1, Y1+inputTop, X2, Y2+inputTop);
    }

    
// DESENHO DA SAÍDA
    drawLine=true;
    stroke(0,0,255);
    Y1 = OutputData[i];
    Y2 = OutputData[i+1];

    y1Above = (Y1>outputHeight);                   // se ambos os pontos estiverem fora do  
    y1Below = (Y1<0);                              // mínimo ou máximo, 
    y2Above = (Y2>outputHeight);                   // não desenhe a linha.
    y2Below = (Y2<0);                              // Se apenas um ponto estiver fora, limite-o 
    if(y1Above)                                    // ao limite e deixe o outro isolado.
    {                                              //
      if(y2Above) drawLine=false;                  //
      else if(y2Below) {                           //
        Y1 = (int)outputHeight;                    //
        Y2 = 0;                                    //
      }                                            //
      else Y1 = (int)outputHeight;                 //
    }                                              //
    else if(y1Below)                               //
    {                                              //
      if(y2Below) drawLine=false;                  //
      else if(y2Above) {                           //
        Y1 = 0;                                    //
        Y2 = (int)outputHeight;                    //
      }                                            //  
      else Y1 = 0;                                 //
    }                                              //
    else                                           //
    {                                              //
      if(y2Below) Y2 = 0;                          //
      else if(y2Above) Y2 = (int)outputHeight;     //
    }                                              //

    if(drawLine)
    {
      line(X1, outputTop + Y1, X2, outputTop + Y2);
    }
  }
  strokeWeight(1);
}

void drawButtonArea()
{
  stroke(0);
  fill(100);
  rect(0, 0, ioLeft, windowHeight);
}

void Toggle_AM() {
  if(AMLabel.getValueLabel().getText()=="Manual") 
  {
    AMLabel.setValue("Automatic");
  }
  else
  {
    AMLabel.setValue("Manual");   
  }
}


void Toggle_DR() {
  if(DRLabel.getValueLabel().getText()=="Direct") 
  {
    DRLabel.setValue("Reverse");
  }
  else
  {
    DRLabel.setValue("Direct");   
  }
}

// Enviar valores de pontos de float para o arduino é bastante complexo 
// Este é o jeito fácil já conhecido:
// -Tome os 6 float que precisamos enviar e coloque-os em uma matriz float de tamanho 6.
// - Use a classe java ByteBuffer e converta essa matriz para uma matriz de 24 bytes
// - envie esses bytes ao arduino
void Send_To_Arduino()
{
  float[] toSend = new float[6];

  toSend[0] = float(SPField.getText());
  toSend[1] = float(InField.getText());
  toSend[2] = float(OutField.getText());
  toSend[3] = float(PField.getText());
  toSend[4] = float(IField.getText());
  toSend[5] = float(DField.getText());
  Byte a = (AMLabel.getValueLabel().getText()=="Manual")?(byte)0:(byte)1;
  Byte d = (DRLabel.getValueLabel().getText()=="Direct")?(byte)0:(byte)1;
  myPort.write(a);
  myPort.write(d);
  myPort.write(floatArrayToByteArray(toSend));
  justSent=true;
} 


byte[] floatArrayToByteArray(float[] input)
{
  int len = 4*input.length;
  int index=0;
  byte[] b = new byte[4];
  byte[] out = new byte[len];
  ByteBuffer buf = ByteBuffer.wrap(b);
  for(int i=0;i<input.length;i++) 
  {
    buf.position(0);
    buf.putFloat(input[i]);
    for(int j=0;j<4;j++) out[j+i*4]=b[3-j];
  }
  return out;
}



// pega a string que o arduino nos envia e analisa
void serialEvent(Serial myPort)
{
  String read = myPort.readStringUntil(10);
  if(outputFileName!="") output.print(str(millis())+ " "+read);
  String[] s = split(read, " ");

  if (s.length ==9)
  {
    Setpoint = float(s[1]);           // * retire as informações de que precisamos 
    Input = float(s[2]);              //   e coloque-as onde for 
    Output = float(s[3]);             //   necessário
    SPLabel.setValue(s[1]);           //   
    InLabel.setValue(s[2]);           //
    OutLabel.setValue(trim(s[3]));    //
    PLabel.setValue(trim(s[4]));      //
    ILabel.setValue(trim(s[5]));      //
    DLabel.setValue(trim(s[6]));      //
    AMCurrent.setValue(trim(s[7]));   //
    DRCurrent.setValue(trim(s[8]));
    if(justSent)                      // * se esta é a primeira leitura desde que 
    {                                 //   enviamos valores ao arduino,  
      SPField.setText(trim(s[1]));    //   pegue os valores atuais e 
      InField.setText(trim(s[2]));    //   coloque-os nos campos de entrada
      OutField.setText(trim(s[3]));   
      PField.setText(trim(s[4]));     
      IField.setText(trim(s[5]));     
      DField.setText(trim(s[6]));     
     // mode = trim(s[7]);              
      AMLabel.setValue(trim(s[7]));        
      //dr = trim(s[8]);                
      DRCurrent.setValue(trim(s[8]));        
      justSent=false;                 
    }                                 

    if(!madeContact) madeContact=true;
  }
}






