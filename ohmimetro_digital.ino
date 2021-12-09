**********************************************************************
char pino_leitura = 0; // Pino para a leitura dos valores da tensão Vx 

int leitura = 0;  // Variavel para armazenar o valor lido pela entrada analógica (valor entre 0 e 1023)

float Vx = 0.0;   // Variavel para calcular o valor lido pela entrada analogica em volts 

float R = 0.0;   // Variavel para receber o valor do resistor de referencia a ser dado pelo usuario via serial monitor

float resultado = 0.0;  // Variavel para calcular o valor do resistor utilizando a formula (3) do embasamento teorico

void setup(){                                            

Serial.begin(9600); // ConFiguraçao do serial monitor

pinMode(pino_leitura, INPUT); // ConFiguracao do pino de leitura como entrada de dados \\ 

}
void loop(){

leitura = analogRead(pino_leitura);                    // Variavel leitura recebe o valor lido pela porta serial A0

Vx = (4.9*leitura)/1023.0;  // relação para calcular VX( regra de 3 simples).
//O valor 4.9 e a tensão medida na saida PWM com duty-cicle 100%   

if(Serial.available()>0){   // Se existir dados na porta serial a serem transmitidos, então executa o if

R = float(Serial.parseInt());    // R recebe o próximo valor inteiro a ser enviado pela serial. Este valor corresponde ao resistor 9780 ohms(valor medido) presente no divisor de tensão com LDR.

resultado = ((4.9*R)/Vx) - R;      // Calculo do resultado 

Serial.print("O valor da Resistencia do LDR e:");  // Imprime no serial monitor a frase entre aspas

Serial.println(resultado);   // Imprime no serial monitor o resultado calculado

Serial.flush();                                      

}
}
