
/* VERSÃO 1.0 rebuilt */



//############################################################################################################################################//

/* Biblioteca utilizada para acessar o watchdog do microcontrolador */
#include <avr/wdt.h> //Carraga os comandos do watchdog (para reset do uC - evitar o travamento)

/* LIBS PARA USO DO MODULO CAN MCP2515 */
#include <SPI.h>
#define CAN_2515

const int SPI_CS_PIN = 10;
const int CAN_INT_PIN = 18; // Pino usado para gerar a interrupção pelo modulo MCP2515

//https://wiki.seeedstudio.com/CAN-BUS_Shield_V2.0/#apis
#ifdef CAN_2515
#include "mcp2515_can.h"
mcp2515_can CAN(SPI_CS_PIN); // Set CS pin
#define MAX_DATA_SIZE 8
#endif

boolean flagCANInit = true; //se false indica que o modulo CAN não foi inicializado com sucesso

unsigned int idMsgCANRec = 0; //essa variavel armazena o id da msg CAN q enviou uma msg CAN

boolean flagRecv = false; //se true indica q uma msg foi recebida via CAN

unsigned long tControle = 0; //contador que controla o intervalo de tempo q o PID irá ser aplicado (verificar necessidade)
unsigned long tEnviaMsgCAN = 0; //contador que controla o intervalo de tempo q as msg CAN serão enviadas (padrão 50ms)

byte bufCANRec[8] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}; //usado para armazenar os dados recebidos via CAN
//byte spDAux = 0x00; //Evita o Bug do PWM

byte erroCalibraDirecao = 0x00; //armazena algum erro (o ultimo erro) ocorrido na rotina de calibração de direção

/* USADO PARA LER A VELOCIDADE DO CARRO - VIA CAN */
double spMEsq = 0, spMdir = 0, rpmRodaEsq = 0,  rpmRodaDir = 0; //variaveis usadas para ler a velocidade do carro

/* LIB PARA USO DO WATCHDOG */
#include <avr/wdt.h> //Carraga os comandos do watchdog (para reset do uC)


/* LIB PARA USO DA EEPROM DO ARDUINO */
#include <EEPROM.h>//Carrega a biblioteca para uso da EEPROM interna 


/* DIRETIVAS PARA AJUSTES DE FUNCIONAMENTO DO PROGRAMA */
//HABILITA OU DESABILITA O MODO DEBUG (MSG OU GRAFICOS VIA SERIAL)
#define debugar 1 //0 - O programa funciona normalmente, 1 - o programa entra em modo debug e envia dados via serial


//DEFINE A FORMA DE LEITURA DOS SENSORES DE VELOCIDADE (ACOMPLADO AS RODAS TRAZEIRAS)
#define modoLeituraVel 4 //3 - modo media móvel (mede o tempo entre pulsos e faz a media movel), 4 - modo leitura lenta (conta a quantidade de pulsos em 500ms)

#define geraGrafVel 9 //USADO NA RORTINA DE CONTROLE DA VELOCIDADE COM PID, 0 - (GERA TEXTO NA SERIAL) envia, via serial, texto com valores de setpoint, medida de velocidade e valor de pwm, 1 - (GERA GRAFICO) envia numeros para gerar grafico de setpoint e velocidade no plotter serial (IDE arduino)
#define geraGrafDirec 9 //USADO NA RORTINA DE CONTROLE DA DIREÇÃO, 0 - (GERA TEXTO NA SERIAL) envia via serial, texto com valores de setpoint, angulo e valor de pwm da direção, 1 - (GERA GRAFICO) envia dados para gerar grafico de setpoint e angulo no plotter serial (IDE arduino)


/* VARIAVEIS USADA PARA CONFIGRA OS PINOS DE ACIONAMENTO DAS PONTE H */
//Ponte H3  BTS 7960 (volante - direção)
#define R_PWM3 3
#define L_PWM3 2

#define pinSinaliza 41 //usado para sinalizar


/* VARIAVEIS USADAS PARA A CALIBRAÇÃO DA DIREÇÃO - RODAS DIANTEIRAS */
double posMin = 0, posMax = 0, posCenter = 0, posAct = 0; //usado para leitura de um potenciometro
boolean flagRotinaCalibraDirec = false; //Se true idenfifica que houve algum erro na rotina de calibração da direção

//pino q gera interrupção para medir o angulo da direção (quando se usa encoder na direção) conta pulsos
const byte interruptPinDirec = 18;

#define pinDirec 22 //Pino usado para identificar se a direção está girando para direita ou esquerda

// essa variavel identifica pra que lado a direção está girando ou qual foi o ultimo movimeto executado(se true a direção está girando pra esquerda)
boolean flagDirec = false; 

// Pinos conectados ao sensores fim de curso da rodas da frente - Usados para determinar a limitação de giro da direção
#define fimDeCursoDir 26
#define fimDeCursoEsq 28


/* VARIAVEIS USADAS PARA O CALCULO DE RESISTENCIA ATUAL DO POTENCIOMETRO - USADO NA LEITURA DE POSIÇÃO DAS RODAS */
unsigned int valorPotenciometro = 10000; //potenciometro linear de 10Kohm (OBS: O potenciometro deve ser de 1K até 60K)
int tensaoAlimentacao = 5; //tensão se alimentação aplicada ao potenciometro (em relação ao terra, ou seja, entre os pinos mais externos do potenciometro)

//Converte o valor dos ganhos do PID rpara 32 bits divididos num array(CONVERTE UM INTEIRO EM 32 BITES DIVIDIDO NUM ARRAY DE TAMNHO 4)
uint8_t ganhosPID[sizeof(int32_t)]; //vetor q armazena o inteiro convertido

//OBS: Com o potenciometro de 10K a resolução da medida sera de (10K/65536) 0,15 ohm, ou seja, 0x0001 = 0,15 ohm e 0xFFFF = 10K

    
/* LIB PARA USO DO CONTROLE PID */
#include <PID_v1.h> //Carrega a biblioteca dos PIDs

double spD = 0, outD = 0; //variaveis do PID usadas  nos motores 


PID pidDirecao(&posAct, &outD, &spD, 0.1, 0.5, 0, DIRECT);

boolean flagPID = true;

/* PINO USADO PARA LER O VALOR DE SAIDA DO POTENCIOMETRO ACOPLADO A DIREÇÃO DO CARRO (TRANSDUTOR DE ANGULO) */
#define potDir A0 //


void setup(){

  wdt_disable();//desabilita o watchdog (usado na rotina de reset)
  
  Serial.begin(115200); //porta de comunicação com a ide do arduino
  Serial.println("INICIANDO ECU DIRECAO...");
  
  //Serial3.begin(9600); //porta de comunicação com a ide do arduino

  while(!Serial) {
      ;// wait for serial port to connect. Needed for native USB port only
  }

 
  //usado pra medir tempo nas rotinas de timeout
  unsigned long tStart = 0, timeOut = 0;

  tStart = millis();
  timeOut = 8000; //(3 segundos)
  //aguarda incializar o shield CAN
  Serial.println("Connecting CAN...");
  while((millis() - tStart) < timeOut){ //aguarda o timeout

    if(CAN_OK == CAN.begin(CAN_500KBPS, MCP_8MHz)){
      Serial.println("CAN init ok!!!");
      flagCANInit = true; //marca a flag q indica q inicialização correta da CAN  
      break; //sai do laço 
    }

    flagCANInit = false; //marca a flag q indica q houve problema na inicialização da CAN
        
  }


  //se houve erro na CAN mostra 
  if(flagCANInit == false){
    Serial.println("CAN error!!!");
  }
  

  // there are 2 mask in mcp2515, you need to set both of them
  /*  aceita idcan de 0x80 a 0x9F */
  CAN.init_Mask(0, 0, 0x7F0);
  
  CAN.init_Filt(0, 0, 0x080);  //    
  CAN.init_Filt(1, 0, 0x080);  //    


  CAN.init_Mask(1, 0, 0x7F0);
  
  CAN.init_Filt(2, 0, 0x090);  //   
  CAN.init_Filt(3, 0, 0x090);  //  
  CAN.init_Filt(4, 0, 0x090);  //  
  CAN.init_Filt(5, 0, 0x090);  //  
  
  
  //PINOS DE SINAL PWM DA DIREÇÃO
  pinMode(R_PWM3, OUTPUT);
  pinMode(L_PWM3, OUTPUT);

  /* PINOS 26 E 28 ESTÃO CONECTADOS AOS SENSORES FIM DE CURSO NAS RODAS DA FRENTE - USADOS PARA DETERMINAR A LIMITAÇÃO DE GIRO DA DIREÇÃO */
  pinMode(fimDeCursoDir, INPUT_PULLUP); //liga o pull-up do pino
  pinMode(fimDeCursoEsq, INPUT_PULLUP); //liga o pull-up do pino
 
  pinMode(pinDirec, INPUT); //Pino usado para identificar se a direção está girando para direita ou esquerda (funciona para o caso de uso de encoder)


  /*
  // LINHAS TESTE 
  EEPROM.write(0x2A, 0x0a);
  EEPROM.write(0x2C, 0x00);
  EEPROM.write(0x2E, 0x00);
  */

  /*
  double Kp, Ki, Kd;

  //ajusta os ganhos do PID do motor da direção
  //EEPROM.read(address)
  Kp = double(EEPROM.read(0x2A))/10;
  Ki = double(EEPROM.read(0x2C))/10;
  Kd = double(EEPROM.read(0x2E))/10;  

  //Serial.println(Kp); Serial.println(Ki); Serial.println(Kd);//linha teste 
  
  pidDirecao.SetTunings(Kp, Ki, Kd);
  */ 
  
  pidDirecao.SetTunings(1, 0, 0);//serve pra algo????

  //turn the PID on
  //pidDirecao.SetMode(AUTOMATIC);
  pidDirecao.SetMode(1);
  pidDirecao.SetSampleTime(100);
  pidDirecao.SetOutputLimits(-255, 255); //ajusta os limites minimos e maximos da saida do PID  

  
  //calibra a direção (identifica os limites extremos da direção e a leitura dos sensores para cada extremo - necessário para o controle da direção)
  calibraDirecao();
  
  //Desliga todos os motores
  //desligaMotores();
  

  pinMode(pinSinaliza, OUTPUT);//usado para sinalizar qndo a rotina principal esta ativa

  attachInterrupt(digitalPinToInterrupt(CAN_INT_PIN), MsgRecCAN, FALLING); // declara a interrupção gerada a qndo uma msg chega via CAN

  
  /*      FAZ COM QUE O BIP TOQUE       */
  digitalWrite(pinSinaliza, HIGH);
  delay(150);//espera 1 segundo para inicar o programa
  digitalWrite(pinSinaliza, LOW);
  delay(150);//espera 1 segundo para inicar o programa
  digitalWrite(pinSinaliza, HIGH);
  delay(150);//espera 1 segundo para inicar o programa
  digitalWrite(pinSinaliza, LOW);

  //pidDirecao.SetTunings(0.5, 2, 0.01); //GIRO MAIS RAPIDO
  pidDirecao.SetTunings(1.0, 2, 0.0); //GIRO MAIS LENTO

  
  CAN.sendMsgBuf(0x80, 0, 8, bufCANRec); //envia a msg CAN para mostrar q a ECU está funcional

  // Ativamos o watchdog e definimos o tempo de 2 seg.
  wdt_enable(WDTO_2S); // WDTO_1S, WDTO_2S, WDTO_4S, WDTO_8S

  Serial.println("ECU Direção RUNNING...");
  
}


void loop(){

  wdt_reset(); //reinicia o watchdog

  if (flagPID == true){
    controleDirecao();
  }
  
  /*  
  if((millis() - tControle) > 50){ //Rotina q determina o intervalo q o PID ira aplicar a logica de controle (100 = 100ms) (verificar nescessidade)
      tControle = millis();//reinicia a contagem
      controleDirecao();
  }
  */

  
  if((millis() - tEnviaMsgCAN) > 228){ //Rotina q determina o intervalo de envio das mensgen de status
    tEnviaMsgCAN = millis();//reinicia a contagem
    enviaMsgStatus();
  }


  if (flagRecv == true){
    trataMsgRecCAN(); //rotina q trata qndo uma msg chega via can
  }


}


//interrupção gerada qndo o modulo CAN recebe uma msg via CAN
void MsgRecCAN(){
    flagRecv = true; //flag que indica que uma msg foi recebida via CAN
    //Serial.println("teste direc");
}


//rotina que envia msg periodicas via CAN sobre o status atual da ECU
void enviaMsgStatus(){

 
  /* 
   *  FORMATO DA MSG
   * |   BYTE 0  |   BYTE 1   |    BYTE 2     |     BYTE 3     |    BYTE 4   |   BYTE 5   |   BYTE 6  |      BYTE 7      |
   * |     -     |      -     |    msgType    | Resistencia do potenciômetro | PWM Signal | PWM Value |   Sinal / Erro   |
   * |           |            |      0x00     |         0x0000-0xFFFF        | 0x00-0x01  | 0x00-0xFF |  0x00,0x0E1-0xEE |
   * 
   */

    bufCANRec[0] = 0x00;//
    bufCANRec[1] = 0x00;//

    bufCANRec[2] = 0x00;//msgType

    //envia msg via CAN sobre o status atual
    bufCANRec[3] = char(map(posAct, posMin, posMax, 0x01, 0x32));//posição atual da direção 
    
    bufCANRec[4] = char(map(spD, posMin, posMax, 0x01, 0x32));//setpoint da direção
    
    unsigned int resistenciaAtual = (((posAct * 5)/1024) * valorPotenciometro) / tensaoAlimentacao; //calcula o valor da resitencia na saida do potenciometro

    //valor da resistencia atual na saida do potenciometro (do pino central para o terra)
    bufCANRec[5] = highByte(resistenciaAtual);
    bufCANRec[6] = lowByte(resistenciaAtual);

    bufCANRec[7] = erroCalibraDirecao;//salva o erro gerado na rotina de calibração da direção (essa variável armazena apenas o ultimo erro detectado)
    
    CAN.sendMsgBuf(0x80, 0, 8, bufCANRec); //envia a msg CAN de resposta
    
}


//rotina que le a msg CAN no controlador
void trataMsgRecCAN(){
  // check if get data
  
  flagRecv = false; // clear flag

  unsigned char len = 0; //armazena o tamanho da msg CAN (qtd de dados recebidos)

  //CAN_MSGAVAIL：Indicates there are data in FIFO buffer to be read
  //CAN_NOMSG：Indicates there is no data in the FIFO buffer
  while (CAN_MSGAVAIL == CAN.checkReceive()){

      // read data,  len: data length, buf: data buf
      CAN.readMsgBuf(&len, bufCANRec);

      idMsgCANRec = CAN.getCanId(); //pega o id do dispositivo q enviou a msg CAN

      //verifica o tipo de msg redebida
      uint8_t type = (CAN.isExtendedFrame() << 0) | (CAN.isRemoteRequest() << 1); // Se 0: ext, Se 1: rtr

      
      Serial.print("Recieve by CAN: id ");

      Serial.print(idMsgCANRec);
      Serial.print(" (");
      Serial.print(idMsgCANRec, HEX);
      Serial.print(")HEX");
      Serial.print("\t");
  
      // print the data
      //for (int i = 0; i < sizeof(buf); i++){
      for (int i = 0; i < len; i++){
        
          //SERIAL_PORT_MONITOR.print(buf[i]); 
          //SERIAL_PORT_MONITOR.print("\t");
  
          Serial.print((bufCANRec[i]),HEX); 
          Serial.print("\t");
      }
      
      Serial.println();
      
      
  }


  executaFuncao();//após receber a msg CAN executa a função relativa a msg

  
}


//verifica qual função deve ser executada (de acordo com o id da msg recebida)
void executaFuncao(){

  byte bufCANResp[8] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}; //usado para enviar a resposta via CAN


  //AJUSTA O ANGULO DAS RODAS
  if(idMsgCANRec == 0x82){
    
      //verifica se a rotina de calibração da direção foi completamente executada
      if(flagRotinaCalibraDirec == true){

        flagPID = true;
        
        //spDAux = bufCANRec[7];
  
        if(bufCANRec[7] < 0x01){ //se o valor recebido for menor que o limite inferir ajusta o valor
          bufCANRec[7] = 0x01; //1 grau
        }
  
        if(bufCANRec[7] > 0x32){ //se o valor recebido for maior que o limite superior ajusta o valor
          bufCANRec[7] = 0x32; //50 graus
        }
        
        
        //    (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
        spD = (((bufCANRec[7] - 0x01) * (posMax - posMin)) / (0x32 - 0x01)) + posMin; //a soma é a ultima operação executada

        //monta a msg de resposta CAN

      }
                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                        
  }


  //CALIBRA A DIREÇÃO  
  if(idMsgCANRec == 0x84){
    calibraDirecao(); //executa a rotina de calibração da direção (pode demorar até 10 segundos para responder)
  }
  

  // LE O VALOR DO PINO A0 (VALOR DE TENSÃO DO POTENCIOMETRO)
  if(idMsgCANRec == 0x86){
      
      //map(leitura_pot, in_min, in_max, out_min, out_max)
      //bufCANResp[3] = (map(int(lePosDirecao()), 0, 1023, 0, 255));
  
      unsigned int resistenciaAtual = (((posAct * tensaoAlimentacao)/1024) * valorPotenciometro) / tensaoAlimentacao; //calcula o valor da resitencia na saida do potenciometro
  
      //valor da resistencia atual na saida do potenciometro (do pino central para o terra)
      bufCANResp[6] = highByte(resistenciaAtual);
      bufCANResp[7] = lowByte(resistenciaAtual);
  
      CAN.sendMsgBuf(idMsgCANRec + 1, 0, 8, bufCANResp); //envia a msg CAN de resposta

  } 

  
  //SETA OS GANHOS DO PID DA DIREÇÃO    
  if(idMsgCANRec == 0x88){

    //OBS: Os valores devem ser enviados em HEX ou seja 0x1E equivale a 30 = 3.0, do mesmo modo 0x50 equiavale a 80 = 8.0
    double Kp, Ki, Kd;
    
    //EEPROM.write(endereco, numero);
    //proporcional
    EEPROM.write(0x2A, bufCANRec[2]);
    EEPROM.write(0x2B, bufCANRec[3]);

    //integral
    EEPROM.write(0x2C, bufCANRec[4]);
    EEPROM.write(0x2D, bufCANRec[5]);

    //derivativo
    EEPROM.write(0x2E, bufCANRec[6]);
    EEPROM.write(0x2F, bufCANRec[7]);

    unsigned int gain = 0;

    gain = (bufCANRec[2]<<8) + (bufCANRec[3]);
    Kp = gain/100;

    gain = (bufCANRec[4]<<8) + (bufCANRec[5]);
    Ki = gain/100;

    gain = (bufCANRec[6]<<8) + (bufCANRec[7]);
    Kd = gain/100;

    //Serial.println(Kp);
    //Serial.println(Ki);
    //Serial.println(Kd);
    
    pidDirecao.SetTunings(Kp, Ki, Kd);
    
  }


  //LE OS GANHOS ATUAIS DOS PIDs DA DIREÇÃO    
  if(idMsgCANRec == 0x8A){

    unsigned int gain = 0;

    gain = 100*pidDirecao.GetKp();
    bufCANResp[2] = highByte(gain);
    bufCANResp[3] = lowByte(gain);

    gain = 100*pidDirecao.GetKi();
    bufCANResp[4] = highByte(gain);
    bufCANResp[5] = lowByte(gain);

    gain = 100*pidDirecao.GetKd();
    bufCANResp[6] = highByte(gain);
    bufCANResp[7] = lowByte(gain);

    //Serial.println(bufCANResp[2]);
    //Serial.println(bufCANResp[3]);
 
    CAN.sendMsgBuf(idMsgCANRec + 1, 0, 8, bufCANResp); //envia a msg CAN de resposta

  }


  //desliga motores da direção
  if(idMsgCANRec == 0x8C){

    flagPID = false;
    
    analogWrite(L_PWM3, 0);
    analogWrite(R_PWM3, 0);
    
  }


  //RESETA ECU
  if(idMsgCANRec == 0x8E){
    Serial.println("Reseando ECU!!!");
    //trava o arduino ate o watchdog resetar
    while (1){
      
    }

  }

}




//Alinha a direção com PID
void controleDirecao(){

  if(flagRotinaCalibraDirec == true){ //verifica se a rotina de calibraççao foi executada sem erros

    //spD = 11; //setpoint
  
    //analogWrite(L_PWM3, 0);
    //analogWrite(R_PWM3, 30);
  
    posAct = lePosDirecao(); //Le o valor do potenciometro (pega a posição atual)

    pidDirecao.Compute(); //faz o calculo da resposta do PID
    //Serial.print("PWM: ");
    //Serial.println(outD);
  
    if(outD == 0){
      //Aplica o resultado do PID na ponte H
      analogWrite(L_PWM3, 0); //Gira pra esquerda
      analogWrite(R_PWM3, 0);
    }else if(outD > 0){
      //Aplica o resultado do PID na ponte H
      analogWrite(L_PWM3, 0); //Gira pra esquerda
      analogWrite(R_PWM3, outD);
    }else{
      //Aplica o resultado do PID na ponte H
      analogWrite(L_PWM3, ((-1)*outD));
      analogWrite(R_PWM3, 0);  //Gira pra direita
    }
  
 }
  
  #if(debugar == 1) //verifica se o modo debugar foi acionado
  
    #if(geraGrafDirec == 0)    //envia informações via serial para mostrar pwm, angulo da direção e setpoint
      Serial.print("Setp Direcao: ");
      Serial.print(spD);
      Serial.print(" >>> Pos atual: ");
      Serial.print(posAct);
      Serial.print(" >>> PWM: ");
      Serial.print(outD);
      Serial.print(" ");
      if(flagDirec == true){
        Serial.println("esq <<<");
      }else{
        Serial.println("dir >>>");
      }
    #endif  
    
  
    #if(geraGrafDirec == 1)  //envia informações via serial para gerar grafico do angulo da direção e setpoint
      /* USADO PARA GERAR UM GRAFICO NA IDE DO ARDUINO */
      Serial.print(int(spD));
      Serial.print(" ");
      Serial.println(int(posAct));
    #endif

  #endif

 
}


//calibra o sensor de giro da direçao
void calibraDirecao(){
  
    //usado pra medir tempo nas rotinas de timeout
    unsigned long tStart = 0, timeOut = 0;
    boolean flagFimDeCurso = false; //usado para identificar o funcionmento dos fim de curso
    boolean flagFimDeCursoDir = false, flagFimDeCursoEsq = false; //indicam erro no sensor fim de curso
    
    int cont_erros = 0; //usado para contar o numero de erros encontrados (serve para indicar o tamanho do pacote)
    byte erros[16] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}; // array que guarda os erros encontrados (não usa as posições 0 a 2 [msgType])
    

    Serial.println("Calibrando direcao...");

    
    delay(1000);

    //Serial.println("Testando sensor de fim de curso direito...");
    analogWrite(L_PWM3, 250);
    analogWrite(R_PWM3, 0);

   
    tStart = millis();
    timeOut = 3000; //(3 segundos)
    flagFimDeCurso = false;
    //aguarda o toque no sensor de fim de curso direito
    while((millis() - tStart) < timeOut){ //aguarda o timeout
      if(digitalRead(fimDeCursoEsq) == false){

        Serial.println("Rodas viradas a esquerda");
        Serial.println("Fim de curso esquerdo OK");
            
        flagFimDeCurso = true; //flag que sinaliza q o sensor respondeu
        break; //sai do laço
      }
    }

    //se não encontrou o sensor gera o aviso de erro
    if(flagFimDeCurso == false){

      Serial.println("ERRO: Sensor de fim de curso esquerdo nao detectado!");

      //Incluir cod de sinalização visual para indicar erro
      flagFimDeCursoDir = true; //indica defeito no sensor de fim de curso
      erros[cont_erros] = 0xE1; 
      cont_erros++;      
    }  

    delay(500);

    /* OBS: A mudança de lado do potenciometro pode alterar exigir a troca da variavel posmax por posmim */
    posMin = lePosDirecao();//Le o valor do potenciometro (pega o limite máximo da esquerda)
    //Serial.print("posMin: ");
    //Serial.println(posMin);


    //desliga o PWM 
    analogWrite(L_PWM3, 0);
    analogWrite(R_PWM3, 0);

    delay(100);


    //Serial.println("Testando sensor de fim de curso esquerdo...");    
    analogWrite(L_PWM3, 0);
    analogWrite(R_PWM3, 250);

    
    tStart = millis();
    timeOut = 3000; //(3 segundos)
    flagFimDeCurso = false;
    //aguarda o toque no sensor de fim de curso esquerdo
    while((millis() - tStart) < timeOut){ //aguarda o timeout
      if(digitalRead(fimDeCursoDir) == false){

        Serial.println("Rodas viradas a direita");
        Serial.println("Fim de curso direito OK");

        flagFimDeCurso = true; //flag que sinaliza q o sensor respondeu
        break; //sai do laço
      }
      
    }
    

    //se não encontrou o sensor gera o aviso de erro
    if(flagFimDeCurso == false){

       Serial.println("ERRO: Sensor de fim de curso direito nao detectado!");

      //Incluir cod de sinalização visual para indicar erro
      flagFimDeCursoDir = true; //indica defeito no sensor de fim de curso
      erros[cont_erros] = 0xE2; 
      cont_erros++;      
    }


    delay(500);

    posMax = lePosDirecao(); //Le o valor do potenciometro (pega o limite minimo na direita)
    //Serial.print("posMax: ");
    //Serial.println(posMax);


    //desliga o PWM
    analogWrite(L_PWM3, 0);
    analogWrite(R_PWM3, 0);

    //EVITA Q AS RODAS GIREM ATE O FIM E TRAVANDO O MOTOR
    
    posMin = posMin*0.90;
    posMax = posMax*0.90;
    
    
    posAct = posMax;//atualiza o valor do angulo da direção
    posCenter = posMin + ((posMax - posMin)/2); //Faz uma média e encontra a posição de centro da direção
    spD = posCenter; // ajusta o setpoint para centralizar a direção


    Serial.print("posMax: ");
    Serial.println(posMax);
    Serial.print("posMin: ");
    Serial.println(posMin);
    Serial.print("posCenter = ");
    Serial.println(posCenter);
    Serial.print("posAct = ");
    Serial.println(posAct);


    /* MOVE OS PNEUS PARA DIREÇÃO CENTRAL */
    delay(1000);

    //APAGAR - bypassa o erro do sensores fim de curso
    flagFimDeCursoDir = false;
    flagFimDeCursoEsq = false;

    //se não for detectado erros no sensores de fim de curso
    if((flagFimDeCursoDir == false) && (flagFimDeCursoEsq == false)){
        
        Serial.println("Centralizando pneus...");

        //pidDirecao.SetTunings(0.2, 0.1, 0.0); //GIRO MAIS LENTO

        flagRotinaCalibraDirec = true; //Sinaliza que nao há erro na rotina de calibração da direção (sensores de fim de curso ok)
        flagPID = true;
    
        tStart = millis();
        timeOut = 5000; //(10000 = 10 segundos)
        while((millis() - tStart) < timeOut){ //aguarda o timeout
        //while(true){
    

         posAct = lePosDirecao(); //Le o valor do potenciometro (pega a posição atual)

          
          pidDirecao.Compute(); //faz o calculo da resposta do PID
          //Serial.print("PWM: ");
          //Serial.println(outD);
        
          if(outD == 0){
            //Aplica o resultado do PID na ponte H
            analogWrite(L_PWM3, 0); //Gira pra esquerda
            analogWrite(R_PWM3, 0);
          }else if(outD > 0){
            //Aplica o resultado do PID na ponte H
            analogWrite(L_PWM3, 0); //Gira pra esquerda
            analogWrite(R_PWM3, outD);
          }else{
            //Aplica o resultado do PID na ponte H
            analogWrite(L_PWM3, ((-1)*outD));
            analogWrite(R_PWM3, 0);  //Gira pra direita
          }
    
          #if(debugar == 1) //verifica se o modo debugar foi acionado
            #if(geraGrafDirec == 0) //habilia o modo texto do debug, envia para serial valor do centro (setpint), posição atual e valor de PWM
              Serial.print("posCentro = ");
              Serial.print(posCenter);
              Serial.print(" >>> posAct = ");
              Serial.print(posAct);
              Serial.print(" >>> PWM = ");
              Serial.println(outD);
            #endif
    
    
            #if(geraGrafDirec == 1) //habilia o modo para geração de grafico com valor de centro (setpoint) e posição atual
              Serial.print(spD);
              Serial.print(" ");
              Serial.println(posAct);
            #endif
          #endif
          
        }

    }

  //Serial.print("posAct = ");
  //Serial.println(lePosDirecao());
  posAct = lePosDirecao(); //atualiza a posição da direção

  // OBS: OS QUALQUER ERRO ABAIXO DEVE DESABILITAR A DIREÇÃO - não implementado

  if(posMin > posMax){
    erros[cont_erros] = 0xE3; 
    cont_erros++;     
  }

  //Se não houver diferença entre posição minima e máxima gera erro (erro de leitura de sensores)
  //if (posMin == posMax){
  if(!((posMin < posMax*0.9)&&(posMin*1.1 < posMax))){
    erros[cont_erros] = 0xE4; 
    cont_erros++;     
  }

  //Se não houver diferença entre posição máxima e a atual gera erro
  //if (posAct == posMax){
  if (posAct >= posMax*0.9){//leva em conta uma variação de 10% para menos do valor maior 
    erros[cont_erros] = 0xE5; 
    cont_erros++;     
  }

  //Se não houver diferença entre posição atual e a minima gera erro 
  //if (posAct == posMin){
  if (posAct <= posMin*1.1){ //leva em conta uma variação de 10% para mais do valor menor
    erros[cont_erros] = 0xE6; 
    cont_erros++;     
  }


    //Se não centralizar a direçao (com potenciometro) (tolerancia de 10%)
    if ((posAct >= posCenter*1.1)||(posAct <= posCenter*0.9)){
      erros[cont_erros] = 0xE7; 
      cont_erros++;     
    }

  
  
  if(cont_erros > 0){

    erroCalibraDirecao = erros[cont_erros - 1];

    Serial.print("Erros: ");
    for(int i = 0; i < cont_erros; i++){
      Serial.print(erros[i], HEX);
      Serial.print(" ");
    }
    Serial.println();

    //CAN.sendMsgBuf(idMsgCANSend, 0, 8, erros); //envia a msg CAN com os erros encontrados
  
    //envia a resposta com os erros via serial
    //Serial.write(0xAA);
    //Serial.write(cont_erros + 3);
    //Serial.write(0x30); //dev

    /*
    for(int i = 0; i < cont_erros; i++){
      //Serial.write(erros[i]); //envia os erros identificados (modo HEX)
      erroCalibraDirecao = erros[i]; //apenas o ultimo erro fica gravado no registrador
  
      #if(debugar == 1) //verifica se o modo debugar foi acionado
        #if(geraGrafDirec == 0) //habilia o modo texto do debug
          Serial.println(erros[i],DEC); //envia os erros identificados (modo TEXT)
        #endif
      #endif
      
    }

    */

  }
  
 
}
                                                                                                                                                                                                                            

//Faz a media aritmética das medidas analógicas de A0 (le o valor do potenciometro para controle das rodas dianteiras)
  float lePosDirecao(){       
  
   float pos = 0;
      
   for(int i = 0; i<=64; i++){
      pos = pos + analogRead(potDir);
   }

    return pos/64;
    
}
