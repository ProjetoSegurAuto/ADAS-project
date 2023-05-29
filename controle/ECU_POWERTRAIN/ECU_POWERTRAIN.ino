
/* VERSÃO 1.0 rebuilt */



//############################################################################################################################################//
#include <math.h>

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

boolean flagCANInit = false; //se false indica que o modulo CAN não foi inicializado com sucesso

//unsigned int idThisDevice = 105; //id desse dispositivo CAN
unsigned int idMsgCANRec = 0; //essa variavel armazena o id dispositivo q envou uma msg CAN

boolean flagRecv = false; //se true indica q uma msg foi recebida via CAN
                                                    
unsigned long tControle = 0; //contador que controla o intervalo de tempo q o PID irá ser aplicado (verificar necessidade)
unsigned long tEnviaMsgCAN = 0; //contador que controla o intervalo de tempo q as msg CAN serão enviadas (padrão 50ms)
unsigned long tFalhaCentral = 0; //contador que verificar se as msgs da VECTOR/ZED (central) estão chegando

byte bufCANRec[8] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}; //usado para armazenar os dados recebidos via CAN


/* LIB PARA USO DO WATCHDOG */
#include <avr/wdt.h> //Carraga os comandos do watchdog (para reset do uC)


/* LIB PARA USO DA EEPROM DO ARDUINO */
#include <EEPROM.h>//Carrega a biblioteca para uso da EEPROM interna 



/* DIRETIVAS PARA AJUSTES DE FUNCIONAMENTO DO PROGRAMA */
//HABILITA OU DESABILITA O MODO DEBUG (MSG OU GRAFICOS VIA SERIAL )
#define debugar 1 //0 - O programa funciona normalmente, 1 - o programa entra em modo debug e envia dados via serial

//DEFINE A FORMA DE LEITURA DOS SENSORES DE VELOCIDADE (ACOMPLADO AS RODAS TRAZEIRAS)
#define modoLeituraVel 5 //3 - modo media móvel (mede o tempo entre pulsos e faz a media movel), 4 - modo leitura lenta (conta a quantidade de pulsos em 500ms) com media,  5 - modo leitura lenta (conta a quantidade de pulsos em 500ms) com media movel

#define geraGrafVel 9 //USADO NA RORTINA DE CONTROLE DA VELOCIDADE COM PID, 0 - (GERA TEXTO NA SERIAL) envia, via serial, texto com valores de setpoint, medida de velocidade e valor de pwm, 1 - (GERA GRAFICO) envia numeros para gerar grafico de setpoint e velocidade no plotter serial (IDE arduino)
#define geraGrafMedVel 9 //USADO NA ROTINA PARA MEDIR A VELOCIDADE (DADOS CRUS),  0 - (GERA TEXTO NA SERIAL) envia via serial, texto com valores de velociade e ... , 1 - envia numeros para gerar grafico de velocidade e ...

#define diametroRoda 95.7/100 //diametro da roda em metros (95.7 cm)


/* VARIAVEIS USADA PARA CONFIGRA OS PINOS DE ACIONAMENTO DAS PONTE H */
//Ponte H1  BTS 7960 (roda esquerda)
#define R_PWM1 5
#define L_PWM1 4

//Ponte H2  BTS 7960 (roda direita)
#define R_PWM2 7
#define L_PWM2 6

#define pinSinaliza 41 //usado para sinalizar


/* VARIAVEIS USADAS PARA MEDIR A VELOCIDADE DOS MOTORES TRAZEIROS */
//usados para contar os pulsos (numero de interrupções) medir frequencia de rotação
double contFreqEsq = 0, contFreqAuxEsq = 0, freqAuxEsq = 0;
double contFreqDir = 0, contFreqAuxDir = 0, freqAuxDir = 0;

//pinos q geram interrupção para medir frequencia de rotação dos pneus trazeiros
const byte interruptPinMotorDir = 19; //sensor de velocidade motor direito
const byte interruptPinMotorEsq = 20; //sensor de velocidade motor esquerdo

//usado para travar a contagem de pulsos
boolean flagFreqDir = false, flagFreqEsq = false;


//usado pra medir tempo (ou frequencia de rotação das rodas trazeiras)
unsigned long tStartFreqDir, tStartFreqEsq; //usados para contar tempo nas rotinas q medem a rotação dos motores, tambem é usado para detectar se o sensor parou de receber pulsos (indica pneu parado ou sensor com problema) (tb evita o bug q retem sempre o ultimo valor medido)

//variaveis usadas para calculo de medias moveis da frequencia de rotação das rodas trazeiras
#define samplesAvgMov 20 //define quantas leituras serão feitas (usada no calculo da media movel)
unsigned long avgMovMotorEsq[samplesAvgMov] = {0}; //vetor com medidas de tempo do motor esquedo
unsigned long avgMovMotorDir[samplesAvgMov] = {0}; //vetor com medidas de tempo do motor direito

boolean flagAmostrasDir = false, flagAmostrasEsq = false;//evita bug nas medidas iniciais (buffer de medias moveis vzaio - com valores zero)

boolean flagEndoderRotateEsq = false, flagEndoderRotateDir = false; //Se true indicam q o encoder rotativo apresentou um problema ou q o pneu parou de girar


/* LIB PARA USO DO CONTROLE PID */
#include <PID_v1.h> //Carrega a biblioteca dos PIDs

double spMdir = 0, outMdir = 0,  spMesq = 0, outMesq = 0;//variaveis do PID usadas  nos motores
unsigned char  flagFrenteReEsq = 0; //Se 0 faz o carro deve ir pra frente - Se 1 faz o carro ir pra traz
unsigned char  flagFrenteReDir = 0; //Se 0 faz o carro deve ir pra frente - Se 1 faz o carro ir pra traz


boolean flagPID = false; //se true habilita o PID, se false desabilita o PID (msg 0x05, 0x07 e 0x09 desligam o PID) (msg 0x0D, 0x0F e 0x011 liga o PID)

unsigned int distanciaAlvo = 0; //recebe a distancia do algo detectado pela camera


/* PID PARA LEITURA DE FREQUENCIA RAPIDA COM MEDIA MÓVEL */
PID pidRodaEsquerda(&freqAuxEsq, &outMesq, &spMesq, 15.9, 0, 0, DIRECT);
PID pidRodaDireita(&freqAuxDir, &outMdir, &spMdir, 16.5, 0, 0, DIRECT);



/* PINOS QUE SERVEM APENAS PARA SINALIZAÇÃO */
#define pinSinalPWMDir 30 //sinaliza a interrupção gerada pelo sinal PWM (usado na rotina de mede os pulsos do motor 1) 
#define pinSinalPWMEsq 31 //sinaliza a interrupção gerada pelo sinal PWM (usado na rotina de mede os pulsos do motor 2)

/* Pino usados para identificar se o motor esquerdo está dirando ao contrário (reverso)  [Para o caso de uso de encoder rotativo] */
#define detcRevDir 24
#define detcRevEsq 23


void setup(){

  wdt_disable();//desabilita o watchdog (usado na rotina de reset)

  Serial.begin(115200); //porta de comunicação com a ide do arduino
  Serial.println("INICIANDO ECU POWERTRAIN...");


  while (!Serial) {
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

    
    //set mask, set both the mask to 0x3ff
    // there are 2 mask in mcp2515, you need to set both of them

    /* aceita de 0x50 a 0x6F */
    CAN.init_Mask(0, 0, 0x7F0);
    
    CAN.init_Filt(0, 0, 0x050);  //    
    CAN.init_Filt(1, 0, 0x050);  //    
  
  
    CAN.init_Mask(1, 0, 0x7F0);
    
    CAN.init_Filt(2, 0, 0x060);  //   
    CAN.init_Filt(3, 0, 0x060);  //  
    CAN.init_Filt(4, 0, 0x060);  //  
    CAN.init_Filt(5, 0, 0x060);  // 



  //PINOS DE SINAL PWM DA RODA ESQUERDA
  pinMode(R_PWM1, OUTPUT);
  pinMode(L_PWM1, OUTPUT);


  //PINOS DE SINAL PWM DA RODA DIREITA
  pinMode(R_PWM2, OUTPUT);
  pinMode(L_PWM2, OUTPUT);

  /* INTERRUPÇÕES PARA MEDIR A FREQUENCIA */
  //www.arduino.cc/reference/pt/language/functions/external-interrupts/attachinterrupt/
  //configura a interrupção externa no pino 19
  pinMode(interruptPinMotorDir, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(interruptPinMotorDir), freqMotorDir, RISING);

  //configura a interrupção externa no pino 20
  pinMode(interruptPinMotorEsq, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(interruptPinMotorEsq), freqMotorEsq, RISING);

  pinMode(detcRevEsq, INPUT); //Pino usado para identificar se o motor esquerdo está dirando ao contrário (reverso) [Para o caso de uso de encoder rotativo]
  pinMode(detcRevDir, INPUT); //Pino usado para identificar se o motor direito está dirando ao contrário (reverso) [Para o caso de uso de encoder rotativo]

  /* ESSES PINOS SINALIZAM A INTERRUPÇÃO PARA MEDIÇÃO DE FREQUENCIA */
  pinMode(pinSinalPWMDir, OUTPUT);
  digitalWrite(pinSinalPWMDir, LOW);
  pinMode(pinSinalPWMEsq, OUTPUT);
  digitalWrite(pinSinalPWMEsq, LOW);

  //turn the PID on
  pidRodaDireita.SetMode(1);//se mudar para modo MANUAL o PID não funciona
  pidRodaDireita.SetSampleTime(100);
  pidRodaDireita.SetOutputLimits(0, 255); //ajusta os limites minimos e maximos da saida do PID

  pidRodaEsquerda.SetMode(1);
  pidRodaEsquerda.SetSampleTime(100);
  pidRodaEsquerda.SetOutputLimits(0, 255); //ajusta os limites minimos e maximos da saida do PID

  
  // LINHAS TESTE
  /*
  //PID motor da Esquerda
  EEPROM.write(0x0A, 0x01);
  EEPROM.write(0x0C, 0x04);
  EEPROM.write(0x0E, 0x01);

  //PID motor da direita
  EEPROM.write(0x1A, 0x01);
  EEPROM.write(0x1C, 0x04);
  EEPROM.write(0x1E, 0x01);
 */

  /*
  double Kp, Ki, Kd;

  //ajusta os ganhos do PID da roda esquerda
  //EEPROM.read(address)
  Kp = double(EEPROM.read(0x0A)) / 10;
  Ki = double(EEPROM.read(0x0C)) / 10;
  Kd = double(EEPROM.read(0x0E)) / 10;

  pidRodaEsquerda.SetTunings(Kp, Ki, Kd);


  //ajusta os ganhos do PID da roda direita
  //EEPROM.read(address)
  Kp = double(EEPROM.read(0x1A)) / 10;
  Ki = double(EEPROM.read(0x1C)) / 10;
  Kd = double(EEPROM.read(0x1E)) / 10;

  pidRodaDireita.SetTunings(Kp, Ki, Kd);
  */
  
  //Serial.println(Kp); Serial.println(Ki); Serial.println(Kd);//linha teste

  //Desliga todos os motores
  desligaMotores();

  delay(1000);//espera 1 segundo para inicar o programa

  pinMode(pinSinaliza, OUTPUT);//usado para sinalizar qndo a rotina principal esta ativa

  attachInterrupt(digitalPinToInterrupt(CAN_INT_PIN), MsgRecCAN, FALLING); // declara a interrupção gerada a qndo uma msg chega via CAN

  pidRodaEsquerda.SetTunings(0.3, 0.5, 0.01); 
  pidRodaDireita.SetTunings(0.3, 0.5, 0.01); 

  CAN.sendMsgBuf(0x50, 0, 8, bufCANRec); //envia a msg CAN para mostrar q a ECU está funcional

  // Ativamos o watchdog e definimos o tempo de 2 seg.
  wdt_enable(WDTO_2S); // WDTO_1S, WDTO_2S, WDTO_4S, WDTO_8S

  Serial.println("ECU Powertrain RUNNING...");

}


void loop() {

  wdt_reset(); //reinicia o watchdog

  //medeVelocidade();//envia informações via serial sobre a velocidade e PWM (PARA TESTES)
  if((millis() - tControle) > 10){ //Rotina q determina o intervalo q o PID ira aplicar a logica de controle (100 = 100ms) (verificar nescessidade)
    tControle = millis();//reinicia a contagem
    //if (flagPID == true){ //liga o PID (msg 0x05, 0x07 e 0x09 desligam o PID) (msg 0x0D, 0x0F e 0x011 liga o PID)
      controleDeVelocidade();
    //}
  }

  monitoraSensoresRotativos();//CRIAR AVISOS DE ERROS EM CASO DE FALHA


  if ((millis() - tEnviaMsgCAN) > 269) { //Rotina q determina o intervalo de envio das mensgens periódicas
    tEnviaMsgCAN = millis();//reinicia a contagem
    enviaMsgStatus();    
  }


  if (flagRecv == true) {
    trataMsgRecCAN(); //rotina q trata qndo uma msg chega via can
  }

}


//interrupção gerada qndo o modulo CAN recebe uma msg via CAN
void MsgRecCAN() {
  flagRecv = true; //flag que indica que uma msg foi recebida via CAN
  //Serial.println("teste power");
}


//rotina que envia msg periodicas via CAN sobre o status atual da ECU
void enviaMsgStatus(){

  /*
      FORMATO DA MSG
     |   BYTE 0   |   BYTE 1   |    BYTE 2     |    BYTE 3     |    BYTE 4    |    BYTE 5   |    BYTE 6   |         BYTE 7        |
     |      -     |    PIDs    |    msgType    |  RPM Esquerda |  Sp PWM Esq  | RPM Direita | Sp RPM Dir  |      Sinal / Erro     |
     |            |  0x00-0x1  |     0x00      |   0x00-0xFF   |   0x00-0x09  |  0x00-0xFF  |  0x00-0x09  | 0x00, 0x01, 0xE0-0xEF |

  */

  
  bufCANRec[0] = 0x00;//reservado

  //verifica se os PIDs estão ligados
  if (flagPID == true) {
    bufCANRec[1] = 0x01;
  } else {
    bufCANRec[1] = 0x00;
  }


  bufCANRec[2] = 0x00;

  bufCANRec[3] = int(freqAuxEsq); 
  //bufCANRec[4] = int((freqAuxEsq - int(freqAuxEsq)) * 10);
  bufCANRec[4] = int(spMesq);
  
  bufCANRec[5] = int(freqAuxDir); 
  //bufCANRec[6] = int((freqAuxDir - int(freqAuxDir)) * 10);
  bufCANRec[6] = int(spMdir);


  //verifica se há algum erro nos encoder rotativos
  if ((flagEndoderRotateEsq == false) && (outMdir > 0)&& (flagEndoderRotateDir == true)) {
    bufCANRec[7] = 0xE1;
  } else if ((outMesq > 0) && (flagEndoderRotateEsq == true) && (flagEndoderRotateDir == false)) {
    bufCANRec[7] = 0xE2;
  } else if ((outMesq > 0) && (flagEndoderRotateEsq == true) && (outMdir > 0) && (flagEndoderRotateDir == true)) {
    bufCANRec[7] = 0xE3;
  }else{
    bufCANRec[7] = 0x00;
  }
  

  CAN.sendMsgBuf(0x50, 0, 8, bufCANRec); //envia a msg CAN de resposta
  
}



//função responsavel por monitorar se os pneus pararam de girar ou se houve algum problema nos encoders rotativos
//se não receber pulsos num intervalo de tempo, gera a msg de erro
void monitoraSensoresRotativos() {

  //so dispara o alarme caso a velocidade setada seja diferente de zero (não dispara o alarme se a velocidade for configurada para zero)
  //if (outMesq > 0){
    //gera erro de timeout para o motor esquerdo
    if ((millis() - tStartFreqEsq) > 1500) { //verifica se o sensor do motor esquerdo não receber sinal da roda (roda parada, senso com problema) em 2 segundos gera o aviso de erro
      tStartFreqEsq = millis(); //reinicia a contagem
      //zera variaveis de contagem de pulsos
      freqAuxEsq = 0;
      contFreqAuxEsq = 0;
      flagEndoderRotateEsq = true; //indica erro no encoder
      //escrever cod q manda parar o pneu
    }
  //}


  //if (outMdir > 0){
    if ((millis() - tStartFreqDir) > 1500) { //verifica se o sensor do motor direito não receber sinal da roda (roda parada, senso com problema) em 2 segundos gera o aviso de erro
      tStartFreqDir = millis();//reinicia a contagem
      //zera variaveis de contagem de pulsos
      freqAuxDir = 0;
      contFreqAuxDir = 0;
      flagEndoderRotateDir = true; //indica erro no encoder
      //escrever cod q manda parar o pneu
    }
  //}

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



//verifica qual dev deve ser acionado (de acordo com msg recebida)
void executaFuncao(){

  byte bufCANResp[8] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}; //usado para enviar a resposta via CAN


  //AJUSTA A VELOCIDADE DO MOTOR ESQUERDO
  if(idMsgCANRec == 0x52){

    flagPID = true; //liga o PID

    flagFrenteReEsq = bufCANRec[6]; //define se o carro anda pra frente ou pra tras  
    spMesq = bufCANRec[7]; //ajusta a velocidade do carro
    
  }


  //AJUSTA A VELOCIDADE DO MOTOR DIREITO    
  if(idMsgCANRec == 0x54){

    flagPID = true; //liga o PID

    flagFrenteReDir = bufCANRec[6]; //define se o carro anda pra frente ou pra tras
    spMdir = bufCANRec[7]; //ajusta a velocidade do carro
 
  }


  //AJUSTA A VELOCIDADE AMBOS MOTORES    
  if(idMsgCANRec == 0x56){

    flagPID = true; //liga o PID

    flagFrenteReEsq = bufCANRec[4]; //define se o carro anda pra frente ou pra tras
    flagFrenteReDir = bufCANRec[6]; //define se o carro anda pra frente ou pra tras

    spMesq = bufCANRec[5]; //ajusta a velocidade do carro
    spMdir = bufCANRec[7]; //ajusta a velocidade do carro
 
  }


  //AJUSTA O PWM DO MOTOR ESQUERDO
  if(idMsgCANRec == 0x58){

    flagPID = false; //desliga o PID

    flagFrenteReEsq = bufCANRec[6]; //define se o carro anda pra frente ou pra tras
    outMesq = bufCANRec[7];
    
    //foward
    if (flagFrenteReEsq == 0x01) {
      analogWrite(L_PWM1, 0);
      analogWrite(R_PWM1, outMesq);
    }
  
    //reverse
    if (flagFrenteReEsq == 0x02) {
      analogWrite(L_PWM1, outMesq);
      analogWrite(R_PWM1, 0);
    }
  
  }


  //AJUSTA O PWM DO MOTOR DIREITO
  if(idMsgCANRec == 0x5A){

    flagPID = false; //desliga o PID

    flagFrenteReDir = bufCANRec[6]; //define se o carro anda pra frente ou pra tras
    outMdir = bufCANRec[7];
    
    //foward
    if (flagFrenteReDir == 0x01) {
      analogWrite(L_PWM2, 0);
      analogWrite(R_PWM2, outMdir);
    }
  
    //reverse
    if (flagFrenteReDir == 0x02) {
      analogWrite(L_PWM2, outMdir);
      analogWrite(R_PWM2, 0);
    }
    
  }


  //AJUSTA O PWM DE AMBOS MOTORES
  if(idMsgCANRec == 0x5C){

    flagPID = false; //desliga o PID

    flagFrenteReEsq = bufCANRec[4]; //define se o carro anda pra frente ou pra tras
    outMesq = bufCANRec[5];

    flagFrenteReDir = bufCANRec[6]; //define se o carro anda pra frente ou pra tras
    outMdir = bufCANRec[7];

      
    //foward
    if (flagFrenteReEsq == 0x01) {
      analogWrite(L_PWM1, 0);
      analogWrite(R_PWM1, outMesq);
    }
  
    //reverse
    if (flagFrenteReEsq == 0x02) {
      analogWrite(L_PWM1, outMesq);
      analogWrite(R_PWM1, 0);
    }
    
    //foward
    if (flagFrenteReDir == 0x01) {
      analogWrite(L_PWM2, 0);
      analogWrite(R_PWM2, outMdir);
    }
  
    //reverse
    if (flagFrenteReDir == 0x02) {
      analogWrite(L_PWM2, outMdir);
      analogWrite(R_PWM2, 0);
    }
    

  }


  //SETA OS GANHOS DO PID DO MOTOR ESQUERDO
  if(idMsgCANRec == 0x5E){

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
    
    pidRodaEsquerda.SetTunings(Kp, Ki, Kd);
    

  }


  //SETA OS GANHOS DO PID DO MOTOR DIREITO     
  if(idMsgCANRec == 0x60){

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
    
    pidRodaDireita.SetTunings(Kp, Ki, Kd);

    
  }


  //LE OS GANHOS DO PID DA ROTA ESQUERDA
  if(idMsgCANRec == 0x62){

    unsigned int gain = 0;

    gain = 100*pidRodaEsquerda.GetKp();
    bufCANResp[2] = highByte(gain);
    bufCANResp[3] = lowByte(gain);

    gain = 100*pidRodaEsquerda.GetKi();
    bufCANResp[4] = highByte(gain);
    bufCANResp[5] = lowByte(gain);

    gain = 100*pidRodaEsquerda.GetKd();
    bufCANResp[6] = highByte(gain);
    bufCANResp[7] = lowByte(gain);

    //Serial.println(bufCANResp[2]);
    //Serial.println(bufCANResp[3]);
 
    CAN.sendMsgBuf(idMsgCANRec + 1, 0, 8, bufCANResp); //envia a msg CAN de resposta

  }


  //LE OS GANHOS DO PID DA ROTA DIREITA
  if(idMsgCANRec == 0x64){

    unsigned int gain = 0;

    gain = 100*pidRodaDireita.GetKp();
    bufCANResp[2] = highByte(gain);
    bufCANResp[3] = lowByte(gain);

    gain = 100*pidRodaDireita.GetKi();
    bufCANResp[4] = highByte(gain);
    bufCANResp[5] = lowByte(gain);

    gain = 100*pidRodaDireita.GetKd();
    bufCANResp[6] = highByte(gain);
    bufCANResp[7] = lowByte(gain);

    //Serial.println(bufCANResp[2]);
    //Serial.println(bufCANResp[3]);
 
    CAN.sendMsgBuf(idMsgCANRec + 1, 0, 8, bufCANResp); //envia a msg CAN de resposta

  }


  if(idMsgCANRec == 0x66){

    flagPID = false; //desliga o PID

    desligaMotores();

  }
  

  //envia o valor de PWM insntantaneo dos dois motores
  if (idMsgCANRec == 0x68) {

    bufCANResp[4] = flagFrenteReEsq;
    bufCANResp[5] = outMesq;

    bufCANResp[6] = flagFrenteReDir;
    bufCANResp[7] = outMdir;

    CAN.sendMsgBuf(idMsgCANRec + 1, 0, 8, bufCANResp); //envia a msg CAN de resposta
    
  }

  
  //Reseta a ECU
  if (idMsgCANRec == 0x6A) {
    Serial.println("Reseando ECU!!!");
    //trava o arduino ate o watchdog resetar
    while (1) {
      
    }

  }


}


//desativa todos os motores
void desligaMotores() {

  spMdir = 0; //zera o setpoint de velocidade dos motores
  spMesq = 0; //zera o setpoint de velocidade dos motores

  analogWrite(L_PWM1, 0);
  analogWrite(R_PWM1, 0);
  
  analogWrite(L_PWM2, 0);
  analogWrite(R_PWM2, 0);

}



//Controla a velocidade dos motores com PID
void controleDeVelocidade() {


#if(modoLeituraVel == 3)
  /* ROTINA DE CALIBRAÇÃO PARA MEDIÇÃO CURTA - TEMPO ENTRE PULSOS COM MEDIA MOVEL */

    if (contFreqAuxEsq == 0) {
    freqAuxEsq = 0;
  } else {
    //y = 3184,9x^-1,003
    freqAuxEsq = 3115.1 * pow(contFreqAuxEsq, -0.997);
  }

  
  if (contFreqAuxDir == 0) {
    freqAuxDir = 0;
  } else {
    //y = 3184,9x^-1,003
     freqAuxDir = 3115.1 * pow(contFreqAuxDir, -0.997);
  }

#endif


#if(modoLeituraVel == 4)
  /* ROTINA DE CALIBRAÇÃO PARA MEDIÇÃO LONGA - 500 MILISEGUNDOS */

  if (contFreqAuxEsq == 0) {
    freqAuxEsq = 0;
  } else {
    //y = 6,1251x - 10,745
    freqAuxEsq = (6.1251 * contFreqAuxEsq) - 10.745;

  }
  
  if (contFreqAuxDir == 0) {
    freqAuxDir = 0;
  } else {
    //y = 6,1251x - 10,745
    freqAuxDir = (6.1251 * contFreqAuxDir) - 10.745;

  }

#endif


#if(modoLeituraVel == 5)
/* ROTINA DE CALIBRAÇÃO PARA MEDIÇÃO LONGA - 500 MILISEGUNDOS COM MEDIA MOVEL */

  if (contFreqAuxEsq == 0) {
    freqAuxEsq = 0;
  } else {
    //y = 120,07x - 9,839
    freqAuxEsq = (120.07 * contFreqAuxEsq) - 9.839;
    freqAuxEsq = (0.9849 * freqAuxEsq) + 2.59;
  }
  

  if (contFreqAuxDir == 0) {
    freqAuxDir = 0;
  } else {
    //y = 120,07x - 9,839
    freqAuxDir = (120.07 * contFreqAuxDir) - 9.839;
    freqAuxDir = (0.9849 * freqAuxDir) + 2.59;
  }

#endif

 
  /* REMOVER */
  //spMesq = 120; //setpoint rpm
  //spMdir = 120; //setpoint rpm
  //bufCANRec[3]= 0x01; //faz o motor girar pra frente

  if (flagPID == true){ //liga o PID (msg 0x05, 0x07 e 0x09 desligam o PID) (msg 0x0D, 0x0F e 0x011 liga o PID)
    pidRodaEsquerda.Compute(); //faz o calculo da resposta do PID
    pidRodaDireita.Compute(); //faz o calculo da resposta do PID


    //outMdir = 2.1319 * spMdir + outMdir;
    //outMesq = 2.1319 * spMesq + outMesq;
  
  
    //motor gira pra frente
    if (flagFrenteReEsq == 0x01) {
      //Aplica o resultado do PID na ponte H
      analogWrite(L_PWM1, 0);
      analogWrite(R_PWM1, byte(outMesq));
  
      analogWrite(L_PWM2, 0);
      analogWrite(R_PWM2, byte(outMdir));
    }
  
  
    //motor gira pra tras (marcha ré)
    if (flagFrenteReDir == 0x02) {
      
      //Aplica o resultado do PID na ponte H
      analogWrite(L_PWM1, byte(outMesq));
      analogWrite(R_PWM1, 0);
  
      analogWrite(L_PWM2, byte(outMdir));
      analogWrite(R_PWM2, 0);
    }
    
  }

  /*
  Serial.print("spDir: ");
  Serial.print(spMdir);
  Serial.print(" spEsq: ");
  Serial.print(spMesq);
  
  Serial.print(" Mdir: ");
  Serial.print(freqAuxDir);
  Serial.print(" PWMdir: ");
  Serial.print(outMdir);

  Serial.print(" >>> ");

  Serial.print("Mesq: ");
  Serial.print(freqAuxEsq);
  Serial.print(" PWMesq: ");
  Serial.println(outMesq);
  */

}



//Mede a velocidade dos veiculos (Rotina usada em testes)
void medeVelocidade() {

  /*
  char x = 15;

  //motor da esquerda
  analogWrite(L_PWM1, 0);
  analogWrite(R_PWM1, x);

  //motor da direita
  analogWrite(L_PWM2, 0);
  analogWrite(R_PWM2, x);
  */


  /*
  Serial.print("sp: ");
  Serial.print(spMdir);
  Serial.print(" Mdir: ");
  Serial.print(contFreqAuxDir);
  Serial.print(" PWMdir: ");
  Serial.print(outMdir);

  Serial.print(" >>> ");

  Serial.print("Mesq: ");
  Serial.print(contFreqAuxEsq);
  Serial.print(" PWMesq: ");
  Serial.println(outMesq);
  */

  Serial.print("cru: ");
  Serial.print(contFreqAuxDir);
  
  Serial.print(" >>> 3: ");
  Serial.print((3115.1 * pow(contFreqAuxDir, -0.997)));//y = 3115,1x^-0,997 

  Serial.print(" >>> 4: ");
  Serial.print((6.1251 * contFreqAuxDir) - 10.745);//y = 6,1251x - 10,745
  
  Serial.print(" >>> 5: ");
  
  Serial.println((0.9849 * ((120.07 * contFreqAuxDir) - 9.839)) + 2.59);//y = 120,07x - 9,839 -->  y = 0,9849x + 2,59
  

  //Gera Grafico
  /*
    Serial.print(int(spMdir));
    Serial.print(" ");
    Serial.print(int(spMesq));
    Serial.print(" ");
    Serial.print(int(freqAuxDir));
    Serial.print(" ");
    Serial.println(int(freqAuxEsq));
  */


}



//Mede a frequencia de rotação do motor direito
void freqMotorDir() {

  flagEndoderRotateDir = false;  //apaga a flag q indica erro no sensor

  digitalWrite(pinSinalPWMDir, !digitalRead(pinSinalPWMDir)); //sinaliza a interrupção
  
#if(modoLeituraVel == 3)
  //Mede o tempo entre os pulsos e faz a média movel (requer equação para calibração) OBS: Sinal com periodo menor que 20ms confunde a leitura
  if (flagFreqDir == false) {
    flagFreqDir = true;
    tStartFreqDir = millis();
  } else {
    avgMovMotorDir[0] = millis() - tStartFreqDir;
    tStartFreqDir = millis();

    //inicia o vetor com a primeira leitura (repete a primeira leitura em todo o vetor)
    if (flagAmostrasDir == false) {
      flagAmostrasDir = true;
      for (int i = 1; i < int(samplesAvgMov); i++) {
        avgMovMotorDir[i] = avgMovMotorDir[0] ;
      }
    }


    //contFreqAuxDir = 0;
    //soma os elementos do filtro de media movel
    for (int i = 0; i < int(samplesAvgMov); i++) {
      contFreqAuxDir = contFreqAuxDir + avgMovMotorDir[i];
      //contFreqAuxDir += avgMovMotorDir[i];
    }

    //faz o calculo de media movel
    contFreqAuxDir = contFreqAuxDir / int(samplesAvgMov);
  }

  //descola as medidas dentro do vetor (medida zero vai pra posição 1)
  for (int i = samplesAvgMov - 1; i > 0 ; i-- ) {
    avgMovMotorDir[i] = avgMovMotorDir[i - 1];
  }
#endif

#if(modoLeituraVel == 4)
  /*
    //conta os pulsos num espaço de tempo (testado com 500ms)
    if((millis() - tStartFreqDir) <= 500){
    contFreqDir++;
    }
  */

  //conta os pulsos durante 1 segundo
  if (flagFreqDir == false) {
    flagFreqDir = true;
    tStartFreqDir = millis();
    contFreqDir = 0;
  }

  contFreqDir++;

  //verifica se ja passou 1 segundo (1000 ms)
  if ((millis() - tStartFreqDir) >= 501){
    contFreqAuxDir = contFreqDir;
    contFreqDir = 0;
    flagFreqDir = false;
  }
#endif

#if(modoLeituraVel == 5)
// MEDIA MOVEL

  if (flagFreqDir == false) {
    flagFreqDir = true;
    tStartFreqDir = millis();
    contFreqDir = 0;
  }

  contFreqDir++;

  //verifica se ja passou 1 segundo (1000 ms)
  if ((millis() - tStartFreqDir) >= 501){

    //soma os elementos do filtro de media movel
    for (int i = 0; i < int(samplesAvgMov); i++) {
      contFreqAuxDir = contFreqDir + avgMovMotorDir[i];
    }

    //faz o calculo de media movel
    contFreqAuxDir = contFreqAuxDir / int(samplesAvgMov);

    //descola as medidas dentro do vetor (medida zero vai pra posição 1)
    for (int i = samplesAvgMov - 1; i > 0 ; i-- ) {
      avgMovMotorDir[i] = avgMovMotorDir[i - 1];
    }
    
    contFreqDir = 0;
    flagFreqDir = false;
  }

   
#endif


}



//Mede a frequencia de rotação do motor esquerdo
void freqMotorEsq() {

  flagEndoderRotateEsq = false; //apaga a flag q indica erro no sensor

  digitalWrite(pinSinalPWMEsq, !digitalRead(pinSinalPWMEsq)); //sinaliza a interrupção

#if(modoLeituraVel == 3)
  //Mede o tempo entre os pulsos e faz a média movel (requer equação para calibração) OBS: Sinal com periodo menor que 20ms confunde a leitura
  if (flagFreqEsq == false) {
    flagFreqEsq = true;
    tStartFreqEsq = millis();
  } else {
    avgMovMotorEsq[0] = millis() - tStartFreqEsq;
    tStartFreqEsq = millis();

    //inicia o vetor com a primeira leitura (repete a primeira leitura em todo o vetor)
    if (flagAmostrasEsq == false) {
      flagAmostrasEsq = true;
      for (int i = 1; i < int(samplesAvgMov); i++) {
        avgMovMotorEsq[i] = avgMovMotorEsq[0];
      }
    }

    //contFreqAuxEsq = 0;
    //soma os elementos do filtro de media movel
    for (int i = 0; i < int(samplesAvgMov); i++) {
      contFreqAuxEsq = contFreqAuxEsq + avgMovMotorEsq[i];
      //contFreqAuxEsq += avgMovMotorEsq[i];
    }
    
    //faz o calculo de media movel
    contFreqAuxEsq = contFreqAuxEsq / int(samplesAvgMov);
  }

  //descola as medidas dentro do vetor (medida zero vai pra posição 1)
  for (int i = samplesAvgMov - 1; i > 0 ; i-- ) {
    avgMovMotorEsq[i] = avgMovMotorEsq[i - 1];
  }

#endif


#if(modoLeituraVel == 4)
// MEDIA SIMPLES
  /*
    //conta os pulsos num espaço de tempo
    if(((millis() - tStartFreqEsq) <= 500)&&(flagFreqEsq == true)){
    contFreqEsq++;
    }
  */

  if (flagFreqEsq == false) {
    flagFreqEsq = true;
    tStartFreqEsq = millis();
    contFreqEsq = 0;
  }

  contFreqEsq++;

  //verifica se ja passou 1 segundo (1000 ms)
  if ((millis() - tStartFreqEsq) >= 501){
    contFreqAuxEsq = contFreqEsq;
    contFreqEsq = 0;
    flagFreqEsq = false;
  }
#endif


#if(modoLeituraVel == 5)
// MEDIA MOVEL

  if (flagFreqEsq == false) {
    flagFreqEsq = true;
    tStartFreqEsq = millis();
    contFreqEsq = 0;
  }

  contFreqEsq++;

  //verifica se ja passou 1 segundo (1000 ms)
  if ((millis() - tStartFreqEsq) >= 501){

    //soma os elementos do filtro de media movel
    for (int i = 0; i < int(samplesAvgMov); i++) {
      contFreqAuxEsq = contFreqEsq + avgMovMotorEsq[i];
    }

    //faz o calculo de media movel
    contFreqAuxEsq = contFreqAuxEsq / int(samplesAvgMov);

    //descola as medidas dentro do vetor (medida zero vai pra posição 1)
    for (int i = samplesAvgMov - 1; i > 0 ; i-- ) {
      avgMovMotorEsq[i] = avgMovMotorEsq[i - 1];
    }
    
    contFreqEsq = 0;
    flagFreqEsq = false;
  }

   
#endif



}
