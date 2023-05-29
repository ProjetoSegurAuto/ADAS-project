//MODULO CONECTADO AO CARRO - PONTE RF - CAN

#include <Arduino.h>
#include <esp_now.h>
#include <WiFi.h>
#include <SPI.h>

//https://nrf24.github.io/RF24/index.html
//https://maniacbug.github.io/RF24/classRF24.html


// REPLACE WITH YOUR RECEIVER MAC Address
uint8_t broadcastAddress[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
uint8_t MACAdressOfSender[] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00}; //MAC do device q enviou a msg RF

esp_now_peer_info_t peerInfo;


const int RF_INT_PIN = 26; //pino que gera a interrupção ao receber uma msg via RF


//https://wiki.seeedstudio.com/CAN-BUS_Shield_V2.0/#4get-can-id
#define CAN_2515

const int SPI_CS_PIN = 5; //CAN = VSPI (CSN=D5,MOSI=D23,MISO=D19,SCK=18)
const int CAN_INT_PIN = 27; //D27 = pino que gera a interrupção ao receber uma msg via CAN

#include "mcp2515_can.h"
mcp2515_can CAN(SPI_CS_PIN);

boolean flagCANInit = true; //se false indica que o modulo CAN não foi inicializado com sucesso

/**********************************************************************************/

unsigned long intervalMsg;//conta tempo entre as msgs
unsigned long  intervalTeste = 0;

boolean flagRecRF = false; //flag q indica q uma msg chegou via RF
boolean flagRecCAN = false; //flag q indica q uma msg chegou via CAN

/**********************************************************************************/
/*     buffer com info das das ECUS       */
//[MSGPlatoon(2 bytes) idMsgCAN(2 bytes) dadosCAN(8 bytes)]
byte bufSendMsgRF[12] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}; //usado para armazenar os dados recebidos via CAN
byte bufRecMsgRF[12] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}; //usado para armazenar os dados recebidos via RF

unsigned long timeOutRF = 0; //varivel q controla o tempo de envio dos buffers via RF (caso não haja requisição via RF)

void MsgRecCAN();
void trataMsgRecCAN();
void MsgRecRF();
void trataMsgRecRF();
void requestData();
void enviaMsgCAN();




//[msgPlatoon(2 bytes) idMsgCAN(2 bytes) dadosCAN(8 bytes)]
typedef struct struct_message  {
  byte msgPlatoon[2] = {0x00, 0x00};
  byte idMsgCAN[2] = {0x00, 0x00};
  byte dadosCAN[8] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
} struct_message;



struct_message structMsgRFSend;
struct_message structMsgRFRec;
struct_message incomingReadings;


// callback when data is sent
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  //Serial.print("\r\nLast Packet Send Status:\t");
  //Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
}


// callback function that will be executed when data is received
void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len){

  flagRecRF = true;
  
  memcpy(&incomingReadings, incomingData, sizeof(incomingReadings));

  memcpy(MACAdressOfSender, mac, sizeof(MACAdressOfSender));
  
  memcpy(structMsgRFRec.msgPlatoon, incomingReadings.msgPlatoon, sizeof(structMsgRFRec.msgPlatoon));
  memcpy(structMsgRFRec.idMsgCAN, incomingReadings.idMsgCAN, sizeof(structMsgRFRec.idMsgCAN));
  memcpy(structMsgRFRec.dadosCAN, incomingReadings.dadosCAN, sizeof(structMsgRFRec.dadosCAN));
  
}


void setup(){
  
  Serial.begin(115200);
  Serial.println("INICIANDO ECU COMMUNICATION...");

  WiFi.mode(WIFI_STA);
  Serial.println(WiFi.macAddress());
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }
  
  
  // Register peer
  memcpy(peerInfo.peer_addr, broadcastAddress, 6);
  peerInfo.channel = 0;  
  peerInfo.encrypt = false;

  
  // Add peer        
  if (esp_now_add_peer(&peerInfo) != ESP_OK){
    Serial.println("Failed to add peer");
    return;
  }
  
  
  esp_now_register_recv_cb(OnDataRecv);
  esp_now_register_send_cb(OnDataSent);

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

  attachInterrupt(digitalPinToInterrupt(CAN_INT_PIN), MsgRecCAN, FALLING); // declara a interrupção gerada a qndo uma msg chega via CAN

  intervalTeste = millis();

  byte msgCAN [] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}; //variavel não usada para nada
  
  CAN.sendMsgBuf(0x20, 0, 8, msgCAN); //envia a msg CAN para mostrar q a ECU está funcional

  timeOutRF = millis();

  Serial.println("ECU Comunicação RUNNING...");
  
}


void loop(){
    
  if((millis() - timeOutRF) > 1000){ //se não houver requisição o sistema em 1s envia os dados mesmo assim
   //Serial.println("Auto update RF"); 
   //requestData();
  }
  

  if(flagRecCAN == true){
    trataMsgRecCAN();
  }
  

  if(flagRecRF == true){
    trataMsgRecRF();
  }
  
}


//interrupção gerada qndo o modulo CAN recebe uma msg via CAN
void MsgRecCAN(){
    //detachInterrupt(digitalPinToInterrupt(CAN_INT_PIN)); // Caso essa boa prática não seja implementada, há problema de interrupt timetout
    flagRecCAN = true; //seta o flag q indica que há uma msg CAN no buffer
} 


//rotina que trata quando há msg CAN no controlador
void trataMsgRecCAN(){
  
  flagRecCAN = false;  // limpa a flag
  byte bufCANAux[8]; //recebe a msg vinda via CAN
  
  //Serial.println("Message CAN recived...");

  uint32_t idRecCAN = 0; //id da msg CAN
  unsigned char len = 0;


  while (CAN_MSGAVAIL == CAN.checkReceive()){

      // read data,  len: data length, buf: data buf
      //CAN.readMsgBuf(&len, bufCANAux);
      //idRecCAN = CAN.getCanId(); 

      CAN.readMsgBuf(&len, structMsgRFSend.dadosCAN);
      idRecCAN = CAN.getCanId(); 

      uint8_t type = (CAN.isExtendedFrame() << 0) | (CAN.isRemoteRequest() << 1); // Se 0: ext, Se 1: rtr

      //bufSendMsgRF[0] =  0x00;//msgPlatoon
      //bufSendMsgRF[1] =  0x00;//msgPlatoon

      //bufSendMsgRF[2] = highByte(idRecCAN); //o id do device é colocado no incio do pacote enviado via RF
      //bufSendMsgRF[3] = lowByte(idRecCAN); //o id do device é colocado no incio do pacote enviado via RF

      structMsgRFSend.msgPlatoon[0] = 0x00;
      structMsgRFSend.msgPlatoon[1] = 0x00;

      structMsgRFSend.idMsgCAN[0] = highByte(idRecCAN);
      structMsgRFSend.idMsgCAN[1] = lowByte(idRecCAN);
  
      /****************************************/
      Serial.print("Recieve by CAN: id ");

      Serial.print(idRecCAN);
      Serial.print(" (");
      Serial.print(idRecCAN, HEX);
      Serial.print(")HEX");
      Serial.print("\t");
      
      // print the data
      //for (int i = 1; i < sizeof(bufCAN); i++){
      for (int i = 0; i < len; i++){

        //[MSGPlatoon(2 bytes) idMsgCAN(2 bytes) dadosCAN(8 bytes)]
        //bufSendMsgRF[i+4] = bufCANAux[i];//copia os dados recebidos via CAN pra o buffer que envia os dados via RF
                
        //Serial.print(bufCANAux[i], HEX); 
        Serial.print(structMsgRFSend.dadosCAN[i], HEX); 
        Serial.print("\t");
        
      }
      
      Serial.println();
  }

  enviaMsgRF();
  
  //attachInterrupt(digitalPinToInterrupt(CAN_INT_PIN), MsgRecCAN, FALLING);
  
}


//rotina que trata quando há msg RF no controlador
void enviaMsgRF(){

  /*
  //MOSTRA A MSG ENVIADA VIA RF (DA CAN PARA RF)
  Serial.print("MSG to RF: ");
  for (int i = 0; i < sizeof(bufSendMsgRF); i++){
    Serial.print(bufSendMsgRF[i], HEX); 
    Serial.print("\t");
  }

  Serial.println("");
  */

  
  //envia a MSG via RF
  esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *) &structMsgRFSend, sizeof(structMsgRFSend));
  
}  


//rotina que envia msg via CAN (recebe a msg via RF envia via CAN)
void trataMsgRecRF(){

  flagRecRF = false;

  //[MSGPlatoon(2 bytes) idMsgCAN(2 bytes) dadosCAN(8 bytes)] FORMATO DA MSG

  /*
  for(int i = 0; i < sizeof(structMsgRF.msgPlatoon); i++){
    Serial.write(structMsgRF.msgPlatoon[i]);    
  }
  */
       
  Serial.print("Recieve by RF: MAC ");

  //MAC do device q enviou o msg via RF
  for (int i = 0; i < sizeof(MACAdressOfSender); i++){ 
    Serial.print(MACAdressOfSender[i], HEX); 
    Serial.print(":");  
  }

  Serial.print("\t");


  for(int i = 0; i < sizeof(structMsgRFRec.msgPlatoon); i++){
    Serial.print(structMsgRFRec.msgPlatoon[i], HEX);
    Serial.print("\t");
  }
  
  for(int i = 0; i < sizeof(structMsgRFRec.idMsgCAN); i++){
    Serial.print(structMsgRFRec.idMsgCAN[i], HEX);
    Serial.print("\t");  
  }
  
  for(int i = 0; i < sizeof(structMsgRFRec.dadosCAN); i++){
    Serial.print(structMsgRFRec.dadosCAN[i], HEX);
    Serial.print("\t");
  }
  
  Serial.println("");
  
  //CAN.sendMsgBuf((bufRecMsgRF[2]<<8) + (bufRecMsgRF[3])), 0, 8, bufRF); //envia do dado recebido por RF via CAN
  CAN.sendMsgBuf(structMsgRFRec.idMsgCAN[1], 0, 8, structMsgRFRec.dadosCAN); //envia do dado recebido por RF via CAN
  
}
