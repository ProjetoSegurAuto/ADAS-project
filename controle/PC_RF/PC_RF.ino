//Code for Receiver
// MODULO CONECTADO AO PC (UNO)
// RECEBE DADOS VIA SERIAL E ENVIA VIA RF OU RECEBE DADOS VIA RF E ENVIA VIA SERIAL

#include <Arduino.h>
#include <esp_now.h>
#include <SPI.h>
#include <WiFi.h>

//REPLACE WITH YOUR RECEIVER MAC Address
uint8_t broadcastAddress[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
uint8_t MACAdressOfSender[] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00};

/*
uint8_t MACThisDevice[] = {0xA4, 0xCF, 0x12, 0x9A, 0xA9, 0xB4};
uint8_t car1Address[] = {0xA8, 0x03, 0x2A, 0x19, 0xB1, 0x08};
uint8_t car2Address[] = {0x98, 0xCD, 0xAC, 0xA9, 0x7D, 0xE4};
uint8_t car3Address[] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
*/


//[msgPlatoon(2 bytes) idMsgCAN(2 bytes) dadosCAN(8 bytes)]
typedef struct struct_message {
  byte msgPlatoon[2] = {0x00, 0x00};
  byte idMsgCAN[2] = {0x00, 0x00};
  byte dadosCAN[8] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
} struct_message;


struct_message structMsgRFRec;
struct_message incomingReadings;


unsigned long intervalMsg;//conta tempo entre as msgs

boolean flagRecRF = false; //flag q indica q uma msg chegou via RF
boolean flagRecSerial = false; //flag q indica q uma msg chegou via Serial

byte bufRecMsgRF[12];//usado para receber dados via RF [MSGPlatoon(2 bytes) idMsgCAN(2 bytes) dadosCAN(8 bytes)]
//byte bufSerial[18] = {0x00}; //usado para receber texto via serial [MACCAR(6 bytes) MSGPlatoon(2 bytes) idMsgCAN(2 bytes) dadosCAN(8 bytes)]

unsigned long tLeituraSerial = 0;//usada para controlar o tempo q leitura de dados vindos via serial

void trataMsgRecRF();
void sendMsg();

// callback when data is sent
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status){
  //Serial.print("\r\nLast Packet Send Status:\t");
  //Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
}


// callback function that will be executed when data is received
void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len) {
  
  flagRecRF = true;//indica q uma msg chegou via RF
 
  memcpy(&incomingReadings, incomingData, sizeof(incomingReadings));

  memcpy(MACAdressOfSender, mac, sizeof(MACAdressOfSender));
  
  memcpy(structMsgRFRec.msgPlatoon, incomingReadings.msgPlatoon, sizeof(structMsgRFRec.msgPlatoon));
  memcpy(structMsgRFRec.idMsgCAN, incomingReadings.idMsgCAN, sizeof(structMsgRFRec.idMsgCAN));
  memcpy(structMsgRFRec.dadosCAN, incomingReadings.dadosCAN, sizeof(structMsgRFRec.dadosCAN));

}


void setup(){

  Serial.begin(115200);
  
  WiFi.mode(WIFI_STA);
  Serial.println(WiFi.macAddress());
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }

  
  esp_now_register_send_cb(OnDataSent); //callback
  esp_now_register_recv_cb(OnDataRecv); //callback
  

  delay(1000); 
}


void loop(){

  sendMsg();//verifica se chegaram dados via serial e envia via RF

  if(flagRecRF == true){//Verifica se chegaram dados via RF
    trataMsgRecRF();    
  } 
  
}


//rotina que trata quando há msg RF no controlador
void trataMsgRecRF(){

  flagRecRF = false; //limpa o flag

  
  // print the data
  for (int i = 0; i < sizeof(MACAdressOfSender); i++){
    Serial.write(MACAdressOfSender[i]); 
  }
  
  
  for(int i = 0; i < sizeof(structMsgRFRec.msgPlatoon); i++){
    Serial.write(structMsgRFRec.msgPlatoon[i]);    
  }
  
  for(int i = 0; i < sizeof(structMsgRFRec.idMsgCAN); i++){
    Serial.write(structMsgRFRec.idMsgCAN[i]);    
  }
  
  for(int i = 0; i < sizeof(structMsgRFRec.dadosCAN); i++){
    Serial.write(structMsgRFRec.dadosCAN[i]);    
  }
  
  Serial.println();//marca o fim da msg 0x0d 0x0a
  
}

 
//le e os dados vindos via Serial e envia via RF
void sendMsg(){

  struct_message structMsgRFSend;

  int contFields = 0; //conta os caracteres q serão enviados
  byte destinoMsgRF[6]; //id do destino da mensagem [MACCAR(6 bytes)]

  byte bufSerialSendRF[12] = {0x00}; //usado para receber texto via serial [MSGPlatoon(2 bytes) idMsgCAN(2 bytes) dadosCAN(8 bytes)]

  if(Serial.available() > 0){
    tLeituraSerial = millis();//inicia a contagem
    while((millis() - tLeituraSerial) < 200){ //Rotina q determina o tempo de leitura das msg serial (fica preso no loop lendo os dados vindos da serial)
      
      if(Serial.available() > 0){
        byte b = Serial.read();
        /*
        if(contFields < 6){ //os dois primeiros bytes são do endereço do testino da msg
          destinoMsgRF[contFields] = b;
        }else{
          bufSerialSendRF[contFields - 6] = b; //os outros bytes são a msg
        }
        */

        if(contFields < 6){ //os dois primeiros bytes são do endereço do testino da msg
          destinoMsgRF[contFields] = b;
        }else if(contFields < 8){
          structMsgRFSend.msgPlatoon[contFields - 6] = b; //
        }else if(contFields < 10){
          structMsgRFSend.idMsgCAN[contFields - 8] = b; //
        }else{
          structMsgRFSend.dadosCAN[contFields - 10] = b; //
        }
        
        contFields++;
        
        //bufSerial[contFields] = b;
        //contFields++;
      
      }
      
    }
    
  }


  if(contFields >= 10){//se receber um pacote de 10 byes via serial envia ele via RF

    for(int i = 0; i < sizeof(destinoMsgRF); i++){
      Serial.write(destinoMsgRF[i]);    
    }

    for(int i = 0; i < sizeof(structMsgRFSend.msgPlatoon); i++){
      Serial.write(structMsgRFSend.msgPlatoon[i]);    
    }

    for(int i = 0; i < sizeof(structMsgRFSend.idMsgCAN); i++){
      Serial.write(structMsgRFSend.idMsgCAN[i]);    
    }

    for(int i = 0; i < sizeof(structMsgRFSend.dadosCAN); i++){
      Serial.write(structMsgRFSend.dadosCAN[i]);    
    }
  
    Serial.println("");
   

    //A8:03:2A:19:B1:08
    //uint8_t testeAdrr[] = {0xA8, 0x03, 0x2A, 0x19, 0xB1, 0x08};

  
    if(!esp_now_is_peer_exist(destinoMsgRF)){//testa se o peer ja foi cadastrado
      
      esp_now_peer_info_t peerInfo_Car;
      
      memcpy(peerInfo_Car.peer_addr, destinoMsgRF, 6);
      
      peerInfo_Car.channel = 0;  
      peerInfo_Car.encrypt = false;
      if (esp_now_add_peer(&peerInfo_Car) != ESP_OK){
          Serial.println("Failed to add peer");
          return;
      }
          
   }
       
    esp_err_t result = esp_now_send(destinoMsgRF, (uint8_t *) &structMsgRFSend, sizeof(structMsgRFSend));

  }    

}
  
