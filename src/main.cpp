#include <Arduino.h>

/*
#include <Wire.h>
#include <ESP8266WiFi.h>
#include <SoftwareSerial.h>
#include <WiFiUdp.h>
#include <ArduinoOTA.h>
// #include <FirebaseESP8266.h>
// #include <ESP8266HTTPClient.h>
// #include <ArduinoJson.h>

// Define Constantes do Firebase
//#define FIREBASE_HOST "teste-ndmc-db.firebaseio.com/"                 // URL DATABASE
//#define FIREBASE_AUTH "6hTAnWY6vnHrF4vZB68CA2osuDyejpwdrzcuzJX4"      // SECRET KEY

//Define FirebaseESP8266 data object
// FirebaseData firebaseData;
// FirebaseJson json;

// Define Constantes da conexao WiFi
#define WIFI_SSID "AP 303"                                                
#define WIFI_PASSWORD "c662d8983e" 

bool conexao_status = false;


void iniciarArduinoOTA ();
void iniciarWifi();

void setup() {
  // put your setup code here, to run once:
}

void loop() {
  // put your main code here, to run repeatedly:
}

void iniciarWifi(){
  // TODO : SETAR UM IP FIXO PARA O NODEMCU
  IPAddress ip;

  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  Serial.print("Conectando com o Wi-Fi");
  while (WiFi.status() != WL_CONNECTED)
  {
    Serial.print(".");
    delay(300);
  }
  
  Serial.println();
  Serial.print("CONECTADO COM O IP: ");
  Serial.println(WiFi.localIP());
  Serial.println();

  // Firebase.begin(FIREBASE_HOST, FIREBASE_AUTH);
  // Firebase.reconnectWiFi(true);

  //Set the size of WiFi rx/tx buffers in the case where we want to work with large data.
  // firebaseData.setBSSLBufferSize(1024, 1024);

  //Set the size of HTTP response buffers in the case where we want to work with large data.
  // firebaseData.setResponseSize(1024);

  //Set database read timeout to 1 minute (max 15 minutes)
  // Firebase.setReadTimeout(firebaseData, 1000 * 60);
  //tiny, small, medium, large and unlimited.
  //Size and its write timeout e.g. tiny (1s), small (10s), medium (30s) and large (60s).
  // Firebase.setwriteSizeLimit(firebaseData, "tiny");

  conexao_status = WiFi.status();
  ip = WiFi.localIP();
  // Firebase.setBool(firebaseData, "/Conexao/conexao_status", conexao_status);
  // Firebase.setString(firebaseData, "/Conexao/ip",ip.toString());
  iniciarArduinoOTA();
}

void iniciarArduinoOTA () {
  // Porta padrao do ESP8266 para OTA eh 8266 - Voce pode mudar ser quiser, mas deixe indicado!
  // ArduinoOTA.setPort(8266);
 
  // O Hostname padrao eh esp8266-[ChipID], mas voce pode mudar com essa funcao
  ArduinoOTA.setHostname("teste_gpio");
 
  // Nenhuma senha eh pedida, mas voce pode dar mais seguranca pedindo uma senha pra gravar
  // NÃO FUNCIONA
  // ArduinoOTA.setPassword((const char *)"123");

  ArduinoOTA.onStart([]() {
    Serial.println("Inicio...");
  }); 
  ArduinoOTA.onEnd([]() {
    Serial.println("nFim!");
  });
  ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
    Serial.printf("Progresso: %u%%r", (progress / (total / 100)));
  });
  ArduinoOTA.onError([](ota_error_t error) {
    Serial.printf("Erro [%u]: ", error);
    if (error == OTA_AUTH_ERROR) Serial.println("Autenticacao Falhou");
    else if (error == OTA_BEGIN_ERROR) Serial.println("Falha no Inicio");
    else if (error == OTA_CONNECT_ERROR) Serial.println("Falha na Conexao");
    else if (error == OTA_RECEIVE_ERROR) Serial.println("Falha na Recepcao");
    else if (error == OTA_END_ERROR) Serial.println("Falha no Fim");
  });
  ArduinoOTA.begin();
  Serial.println("Pronto");
  Serial.print("Endereco IP: ");
  Serial.println(WiFi.localIP());
}
*/

#include <Wire.h> // specify use of Wire.h library.

//endereço I2C do MCP23016
#define MCPAddress  0x20

// COMMAND BYTE TO REGISTER RELATIONSHIP : Table: 1-3 of Microchip MCP23016 - DS20090A 
//ENDEREÇOS DE REGISTRADORES
#define GP0   0x00   // DATA PORT REGISTER 0 
#define GP1   0x01   // DATA PORT REGISTER 1 
#define OLAT0   0x02   // OUTPUT LATCH REGISTER 0 
#define OLAT1   0x03   // OUTPUT LATCH REGISTER 1 
#define IPOL0   0x04  // INPUT POLARITY PORT REGISTER 0 
#define IPOL1   0x05  // INPUT POLARITY PORT REGISTER 1 
#define IODIR0  0x06  // I/O DIRECTION REGISTER 0 
#define IODIR1  0x07  // I/O DIRECTION REGISTER 1 
#define INTCAP0 0x08  // INTERRUPT CAPTURE REGISTER 0 
#define INTCAP1 0x09  // INTERRUPT CAPTURE REGISTER 1 
#define IOCON0  0x0A  // I/O EXPANDER CONTROL REGISTER 0 
#define IOCON1  0x0B  // I/O EXPANDER CONTROL REGISTER 1 

void configurePort(uint8_t port, uint8_t custom);
void writeBlockData(uint8_t reg, uint8_t data);
void writePinData(int pin, int value, uint8_t gp);
uint8_t readPin(uint8_t pin, uint8_t gp);
uint8_t valueFromPin(uint8_t pin, uint8_t statusGP);

void setup()   {
  Serial.begin(9600);

  delay(1000);
//  Wire.begin(19,23); //ESP32
//  Wire.begin(D2,D1); //nodemcu ESP8266
  Wire.begin(0,2); //ESP-01
  Wire.setClock(200000); //frequencia

  //configura o GPIO0 como OUTPUT (todos os pinos)
  configurePort(IODIR0, OUTPUT);
  //configura o GPIO1 como OUTPUT (todos os pinos)
  configurePort(IODIR1, OUTPUT);
  //seta todos os pinos do GPIO0 como LOW
  writeBlockData(GP0, B00000000);
  //seta todos os pinos do GPIO1 como LOW
  writeBlockData(GP1, B00000001);
}


void loop() {
     //seta o pino 7 do GP0 como HIGH e os demais como LOW
     writeBlockData(GP0, B10000000);
    //seta todos os pinos do GPIO1 como LOW
    writeBlockData(GP1, B00000000);

    delay(1000);

    //seta todos os pinos do GPIO0 como LOW
    writeBlockData(GP0, B00000000);
    //seta o pino 0 do GP1 como HIGH e os demais como LOW 
    writeBlockData(GP1, B00000001);

  delay(1000);
   
} // end loop


//configura o GPIO (GP0 ou GP1)
//como parametro passamos:
//port: GP0 ou GP1 
//custom: INPUT para todos as portas do GP trabalharem como entrada
//        OUTPUT para todos as portas do GP trabalharem como saida
//        custom um valor de 0-255 indicando o modo das portas (1=INPUT, 0=OUTPUT) 
//        ex: 0x01 ou B00000001 ou  1 : indica que apenas o GPX.0 trabalhará como entrada, o restando como saida
void configurePort(uint8_t port, uint8_t custom)
{
  if(custom == INPUT)
  {
    writeBlockData(port, 0xFF);
  }
  else if(custom == OUTPUT)
  {
    writeBlockData(port, 0x00);
  }
  else
  {
    writeBlockData(port, custom);
  }
}

//envia dados para o MCP23016 através do barramento i2c
//reg: REGISTRADOR
//data: dados (0-255)
void writeBlockData(uint8_t reg, uint8_t data)
{
  Wire.beginTransmission(MCPAddress);
  Wire.write(reg);
  Wire.write(data);
  Wire.endTransmission();
  delay(10);
}

//seta o valor de um pino especifico
//pin: pino desejado (0-7)
//value: 0 ou 1
//gp : GP0 ou GP1
void writePinData(int pin, int value, uint8_t gp)
{
    uint8_t statusGP = 0;
    Wire.beginTransmission(MCPAddress);
    Wire.write(gp);
    Wire.endTransmission();
    //requisita a leitura de 1 byte vindo do GP
    Wire.requestFrom(MCPAddress, 1);
    statusGP = Wire.read();    

    //se o valor a setar for ZERO fazemos uma operação de subtração binária
    if (value == 0)
    {
      statusGP &= ~(B00000001 << (pin)); // muda o pino passado para ZERO (LOW)
    }
    //caso seja UM, faremos uma operação de soma binária
    else if (value == 1)
    {
      statusGP |= (B00000001 << (pin)); // muda o pino passado para UM (HIGH)
    }
    Wire.beginTransmission(MCPAddress);
    Wire.write(gp);
    Wire.write(statusGP);
    Wire.endTransmission();     
}

//faz a leitura de um pino específico
//pin: pino desejado (0-7)
//gp: GP0 ou GP1
//retorno: 0 ou 1
uint8_t readPin(uint8_t pin, uint8_t gp)
{
    uint8_t statusGP = 0;
    Wire.beginTransmission(MCPAddress);
    Wire.write(gp); 
//    Serial.println(Wire.endTransmission());
    Wire.endTransmission();
    Wire.requestFrom(MCPAddress, 1); //lê do chip 1 byte
    statusGP = Wire.read(); 
    return valueFromPin(pin, statusGP);
}

//retorna o valor do bit na posição desejada
//pin: posição do bit (0-7)
//statusGP: valor lido do GP (0-255)
uint8_t valueFromPin(uint8_t pin, uint8_t statusGP)
{
  return (statusGP &( 0x0001 << pin)) == 0 ? 0 : 1;
}