/*************************************
 * Projeto Final - Locker
 * 
 * Nome: Leandro Iglesias Bresolin
 * R.A.: 190009972
 * 
 ************************************/

// Inclusão das bibliotecas
#include <Arduino.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"

#include <WiFi.h>
#include <PubSubClient.h>
#include <LiquidCrystal_I2C.h>
#include <Wire.h>
#include <SPI.h>
#include <MFRC522.h>
#include "cJSON.h"

// Definições dos pinos do leitor RFID
#define RFID_SDA         5          
#define RFID_RST         36  
#define BEEP             15      

// Definições dos pinos dos LEDs
#define LED_RED          13          
#define LED_GREEN        12    
#define LED_BLUE         14
#define LED_RFID         2

// Definições do Display
#define DISPLAY_ADDRESS  0x27
#define DISPLAY_COL      16
#define DISPLAY_ROW      2  
#define SDA_1            21
#define SCL_1            22

// Definições do Sensor de porta aberta/fechada
#define DOOR_STATUS      33

// Definições do Rele
#define RELE             4

// Definições do Wi-Fi
const char* SSID = "YOUR_SSID";
const char* PASSWORD = "YOUR_PASSWORD";

// Definições do broker MQTT
const char* ID_MQTT = "Leandro_UNISAL";
const char* BROKER_MQTT = "broker.hivemq.com";
int BROKER_PORT = 1883;

#define TOPICO_STATUS "/unisal/lorena/projeto/2020/status/"
#define TOPICO_SERVER "/unisal/lorena/projeto/2020/server/"

// Definições do dispositivo
const int DEVICE_ID = 1234;

// Protótipos das Tasks
void vTaskRFID( void * pvParameters );
void vTaskDoor( void * pvParameters );
void vTask_Mqtt( void * pvParameters );

// Handles das tasks
TaskHandle_t xTaskRFIDHandle;  
TaskHandle_t xTaskDoorHandle;  
TaskHandle_t xTaskMqttHandle;  

// Handle do Semaforo
SemaphoreHandle_t xMutex;

// Handle das filas
QueueHandle_t xFilaDeviceStatus; 

// Funções auxiliares
void vInitHW(void);
void wifiConnect(void);
void setLed(int);
void setLedRFID(int);
void setRele(int);
void sendMqtt(void);
void parseCommand(char*);
void setDisplayMessage(int);
void connectMQTT(void);
void mqttCallback(char* topic, byte* payload, unsigned int length);


// Definições das bibliotecas 
LiquidCrystal_I2C lcd(DISPLAY_ADDRESS, DISPLAY_COL, DISPLAY_ROW);
WiFiClient wifi_comm;
PubSubClient mqtt(wifi_comm);
MFRC522 rfid(RFID_SDA, RFID_RST);

// Definindo struct de dados do dispositivo
struct device_struct
{
  int device_id;
  int door_status;
  int uid;
};
struct device_struct device_status;

// Definindo struct de dados do servidor
struct server_struct
{
  int device_id;
  int autorization;
  int card_status;
};
struct server_struct server_status;

// ================================================
// SETUP
// ================================================
void setup() { 
  
  //Inicia Hardware 
  vInitHW();
  
  // Cria a fila
  xFilaDeviceStatus = xQueueCreate(1, sizeof(device_struct));

  // Cria o Mutex
  xMutex = xSemaphoreCreateMutex();

  //Cria vTaskDoor
  xTaskCreatePinnedToCore(
    vTaskDoor, 
    "Task Door", 
    configMINIMAL_STACK_SIZE + 2048, 
    NULL, 
    1, 
    &xTaskDoorHandle,APP_CPU_NUM); 

  //Cria vTaskRFID
  xTaskCreatePinnedToCore(
    vTaskRFID, 
    "Task RFID", 
    configMINIMAL_STACK_SIZE + 2048, 
    NULL, 
    5, 
    &xTaskRFIDHandle,APP_CPU_NUM); 
 
  //Cria vTask_Mqtt
  xTaskCreate(
    vTask_Mqtt, 
    "Task MQTT", 
    configMINIMAL_STACK_SIZE + 2048, 
    NULL, 
    2, 
    &xTaskMqttHandle); 

}

// ================================================
// LOOP
// ================================================
void loop() {

  xSemaphoreTake(xMutex, 20);
  setDisplayMessage(1);
  setLed(0);

  // Envia status por MQTT
  sendMqtt();

  xSemaphoreGive(xMutex);  

  vTaskDelay(pdMS_TO_TICKS(5000));

}


// ================================================
// TASKS
// ================================================

// Implementação do status da porta
void vTaskDoor(void *pvParameters )
{
  (void) pvParameters;
  
  while(1)
  {
    xSemaphoreTake(xMutex, 20);
    
    device_status.door_status = digitalRead(DOOR_STATUS);
    xQueueOverwrite(xFilaDeviceStatus, &device_status);

    if (device_status.door_status == 0){
      setDisplayMessage(5);
      setLed(3);
    }
        
    xSemaphoreGive(xMutex);
    vTaskDelay(pdMS_TO_TICKS(500));
  }
}

// Implementação do status do RFID
void vTaskRFID(void *pvParameters )
{
  (void) pvParameters;
  String uidString;

  while(1)
  {
    xSemaphoreTake(xMutex, 20);

    if ( rfid.PICC_IsNewCardPresent())
    {
      if ( rfid.PICC_ReadCardSerial())
      { 
        uidString = String(rfid.uid.uidByte[0]) +  
                    String(rfid.uid.uidByte[1]) + 
                    String(rfid.uid.uidByte[2]) + 
                    String(rfid.uid.uidByte[3]);
       
        setLedRFID(200);

        device_status.uid = uidString.toInt();
        xQueueOverwrite(xFilaDeviceStatus, &device_status);
        
        // Envia status por MQTT
        sendMqtt();
      }
    }
    
    xSemaphoreGive(xMutex);  
    vTaskDelay(pdMS_TO_TICKS(500));
  }
}

// Implementação da Task MQTT
void vTask_Mqtt(void *pvParameters){
  (void) pvParameters;
  
  // Inicializa conexão com o broker MQTT
  mqtt.setServer(BROKER_MQTT, BROKER_PORT);
  mqtt.setCallback(mqttCallback);

  while(1)
  {
    xSemaphoreTake(xMutex, 20);
    if(!mqtt.connected()){
      connectMQTT();
    }
    xSemaphoreGive(xMutex);
    mqtt.loop();
    vTaskDelay(pdMS_TO_TICKS(500));

  }
}

// ================================================
// FUNÇÕES AUXILIARES
// ================================================

// Função Init Hardware
void vInitHW(void)
{
  // Configura o ID do dispositivo
  device_status.device_id = DEVICE_ID;

  // Inicializando a serial
  Serial.begin(115200);
  
  // Conecta na rede wi-fi definida.
  wifiConnect();

  // Inicianlizando SPI e leitor de RFID  
  SPI.begin();             
  rfid.PCD_Init(); 
  pinMode(LED_RFID, OUTPUT);
  pinMode(BEEP, OUTPUT);
  
  // Configurando os LEDs
  pinMode(LED_RED, OUTPUT);
  pinMode(LED_GREEN, OUTPUT);
  pinMode(LED_BLUE, OUTPUT);
  
  // Configurando o pino da porta
  pinMode(DOOR_STATUS, INPUT_PULLUP);
 
  // Configurando o pino do rele
  pinMode(RELE, OUTPUT);

  // Inicializando com o LED apagado
  setLed(0);
  
  // Inicializando display
  lcd.init();
  lcd.backlight();
  setDisplayMessage(0); 
    
}

// Função para escrever mensagens no display
void setDisplayMessage(int msg){
  switch (msg)
  {
    case 0:
      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print("Aguarde");
      lcd.setCursor(0, 1);
      lcd.print("Inicializando...");
      break;
    
    case 1: 
      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print("Aproxime o ");
      lcd.setCursor(0, 1);
      lcd.print("cartao do leitor");
      break;

    case 2: 
      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print("Acesso nao ");
      lcd.setCursor(0, 1);
      lcd.print("autorizado");
      break;

    case 3: 
      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print("Acesso ");
      lcd.setCursor(0, 1);
      lcd.print("autorizado");
      break;

    case 4: 
      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print("Cartao ");
      lcd.setCursor(0, 1);
      lcd.print("Bloqueado");
      break;

    case 5: 
      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print("Atencao! ");
      lcd.setCursor(0, 1);
      lcd.print("Porta aberta.");
      break;

    case 6: 
      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print("Por favor,");
      lcd.setCursor(0, 1);
      lcd.print("aguarde...");
      break;

    default:
      break;
  }
}

//Função para conectar com o Wi-Fi
void wifiConnect(void)
{ 
  uint8_t count = 0;

  Serial.println();
  Serial.print("Tentando se conectar com a rede: " + String(SSID) + "\n");
  
  WiFi.begin(SSID, PASSWORD);

  while (WiFi.status() != WL_CONNECTED) {
    vTaskDelay(pdMS_TO_TICKS(1000));
    count ++;
    Serial.println("Conectando ao WiFi...");
    Serial.println("Tentativa: " + String(count));
    WiFi.begin(SSID, PASSWORD);
    if (count >= 3){
      count = 0;
      break;
    }
  }

  if(WiFi.status() != WL_CONNECTED){
    Serial.println("WiFi não conectado.");
  }else{
    Serial.println("");
    Serial.println("WiFi conectado!");
    Serial.println("Endereço IP: ");
    Serial.println(WiFi.localIP());

    setDisplayMessage(1);
  }
  
}

// Função para ligar/desligar o LED RGB 
// 0 --> Apagado
// 1 --> RED
// 2 --> GREEN
// 3 --> BLUE
void setLed(int color){

  if (color == 1){
    digitalWrite(LED_RED, HIGH);
    digitalWrite(LED_GREEN, LOW);
    digitalWrite(LED_BLUE, LOW);
  }else if (color == 2){
    digitalWrite(LED_RED, LOW);
    digitalWrite(LED_GREEN, HIGH);
    digitalWrite(LED_BLUE, LOW);
  }else if (color == 3){
    digitalWrite(LED_RED, LOW);
    digitalWrite(LED_GREEN, LOW);
    digitalWrite(LED_BLUE, HIGH);
  }else{
    digitalWrite(LED_RED, LOW);
    digitalWrite(LED_GREEN, LOW);
    digitalWrite(LED_BLUE, LOW);
     
  }
}

// Função para ligar/desligar o Rele
void setLedRFID(int delay){

  digitalWrite(LED_RFID, HIGH);
  digitalWrite(BEEP, HIGH);
  vTaskDelay(pdMS_TO_TICKS(delay));
  digitalWrite(LED_RFID, LOW);
  digitalWrite(BEEP, LOW);

}


// Função para ligar/desligar o LED do RFID 
void setRele(int delay){

  digitalWrite(RELE, HIGH);
  vTaskDelay(pdMS_TO_TICKS(delay));
  digitalWrite(RELE, LOW);
  
}


// Função para conexão com o broker MQTT
void connectMQTT(void) 
{
  Serial.print("* Tentando se conectar ao Broker MQTT: ");
  Serial.println(BROKER_MQTT);
  if (mqtt.connect(ID_MQTT)) 
  {
    Serial.println("Conectado com sucesso ao broker MQTT!");
    mqtt.subscribe(TOPICO_SERVER, 1);
  } 
  else 
  {
    Serial.println("Falha ao reconectar no broker.");
    Serial.println("Nova tentativa de conexao em 2s");
  }
}

// Função para enviar os dados por MQTT
void sendMqtt()
{
  
  cJSON *parent;
  char *rendered;
  parent = cJSON_CreateObject();

  xQueuePeek(xFilaDeviceStatus, &device_status, 20);
  cJSON_AddItemToObject(parent, "device_id", cJSON_CreateNumber(device_status.device_id));
  cJSON_AddItemToObject(parent, "door_status", cJSON_CreateNumber(device_status.door_status));
  cJSON_AddItemToObject(parent, "uid", cJSON_CreateNumber(device_status.uid));
  
  rendered = cJSON_Print(parent);
  
  Serial.println("ENVIADO: ");
  Serial.println(rendered);
  Serial.println("");

  cJSON_Delete(parent);

  device_status.uid = 0;
  xQueueOverwrite(xFilaDeviceStatus, &device_status);

  if(mqtt.connected()){
    mqtt.publish(TOPICO_STATUS, rendered);
  }
  
}

// Callback do MQTT
/*Sempre que chegar uma mensagem pelo MQTT, essa
  função será chamada */
void mqttCallback(char* topic, byte* payload, unsigned int length) 
{
  String msg;
  String tpc;

  payload[length] = '\0';
  msg = String((char*)payload);
  tpc = String((char*)topic);
  
  Serial.print(tpc);
  Serial.println(msg);

  Serial.println("RECEBIDO: ");
  Serial.println(msg);
  Serial.println("");

  parseCommand((char*)payload);

}


// Função tratar comandos do servidor
void parseCommand(char *dado)
{
    cJSON *root = cJSON_Parse(dado);
    server_status.device_id = cJSON_GetObjectItem(root, "device_id")->valueint;
    server_status.autorization = cJSON_GetObjectItem(root, "autorization")->valueint;
    server_status.card_status = cJSON_GetObjectItem(root, "card_status")->valueint;

    if (server_status.device_id == DEVICE_ID){

      if(server_status.card_status == 0)
      {
        setDisplayMessage(4);
        setLed(1);
        Serial.println("====== CARTAO BLOQUEADO ;-(");
      }else{
        if(server_status.autorization == 0)
        {
          setDisplayMessage(2);
          setLed(1);
          Serial.println("====== CARTAO NAO AUTORIZADO ;-(");

        }else if (server_status.autorization == 1)
        {
          setDisplayMessage(3);
          setLed(2);
          setRele(500);
          Serial.println("====== AUTORIZADO ;-)");
        }
      }
    }
    vTaskDelay(pdMS_TO_TICKS(3000));
    setLed(0);
    setDisplayMessage(1);
}

