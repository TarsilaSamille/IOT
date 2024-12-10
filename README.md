# IOT

https://www.youtube.com/watch?v=UpODicJY51M

https://wokwi.com/projects/416800871394149377

#include <WiFi.h>
#include <Adafruit_MQTT.h>
#include <Adafruit_MQTT_Client.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <HardwareSerial.h>

// Initialize Serial2 for UART
HardwareSerial Serial2(2);

const int rxPin = 16; // RX pin
const int baudRate = 9600; // UART baud rate


/********* Configurações do Adafruit IO ********/
#define WLAN_SSID       "Wokwi-GUEST"
#define WLAN_PASS       ""
#define AIO_SERVER      "io.adafruit.com"
#define AIO_SERVERPORT  1883
#define AIO_USERNAME    "tarsillasamile"
#define AIO_KEY         ""

/**** Configuração do Cliente WiFi e MQTT ****/
WiFiClient client;
Adafruit_MQTT_Client mqtt(&client, AIO_SERVER, AIO_SERVERPORT, AIO_USERNAME, AIO_KEY);

// Feeds Adafruit IO
Adafruit_MQTT_Publish entradasFeed = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/entradas");
Adafruit_MQTT_Publish objetoPresenteFeed = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/objetoPresente");


void connectToWiFi() {
  Serial.print("Conectando a ");
  Serial.println(WLAN_SSID);
  WiFi.begin(WLAN_SSID, WLAN_PASS);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nWiFi conectado!");
}

void connectToMQTT() {
  while (!mqtt.connected()) {
    Serial.print("Conectando ao MQTT... ");
    if (mqtt.connect()) {
      Serial.println("Conectado!");
    } else {
      Serial.print("Falha na conexão MQTT. Verifique as credenciais.");
      Serial.println(". Tentando novamente em 5 segundos...");
      delay(5000);
    }
  }
}
void displayAndPrint(String i) {
    Serial.print("Publicando entradas: ");
    Serial.println(i);
    int32_t valor = i.toInt(); // Converte a String para int
    entradasFeed.publish(valor); // Publica o valor convertido
}


void setup() {

  Serial.begin(115200);
  Serial.println(ESP.getFreeHeap());

  connectToWiFi();
  connectToMQTT();
    // Initialize UART for reception
    Serial2.begin(baudRate, SERIAL_8N1, rxPin, -1); // TX pin set to -1 (unused)
    Serial.begin(115200); // Debugging
    Serial.println("Receiver Initialized");
}

void loop() {
    // Verifica a conexão com o MQTT
    if (!mqtt.connected()) {
      connectToMQTT();
    }
    mqtt.processPackets(10);
    mqtt.ping();



       // Check if data is available
    if (Serial2.available()) {
        String receivedData = Serial2.readStringUntil('\n'); // Read until newline
        Serial.printf("Received: %s\n", receivedData.c_str()); // Debugging output
        displayAndPrint(receivedData.c_str());
    }
}

#include <WiFi.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <PubSubClient.h>

// Definições do display OLED
#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels

#define OLED_RESET     -1 // Reset pin
#define SCREEN_ADDRESS 0x3C

// Informações da sua rede Wi-Fi
//#define WIFI_SSID "TEIXEIRA"        // Nome da sua rede Wi-Fi
//#define WIFI_PASS "20011005"      // Senha da sua rede Wi-Fi

#define IO_USERNAME    "tarsillasamile"
#define IO_KEY         ""

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

const char* mqttserver = "io.adafruit.com";
const int mqttport = 1883;
const char* mqttUser = IO_USERNAME;
const char* mqttPassword = IO_KEY;
const char* subscribe_topic = "tarsillasamile/feeds/entrada"; //

// Inicializa o cliente Adafruit IO com Wi-Fi
WiFiClient espClient;
PubSubClient client(espClient);

void savePersonCountToFile(String count) {
  File file = SPIFFS.open("/estado.txt", FILE_WRITE);
  if (file) {
    file.print(count);
    file.close();
  } else {
    Serial.println("Falha ao abrir o arquivo para escrita.");
  }
}

void displayCount(String count) {
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(WHITE);
  display.setCursor(0, 0);
  display.print("Pessoas na sala: ");
  display.println(count);
  display.display();
}

void connectToWiFi() {
  Serial.print("Conectando a ");
  Serial.println(WIFI_SSID);
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nWiFi conectado!");
}

// Callback para mensagens recebidas
void callback(char* topic, byte* payload, unsigned int length) {
  Serial.print("Mensagem recebida no tópico: ");
  Serial.println(topic);
  String message = "";
  for (int i = 0; i < length; i++) {
    message += (char)payload[i];
  }
  Serial.print("Leitura: ");
  Serial.println(message);
  savePersonCountToFile(message);
  displayCount(message);
}

void reconnect() {
  while (!client.connected()) {
    Serial.print("Tentando conectar ao MQTT...");
    if (client.connect("ESP32Client", mqttUser, mqttPassword)) {
      Serial.println("Conectado ao MQTT!");
      client.subscribe(subscribe_topic); // Assina o feed de entrada
    } else {
      Serial.print("Falha, rc=");
      Serial.print(client.state());
      Serial.println(" tentando novamente em 5 segundos...");
      delay(5000);
    }
  }
}

void readFile(){
  File file = SPIFFS.open("/teste.txt", "r");
  if (!file) {
    Serial.println("Erro ao abrir arquivo para leitura!");
    return;
  }
  Serial.println("Conteúdo do arquivo:");
  while (file.available()) {
    Serial.write(file.read());
  }
  file.close();
}


void setup() {
  Serial.begin(115200);
  connectToWiFi();

  client.setServer(mqttserver, mqttport);
  client.setCallback(callback);

  // Inicializar o display OLED
  if(!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
    Serial.println(F("SSD1306 allocation failed"));
    for(;;);
  }
  display.display();
  delay(2000);
  display.clearDisplay();
  readFile();

  Serial.println("Setup completo!");
}

void loop() {
  if (!client.connected()) {
    reconnect();
  }
  client.loop();
  delay(5000);
}

#include <WiFi.h>
#include <Adafruit_MQTT.h>
#include <Adafruit_MQTT_Client.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

#include <SPIFFS.h> 
#include <SPI.h>
#include <FS.h>

// Definições do display OLED
#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels

// Declaração para o display SSD1306 conectado via I2C
#define OLED_RESET     -1 // Reset pin
#define SCREEN_ADDRESS 0x3C
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

/************************* Configurações do Adafruit IO ************************/
#define WLAN_SSID       "Wokwi-GUEST"
#define WLAN_PASS       ""
#define AIO_SERVER      "io.adafruit.com"
#define AIO_SERVERPORT  1883
#define AIO_USERNAME    "tarsillasamile"
#define AIO_KEY         ""

// Pinos do sensor de distância HC-SR04
#define TRIG_PIN_1 26
#define ECHO_PIN_1 25

// Pinos do sensor de distância HC-SR04 2
#define TRIG_PIN_2 33
#define ECHO_PIN_2 32


#define DISTANCE 32


/************ Configuração do Cliente WiFi e MQTT ************/
WiFiClient client;
Adafruit_MQTT_Client mqtt(&client, AIO_SERVER, AIO_SERVERPORT, AIO_USERNAME, AIO_KEY);

// Feeds Adafruit IO
Adafruit_MQTT_Publish entradasFeed = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/entradas");
Adafruit_MQTT_Publish objetoPresenteFeed = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/objetoPresente");


int currentState = 0;        
int previousState = 0;       
int currentState2 = 0;   
int previousState2 = 0;      
int i = 0;                  


void connectToWiFi() {
  Serial.print("Conectando a ");
  Serial.println(WLAN_SSID);
  WiFi.begin(WLAN_SSID, WLAN_PASS);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nWiFi conectado!");
}

void connectToMQTT() {
  while (!mqtt.connected()) {
    Serial.print("Conectando ao MQTT... ");
    if (mqtt.connect()) {
      Serial.println("Conectado!");
    } else {
      Serial.print("Falha na conexão MQTT. Verifique as credenciais.");
      Serial.println(". Tentando novamente em 5 segundos...");
      delay(5000);
    }
  }
}

long measureDistance(int trigPin, int echoPin) {
    digitalWrite(trigPin, LOW);
    delayMicroseconds(2);
    digitalWrite(trigPin, HIGH);
    delayMicroseconds(10);
    digitalWrite(trigPin, LOW);
    long duration = pulseIn(echoPin, HIGH);
    return (duration / 2) / 29.1; // Convertir en centimètres
}

void displayAndPrint(int i) {
    
      display.clearDisplay();
      display.setTextSize(1);
      display.setTextColor(WHITE);
      display.setCursor(0, 0);
      display.print("Pessoas na sala: ");
      display.println(i);
      display.display();

      Serial.print("Publicando entradas: ");
    Serial.println(i);
    entradasFeed.publish((int32_t)i); 

     File file = SPIFFS.open("/estado.txt", FILE_WRITE);
    if (file) {
      file.print(i);
      file.print(" ");
      file.close();
    } else {
      Serial.println("Falha ao abrir o arquivo para escrita.");
    }
}

void setup() {

  Serial.begin(115200);
  Serial.println(ESP.getFreeHeap());

  // Inicializar comunicação I2C
  Wire.begin(21, 22); // SDA, SCL para ESP32
  // Inicializar o display OLED
  if(!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
    Serial.println(F("SSD1306 allocation failed"));
    for(;;);
  }
  display.display();
  delay(2000);
  display.clearDisplay();

  connectToWiFi();
  connectToMQTT();

  // Inicialização do SPIFFS - Comentado
  if (!SPIFFS.begin(true)) {
    Serial.println("Falha ao montar o sistema de arquivos");
    return;
  }
  // Leitura inicial do estado - Comentado
  File file = SPIFFS.open("/estado.txt", FILE_READ);
  if (file) {
    i = file.parseInt();
    file.close();
  } else {
    Serial.println("Arquivo não encontrado. Usando valores padrão.");
  }

  // Configurar pinos do HC-SR04
  pinMode(TRIG_PIN_1, OUTPUT);
  pinMode(ECHO_PIN_1, INPUT);
  pinMode(TRIG_PIN_2, OUTPUT);
  pinMode(ECHO_PIN_2, INPUT);
}

void loop() {
    // Verifica a conexão com o MQTT
    if (!mqtt.connected()) {
      connectToMQTT();
    }
    mqtt.processPackets(10);
    mqtt.ping();


    long duration, distance;
    long duration2, distance2;
    distance = measureDistance(TRIG_PIN_1, ECHO_PIN_1);
    distance2 = measureDistance(TRIG_PIN_2, ECHO_PIN_2);

    // Verificar entrada de objeto no sistema
    if (distance < DISTANCE) {
        currentState = 1;
    } else {
        currentState = 0;
    }

    if (currentState != previousState) {
        while (currentState == 1) {
            distance2 = measureDistance(TRIG_PIN_2, ECHO_PIN_2);

            if (distance2 < DISTANCE) {
                currentState2 = 1;
            } else {
                currentState2 = 0;
            }

            if (currentState2 != previousState2) {
                if (currentState2 == 1) {
                    i++; // Incrementar contador
                    displayAndPrint(i);

                }

                delay(1000);
                return;
            }
        }
    }
    // Verificar saída de objeto do sistema
    if (distance2 < DISTANCE) {
        currentState2 = 1;
    } else {
        currentState2 = 0;
    }

    if (currentState2 != previousState2) {
        while (currentState2 == 1) {
            distance = measureDistance(TRIG_PIN_1, ECHO_PIN_1);

            if (distance < DISTANCE) {
                currentState = 1;
            } else {
                currentState = 0;
            }

            if (currentState != previousState) {
                if (currentState == 1) {
                    i--; // Decrementar contador
                    displayAndPrint(i);
                }
                delay(1000);
                return;
            }
        }
    }
   
}
