#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <SPIFFS.h> 
#include <SPI.h>
#include <FS.h>
#include <HardwareSerial.h>

// Initialize Serial2 for UART

const int txPin = 17; // TX pin
const int baudRate = 9600; // UART baud rate

// Definições do display OLED
#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels

// Declaração para o display SSD1306 conectado via I2C
#define OLED_RESET     -1 // Reset pin
#define SCREEN_ADDRESS 0x3C
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

// Pinos do sensor de distância HC-SR04
#define TRIG_PIN_1 14
#define ECHO_PIN_1 27

// Pinos do sensor de distância HC-SR04 2
#define TRIG_PIN_2 33
#define ECHO_PIN_2 32


#define DISTANCE 60

int currentState = 0;        
int previousState = 0;       
int currentState2 = 0;   
int previousState2 = 0;      
int i = 0;                  


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
  if(i>=0){
      display.clearDisplay();
      display.setTextSize(1);
      display.setTextColor(WHITE);
      display.setCursor(0, 0);
      display.print("Pessoas na sala: ");
      display.println(i);
      display.display();

     File file = SPIFFS.open("/estado.txt", FILE_WRITE);
    if (file) {
      file.print(i);
      file.print(" ");
      file.close();
    } else {
      Serial.println("Falha ao abrir o arquivo para escrita.");
    }

        Serial2.println(String(i));
    Serial.printf("Sent: %d\n", i); // Debugging output
    
  }else{
        i= 0;

    }
}

void setup() {

  Serial.begin(115200);
  Serial.println(ESP.getFreeHeap());

  // Inicializar comunicação I2C
  Wire.begin(25, 26); // SDA, SCL para ESP32
  // Inicializar o display OLED
  if(!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
    Serial.println(F("SSD1306 allocation failed"));
    for(;;);
  }
  display.display();
  delay(2000);
  display.clearDisplay();


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
    // Initialize UART for transmission
    Serial2.begin(baudRate, SERIAL_8N1, -1, txPin); // RX pin set to -1 (unused)
    Serial.begin(115200); // Debugging
    Serial.println("Transmitter Initialized");
}

void loop() {
//                    displayAndPrint(0);


    long duration, distance;
    long duration2, distance2;
    distance = measureDistance(TRIG_PIN_1, ECHO_PIN_1);
    distance2 = measureDistance(TRIG_PIN_2, ECHO_PIN_2);
//
    Serial.println(distance);
    Serial.println(distance2);
    Serial.println(i);

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
   
}#include <WiFi.h>
#include <Adafruit_MQTT.h>
#include <Adafruit_MQTT_Client.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <HardwareSerial.h>

// Initialize Serial2 for UART
// HardwareSerial Serial2(2);

const int rxPin = 16; // RX pin
const int baudRate = 9600; // UART baud rate


/********* Configurações do Adafruit IO ********/
#define WLAN_SSID       "tars"
#define WLAN_PASS       "12345678"
#define AIO_SERVER      "io.adafruit.com"
#define AIO_SERVERPORT  1883
#define AIO_USERNAME    "tarsillasamile"
#define AIO_KEY         "aaa"

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
void displayAndPrint(int i) {
    Serial.print("Publicando entradas: ");
    Serial.println( i);
    int32_t valor = i;
    entradasFeed.publish(valor); 
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


    if (Serial2.available()) {
        String receivedData = Serial2.readStringUntil('\n'); // Read until newline
        Serial.printf("Received: %d\n", receivedData.toInt()); // Debugging output
        displayAndPrint(receivedData.toInt());
    }

}
