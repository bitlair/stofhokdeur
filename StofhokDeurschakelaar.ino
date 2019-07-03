#include <ESP8266WiFi.h>
#include <PubSubClient.h>
#include "config.h"
#include "OneWire.h"

#define ADDRSIZE 8
#define DOOR_OPEN_DELAY 5000

#define MosfetPin D1
#define OpenButtonPin D2
#define GreenLedPin D7
#define RedLedPin D8
#define IButtonReaderPin D6

void printButtonAddress(uint8_t *addr) {
  char buf[3];
  
  for(int i = 0; i < ADDRSIZE; i++) {
    snprintf(buf, sizeof(buf), "%02x", addr[i]);
    Serial.print(buf);
  }
}

WiFiClient espClient;
PubSubClient mqttClient(espClient);
OneWire iButtonOneWire(IButtonReaderPin);
char last_bitlair_state[20];

void setup() {
  pinMode(MosfetPin, OUTPUT);
  pinMode(OpenButtonPin, INPUT_PULLUP);
  pinMode(GreenLedPin, OUTPUT);
  pinMode(RedLedPin, OUTPUT);
  pinMode(IButtonReaderPin, INPUT);

  digitalWrite(RedLedPin, HIGH);
  digitalWrite(GreenLedPin, LOW);
  
  Serial.begin(9600);

  setupWIFI();
  mqttClient.setServer(mqtt_server, 1883);
  mqttClient.setCallback(mqttCallback);
}

uint8_t addr[ADDRSIZE];

void mqttCallback(char* topic, byte* payload, unsigned int length) {
  static char value[256];
  memset(value, 0, sizeof(value));
  memcpy(value, payload, min(length, sizeof(value) - 1));

  if (strcmp(bitlair_state_topic, topic) == 0) {
    if (strcmp(last_bitlair_state, value) != 0) {
      Serial.print("Bitlair changed state to ");
      Serial.println(value);
      memcpy(last_bitlair_state, value, sizeof(last_bitlair_state));
    }
  } 
}

void loop() {
  if (!mqttClient.connected()) {
    reconnectMQTT();
  }
  mqttClient.loop();

  if (strcmp(last_bitlair_state, "open") == 0) {
    digitalWrite(MosfetPin, LOW);
    digitalWrite(RedLedPin, LOW);
    digitalWrite(GreenLedPin, HIGH);
  } else {
    digitalWrite(MosfetPin, HIGH);
    digitalWrite(RedLedPin, HIGH);
    digitalWrite(GreenLedPin, LOW);
  }

  if (!digitalRead(OpenButtonPin)) {
    Serial.println("Door opened using inside button");
    digitalWrite(MosfetPin, LOW);
    delay(DOOR_OPEN_DELAY);
    digitalWrite(MosfetPin, HIGH);
  }

  iButtonOneWire.reset_search();
  
  if (iButtonOneWire.search(addr) && OneWire::crc8(addr, 7) == addr[7]) {
      Serial.print("Authenticating   ");

      digitalWrite(GreenLedPin, HIGH);
      digitalWrite(RedLedPin, LOW);

      printButtonAddress(addr);
      Serial.print("\n");

      digitalWrite(MosfetPin, LOW);
      delay(DOOR_OPEN_DELAY);
      digitalWrite(MosfetPin, HIGH);

  }
}

void reconnectMQTT() {
  // Loop until we're reconnected
  while (!mqttClient.connected()) {
    Serial.print("Attempting MQTT connection...");
    // Attempt to connect
    if (mqttClient.connect(mqtt_client_id)) {
      Serial.println("connected");
      mqttClient.subscribe(bitlair_state_topic);
    } else {
      Serial.print("failed, rc=");
      Serial.print(mqttClient.state());
      Serial.println(" try again in 5 seconds");
      // Wait 5 seconds before retrying
      delay(5000);
    }
  }
}

void setupWIFI() {
  delay(10);
  // We start by connecting to a WiFi network
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(ssid);

  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);

  WiFi.printDiag(Serial);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  Serial.println("");
  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
}
