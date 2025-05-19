#include <SPI.h>
#include <WiFiNINA.h>
#include <PubSubClient.h>
///////please enter your sensitive data in the Secret tab/arduino_secrets.h
char ssid[] = "iPhone van Andreas";        // your network SSID (name)
char pass[] = "Poekkie2011";    // your network password (use for WPA, or use as key for WEP)
int keyIndex = 0;            // your network key Index number (needed only for WEP)

int status = WL_IDLE_STATUS;
// if you don't want to use DNS (and reduce your sketch size)
// use the numeric IP instead of the name for the server:
//IPAddress server(74,125,232,128);  // Als je een MQTT server wilt gebruiken zonder DNS (no DNS)
//char server[] = "broker.emqx.io";    // name address for Cloud MQTT server (using DNS)
//char server[] = "test.mosquitto.org"; 
//char server[] = "broker.hivemq.com"; 
char server[] = "public-mqtt-broker.bevywise.com";
int teller = 0;
int messageOntvangen = 0;



WiFiClient clientwifi;
PubSubClient client(clientwifi);


// MQTT Broker Details
//const char* mqttServer = "YOUR_MQTT_SERVER"; // Bijv. "broker.hivemq.com"
//const int mqttPort = 1883;                   // Standaard MQTT poort
//const char* mqttUser = "";                  // Indien nodig
//const char* mqttPassword = "";              // Indien nodig
//const char* mqttClientId = "ArduinoClient";

// Sensor Pins
const int TMP36_PIN = A0;
const int LDR_PIN = A1;

// Actuator Pins
const int MOTOR_ENABLE_PIN = D2;
const int MOTOR_IN1_PIN = D3;
const int MOTOR_IN2_PIN = D4;
const int RGB_RED_PIN = D9;
const int RGB_GREEN_PIN = D10;
const int RGB_BLUE_PIN = D11;


void callback(char* topic, byte* payload, unsigned int length) {
  Serial.print("Message arrived [");
  Serial.print(topic);
  Serial.print("] ");
  for (int i=0;i<length;i++) {
    Serial.print((char)payload[i]);
  }
  Serial.println();
  messageOntvangen++;
}



void reconnect() {
  // Loop until we're reconnected
  while (!client.connected()) {
    Serial.print("Attempting MQTT connection...");
    // Attempt to connect
    if (client.connect("arduinoClient-avl")) {
      Serial.println("connected");
      // Once connected, publish an announcement...
      client.publish("outTopic25","Maarten");

      // ... and resubscribe
      // neem inTopic als topic
      client.subscribe("inTopic");
    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      // Wait 5 seconds before retrying
      delay(5000);
    }
  }
}

void setup() {
  //Initialize serial and wait for port to open:
  Serial.begin(9600);
  while (!Serial) {
    ; // wait for serial port to connect. Needed for native USB port only
  }

  // check for the presence of the shield:
  if (WiFi.status() == WL_NO_SHIELD) {
    Serial.println("WiFi shield not present");
    // don't continue:
    while (true);
  }

  // attempt to connect to WiFi network:
  while (status != WL_CONNECTED) {
    Serial.print("Attempting to connect to SSID: ");
    Serial.println(ssid);
    // Connect to WPA/WPA2 network. Change this line if using open or WEP network:
    status = WiFi.begin(ssid, pass);

    // wait 10 seconds for connection:
    delay(10000);
  }
  Serial.println("Connected to wifi");
  printWiFiStatus();

  
  
  
  client.setServer(server, 1883);
  client.setCallback(callback);

}

void loop()
{
  if (!client.connected()) {
    reconnect();
  }
  client.loop();
  if(messageOntvangen != 0){
    //Publisht een teller die bijhoudt hoeveel keer een message is ontvangen
    teller++;
    char snum[5];
    itoa(teller, snum, 10);
    client.publish("KDGteller",snum);
    messageOntvangen = 0;
  }
}

void printWiFiStatus() {
  // print the SSID of the network you're attached to:
  Serial.print("SSID: ");
  Serial.println(WiFi.SSID());

  // print your WiFi shield's IP address:
  IPAddress ip = WiFi.localIP();
  Serial.print("IP Address: ");
  Serial.println(ip);

  // print the received signal strength:
  long rssi = WiFi.RSSI();
  Serial.print("signal strength (RSSI):");
  Serial.print(rssi);
  Serial.println(" dBm");
}

/*
// Functie prototypes
void connectToWiFi();
void connectToMQTT();
void publishSensorData();
void setMotorSpeed(int speed);
void setRgbColor(int red, int green, int blue);
float readTemperature();
float readLightLevel();

void setup() {
  Serial.begin(115200);
  connectToWiFi();
  connectToMQTT();

  // Actuator pinnen instellen
  pinMode(MOTOR_ENABLE_PIN, OUTPUT);
  pinMode(MOTOR_IN1_PIN, OUTPUT);
  pinMode(MOTOR_IN2_PIN, OUTPUT);
  pinMode(RGB_RED_PIN, OUTPUT);
  pinMode(RGB_GREEN_PIN, OUTPUT);
  pinMode(RGB_BLUE_PIN, OUTPUT);
}

void loop() {
  if (mqttClient.connected()) {
    mqttClient.loop();
    publishSensorData();
    // Ontvangen berichten verwerken (voor besturing vanaf webapp)
  } else {
    connectToMQTT();
  }
  delay(1000); // Aanpassen naar behoefte
}

void connectToWiFi() {
  Serial.print("Connecting to WiFi...");
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nConnected to WiFi");
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());
}

void connectToMQTT() {
  mqttClient.setServer(mqttServer, mqttPort);
  mqttClient.setCallback(mqttCallback); // Callback functie voor inkomende berichten

  Serial.print("Connecting to MQTT...");
  while (!mqttClient.connect(mqttClientId, mqttUser, mqttPassword)) { //Gebruikersnaam en wachtwoord indien nodig
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nConnected to MQTT");
  // Abonneer op topics voor actuatorbesturing
  mqttClient.subscribe("kweekkast/ventilator/set");
  mqttClient.subscribe("kweekkast/rgb/set");
}

void mqttCallback(char* topic, byte* payload, unsigned int length) {
  Serial.print("Message arrived [");
  Serial.print(topic);
  Serial.print("] ");
  for (int i = 0; i < length; i++) {
    Serial.print((char)payload[i]);
  }
  Serial.println();

  // Berichten verwerken voor ventilator
  if (strcmp(topic, "kweekkast/ventilator/set") == 0) {
    if (strncmp((char*)payload, "ON", length) == 0) {
      setMotorSpeed(255); // Volle snelheid
      Serial.println("Ventilator AAN");
    } else if (strncmp((char*)payload, "OFF", length) == 0) {
      setMotorSpeed(0);
      Serial.println("Ventilator UIT");
    }
  }

  // Berichten verwerken voor RGB LED
    if (strcmp(topic, "kweekkast/rgb/set") == 0) {
        // Verwacht payload formaat: "R,G,B" (bijv. "255,0,0")
        char r_str[4], g_str[4], b_str[4];
        int r, g, b;
        sscanf((char*)payload, "%[^,],%[^,],%s", r_str, g_str, b_str); // Parseer de string
        r = atoi(r_str);
        g = atoi(g_str);
        b = atoi(b_str);
        setRgbColor(r, g, b);
        Serial.printf("RGB LED: R=%d, G=%d, B=%d\n", r, g, b);
    }
}

void publishSensorData() {
  float temp = readTemperature();
  float lightLevel = readLightLevel();

  // Temperatuur publiceren
  char tempString[10];
  dtostrf(temp, 2, 2, tempString); // 2 decimalen, float naar string
  mqttClient.publish("kweekkast/temperatuur", tempString);
  Serial.print("Temperature: ");
  Serial.println(tempString);

  // Lichtniveau publiceren
  char lightString[10];
  dtostrf(lightLevel, 2, 2, lightString);
  mqttClient.publish("kweekkast/licht", lightString);
  Serial.print("Light Level: ");
  Serial.println(lightString);
}

float readTemperature() {
  int sensorValue = analogRead(TMP36_PIN);
  float voltage = sensorValue * (3.3 / 1023.0);
  float temperatureC = (voltage - 0.5) * 100;
  return temperatureC;
}

float readLightLevel() {
  int sensorValue = analogRead(LDR_PIN);
  // Omgekeerde waarde omdat lagere analoge waarde meer licht betekent
  float lightLevel = 1023.0 - sensorValue;
  return lightLevel;
}

void setMotorSpeed(int speed) {
  if (speed > 0) {
    digitalWrite(MOTOR_IN1_PIN, HIGH);
    digitalWrite(MOTOR_IN2_PIN, LOW);
    analogWrite(MOTOR_ENABLE_PIN, speed); // PWM voor snelheid
  } else {
    digitalWrite(MOTOR_ENABLE_PIN, LOW);
  }
}

void setRgbColor(int red, int green, int blue) {
  analogWrite(RGB_RED_PIN, red);
  analogWrite(RGB_GREEN_PIN, green);
  analogWrite(RGB_BLUE_PIN, blue);
}

*/
