#include <WiFi.h>
#include <PubSubClient.h>

// WiFi
const char* ssid = "AGUNG";
const char* password = "agungsuryana18";

// MQTT
const char* mqtt_server = "192.168.0.186";
const int mqtt_port = 1884;
const char* topic = "robot/status";

WiFiClient espClient;
PubSubClient client(espClient);

// L298N pin kontrol motor
const int IN1 = 14;
const int IN2 = 27;
const int IN3 = 26;
const int IN4 = 25;

// ENA & ENB (PWM kontrol kecepatan motor)
const int ENA = 33;
const int ENB = 32;
int kecepatanMotor = 200; // nilai PWM (0 - 255)

// LED indikator
const int ledKiri = 2;
const int ledKanan = 4;
const int ledTengah = 5;

// Ultrasonik HC-SR04
const int trigPin = 12;
const int echoPin = 13;
long durasi;
float jarak;

void setup_wifi() {
  delay(10);
  Serial.println("Menghubungkan ke WiFi...");
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nWiFi terhubung");
  Serial.print("IP Address: ");
  Serial.println(WiFi.localIP());
}

// Fungsi ultrasonik
float bacaJarak() {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  durasi = pulseIn(echoPin, HIGH);
  return durasi * 0.034 / 2; // Konversi ke cm
}

// Fungsi kendali motor
void maju() {
  analogWrite(ENA, kecepatanMotor);
  analogWrite(ENB, kecepatanMotor);
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
}

void kiri() {
  analogWrite(ENA, kecepatanMotor);
  analogWrite(ENB, kecepatanMotor);
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
}

void kanan() {
  analogWrite(ENA, kecepatanMotor);
  analogWrite(ENB, kecepatanMotor);
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
}

void berhenti() {
  analogWrite(ENA, 0);
  analogWrite(ENB, 0);
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
}

// Callback MQTT
void callback(char* topic, byte* payload, unsigned int length) {
  String pesan = "";
  for (unsigned int i = 0; i < length; i++) {
    pesan += (char)payload[i];
  }

  pesan.trim();
  Serial.print("Pesan diterima: ");
  Serial.println(pesan);

  digitalWrite(ledKiri, LOW);
  digitalWrite(ledKanan, LOW);
  digitalWrite(ledTengah, LOW);

  if (pesan == "Kiri") {
    kiri();
    digitalWrite(ledKiri, HIGH);
  } else if (pesan == "Kanan") {
    kanan();
    digitalWrite(ledKanan, HIGH);
  } else if (pesan == "Tengah" || pesan == "Atas") {
    maju();
    digitalWrite(ledTengah, HIGH);
  } else if (pesan == "Bawah") {
    berhenti();
  }
}

// Reconnect MQTT
void reconnect() {
  while (!client.connected()) {
    Serial.print("Menghubungkan ke MQTT...");
    if (client.connect("ESP32Client")) {
      Serial.println("Terhubung");
      client.subscribe(topic);
    } else {
      Serial.print("Gagal, rc=");
      Serial.print(client.state());
      Serial.println(" coba lagi dalam 5 detik");
      delay(5000);
    }
  }
}

void setup() {
  Serial.begin(115200);

  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  pinMode(ENA, OUTPUT);
  pinMode(ENB, OUTPUT);

  pinMode(ledKiri, OUTPUT);
  pinMode(ledKanan, OUTPUT);
  pinMode(ledTengah, OUTPUT);

  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);

  setup_wifi();
  client.setServer(mqtt_server, mqtt_port);
  client.setCallback(callback);
}

void loop() {
  if (!client.connected()) {
    reconnect();
  }
  client.loop();

  // Cek jarak
  jarak = bacaJarak();
  Serial.print("Jarak: ");
  Serial.print(jarak);
  Serial.println(" cm");

  if (jarak <= 2.0) {
    berhenti();
    Serial.println("Benda terlalu dekat, berhenti.");
  }

  delay(100);
}
