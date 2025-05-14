#include <WiFi.h>
#include <PubSubClient.h>
#include <Servo.h>
#include <L298N.h>  // Pastikan library ini sudah diinstall

// WiFi
const char* ssid = "AGUNG";
const char* password = "agungsuryana18";

// MQTT
const char* mqtt_server = "192.168.0.186";
const int mqtt_port = 1884;
const char* topic = "robot/status";

WiFiClient espClient;
PubSubClient client(espClient);

// Motor Driver L298N
const int ENA = 13;
const int IN1 = 12;
const int IN2 = 14;
const int ENB = 25;
const int IN3 = 27;
const int IN4 = 26;

int speed = 255;

L298N motorA(ENA, IN1, IN2); // Motor A (kiri)
L298N motorB(ENB, IN3, IN4); // Motor B (kanan)

// Ultrasonik HC-SR04
const int trigPin = 33;
const int echoPin = 32;

// Servo
const int servoPin = 15;
Servo servo;

long bacaJarak() {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  long durasi = pulseIn(echoPin, HIGH);
  return durasi * 0.034 / 2;
}

// Fungsi gerak motor
void maju() {
  motorA.setSpeed(speed);
  motorB.setSpeed(speed);
  motorA.forward();
  motorB.forward();
}

void kiri() {
  motorA.setSpeed(speed);
  motorB.setSpeed(speed);
  motorA.backward();
  motorB.forward();
}

void kanan() {
  motorA.setSpeed(speed);
  motorB.setSpeed(speed);
  motorA.forward();
  motorB.backward();
}

void berhenti() {
  motorA.stop();
  motorB.stop();
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
  } else if (pesan == "Kanan") {
    kanan();
  } else if (pesan == "Tengah") {
    maju();
  } else if (pesan == "Bawah") {
    berhenti();
  } else if (pesan == "Atas") {
    maju();
  }
}

// MQTT
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

// WiFi
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

// Setup awal
void setup() {
  Serial.begin(115200);

  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);

  pinMode(ledKiri, OUTPUT);
  pinMode(ledKanan, OUTPUT);
  pinMode(ledTengah, OUTPUT);

  servo.attach(servoPin);
  servo.write(0); // posisi awal

  setup_wifi();
  client.setServer(mqtt_server, mqtt_port);
  client.setCallback(callback);
}

void loop() {
  if (!client.connected()) {
    reconnect();
  }
  client.loop();

  long jarak = bacaJarak();
  Serial.print("Jarak: ");
  Serial.println(jarak);

  if (jarak <= 2) {
    berhenti();
    Serial.println("Objek dekat - servo aktif");

    servo.write(90); // ambil
    delay(1000);
    servo.write(0);  // kembali
    delay(1000);
  }
}
