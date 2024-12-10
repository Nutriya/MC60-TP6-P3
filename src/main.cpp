#define BLYNK_TEMPLATE_ID "TMPL5037QLVnF"                     // ID du modèle Blynk
#define BLYNK_TEMPLATE_NAME "uwu"                             // Nom du modèle Blynk
#define BLYNK_AUTH_TOKEN "izwoM5CCikaAlmDQGJ2MBbuyXl4XeUiH"   // Token d'authentification Blynk
#define BLYNK_PRINT Serial                                    // Définir la sortie de débogage sur le port série

#define LED_PIN         GPIO_NUM_13                               // Broche de la LED

#define PWM_CHANNEL     (uint8_t) 0                               // Canal PWM
#define PWM_FREQ        (int) 1000                                // Fréquence PWM en Hz
#define PWM_RESOLUTION  (uint8_t) 8                               // Résolution PWM en bits

#define PWM_MAX         (uint8_t) 255

#define CONSIGNE        (uint8_t) 25

#define KP              (float)  15

#include <WiFi.h>                                             // Bibliothèque pour la connexion WiFi
#include <BlynkSimpleEsp32.h>                                 // Bibliothèque Blynk pour ESP32
#include <Ticker.h>                                           // Bibliothèque pour les minuteries

#include <Adafruit_Sensor.h>                                  // Bibliothèque de capteurs Adafruit
#include <Adafruit_BME280.h>                                  // Bibliothèque pour le capteur BME280

char ssid[] = "Galaxy A13B7E0";                               // SSID du réseau WiFi
char pass[] = "txeh8505";                                     // Mot de passe du réseau WiFi

Ticker timer;   // Création d'une instance de minuterie
Adafruit_BME280 bme;    // Création d'une instance pour le capteur BME280 (I2C)

int erreur = 0;
float temperature;
int consigne = CONSIGNE;
int erreur = 0;

// Fonction appelée lorsque la valeur du widget V2 change
BLYNK_WRITE(V3){

  consigne = param.asInt();
  Serial.printf("BLYNK_WRITE called with value: consigne =  %d\n", consigne);
}


// Fonction pour lire les données du capteur
void readSensor() {
  temperature = bme.readTemperature();        // Lire la température
    
  // Afficher les valeurs lues sur le port série
  Serial.print("T_read= ");
  Serial.print(temperature);
  Serial.println(" °C");
  

  erreur = consigne-temperature;
    // Afficher les valeurs lues sur le port série
  Serial.print("T_erreur = ");
  Serial.print(erreur);
  Serial.println(" °C");
  
  // Envoyer les données aux widgets virtuels Blynk
  Blynk.virtualWrite(V1, temperature);

  ledcWrite(PWM_CHANNEL, (PWM_MAX-erreur*KP)); 
}

void setup() {
  Serial.begin(115200);    // Initialiser la communication série à 115200 bauds
  
  pinMode(LED_PIN, OUTPUT);                               // Définir la broche de la LED comme sortie
 
         
  unsigned status = bme.begin(0x76); // Adresse I2C par défaut du BME280
  if (!status) {
    Serial.println("Could not find a valid BME280 sensor, check wiring, address, sensor ID!");
    while (1) delay(10);
  }
  Serial.println("-- BME280 Test --");                    // Message de  démarrage

    // Configurer la minuterie pour appeler readSensor toutes les 2 secondes
  timer.attach(2, readSensor);

  Blynk.begin(BLYNK_AUTH_TOKEN, ssid, pass);              // Connexion à Blynk avec le token et les informations WiFi

  Blynk.virtualWrite(V3, CONSIGNE);
}

void loop() {
  Blynk.run(); // Exécuter Blynk
}
