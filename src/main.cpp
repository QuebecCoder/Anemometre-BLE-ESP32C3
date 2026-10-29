// Anémomètre BLE - ESP32-C3
// Mathis Campagna

#include <Arduino.h>
#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEServer.h>

#define PIN_CAPTEUR 4
#define PIN_LED 7

#define ANTIREBOND_MS 40
#define NB_AIMANTS 2
#define DIAMETRE_M 0.15f
#define CIRCONF_M (DIAMETRE_M * 3.14159f)
#define DELAI_ARRET_MS 3000
#define K_CALIBRATION 7.5 // ajusté avec la station météo

#define NB_MESURES 10
#define TOLERANCE_KMH 2.0f

#define NOM_BLE "ESP32_Mathis"
#define UUID_SERVICE BLEUUID((uint16_t)0x181A) // Environmental Sensing
#define UUID_VITESSE BLEUUID((uint16_t)0x2A70) // True Wind Speed


BLEServer* serveur = nullptr;
BLECharacteristic* caracVitesse = nullptr;
bool clientConnecte = false;

unsigned long dernierFront = 0;
unsigned long dernierEnvoi = 0;
bool etatPrecedent = HIGH;
bool etatLed = false;

float mesures[NB_MESURES];
byte nbMesures = 0;
float vitesseFiltree = 0.0f;


class GestionConnexion : public BLEServerCallbacks {
  void onConnect(BLEServer* s) override {
    clientConnecte = true;
    Serial.println("Client connecté");
  }

  void onDisconnect(BLEServer* s) override {
    clientConnecte = false;
    Serial.println("Client déconnecté");
    delay(200);
    s->getAdvertising()->start(); // permet de se reconnecter
  }
};


float calculerKmh(unsigned long ecartMs) {
  if (ecartMs == 0) return 0.0f;

  float ecartS = ecartMs / 1000.0f;
  float toursParSec = (1.0f / NB_AIMANTS) / ecartS;
  float vitesseMs = toursParSec * CIRCONF_M * K_CALIBRATION;
  return vitesseMs * 3.6f;
}


// Filtre retire le min et le max,
// Garde la valeur qui a le plus de mesures proches
float filtrerMesures() {
  float minimum = mesures[0];
  float maximum = mesures[0];
  for (byte i = 1; i < NB_MESURES; i++) {
    if (mesures[i] < minimum) minimum = mesures[i];
    if (mesures[i] > maximum) maximum = mesures[i];
  }

  byte meilleurScore = 0;
  float valeurRetenue = 0.0f;

  for (byte i = 0; i < NB_MESURES; i++) {
    if (mesures[i] == minimum || mesures[i] == maximum) continue;

    byte voisines = 0;
    for (byte j = 0; j < NB_MESURES; j++) {
      if (mesures[j] == minimum || mesures[j] == maximum) continue;
      if (fabs(mesures[j] - mesures[i]) <= TOLERANCE_KMH) voisines++;
    }

    if (voisines > meilleurScore) {
      meilleurScore = voisines;
      valeurRetenue = mesures[i];
    }
  }

  // Si tout a été retiré on prend la moyenne
  if (meilleurScore == 0) {
    float somme = 0;
    for (byte i = 0; i < NB_MESURES; i++) somme += mesures[i];
    return somme / NB_MESURES;
  }

  return valeurRetenue;
}


void setup() {
  Serial.begin(115200);

  pinMode(PIN_LED, OUTPUT);
  pinMode(PIN_CAPTEUR, INPUT_PULLUP);
  digitalWrite(PIN_LED, LOW);

  BLEDevice::init(NOM_BLE);
  serveur = BLEDevice::createServer();
  serveur->setCallbacks(new GestionConnexion());

  BLEService* service = serveur->createService(UUID_SERVICE);
  caracVitesse = service->createCharacteristic(
    UUID_VITESSE,
    BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY
  );
  service->start();
  serveur->getAdvertising()->start();

  Serial.println("Anémomètre prêt");
  Serial.print("Circonférence : ");
  Serial.print(CIRCONF_M * 100, 1);
  Serial.println(" cm");
}


void loop() {
  unsigned long maintenant = millis();
  bool etat = digitalRead(PIN_CAPTEUR);

  // Front descendant = un aimant passe devant le capteur
  if (etatPrecedent == HIGH && etat == LOW) {
    bool premierFront = (dernierFront == 0);
    bool pasUnRebond = (maintenant - dernierFront) >= ANTIREBOND_MS;

    if (!premierFront && pasUnRebond) {
      mesures[nbMesures++] = calculerKmh(maintenant - dernierFront);

      etatLed = !etatLed;
      digitalWrite(PIN_LED, etatLed);

      if (nbMesures >= NB_MESURES) {
        vitesseFiltree = filtrerMesures();
        nbMesures = 0;
        Serial.print("Nouvelle vitesse : ");
        Serial.print(vitesseFiltree, 2);
        Serial.println(" km/h");
      }
    }

    if (premierFront || pasUnRebond) dernierFront = maintenant;
  }
  etatPrecedent = etat;

  // Rotor arrêté
  if (dernierFront != 0 && (maintenant - dernierFront) > DELAI_ARRET_MS) {
    vitesseFiltree = 0.0f;
    nbMesures = 0;
  }

  // Envoi BLE chaque seconde
  if (maintenant - dernierEnvoi >= 1000) {
    dernierEnvoi = maintenant;

    Serial.print("Vitesse : ");
    Serial.print(vitesseFiltree, 2);
    Serial.println(" km/h");

    if (clientConnecte) {
      uint16_t valeur = (uint16_t)(vitesseFiltree * 100);
      caracVitesse->setValue(valeur);
      caracVitesse->notify();
    }
  }
}