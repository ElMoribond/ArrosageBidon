/** Gestion du bidon d'arrosage du balcon.
    Celui-ci, d'une contenance de 5L possède une electrovanne 12V (via relai donc)
    Ainsi que de deux sondes de niveau (à bulle) placées en haut et en bas.
    Gyroscope (MPU6050) est là pour ne pas remplir si il est tombé.
    Pour forcer le mode Access Point (et accéder configuration) il suffit que le bidon ne puisse pas se connecter au Wifi.
    Après 30 secondes de tentative, il bascule en mode AP et il faut se connecter sur le bidon en Wifi.
    Puis, avec navigateur, sur http://192.168.4.1/
    Cosmic, nov 2019

    Câblage ESP-12 D1 Mini
    D0  16                   -- Relai
    D1  5  SCL               -- I²c BME280 & MPU 6050
    D2  4  SDA               -- I²c
    D5  14                   -- Sonde haut
    D6  12                   -- Sonde bas

    Blynk Virtual Pins:
    V0                       Température
    V1                       Humidité
    V2                       Pression Athmos.
    V3                       Nb secondes depuis 1970
    V4                       Orientation du bidon Ok
    V5                       Relai Allumé
    V6                       Sonde Haute
    V7                       Sonde Basse
    V8                       Bouton App Android pour autorisé/interdire arrosage
**/
#include <Arduino.h>
#include <EEPROM.h>
#include <ESP8266WiFi.h>
#include <ESP8266mDNS.h>
#include <ESP8266WebServer.h>
#include <NTPClient.h>
#include <WiFiUdp.h>
#include "math.h"
#include <DebounceEvent.h>
#include <Thread.h>
#include <StaticThreadController.h>
#include "Wire.h"
#include "I2Cdev.h"
#include "MPU6050.h"
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>

#define BLYNK_SSL_USE_LETSENCRYPT
#define RELAY_PIN          16
#define HAUT_PIN           14
#define BAS_PIN            12
const char* Ap_name        = "*************"; // nom de l'access point (192.168.4.1)
const char* Ap_pass        = "************"; // mot de passe de lAP Wifi & de OTA
const char* www_username   = "**********"; // username pour accès configuration en mode Access Point et MAJ via Wifi (OTA)
const char* www_password   = "*********"; // mot de passe  pour accès configuration en mode Access Point et MAJ via Wifi
const char* NoGoodPassword = "Mauvais mot de passe ~(";
const char* _codesource_   = "https://github.com/ElMoribond/ArrosageBidon";
const char* _url_          = "*******.org";
const long _port_          = 8082;
const char* _title_        = "Alerte Arrosage Bidon Balcon !!!";
const char* _msg_          = "Vérifier au minimum le niveau horizontal.";
const char* _auth_         = "********************************";
const byte DelaiHotSpot    = 30; // délai détection Wifi avant passage mode Access Point en secondes

typedef struct {
  int secs                 = 15; // fréquence d'envoie temp./humid/pression
  int flashCounter         = 34; // nombre de modification EEPROM
  long port                = 9443; // port serveur Blynk
  char auth[33]            = ""; // token Blynk
  char ssid[33]            = "**********"; // SSID Wifi
  char pass[65]            = "**********"; // Mot de passe Wifi
  char url[33]             = ""; // URL mon serveur Blynk
  char mail[65]            = "*******@***.fr"; // email notification
  char title[49]           = ""; // titre email
  char msg[129]            = ""; // msg email
} Config_T;
Config_T Configuration; // données sauvegardées/lues EEPROM
// ne pas oublier d'effacer Flash pour réinitialiser en gérant compteur si changement dESP

#define LOGDEBUG 1
#if defined(LOGDEBUG)
  #define DEBUG_PRINTLN(x)  Serial.println(x)
  #define DEBUG_PRINT(x)    Serial.print(x)
  #define BLYNK_PRINT       Serial
#else
  #define DEBUG_PRINTLN(x)
  #define DEBUG_PRINT(x)
#endif
#include <BlynkSimpleEsp8266.h>

template <class T> bool EEPROMreadAnything(T& value) {
  EEPROM.begin(sizeof(Config_T));
  delay(123);
  byte* p = (byte*)(void*)&value;
  unsigned int i;
  for (i = 0; i < sizeof(Config_T); *p++ = EEPROM.read(i++));
  EEPROM.end();
  delay(123);
  if (i != sizeof(Config_T)) {
    DEBUG_PRINTLN("Erreur Lecture EEPROM");
    return false; }
  DEBUG_PRINTLN("Lecture EEPROM Terminée");
  /*DEBUG_PRINTLN(value.ssid);
  DEBUG_PRINTLN(value.pass);
  DEBUG_PRINTLN(value.url);
  DEBUG_PRINTLN(value.auth);
  DEBUG_PRINTLN(value.port);
  DEBUG_PRINTLN(value.secs);
  DEBUG_PRINTLN(value.flashCounter);
  DEBUG_PRINTLN(value.mail);
  DEBUG_PRINTLN(value.title);
  DEBUG_PRINTLN(value.msg);*/
  return true;
}

int lastConnectionAttempt = millis();
int connectionDelay = 5000; // try to reconnect every 5 seconds

template <class T> bool EEPROMwriteAnything(const T& value) {
  DEBUG_PRINTLN("SAV");
  EEPROM.begin(sizeof(Config_T));
  delay(123);
  const byte* p = (const byte*)(const void*)&value;
  unsigned int i;
  for (i = 0; i < sizeof(Config_T); EEPROM.write(i++, *p++));
  EEPROM.end();
  DEBUG_PRINTLN("Ok");
  return i == sizeof(Config_T);
}

MPU6050 accelgyro;
int16_t ax, ay, az, gx, gy, gz;
float angle;
const float Coeff = 0.28;
const int ORIENTATION_DELTA = 21;

Adafruit_BME280 Bme;
struct Sensor_T { bool StatusBME; float h, t, p; } Sensor;

ESP8266WebServer WebServer(80);
ESP8266WebServer OTAWebServer(80);
struct Color_T { const char background[8] = "#336699", text[8] = "#ecf2f9"; } WebColor;
bool WebServerStatus;

bool ArrosageAutorise;
bool OLDOrientationOK = true;
bool OrientationOk = !OLDOrientationOK;
void SendNotif();
void SendStatBidon(bool sendAutorise = false);

DebounceEvent SondeBasse = DebounceEvent(BAS_PIN, [](uint8_t p, uint8_t e, uint8_t c, uint16_t l) { SendStatBidon(); }, BUTTON_PUSHBUTTON | BUTTON_DEFAULT_HIGH | BUTTON_SET_PULLUP);
DebounceEvent SondeHaute = DebounceEvent(HAUT_PIN, [](uint8_t p, uint8_t e, uint8_t c, uint16_t l) { SendStatBidon(); }, BUTTON_PUSHBUTTON | BUTTON_DEFAULT_HIGH | BUTTON_SET_PULLUP);
// Sonde****.pressed() = True SI bulle de niveau en bas (pas d'eau à ce niveau)

Thread* ThreadOrientation = new Thread();
StaticThreadController<1> Controll(ThreadOrientation);
BlynkTimer Timer; // pour envoyer data du BME280

// LEDs sur smartphone
WidgetLED LEDok(V4);
WidgetLED LEDrelai(V5);
WidgetLED LEDhaut(V6);
WidgetLED LEDbas(V7);

BLYNK_WRITE(V8) { // pour récupérer le bouton du phone
  bool wish = param.asInt();
  if (wish) {
    if (!OrientationOk) {
      ArrosageAutorise = !wish;
      SendStatBidon(true); }
    else ArrosageAutorise = wish; }
  else ArrosageAutorise = wish;
  DEBUG_PRINTLN(String("Remplissage: ") + (ArrosageAutorise ? "Oui" : "Non"));
  SendNotif();
}

void SendBME() { // envoie température, humidité & pression athmo.
  if (Sensor.StatusBME) {
    Sensor.h = Bme.readHumidity();
    Sensor.t = Bme.readTemperature();
    Sensor.p = Bme.readPressure() / 100.0F;
    DEBUG_PRINTLN(String("Température: ") + Sensor.t + " *C  " + "Humidité: " + Sensor.h + " %  " + "Pression: " + Sensor.p + " hPa"); }
  else {
    DEBUG_PRINTLN("Erreur BME");
    Sensor.h = Sensor.t = Sensor.p = 0; }
  if (WiFi.status() == WL_CONNECTED && Blynk.connected()) {
    Blynk.virtualWrite(V2, Sensor.p);
    Blynk.virtualWrite(V0, Sensor.t);
    Blynk.virtualWrite(V1, Sensor.h);
    Blynk.notify(String("Running for ") + millis() / 60000L + " minutes."); }
}

void SendNotif() { // notification email
  if (WiFi.status() == WL_CONNECTED && Blynk.connected()) {
    Blynk.email(Configuration.mail, Configuration.title, Configuration.msg);
    Blynk.notify(String(Configuration.title) + " - " + Configuration.msg); }
}

void CheckRelai() { // vérification de létat du relai
  if (ArrosageAutorise && OrientationOk && SondeBasse.pressed() && SondeHaute.pressed()) {
    if (digitalRead(RELAY_PIN) == HIGH) {
      digitalWrite(RELAY_PIN, LOW);
      DEBUG_PRINTLN("Vanne Ouverte");
      SendStatBidon(); } }
  else if (!(ArrosageAutorise && !SondeBasse.pressed() && SondeHaute.pressed()) || !OrientationOk || !ArrosageAutorise) {
    if (digitalRead(RELAY_PIN) == LOW) {
      digitalWrite(RELAY_PIN, HIGH);
      DEBUG_PRINTLN("Vanne Fermée");
      if (ArrosageAutorise && !OrientationOk)
        SendNotif();
      SendStatBidon(); } }
}

String getHeaderPage() { // entête des pages HTML
  return String("<!DOCTYPE html>\n<html>\n<head>\n<title>Arroseur Balcon by Cosmic</title>\n") +
    "<meta http-equiv='Content-Type' content='text/html; charset=UTF-8' />\n" +
    "<style> body { background-color: " + WebColor.background + "; font-family: Arial, Helvetica, Sans-Serif; Color: " + WebColor.text +
    "; }</style>\n<script type='text/javascript'>\nfunction autoFill() {\n" + 
    "document.getElementById('_auth').value = '" + _auth_ + "';\n" +
    "document.getElementById('_url').value = '" + _url_ + "';\n" +
    "document.getElementById('_port').value = '" + _port_ + "';\n" +
    "document.getElementById('_title').value = '" + _title_ + "';\n" +
    "document.getElementById('_msg').value = '" + _msg_ + "';\n}</script>\n</head>\n<body><h1>Arroseur Balcon</h1>";
}

String getFooterPage(String tmp = "") { return tmp + "</body></html>"; }  // pied de page des pages HTML

void HandleForm() {
  if (!WebServer.authenticate(www_username, www_password)) {
    DEBUG_PRINTLN(NoGoodPassword);
    return WebServer.requestAuthentication(); }
  DEBUG_PRINT("\nSauvegarde EEPROM ");
  if (WebServer.arg("_ssid") == Configuration.ssid && WebServer.arg("_pass") == Configuration.pass && WebServer.arg("_url") == Configuration.url &&
    WebServer.arg("_auth") == Configuration.auth && WebServer.arg("_mail") == Configuration.mail && WebServer.arg("_title") == Configuration.title &&
    WebServer.arg("_msg") == Configuration.msg && atol(WebServer.arg("_port").c_str()) == Configuration.port && atoi(WebServer.arg("_secs").c_str()) == Configuration.secs) {
    DEBUG_PRINT("Annulée");
    WebServer.sendHeader("Location", "/");
    WebServer.send(303); }
  else {
    DEBUG_PRINTLN("Tentée");
    strcpy(Configuration.url,   WebServer.arg("_url").c_str());
    strcpy(Configuration.auth,  WebServer.arg("_auth").c_str());
    strcpy(Configuration.ssid,  WebServer.arg("_ssid").c_str());
    strcpy(Configuration.pass,  WebServer.arg("_pass").c_str());
    strcpy(Configuration.mail,  WebServer.arg("_mail").c_str());
    strcpy(Configuration.title, WebServer.arg("_title").c_str());
    strcpy(Configuration.msg,   WebServer.arg("_msg").c_str());
    Configuration.secs = atoi(WebServer.arg("_secs").c_str());
    Configuration.port = atol(WebServer.arg("_port").c_str());
    Configuration.flashCounter++;
    bool ok;
    WebServer.send(200, "text/html", getFooterPage(getHeaderPage() +
      (ok = EEPROMwriteAnything(Configuration)) ? "Sauvegardé dans EEPROM :)\n\nESP Redémarré" : "Erreur Sauvegarde EEPROM"));
    if (ok)
      ESP.restart(); }
}

void OrientationCallback() { // vérification si bidon tombé
  accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
  angle = Coeff * (angle + float(gy) * 0.01 / 131) + (1 - Coeff) * atan2((double)ax, (double)az) * 180 / PI;
  if (OLDOrientationOK != (OrientationOk = angle > -ORIENTATION_DELTA && angle < ORIENTATION_DELTA)) {
    OLDOrientationOK = OrientationOk;
    DEBUG_PRINTLN(String("Orientation changée ") + " " + (OrientationOk ? "Ok" : "NOK") + " " + angle + "°");
    if (!OrientationOk)
      SendNotif();
    SendStatBidon(); }
}

void NiveauCallback(uint8_t pin, uint8_t event, uint8_t count, uint16_t length) { // vérification des niveaux
  SendStatBidon();
}

void SendStatBidon(bool sendAutorise) { // envoie des datas vers serveur Blynk
  digitalRead(RELAY_PIN) == LOW ? LEDrelai.on() : LEDrelai.off();
  OrientationOk ? LEDok.on() : LEDok.off();
  SondeBasse.pressed() ? LEDbas.off() : LEDbas.on();
  SondeHaute.pressed() ? LEDhaut.off() : LEDhaut.on();
  DEBUG_PRINT(String("Orientation ") + (OrientationOk ? "" : "N-") + "Ok");
  DEBUG_PRINT(String(" Relai ") + (digitalRead(RELAY_PIN) == LOW ? "" : "N-") + "Actif");
  DEBUG_PRINT(String(" Niveau Bas ") + (SondeBasse.pressed() ? "Sec" : "Ok"));
  DEBUG_PRINT(String(" Niveau HauT ") + (SondeHaute.pressed() ? "Sec" : "Ok"));
  DEBUG_PRINTLN(String(" RemplissagE: ") + (ArrosageAutorise && digitalRead(RELAY_PIN) == LOW ? "Oui" : "Non"));
  if (WiFi.status() == WL_CONNECTED && Blynk.connected() && sendAutorise)
    Blynk.virtualWrite(V8, ArrosageAutorise);
}

// Doit être exécuté en cas de changement de l'ESP12 par ex.
void Init_EEPROM_1ere_Fois_SEULEMENT_presque_debut_setup() {
  strcpy(Configuration.auth, _auth_);
  strcpy(Configuration.url, _url_);
  strcpy(Configuration.title, _title_);
  strcpy(Configuration.msg, _msg_);
  DEBUG_PRINTLN("Ecriture COMPLÈTE Flash");
  EEPROMwriteAnything(Configuration);
  DEBUG_PRINTLN("Fini");
  delay(5000);
  for (;; DEBUG_PRINTLN("Mettre en rem dans le source puis....")) delay(3000);
}

void setup() {
  Serial.begin(115200);
  delay(1000);
  for (uint i = 0; i < 10; i++) {
    DEBUG_PRINT("¤");
    delay(1000); }
  DEBUG_PRINTLN("\n-- Démarrage --");
  // ------- Init_EEPROM_1ere_Fois_SEULEMENT_presque_debut_setup(); ---------------//
  pinMode(RELAY_PIN, OUTPUT);
  CheckRelai();
  Sensor.StatusBME = Bme.begin(0x76);
  DEBUG_PRINTLN(String("BME280 ") + (Sensor.StatusBME ? "Ok" : "non détecté"));
  accelgyro.initialize();
  for (; !accelgyro.testConnection(); DEBUG_PRINTLN("Echec MPU6050"))
    delay(1234);
  delay(123);
  ThreadOrientation->onRun(OrientationCallback);
  ThreadOrientation->setInterval(250);
  OrientationCallback();
  if (!EEPROMreadAnything(Configuration)) {
    DEBUG_PRINT("Oups Lecture EEPROM");
    for (uint i = 0; i < 10; i++) {
      DEBUG_PRINT(" !");
      delay(1000); }
    ESP.restart(); }
    /*strcpy(Configuration.url, _url_);
    strcpy(Configuration.title, _title_);
    strcpy(Configuration.msg, _msg_);
    strcpy(Configuration.auth, _auth_);
    Configuration.port = _port_; } */
  DEBUG_PRINTLN(String("Connexion à ") + Configuration.ssid);
  // 1ère connexion Wifi pour vérifier si passage en mode Access Point
  WiFi.disconnect();
  WiFi.mode(WIFI_STA);
  WiFi.begin(Configuration.ssid, Configuration.pass);
  int ii = 0;
  while (WiFi.status() != WL_CONNECTED && !WebServerStatus) {
    delay(500);
    DEBUG_PRINT(".");
    if (++ii > DelaiHotSpot * 2) WebServerStatus = true; }
  if (!WebServerStatus) {
    DEBUG_PRINTLN("\nWiFi Ok"); }
  else { // passage en mode Access Point
    WiFi.disconnect();
    delay(123);
    WiFi.mode(WIFI_AP);
    DEBUG_PRINTLN("\nErreur Connexion Wifi\nMode Serveur Web Activé");
    DEBUG_PRINTLN(String(WiFi.softAP(Ap_name, Ap_pass) ? "Ok " : "Erreur!") + "AP address: " + WiFi.softAPIP().toString());
    delay(123);
    WebServer.on("/FormPage", HTTP_POST, HandleForm); // traitement du formulaire
    WebServer.on("/", HTTP_GET, []() { // page HTML d'accueil
      if (!WebServer.authenticate(www_username, www_password)) {
        DEBUG_PRINTLN(NoGoodPassword);
        return WebServer.requestAuthentication(); }
      WebServer.sendHeader("Connection", "close");
      WebServer.send(200, "text/html", getFooterPage(getHeaderPage() + "<form action='/FormPage' method='POST'>" + "SSID Wifi<br><input type='text' id='_ssid' name='_ssid' required minlength='8' maxlength='32' size='32' value='" +
        Configuration.ssid  + "'><br>" + "Mot de passe<br><input type='text' id='_pass' name='_pass' required minlength='8' maxlength='64' size='32' value='" +
        Configuration.pass  + "'><br>" + "<hr>Periode MAJ (secondes)<br><input type='text' id='_secs' name='_secs' required minlength='2' maxlength='3' size='4' value='" +
        Configuration.secs  + "'><br>" + "<hr>URL Blynk*<br><input type='text' id='_url' name='_url' required minlength='10' maxlength='64' size='24' value='" +
        Configuration.url   + "'><br>" + "Port du serveur Blynk*<br><input type='text' id='_port' name='_port' required minlength='2' maxlength='4' size='5' value='" +
        Configuration.port  + "'><br>" + "Code Auth Blynk*<br><input type='text' id='_auth' name='_auth' required minlength='32' maxlength='32' size='32' value='" +
        Configuration.auth  + "'><br>" + "<hr>Email<br><input type='text' id='_mail' name='_mail' required minlength='8' maxlength='64' size='32' value='" +
        Configuration.mail  + "'><br>" + "Titre Email*<br><input type='text' id='_title' name='_title' required minlength='32' maxlength='32' size='32' value='" +
        Configuration.title + "'><br>" + "Message*<br><textarea id='_msg' name='_msg' rows='5' cols='33' required minlength='12' maxlength='128'>" + Configuration.msg + "</textarea><br>" +
        "<input id='_raz' type='button' value='*Par Défaut' onclick='autoFill();' /><br><input type='submit' value='Enregistrer dans mémoire'></form>" +
        "<hr><a href='" + _codesource_ + "' target='_blank'>Code source</a>")); });
    //WebServer.onNotFound(handleNotFound);
    WebServer.onNotFound([]() { WebServer.send(404, "text/plain", "Erreur 404: Page Introuvable :("); });
    WebServer.begin();
    return; }
  WiFi.disconnect(); // déconnexion pour laisser gérer le Wifi par Blynk
  DEBUG_PRINTLN("-- Déconnection du Wifi Ok pour laisser faire Blynk --");
  Blynk.begin(Configuration.auth, Configuration.ssid, Configuration.pass, Configuration.url, Configuration.port);
  SendStatBidon(true);
  SendBME();
  Timer.setInterval(1000L * Configuration.secs, SendBME);
  DEBUG_PRINTLN("-- Fin de Configuration --");
}

void loop() {
  CheckRelai();
  Controll.run(); // vérif. bidon tombé
  SondeBasse.loop();
  SondeHaute.loop();
  if (WebServerStatus)
    WebServer.handleClient();
  else {
    if (WiFi.status() == WL_CONNECTED && Blynk.connected()) {
      Blynk.run();
      Timer.run(); }
    /*else if (digitalRead(RELAY_PIN) == LOW) {
      digitalWrite(RELAY_PIN, HIGH);
      DEBUG_PRINTLN(String("Vanne Fermée car Déconnecté de ") + Configuration.url); } */
      }
  delay(12);
}
