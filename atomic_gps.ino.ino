// AtomiC-GPS est un logger de GPS au format GPX et un enregistreur des variables environnementales
// M5Atom-GPS + ENV.III (SHT30 + QMP6988)
// version 1.0

#include "M5Atom.h"
#include <WiFi.h>
#include <WiFiClient.h>
#include <WebServer.h>
#include <Arduino.h>
#include <SPI.h>
#include "FS.h"
#include <SD.h>
#include <TinyGPSPlus.h>
#include "esp_log.h"
#include "M5UnitENV.h"  // Bibliothèque pour les capteurs SHT30 et QMP6988
#include <Wire.h>
#include <CSV_Parser.h>

#define TIME_ZONE 2 //valeur du TIME_ZONE par défaut
#define RECORD_FREQUENCY 5000 //Fréquence d'acquisition
#define ACTIVATE_DEEP_SLEEP_ONCLICK true
#define GPS_TIMEOUT 5000
#define DISPLAY_GPS_DATA true

static const char* TAG = "atomic_gps";
// Configuration du serveur web
const char* ssid = "AtomiC_GPS";
const char* password = "87654321";
WebServer server(80);// Server Web

// Prototypes
static void readGPS(unsigned long ms);
static void printGPSData();
static void createDataFile();
static void createEnvFile();
static void addGPXPoint();
static void addEnvData();
static void blink_led_orange();
static void blink_led_red();
static void blink_led_blue();
static void blink_led_green();
static void printWakeupReason();
static void checkSatellites();  // Nouvelle fonction pour vérifier les satellites

TinyGPSPlus gps;// The TinyGPS++ object
HardwareSerial gps_uart(1);//Defintion acces au GPS
SHT3X sht3x;  // Capteur SHT30
QMP6988 qmp;  // Capteur QMP6988

bool firstStart = true;  // Crée le fichier GPX au premier démarrage
char filepath_gpx[25];   // Nom du fichier GPX
char filepath_env[25];   // Nom du fichier de données environnementales
char filepath_stats[25]; // Nom du ficher des statistiques
char today_folder[15];   // Dossier pour les fichiers du jour
char point_date[22]; // Date et heure du point GPS
float prev_lat = 0.0f;  // Aucun point précédent au démarrage
float prev_long = 0.0f;
bool ENABLE_GPS = true;
bool DESACTIVATE_GPS = false;
bool BT_ON = true;
bool BT_OFF = false;
// Variables pour les statistiques de température
float temp_min = 100.0;
float temp_max = -100.0;
float total_distance = 0.0;
String time_temp_min;
String time_temp_max;
String last_measure_time;
String current_time;

#define FORCE_UPDATE_STATS true
#define SPEED_BUFFER_SIZE 10

typedef struct TODAYSTATS
{
  float dist;
  float speed_max;
  float speed_mean; 
  double  speedbuffer[SPEED_BUFFER_SIZE];
  bool  speedbufferfull = false;
  int   speedbufferpos = 0;
} TODAYSTATS_t;

TODAYSTATS_t today_stats;

unsigned long previousMillis = 0;

void external_button_pressed() {
  if (!ENABLE_GPS) {
    // Action si GPS désactivé
  } else {
    blink_led_green();
  }
}
//GENERATION DE LA PAGE WEB
void handleRoot() {
  String html = "<html><head><meta http-equiv='refresh' content='10'></head><body>";
  html += "<h1 style='text-align:center;'>AtomiC-GPS Data</h1>";
  
  // Affichage du nom du dossier en cours d'utilisation
  html += "<h2>Current Folder: " + String(today_folder) + "</h2>";
  
  // Section GPS
  html += "<h2>GPS Data</h2>";
  html += "<p>Satellites: " + String(gps.satellites.value(), 6) + "</p>";
  html += "<p>Latitude: " + String(gps.location.lat(), 6) + "</p>";
  html += "<p>Longitude: " + String(gps.location.lng(), 6) + "</p>";
  html += "<p>Altitude: " + String(gps.altitude.meters()) + " m</p>";
  html += "<p>GPS Time: " + String(current_time) + "</p>";
  // Section Capteurs environnementaux
  html += "<h2>Environmental Sensor Data</h2>";
  html += "<p>Temperature: " + String(sht3x.cTemp) + " C</p>";
  html += "<p>Humidity: " + String(sht3x.humidity) + " %</p>";
  html += "<p>Pressure: " + String(qmp.pressure / 100.0F) + " hPa</p>";
  html += "<p>Altitude: " + String(qmp.altitude) + " m</p>";
   html += "<p>Sensor Time: " + String(point_date) + "</p>";
  // Section des statistiques de température
  html += "<h2>Temperature Statistics</h2>";
  html += "<p>Minimum Temperature: " + String(temp_min) + " C at " + time_temp_min + "</p>";
  html += "<p>Maximum Temperature: " + String(temp_max) + " C at " + time_temp_max + "</p>";
  // Affichage de l'heure de la dernière mesure
  html += "<h2>Last Measurement</h2>";
  html += "<p>Last Measurement Time: " + last_measure_time + "</p>";
  //Affichage du nombre de Km parcouru
  html += "<h2>Km parcouru</h2>";
  html += "<p>Total Distance: " + String(total_distance / 1000.0, 2) + " km</p>";
  //Bas de page
  html += "</body></html>";
  server.send(200, "text/html", html);
}
//Application d'un style perso sur la page web
void handleCSS() {
  String css = "body { font-family: Arial, sans-serif; text-align: center; }";
  css += "h1 { color: #333; }";
  css += "h2 { color: #666; }";
  css += "p { color: #999; }";
  server.send(200, "text/css", css);
}

void setup() {
  // Initialisation des niveaux de log
  esp_log_level_set("*", ESP_LOG_VERBOSE);  // Définit le niveau de log pour tous les modules
  
  M5.begin(true, true, true);// begin(bool SerialEnable , bool I2CEnable , bool DisplayEnable ) 
  delay(50);

  WiFi.softAP(ssid, password);//Activation du hotspot
  Serial.print("Adresse IP : ");
  Serial.println(WiFi.softAPIP());

  Wire.begin(26, 32); // Initialisation du bus I2C

  SPI.begin(23, 33, 19, -1);// Initialisation du bus SPI
  if (!SD.begin(-1, SPI, 40000000)) {
    ESP_LOGE(TAG, "initialization failed!");
    for (int i = 0; i < 3; i++) {
      delay(1000);
      if (SD.begin(-1, SPI, 40000000)) {
        ESP_LOGI(TAG, "SD card reinitialized successfully");
        break;
      }
    }
  } else {
    sdcard_type_t Type = SD.cardType();
    Serial.printf("SDCard Type = %d \\\\r\\\\n", Type);
    Serial.printf("SDCard Size = %d \\\\r\\\\n", (int)(SD.cardSize() / 1024 / 1024));
  }
  // Initialisation du port série pour le GPS
  gps_uart.begin(9600, SERIAL_8N1, 22, -1); 
  delay(250);
 
    // Vérification de l'initialisation du GPS
    if (gps_uart) {
        Serial.println("GPS module initialized successfully");
        M5.dis.drawpix(0, 0, 0);
    } else {
        Serial.println("Failed to initialize GPS module");
    }

 // Désactive le GPS en appuyant sur le bouton du M5Atom Lite (GPIO39)
    pinMode(39, INPUT);
    attachInterrupt(39, [] {
      ENABLE_GPS = !ENABLE_GPS;
      ESP_LOGI(TAG, "Change GPS stat %u", ENABLE_GPS);
      if (!ENABLE_GPS) DESACTIVATE_GPS = true;
    },
    RISING);

  printWakeupReason();

  // Initialisation des capteurs SHT30 et QMP6988
  if (!qmp.begin(&Wire, QMP6988_SLAVE_ADDRESS_L, 26, 32, 400000U)) {
    ESP_LOGE(TAG, "Couldn't find QMP6988");
  }
  if (!sht3x.begin(&Wire, SHT3X_I2C_ADDR, 26, 32, 400000U)) {
    ESP_LOGE(TAG, "Couldn't find SHT3X");
  }
  
  // Configuration de la page web
  server.on("/", handleRoot);
  server.on("/style.css", handleCSS);
  server.begin();
  Serial.println("HTTP server started");
}

void loop() {
  
  server.handleClient();
  M5.update();
  
  unsigned long currentMillis = millis();
  if (currentMillis - previousMillis >= RECORD_FREQUENCY) {
    previousMillis = currentMillis;
    ESP_LOGI(TAG, "Cycle de la boucle exécuté après %lu ms", RECORD_FREQUENCY);
  }
  if (DESACTIVATE_GPS) {
    DESACTIVATE_GPS = false;
    blink_led_orange();
    blink_led_orange();
    if (ACTIVATE_DEEP_SLEEP_ONCLICK) {
      // On met en sommeil le module
      ESP_LOGI(TAG, "Wakeup on button activated");
      esp_sleep_enable_ext0_wakeup(GPIO_NUM_39, 0);
      esp_deep_sleep_start();
    }
  }
  if (ENABLE_GPS) {
    checkSatellites();  // Vérifie le nombre de satellites connectés
    readGPS(500);
    
    if (gps.location.isValid() && gps.date.isValid()) {
      if (firstStart) {
        Serial.println("Creation des fichiers");
        //SerialBT.println("Creation des fichiers");
        createDataFile();
        createEnvFile();
        delay(250);
        firstStart = false;
      }
      addGPXPoint();//Lancement lecture GPS et remplissage fichier GPX
      ESP_LOGI(TAG, "Appel de la fonction addGPXPoint");
      addEnvData();//Lancement lecture GPS et remplissage fichier de données
      ESP_LOGI(TAG, "Appel de la fonction addEnvData");
      //delay(250);
    }
    if (millis() > GPS_TIMEOUT && gps.charsProcessed() < 10)
      ESP_LOGW(TAG, "No GPS data received: check wiring or position");
  }
   // Mise à jour des statistiques de température
  if (sht3x.update()) {
    float temperature = sht3x.cTemp;
    current_time = String(gps.time.hour()) + ":" + String(gps.time.minute()) + ":" + String(gps.time.second());
    
    if (temperature < temp_min) {
      temp_min = temperature;
      time_temp_min = point_date;
    }
    if (temperature > temp_max) {
      temp_max = temperature;
      time_temp_max = point_date;
    }
    // Mise à jour de l'heure de la dernière mesure
    last_measure_time = point_date;
  }

  delay(RECORD_FREQUENCY);
}

static void checkSatellites() {
  int numSatellites = gps.satellites.value();
  if (numSatellites > 0) {
    Serial.printf("Connected to %d satellites\n", numSatellites);
    blink_led_green();  // Indique une connexion réussie
  } else {
    Serial.println("No satellites connected");
    ESP_LOGW(TAG, "No satellites connected");
    blink_led_red();  // Indique l'absence de connexion
  }
}

static void createDataFile() {
  //Creation du dossier du jour
  sprintf(today_folder, "/%04d-%02d-%02d", gps.date.year(), gps.date.month(), gps.date.day());
  if (!SD.exists(today_folder)) {
    SD.mkdir(today_folder);
    ESP_LOGI(TAG, "Create today folder %s", today_folder);
  } else {
    ESP_LOGI(TAG, "Today folder already exists %s\\n", today_folder);
  }
  //Creation du fichier GPX et statistique
  sprintf(filepath_gpx, "%s/track.gpx", today_folder);
  sprintf(filepath_stats, "%s/stats.csv", today_folder);
  ESP_LOGI(TAG, "GPX file path %s\\n", filepath_gpx);
  ESP_LOGI(TAG, "Stats file path %s\\n", filepath_stats);

// Si le fichier existe recharge les statistiques du jour 
  if ( SD.exists(filepath_stats) ) {
    File statsFile = SD.open(filepath_stats, FILE_READ);
    if ( statsFile ) {
      CSV_Parser cp(/*format*/ "sf", /*has_header*/ true, /*delimiter*/ ',');
      cp.readSDfile(filepath_stats);

      // Il faut absolument vérifier que le fichier n'est pas vide, sinon plantage assuré !
      if ( cp.getRowsCount() > 0 ) {
        float *val_col = (float*)cp["value"];

        if (val_col) 
          today_stats.dist        = (float)val_col[0]; 
          today_stats.speed_max   = (float)val_col[1];  
          today_stats.speed_mean  = (float)val_col[2];  
          cp.print();
          //Serial.printf("dist %f | max speed %f | mean speed %f  \n", today_stats.dist, today_stats.speed_max, today_stats.speed_mean);
      } else {
        // Le fichier est probablement corrompu, on le supprime
        ESP_LOGW(TAG,"Stat file removed because probably corrupted. I'll be re-saved next time");
        SD.remove(filepath_stats);
      } 
    }  
  }  

  //Creation du fichier GPX du jour s'il n'existe pas
  if (!SD.exists(filepath_gpx)) {
    ESP_LOGI(TAG, "Create new GPX file %s", filepath_gpx);
    File gpxFile = SD.open(filepath_gpx, FILE_WRITE);
    if (gpxFile) {
      gpxFile.print(F(
        "<?xml version=\"1.0\" encoding=\"UTF-8\"?>\r\n"
        "<gpx version=\"1.1\" creator=\"mon.arrosage.auto.free.fr\" xmlns=\"http://www.topografix.com/GPX/1/1\" \r\n"
        "xmlns:xsi=\"http://www.w3.org/2001/XMLSchema-instance\"\r\n"
        "xsi:schemaLocation=\"http://www.topografix.com/GPX/1/1 http://www.topografix.com/GPX/1/1/gpx.xsd\">\r\n"
        "\t<trk>\r\n<trkseg>\r\n")); 
      gpxFile.print(F("</trkseg>\r\n</trk>\r\n</gpx>\r\n"));
      gpxFile.close();
    } else {
      ESP_LOGW(TAG, "Impossible to open %s GPX file", filepath_gpx);
      blink_led_red();
    }
  } else {
    ESP_LOGI(TAG, "%s file already exists", filepath_gpx);
  }
}

static void createEnvFile() {
  sprintf(filepath_env, "%s/env_data.txt", today_folder);
  ESP_LOGI(TAG, "Env data file path %s\\n", filepath_env);
  if (!SD.exists(filepath_env)) {
    ESP_LOGI(TAG, "Create new Env data file %s", filepath_env);
    File envFile = SD.open(filepath_env, FILE_WRITE);
    if (envFile) {
      envFile.println("Timestamp, Temperature (C), Humidity (%), Pressure (hPa), Altitude(m)");
      envFile.close();
    } else {
      ESP_LOGW(TAG, "Impossible to open %s Env data file", filepath_env);
      blink_led_red();
    }
  } else {
    ESP_LOGI(TAG, "%s file already exists", filepath_env);
  }
}
//LIT LE MESSAGE RENVOYE PAR LE MODULE GPS
static void readGPS(unsigned long ms) {
  ESP_LOGI(TAG, "Read GPS stream");
  bool led_state = true;
  unsigned long start = millis();
  do {
    while (gps_uart.available())
      gps.encode(gps_uart.read());
    if ((millis() - start) % 250 != 80) {
      led_state = !led_state;
    }
    if (led_state)
      M5.dis.drawpix(0, 0, 0x0099ff);
    else
      M5.dis.drawpix(0, 0, 0);
  } while (millis() - start < ms);

  // Vérification du nombre de satellites
  int numSatellites = gps.satellites.value();
  if (numSatellites > 0) {
    Serial.printf("Connected to %d satellites\n", numSatellites);
    if (DISPLAY_GPS_DATA) printGPSData();
  } else {
    Serial.println("No satellites connected");
    ESP_LOGW(TAG, "No satellites connected");
    blink_led_red();  // Indique l'absence de connexion
  }

  M5.dis.drawpix(0, 0, 0);
}

static void addGPXPoint() {
  // Only if speed > 5 km/h, update max/mean speed, distance traveled
  // Uniquement si la vitesse > 5 km/h, actualise vitesse moyenne/max, distance parcourue
  // Randonnee pedestre = 2 km/h
  if ( gps.speed.kmph() > 2 ) {
    // Add new speed point in the buffer
    // Ajoute la vitesse au buffer tournant permettant de calculer la vitesse moyenne
    double current_speed = gps.speed.kmph();

    today_stats.speedbuffer[today_stats.speedbufferpos] = current_speed;
    today_stats.speedbufferpos += 1;

    if ( current_speed > today_stats.speed_max ) today_stats.speed_max = current_speed;
      
    if ( today_stats.speedbufferpos >= 10 ) {
      today_stats.speedbufferpos = 0;
      today_stats.speedbufferfull = true;
    }
      
    if ( today_stats.speedbufferfull ) {
      float speed_mean = 0;
      for (int l = 0; l < SPEED_BUFFER_SIZE; l++)
      {
        speed_mean += today_stats.speedbuffer[l];
        Serial.print("add to mean"); Serial.println(today_stats.speedbuffer[l]);
      }
      Serial.print("speed_mean total"); Serial.println(speed_mean);
      
      speed_mean = speed_mean / SPEED_BUFFER_SIZE;
      today_stats.speed_mean = speed_mean;
    }    
    
    // Estimate distance traveled from the lasted position
    // Estime la distance parcourue depuis la dernière position connue
    if ( prev_lat != NULL ) {
      double _distance = gps.distanceBetween(gps.location.lat(), gps.location.lng(), prev_lat, prev_long);
      Serial.print("distance = "); Serial.println(_distance);
      // Ajoute la distance parcourue si < 1 km
      if ( _distance > 0 ) {
        //Mise à jour de la variable pour affichage
        total_distance += _distance;
        // Stocke la position actuelle pour la prochaine estimation de distance parcourue
        today_stats.dist += _distance;

        updateStatFile();
      }
      prev_lat = gps.location.lat();
      prev_long = gps.location.lng();
      //ESP_LOGI(TAG,"Distance traveled %d", _distance );
    } else {
      prev_lat = gps.location.lat();
      prev_long = gps.location.lng();
    }
  }

  if ( FORCE_UPDATE_STATS ) updateStatFile();
  
  sprintf(point_date, "%4d-%02d-%02dT%02d:%02d:%02dZ", gps.date.year(), gps.date.month(), gps.date.day(), gps.time.hour() + TIME_ZONE, gps.time.minute(), gps.time.second());
  
  File gpxFile = SD.open(filepath_gpx, FILE_WRITE);
  if (!gpxFile) {
    ESP_LOGW(TAG, "Impossible to open %s GPX file", filepath_gpx);
    blink_led_red();
  } else {
    ESP_LOGI(TAG, "Add new point %s", point_date);
    double _lat = gps.location.lat();
    double _lng = gps.location.lng();
    double _alt = gps.altitude.meters();
    double _hdop = gps.hdop.hdop();
    int _sat = gps.satellites.value();
    
    unsigned long filesize = gpxFile.size();
    // back up the file pointer to just before the closing tags
    filesize -= 27;
    gpxFile.seek(filesize);
    gpxFile.print(F("<trkpt lat=\""));
    gpxFile.print(_lat, 6);
    gpxFile.print(F("\" lon=\""));
    gpxFile.print(_lng, 6);
    gpxFile.println(F("\">"));
    gpxFile.print(F("<time>"));
    gpxFile.print(point_date);
    gpxFile.println(F("</time>"));
    // Satellites
    gpxFile.print(F("<sat>"));
    gpxFile.print(_sat);
    gpxFile.println(F("</sat>"));
    // Elevation | Altitude 
    gpxFile.print(F("<ele>"));
    gpxFile.print(_alt, 1);
    gpxFile.print(F("</ele>\r\n<hdop>"));
    gpxFile.print(_hdop, 3);
    gpxFile.println(F("</hdop>\r\n</trkpt>"));
    gpxFile.print(F("</trkseg>\r\n</trk>\r\n</gpx>\r\n"));
    gpxFile.close();

    blink_led_blue();
  }
}
//CREE OU ACTUALISE LES STATS
static void updateStatFile()
{
  File statsFile = SD.open(filepath_stats, FILE_WRITE);
  if(!statsFile) {
    ESP_LOGW(TAG, "Impossible to open %s stats file", filepath_stats);
  } else {
    ESP_LOGI(TAG, "Add stats data to file");
    
    statsFile.print(F("key,value,unit\r\n")); 
    // Distance parcourue aujourd'hui en mètres
    statsFile.print(F("distance,"));
    statsFile.print(today_stats.dist);
    statsFile.print(F(",m"));
    statsFile.print(F("\r\n"));
    // Distance parcourue aujourd'hui en km
    statsFile.print(F("distance,"));
    statsFile.print(today_stats.dist / 1000., 2);
    statsFile.print(F(",km"));
    statsFile.print(F("\r\n"));
    // Vitesse maxi aujourd'hui
    statsFile.print(F("speed_max,"));
    statsFile.print(today_stats.speed_max);
    statsFile.print(F(",hm/h"));
    statsFile.print(F("\r\n"));
    // Vitesse moyenne aujourd'hui
    statsFile.print(F("speed_mean,"));
    statsFile.print(today_stats.speed_mean);
    statsFile.print(F(",km/h"));
    statsFile.print(F("\r\n"));
    statsFile.close();
  }  
}
//AJOUT DES MESURES ENVIRONNEMENTALES
static void addEnvData() {
    // Met à jour les capteurs SHT30 et QMP6988
    bool sht3xUpdated = sht3x.update();
    bool qmpUpdated = qmp.update();
    ESP_LOGI(TAG, "SHT30 update status: %d", sht3xUpdated);
    ESP_LOGI(TAG, "QMP6988 update status: %d", qmpUpdated);
    //Serial.printf("SHT30 update status: %d", sht3xUpdated);
    //Serial.printf("QMP6988 update status: %d", qmpUpdated);

    if (sht3x.update() && qmp.update()) {
        float temperature = sht3x.cTemp;
        float humidity = sht3x.humidity;
        float pressure = qmp.pressure / 100.0F; // Convertir la pression en hPa
        float altitude = qmp.altitude;
        File envFile = SD.open(filepath_env, FILE_APPEND);
        
        // Si le fichier des données environnementales est ouvert avec succès, ajoute les données
        if (envFile) {
            ESP_LOGI(TAG, "Environmental data recorded: Temp=%.2f, Humidity=%.2f, Pressure=%.2f, Altitude=%.2f", temperature, humidity, pressure, altitude);
            envFile.printf("%s, %.2f, %.2f, %.2f, %.2f\n", point_date, temperature, humidity, pressure, altitude);
            envFile.close();
            Serial.printf("%s, %.2f, %.2f, %.2f, %.2f\n", point_date, temperature, humidity, pressure, altitude);
            blink_led_blue(); // Clignote en bleu pour indiquer l'ajout des données environnementales
        } else {
            ESP_LOGW(TAG, "Impossible to open %s Env data file", filepath_env);
            blink_led_red(); // Clignote en rouge si le fichier ne peut pas être ouvert
        }
    } else {
        ESP_LOGW(TAG, "Failed to update sensor data");
        blink_led_red(); // Clignote en rouge si la mise à jour des capteurs échoue
    }
}
//AFFOCHAGE DES DONNEES GPS
static void printGPSData() {
  Serial.print(F("Location: "));
  if (gps.location.isValid()) {
    Serial.print(gps.location.lat(), 6);
    Serial.print(F(","));
    Serial.print(gps.location.lng(), 6);
    Serial.print(F(", age:"));
    Serial.print(gps.location.age(), 6);
    Serial.print(F(", hdop:"));
    Serial.print(gps.hdop.hdop(), 3);
  } else {
    Serial.print(F("INVALID LOCATION"));
  }
  Serial.print(F(" Date/Time: "));
  if (gps.date.isValid()) {
    Serial.print(gps.date.month());
    Serial.print(F("/"));
    Serial.print(gps.date.day());
    Serial.print(F("/"));
    Serial.print(gps.date.year());
  } else {
    Serial.print(F("INVALID DATE"));
  }
  Serial.print(F(" "));
  if (gps.time.isValid()) {
    if (gps.time.hour() < 10) Serial.print(F("0"));
    Serial.print(gps.time.hour());
    Serial.print(F(":"));
    if (gps.time.minute() < 10) Serial.print(F("0"));
    Serial.print(gps.time.minute());
    Serial.print(F(":"));
    if (gps.time.second() < 10) Serial.print(F("0"));
    Serial.print(gps.time.second());
    Serial.print(F("."));
  } else {
    Serial.print(F("INVALID TIME"));
  }
  Serial.print(F(" Altitude (m):"));
  Serial.print(gps.altitude.meters());
  Serial.print(F(" Speed (km/h):"));
  if (gps.speed.isValid()) {
    Serial.print(gps.speed.kmph());
  } else {
    Serial.print(F("INVALID SPEED"));
  }
  Serial.print(F(" Course:"));
  if (gps.course.isValid()) {
    Serial.print(gps.course.deg());
  } else {
    Serial.print(F("INVALID COURSE"));
  }
  Serial.println();
}
//LOG DE LA RAISON DU REVEIL
static void printWakeupReason() {
  esp_sleep_wakeup_cause_t wakeup_reason;
  wakeup_reason = esp_sleep_get_wakeup_cause();
  switch (wakeup_reason) {
    case ESP_SLEEP_WAKEUP_EXT0: Serial.println("Wakeup caused by external signal using RTC_IO"); break;
    case ESP_SLEEP_WAKEUP_EXT1: Serial.println("Wakeup caused by external signal using RTC_CNTL"); break;
    case ESP_SLEEP_WAKEUP_TIMER: Serial.println("Wakeup caused by timer"); break;
    case ESP_SLEEP_WAKEUP_GPIO: Serial.println("Wakeup caused by GPIO"); break;
    case ESP_SLEEP_WAKEUP_UART: Serial.println("Wakeup caused by UART"); break;
    default: Serial.printf("Wakeup was not caused by deep sleep: %d\n", wakeup_reason); break;
  }
}

//--------DO NOT TOUCHE---------

static void blink_led_orange() {
  M5.dis.drawpix(0, 0, 0xe68a00);
  delay(200);
  M5.dis.drawpix(0, 0, 0);
  delay(200);
}

static void blink_led_red() {
  M5.dis.drawpix(0, 0, 0xff3300);
  delay(200);
  M5.dis.drawpix(0, 0, 0);
  delay(200);
}

static void blink_led_blue() {
  M5.dis.drawpix(0, 0, 0x0000cc);
  delay(200);
  M5.dis.drawpix(0, 0, 0);
  delay(200);
}

static void blink_led_green() {
  M5.dis.drawpix(0, 0, 0x00ff00);
  delay(200);
  M5.dis.drawpix(0, 0, 0);
  delay(200);
}
