/* Ph CORBEL 12/2018
  Gestion eclairage tunnel
  Alimentation sur panneaux solaires

  mode deep sleep
  reveille tout les matin avant heure debut
  reception des SMS en attente
  apres 5 min de fonctionnement (ex: 2mn pour reception/suppression 8 SMS, 4mn 14SMS)
  envoie sms signal vie
  analyse calendrier sauvegardé en SPIFFS

	si jour circulé
  on continue normalement
  en fin de journée retour sleep jusqu'a 06h59

  si non circulé,
  retour SIM800 et ESP32 en sleep reveil toute les heures
  au reveil attendre au moins 30s pour que les SMS arrivent,
  quand plus de SMS et traitement retour sleep 1H00

  mode normal
  SIM800 en reception
  entrees coffret, pedale 1 et 2 sur interruption ESP32
  allumage /extinction avec time out de 1 heure

  Si coupure cable(equiv pedale enfoncée en permanence), ou Porte coffret ouverte
  declenchement Alarme

  Surveillance Batterie solaire
	Adc interne instable 2 à 3.5% erreurs!
	mise en place moyenne mobile sur les adc precision <1% avec 4bits

	Circulation = CalendrierCircule ^ flagCircule (OU exclusif)
	CalCircule	|	flagCircule | Circulation
				1			|			0				|			1
				0			|			1				|			1
				0			|			0				|			0
				1			|			1				|			0

	Librairie TimeAlarms.h modifiée
	#define dtNBR_ALARMS 10 (ne fonctionne pas avec 9)   6 à l'origine nombre d'alarmes RAM*11 max is 255

  --- ATTENTION ---
	l'utilisation de l'adc sur GPIO26 pour la mesure du 24V
	est perturbé apres l'utilisation du WIFI
	entrainant une erreur de mesure,
	pouvant ne pas detecter une tension 24V trop basse
	RST reset soft sans effet
	Solution
	reset hard liaison RS<->GPIO13
	apres arret WIFI GPIO13 to LOW
	apres redemarrage adc OK

	apres OTA relancer un RST


	to do
  
  Compilation LOLIN D32,default,80MHz,, ESP32 1.0.2 (1.0.4 bugg?)
	Arduino IDE 1.8.19 : 1008074 76%, 47720 14% sur PC
	Arduino IDE 1.8.19 : 1007970 76%, 47720 14% sur raspi

  V2-12 30/01/2023 installé Canals
  1- Renvoie sur liste restreinte message provenant d'un numéro < 8 chiffres (N° Free)
  2- Comande vide log par SMS
  3- Print __FILE__ au démarrage
  4- Supprimer envoie SMS soimeme si pb majheure,
    remplacé par majheure par defaut 01/08/2022 08:00:00
  5- Ajouter Commande AT par SMS: SENDAT=cdeAT
  6- Efface sms en debut de traitement

  V2-11 05/02/2022 testé sur carte hard V2
  
  V2-1 03/05/2021 pas installé (testé sur carte V2)
  1 - extinction si Fin de journée (31/08/2021)
  2 - pas resolu!
      bug message etat, les lignes apres Batterie : OK, 100%, seulement si demande St alors que Allumé
      les lignes Allume:, jour Circule et Nbr Allumage sont doublées
      suppression ligne: message.reserve(140);
      changement valeur max temps Allumage ALLUME=X, max=12 heures

  V2-0 31/03/2021 installé sur carte V1 suite panne carte V2
  V2-0 03/03/2021 installé 11/03/2021
  1- carte hard V2 identique Cv
     entree E1 devient pedale 1
     entree E2 devient pedale 2
     entree VDR devient coffret, pullup par sortie PinLum
     PinLum Output, HIGH en permanence
  2- remaniement messages pour compatibilité interpretation serveur

  V1-5 14/09/2020 pas installé
  1- upload log en GPRS vers serveur ftp

  V1-4 24/07/2020 installé 22/08/2020
  Allumage entraine par couplage sur cable 350m
  declenchement Alarme Coffret et double detection pedale entree/sortie
  correction declenchement Alarme coffret, supprimer par interruption directe
  uniquement pas comptage dans boucle acquisition (apres 30s)
  apres declenchement pedale allumage, le declenchement pedale opposée inhibé pour 5s

  V1-3 02/06/2020 installé le 05/06/2020
  1- calendrier format json idem Signalisation
  2- remaniement demarrage et decision, calcul timetosleep (idem Signalisation)
     augmentation anticip pour compenser variation du reveil (45minutes 2700s) attendre DebutJour
  3- si commande ALLUME=1à9, force Allumage sans tenir compte des pedales,
     pour 1à9 heures jusqu'à ETEINDRE
  4- corection bug général, pour changer un Alarm.timerRepeat et Alarm.alarmRepeat,
     il faut utiliser la fonction Alarm.write(Id, durée)
  5- logrecord au lancement, Id en tete fichier log

  V1-2 Installé 03/12/2019

*/
#include <Arduino.h>
#include <Battpct.h>
#include <Sim800l.h>              //my SIM800 modifié
#include <Time.h>
#include <TimeAlarms.h>
#include <sys/time.h>             //<sys/time.h>
#include <WiFi.h>
#include <EEPROM.h>               // variable en EEPROM(SPIFFS)
#include <SPIFFS.h>
#include <ArduinoOTA.h>
#include <WiFiClient.h>
#include <WebServer.h>
#include <FS.h>
#include <SPI.h>
#include "passdata.h"
#include <ArduinoJson.h>
#include <credentials_tpcf.h>

String  webpage = "";
#define ServerVersion "1.0"
bool    SPIFFS_present = false;
#include "CSS.h"               // pageweb

// #define RESET_PIN     18   // declaré par Sim800l.h
// #define LED_PIN        5   // declaré par Sim800l.h
#define PinBattProc		35   // liaison interne carte Lolin32 adc
#define PinBattSol		39   // Batterie générale 12V adc VN
#define PinBattUSB		36   // V USB 5V adc VP 36, 25 ADC2 pas utilisable avec Wifi
#define Pin24V				26   // Mesure Tension 24V
#define PinPedale1		32   // Entrée Pedale1 Wake up EXT1
#define PinPedale2		33   // Entrée Pedale2 Wake up EXT1
#define PinCoffret 		34   // Entrée Porte Coffret Wake up EXT1
#define PinEclairage 	19   // Sortie Commande eclairage
#define PinSirene			15   // Sortie Commande Sirene (#0 en sleep 2.3V?)
#define RX_PIN				16   // TX Sim800
#define TX_PIN				17   // RX Sim800
#define PinReset			13   // Reset Hard
#define PinAlimLum    25   // Alimentation Pull up PinLum
#define PinTest       27   // Test sans GSM cc a la masse
#define SIMPIN				1234 // Code PIN carte SIM

#define uS_TO_S_FACTOR 1000000  /* Conversion factor for micro seconds to seconds */

#define nSample (1<<4)    // nSample est une puissance de 2, ici 16 (4bits)
unsigned int adc_hist[4][nSample]; // tableau stockage mesure adc, 0 Batt, 1 Proc, 2 USB, 3 24V
unsigned int adc_mm[4];            // stockage pour la moyenne mobile

uint64_t TIME_TO_SLEEP  = 15;/* Time ESP32 will go to sleep (in seconds) */
unsigned long debut     = 0; // pour decompteur temps wifi
unsigned long timer100  = 0; // pour timer 100ms adc
unsigned long timer1000 = 0; // pour timer 1s rearmement auto si pedale enffoncée en permanence
byte calendrier[13][32]; // tableau calendrier ligne 0 et jour 0 non utilisé, 12*31
char filecalendrier[13]  = "/filecal.csv";  // fichier en SPIFFS contenant le calendrier de circulation
char filecalibration[11] = "/coeff.txt";    // fichier en SPIFFS contenant les data de calibration
char filelog[9]          = "/log.txt";      // fichier en SPIFFS contenant le log

const String soft	= "ESP32_Tunnel.ino.d32"; // nom du soft
String	ver       = "V2-12";
int Magique       = 15;
const String Mois[13] = {"", "Janvier", "Fevrier", "Mars", "Avril", "Mai", "Juin", "Juillet", "Aout", "Septembre", "Octobre", "Novembre", "Decembre"};
String Sbidon 		= ""; // String texte temporaire
String message;
String bufferrcpt;
String fl = "\n";                   //  saut de ligne SMS
String Id ;                         //  Id du materiel sera lu dans EEPROM
char   SIM800InBuffer[64];          //  for notifications from the SIM800
char   replybuffer[255];            //  Buffer de reponse SIM800
volatile int IRQ_Cpt_PDL1  = 0;
volatile int IRQ_Cpt_PDL2  = 0;
volatile int IRQ_Cpt_Coffret = 0;
volatile unsigned long rebond1 = 0;		//	antirebond IRQ
volatile unsigned long rebond2 = 0;
volatile unsigned long rebond3 = 0;
byte DbounceTime = 20;				// antirebond
byte confign = 0;             // position enregistrement config EEPROM
byte recordn = 200;           // position enregistrement log EEPROM
bool Allume = false;
bool FlagPIR = false;
RTC_DATA_ATTR bool FlagAlarmeTension       = false; // Alarme tension Batterie
RTC_DATA_ATTR bool FlagLastAlarmeTension   = false;
RTC_DATA_ATTR bool FlagMasterOff           = false; // Coupure Allumage en cas de pb
RTC_DATA_ATTR bool FirstWakeup             = true;  // envoie premier message vie une seule fois
RTC_DATA_ATTR int  CptAllumage = 0; // Nombre Allumage par jour en memoire RTC
RTC_DATA_ATTR bool WupAlarme   = false; // declenchement alarme externe
RTC_DATA_ATTR bool flagCircule = false; // circule demandé -> inverse le calendrier, valable 1 seul jour
RTC_DATA_ATTR bool FileLogOnce = false; // true si log > seuil alerte

bool FlagAlarme24V           = false; // Alarme tension 24V Allumage
bool FlagLastAlarme24V       = false;
bool FlagAlarmeIntrusion     = false; // Alarme Defaut Cable detectée
bool FlagAlarmeCable1        = false; // Alarme Cable Pedale1
bool FlagAlarmeCable2        = false; // Alarme Cable Pedale2
bool FlagAlarmeCoffret       = false; // Alarme Porte Coffret
bool FlagLastAlarmeIntrusion = false;
bool FirstSonn = false;				// Premier appel sonnerie
bool SonnMax   = false;				// temps de sonnerie maxi atteint
bool FlagReset = false;       // Reset demandé
bool jour      = false;				// jour = true, nuit = false
bool gsm       = true;        // carte GSM presente utilisé pour test sans GSM seulement
int  Nmax      = 0;						// comptage alarme cable avant alarme different Jour/Nuit

int CoeffTension[4];          // Coeff calibration Tension
int CoeffTensionDefaut = 7000;// Coefficient par defaut

bool LastWupAlarme = false;   // memo etat Alarme par Wakeup

int    slot = 0;              //this will be the slot number of the SMS

long   TensionBatterie  = 0; // Tension Batterie solaire
long   VBatterieProc    = 0; // Tension Batterie Processeur
long   VUSB             = 0; // Tension USB
long   Tension24        = 0; // Tension 24V Allumage

WebServer server(80);
File UploadFile;

typedef struct										// declaration structure  pour les log
{
  char 		dt[10];									//	DateTime 0610-1702 9+1
  char 		Act[2];									//	Action A/D/S/s 1+1
  char 		Name[15];								//	14 car
} champ;
champ record[5];

struct  config_t 								// Structure configuration sauvée en EEPROM
{
  int     anticip     ;         // temps anticipation du reveille au lancement s
  int 		magic				;					// num magique
  long    DebutJour 	;					// Heure message Vie, 7h matin en seconde = 7*60*60
  long    FinJour 		;					// Heure fin jour, 20h matin en seconde = 20*60*60
  long    RepeatWakeUp; 				// Periodicité WakeUp Jour non circulé
  int     Tanalyse    ;         // tempo analyse alarme sur interruption
  int			tempoSortie ;					// tempo eclairage apres sorties(s)
  int			timeOutS	 	;					// tempo time out eclairage (s)
  int			timeoutWifi ;					// tempo coupure Wifi si pas de mise a jour (s)
  int 		Dsonn 	;							// Durée Sonnerie
  int 		DsonnMax;							// Durée Max Sonnerie
  int 		Dsonnrepos;						// Durée repos Sonnerie
  bool    Intru   ;							// Alarme Intrusion active
  bool    Silence ;							// Mode Silencieux = true false par defaut
  bool    Pedale1;              // Alarme Pedale1 Active
  bool    Pedale2;              // Alarme Pedale2 Active
  bool    Coffret;              // Alarme Porte Coffret Active
  long    Jour_Nmax;            // Comptage Alarme Jour
  long    Nuit_Nmax;            // Comptage Alarme Nuit
  bool    Pos_Pn_PB[10];        // numero du Phone Book (1-9) à qui envoyer 0/1 0 par defaut
  char    Idchar[11];           // Id
  char    apn[11];              // APN
  char    gprsUser[11];         // user for APN
  char    gprsPass[11];         // pass for APN
  char    ftpServeur[26];       // serveur ftp
  char    ftpUser[8];           // user ftp
  char    ftpPass[16];          // pwd ftp
  int     ftpPort;              // port ftp
} ;
config_t config;


AlarmId loopPrincipale;	// boucle principale
AlarmId TempoAnalyse;		// tempo analyse alarme suite Interruption
AlarmId TempoSortie;		// Temporisation eclairage a la sortie
AlarmId TimeOut;				// TimeOut Allumage
AlarmId DebutJour;      // Debut journée
AlarmId FinJour;				// Fin de journée retour deep sleep
AlarmId TSonn;					// 4 tempo durée de la sonnerie
AlarmId TSonnMax;				// 5 tempo maximum de sonnerie
AlarmId TSonnRepos;			// 6 tempo repos apres maxi

portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;

HardwareSerial *SIM800Serial = &Serial2;// liaison serie FONA SIM800
Sim800l Sim800;  											// to declare the library

//---------------------------------------------------------------------------
void IRAM_ATTR handleInterruptP1() { // Pedale 1

  portENTER_CRITICAL_ISR(&mux);
  if (xTaskGetTickCount() - rebond1 > DbounceTime) {
    IRQ_Cpt_PDL1++;
    rebond1 = xTaskGetTickCount(); // equiv millis()
  }
  portEXIT_CRITICAL_ISR(&mux);

}
void IRAM_ATTR handleInterruptP2() { // Pedale 2

  portENTER_CRITICAL_ISR(&mux);
  if (xTaskGetTickCount() - rebond2 > DbounceTime) {
    IRQ_Cpt_PDL2++;
    rebond2 = xTaskGetTickCount(); // equiv millis()
  }
  portEXIT_CRITICAL_ISR(&mux);

}
void IRAM_ATTR handleInterruptPo() { // Coffret
  portENTER_CRITICAL_ISR(&mux);
  if (xTaskGetTickCount() - rebond3 > DbounceTime) {
    IRQ_Cpt_Coffret++;
    rebond3 = xTaskGetTickCount(); // equiv millis()
  }
  portEXIT_CRITICAL_ISR(&mux);
}
//---------------------------------------------------------------------------

void setup() {
  // message.reserve(140);

  Serial.begin(115200);
  Serial.println();
  Serial.println(__FILE__);
  if(gsm){
    Serial.println(F("lancement SIM800"));
    SIM800Serial->begin(9600); // 4800
    Sim800.begin();
  }

  pinMode(PinEclairage, OUTPUT);
  pinMode(PinPedale1  , INPUT_PULLUP);
  pinMode(PinPedale2  , INPUT_PULLUP);
  pinMode(PinCoffret  , INPUT); // no sw pull up
  pinMode(PinSirene   , OUTPUT);
  pinMode(PinAlimLum  , OUTPUT);
  pinMode(PinTest     , INPUT_PULLUP);
  digitalWrite(PinEclairage, LOW);
  digitalWrite(PinSirene, LOW);
  digitalWrite(PinAlimLum , HIGH);
  adcAttachPin(PinBattProc);
  adcAttachPin(PinBattSol);
  adcAttachPin(PinBattUSB);
  adcAttachPin(Pin24V);

  if (digitalRead(PinTest) == 0) { // lire strap test, si = 0 test sans carte gsm
    gsm = false;
    setTime(12, 00, 00, 15, 07, 2019); // il faut initialiser la date et heure, jour circule et midi
    Serial.println("Lancement test sans carte gsm");
    Serial.println("mise à l'heure 14/07/2019 12:00:00");
    Serial.println("retirer le cavalier Pin27 et reset");
    Serial.println("pour redemarrer normalement");
  }
  init_adc_mm();// initialisation tableau pour adc Moyenne Mobile

  /* Lecture configuration en EEPROM	 */
  EEPROM.begin(512);

  EEPROM.get(confign, config); // lecture config
  recordn = sizeof(config);
  Serial.print("len config ="), Serial.println(sizeof(config));
  EEPROM.get(recordn, record); // Lecture des log
  Alarm.delay(500);
  if (config.magic != Magique) {
    /* verification numero magique si different
    		erreur lecture EEPROM ou carte vierge
    		on charge les valeurs par défaut
    */
    Serial.println(F("Nouvelle Configuration !"));
    config.magic         = Magique;
    config.anticip       = 2700;
    config.DebutJour     = 8 * 60 * 60;
    config.FinJour       = 19 * 60 * 60;
    config.RepeatWakeUp  = 60 * 60;
    config.Tanalyse      = 10 * 60;
    config.Intru         = true;
    config.Silence       = true;
    config.Dsonn         = 60;
    config.DsonnMax      = 90;
    config.Dsonnrepos    = 120;
    config.tempoSortie   = 10;
    config.timeOutS      = 3600;// 3600
    config.timeoutWifi   = 10 * 60;
    config.Pedale1       = false;
    config.Pedale2       = false;
    config.Coffret       = true;
    config.Jour_Nmax     = 3 * 60 / 10; // 3mn /10 temps de boucle Acquisition
    config.Nuit_Nmax     = 30 / 10; // 30s /10 temps de boucle Acquisition
    for (int i = 0; i < 10; i++) {// initialise liste PhoneBook liste restreinte
      config.Pos_Pn_PB[i] = 0;
    }
    // config.Pos_Pn_PB[1]  = 1;	// le premier numero du PB par defaut
    String temp          = "TPCF_Canal";// TPCF_TCnls
    temp.toCharArray(config.Idchar, 11);
    String tempapn       = "free";//"sl2sfr"
    String tempGprsUser  = "";
    String tempGprsPass  = "";
    config.ftpPort       = tempftpPort;
    tempapn.toCharArray(config.apn, (tempapn.length() + 1));
    tempGprsUser.toCharArray(config.gprsUser,(tempGprsUser.length() + 1));
    tempGprsPass.toCharArray(config.gprsPass,(tempGprsPass.length() + 1));
    tempServer.toCharArray(config.ftpServeur,(tempServer.length() + 1));
    tempftpUser.toCharArray(config.ftpUser,(tempftpUser.length() + 1));
    tempftpPass.toCharArray(config.ftpPass,(tempftpPass.length() + 1));

    EEPROM.put(confign, config);
    EEPROM.commit();
    // valeur par defaut des record (log)
    for (int i = 0; i < 5 ; i++) {
      temp = "";
      temp.toCharArray(record[i].dt, 10);
      temp.toCharArray(record[i].Act, 2);
      temp.toCharArray(record[i].Name, 15);
    }
    EEPROM.put(recordn, record);// ecriture des valeurs par defaut
    EEPROM.commit();
  }
  EEPROM.end();
  PrintEEPROM();
  Id  = String(config.Idchar);
  Id += fl;

  // Port defaults to 3232
  // ArduinoOTA.setPort(3232);
  ArduinoOTA.setHostname(config.Idchar);
  ArduinoOTA.setPasswordHash(OTApwdhash);
  ArduinoOTA
  .onStart([]() {
    String type;
    if (ArduinoOTA.getCommand() == U_FLASH)
      type = "sketch";
    else // U_SPIFFS
      type = "filesystem";

    // NOTE: if updating SPIFFS this would be the place to unmount SPIFFS using SPIFFS.end()
    Serial.print(F("Start updating "));
    Serial.println(type);
  })
  .onEnd([]() {
    Serial.println(F("End"));
    delay(100);
    ResetHard();
  })
  .onProgress([](unsigned int progress, unsigned int total) {
    Serial.printf("Progress: %u%%\r", (progress / (total / 100)));
  })
  .onError([](ota_error_t error) {
    Serial.printf("Error[%u]: ", error);
    if 			(error == OTA_AUTH_ERROR) 		Serial.println(F("Auth Failed"));
    else if (error == OTA_BEGIN_ERROR) 		Serial.println(F("Begin Failed"));
    else if (error == OTA_CONNECT_ERROR) 	Serial.println(F("Connect Failed"));
    else if (error == OTA_RECEIVE_ERROR) 	Serial.println(F("Receive Failed"));
    else if (error == OTA_END_ERROR) 			Serial.println(F("End Failed"));
  });

  if (!SPIFFS.begin(true)) {
    Serial.println(F("SPIFFS initialisation failed..."));
    SPIFFS_present = false;
  }
  else {
    Serial.println(F("SPIFFS initialised... file access enabled..."));
    SPIFFS_present = true;
  }

  OuvrirCalendrier();					// ouvre calendrier circulation en SPIFFS
  OuvrirFichierCalibration(); // ouvre fichier calibration en SPIFFS
  // Serial.print(F("temps =")),Serial.println(millis());
  if (gsm) {
    Sim800.reset(SIMPIN);					// lancer SIM800
    MajHeure("");
  }

  loopPrincipale = Alarm.timerRepeat(10, Acquisition); // boucle principale 10s
  Alarm.enable(loopPrincipale);

  TempoAnalyse = Alarm.timerRepeat(config.Tanalyse, FinAnalyse); // Tempo Analyse Alarme sur interruption
  Alarm.disable(TempoAnalyse);

  TempoSortie = Alarm.timerRepeat(config.tempoSortie, Extinction); // tempo extinction a la sortie
  Alarm.disable(TempoSortie);

  TimeOut = Alarm.timerRepeat(config.timeOutS, Extinction); // tempo time out extinction
  Alarm.disable(TimeOut);

  DebutJour = Alarm.alarmRepeat(config.DebutJour,SignalVie);
  FinJour = Alarm.alarmRepeat(config.FinJour, FinJournee); // Fin de journée retour deep sleep
  Alarm.enable(FinJour);

  TSonn = Alarm.timerRepeat(config.Dsonn, ArretSonnerie);	// tempo durée de la sonnerie
  Alarm.disable(TSonn);

  TSonnMax = Alarm.timerRepeat(config.DsonnMax, SonnerieMax); // tempo maximum de sonnerie
  Alarm.disable(TSonnMax);

  TSonnRepos = Alarm.timerRepeat(config.Dsonnrepos, ResetSonnerie); // tempo repos apres maxi
  Alarm.disable(TSonnRepos);

  ActiveInterrupt();

  Serial.print(F("flag Circule :")), Serial.println(flagCircule);

  if (get_wakeup_reason() == PinCoffret && config.Intru) { // Alarme Coffret
    FlagAlarmeCoffret = true;
    FlagAlarmeIntrusion = true;
    FlagPIR = true;
  }
  MajLog("Auto","Lancement");

}
//---------------------------------------------------------------------------
void loop() {
  recvOneChar();

  if (rebond1 > millis()) rebond1 = millis();
  if (rebond2 > millis()) rebond2 = millis();
  if (rebond3 > millis()) rebond3 = millis();

  char* bufPtr = SIM800InBuffer;	//buffer pointer
  if (Serial2.available()) {      	//any data available from the FONA?
    int charCount = 0;
    /* Read the notification into SIM800InBuffer */
    do  {
      *bufPtr = Serial2.read();
      bufferrcpt += *bufPtr;
      Serial.write(*bufPtr);
      // Alarm.delay(1);
      delay(1);
    } while ((*bufPtr++ != '\n') && (Serial2.available()) && (++charCount < (sizeof(SIM800InBuffer) - 1)));
    /* Add a terminal NULL to the notification string */
    *bufPtr = 0;
    if (charCount > 1) {
      // Serial.print(F("Buffer ="));
      Serial.println(bufferrcpt);
    }
    if ((bufferrcpt.indexOf(F("RING"))) > -1) {	// RING, Ca sonne
      Sim800.hangoffCall();									// on raccroche
    }
    /* Scan the notification string for an SMS received notification.
      If it's an SMS message, we'll get the slot number in 'slot' */
    if (1 == sscanf(SIM800InBuffer, "+CMTI: \"SM\",%d", &slot)) {
      traite_sms(slot);
    }
  }

  if (IRQ_Cpt_Coffret > 0) { // Alarme Coffret, pas utilisé ici
    portENTER_CRITICAL(&mux);
    IRQ_Cpt_Coffret = 0;
    portEXIT_CRITICAL(&mux);
    // if (config.Intru && config.Coffret) {
      // if (BattPBpct(TensionBatterie, 6) > 20) {
        // FlagAlarmeCoffret = true;
        // FlagAlarmeIntrusion = true;
        // FlagPIR = true;
        // Acquisition();
      // }
    // }
  }

  if (IRQ_Cpt_PDL1 > 0 || IRQ_Cpt_PDL2 > 0) {
    Serial.print(F("Interruption : "));
    Serial.print(IRQ_Cpt_PDL1);
    Serial.print(" ");
    Serial.println(IRQ_Cpt_PDL2);
  }
  if (IRQ_Cpt_PDL1 > 0) {
    Allumage(1);
    portENTER_CRITICAL(&mux);
    IRQ_Cpt_PDL1 = 0;
    portEXIT_CRITICAL(&mux);
  }
  if (IRQ_Cpt_PDL2 > 0) {
    Allumage(2);
    portENTER_CRITICAL(&mux);
    IRQ_Cpt_PDL2 = 0;
    portEXIT_CRITICAL(&mux);
  }

  ArduinoOTA.handle();
  Alarm.delay(1);

  if (millis() - timer100 > 100) { // toute les 100ms
    timer100 = millis();
    read_adc(PinBattSol, PinBattProc, PinBattUSB, Pin24V); // lecture des adc
  }

  if (millis() - timer1000 > 1000) { // toute les 1s rearmement tempo sortie si pedale enfoncée tout le temps du passage
    timer1000 = millis();
    if (digitalRead(PinPedale1))Allumage(1);
    if (digitalRead(PinPedale2))Allumage(2);
  }

}	//fin loop
//---------------------------------------------------------------------------
void Acquisition() {
  static int8_t nsms;
  static int cpt = 0; // compte le nombre de passage boucle
  static bool firstdecision = false;

  AIntru_HeureActuelle();

  if (cpt > 6 && nsms == 0 && !firstdecision) {
    /* une seule fois au demarrage attendre au moins 70s et plus de sms en attente */
    action_wakeup_reason(get_wakeup_reason());
    firstdecision = true;
  }
  cpt ++;

  if (LastWupAlarme != WupAlarme && nsms == 0) { // fin de la tempo analyse retour sleep
    LastWupAlarme = false;
    WupAlarme     = false;
    Serial.println(F("Fin TempoAnalyse"));
    envoieGroupeSMS(0, 1);				// envoie groupé Etat avec fin analyse
    calculTimeSleep();
    DebutSleep();
  }

  if (CoeffTension[0] == 0 || CoeffTension[1] == 0 || CoeffTension[2] == 0 || CoeffTension[3] == 0) {
    OuvrirFichierCalibration(); // patch relecture des coeff perdu
  }

  if (gsm) {
    if (!Sim800.getetatSIM())Sim800.reset(SIMPIN); // verification SIM
  }
  Serial.print(displayTime(0));
  Serial.print(F(" Freemem = ")), Serial.println(ESP.getFreeHeap());
  static byte nalaTension = 0;
  static byte nRetourTension = 0;
  static byte nalaAllume = 0; // compte le nombre alarme Allume
  static byte nRetourAllume = 0;
  TensionBatterie = map(adc_mm[0] / nSample, 0, 4095, 0, CoeffTension[0]);
  VBatterieProc   = map(adc_mm[1] / nSample, 0, 4095, 0, CoeffTension[1]);
  VUSB            = map(adc_mm[2] / nSample, 0, 4095, 0, CoeffTension[2]);
  Tension24       = map(adc_mm[3] / nSample, 0, 4095, 0, CoeffTension[3]);

  // Serial.print(adc_mm[0]),Serial.print(";");
  // Serial.print(adc_mm[1]),Serial.print(";");
  // Serial.println(adc_mm[2]);

  if (Allume) {
    Serial.print(F("Tension 24V :")), Serial.print(float(Tension24 / 100.0)), Serial.print(F(" "));
    Serial.print("coeff 24V="), Serial.println(CoeffTension[3]);
    if (Tension24 < 1800) { // on attend 5 passages pour mesurer 24V
      nalaAllume ++;
      if(nalaAllume > 3){
        FlagAlarme24V = true;
        nalaAllume = 0;
      }
    }
    else if (Tension24 > 2000){
      nRetourAllume ++;
      if(nRetourAllume > 3){
        nRetourAllume = 0;
        nalaAllume = 0;
        FlagAlarme24V = false;
      }
    }
  }

  if (BattPBpct(TensionBatterie, 6) < 25 || VUSB < 4000) { // || VUSB > 6000
    nalaTension ++;
    if (nalaTension > 3) {
      FlagAlarmeTension = true;
      nalaTension = 0;
    }
  }
  else if (BattPBpct(TensionBatterie, 6) > 80 && VUSB > 4800) { //  && VUSB < 5400	//hysteresis et tempo sur Alarme Batterie
    nRetourTension ++;
    if (nRetourTension > 3) {
      FlagAlarmeTension = false;
      nRetourTension = 0;
      nalaTension = 0;
    }
  }
  else {
    if (nalaTension > 0)nalaTension--;		//	efface progressivement le compteur
  }

  String texte;
  texte = F("Batt Solaire = ");
  texte += float(TensionBatterie / 100.0);
  texte += "V ";
  texte += String(BattPBpct(TensionBatterie, 6));
  texte += "%";
  texte += F(", Batt Proc = ");
  texte += (String(VBatterieProc) + "mV ");
  texte += String(BattLipopct(VBatterieProc));
  texte += (F("%, V USB = "));
  texte += (float(VUSB / 1000.0));
  texte += ("V");
  texte += fl;
  Serial.print(texte);

  static byte nalaPIR1 = 0;
  static byte nalaPIR2 = 0;
  static byte nalaCoffret = 0;
  if (config.Intru) {
    // gestion des capteurs coupé ou en alarme permanente
    // verif sur plusieurs passages consecutifs
    if (digitalRead(PinCoffret) && config.Coffret) {
      nalaCoffret ++;
      if (nalaCoffret > 3) {
        FlagAlarmeCoffret = true;
        FlagPIR = true;
        nalaCoffret = 0;
      }
    }
    else {
      if (nalaCoffret > 0) nalaCoffret --;		//	efface progressivement le compteur
    }

    if (digitalRead(PinPedale1) && config.Pedale1) {
      nalaPIR1 ++;
      if (nalaPIR1 > Nmax) {
        FlagAlarmeCable1 = true;
        FlagPIR = true;
        nalaPIR1 = 0;
      }
    }
    else {
      if (nalaPIR1 > 0) nalaPIR1 --;		//	efface progressivement le compteur
    }

    if (digitalRead(PinPedale2) && config.Pedale2) {
      nalaPIR2 ++;
      if (nalaPIR2 > Nmax) {
        FlagAlarmeCable2 = true;
        FlagPIR = true;
        nalaPIR2 = 0;
      }
    }
    else {
      if (nalaPIR2 > 0) nalaPIR2 --;		//	efface progressivement le compteur
    }

    if (FlagPIR) {
      FlagAlarmeIntrusion = true;
      FlagPIR = false;
      ActivationSonnerie();		// activation Sonnerie
      if (FlagAlarmeCoffret) {
        Serial.println(F("Alarme Coffret"));
      }
      else if (FlagAlarmeCable1 || FlagAlarmeCable2) {
        Serial.println(F("Alarme Cable"));
      }
    }
  }
  else {
    FlagPIR = false;
    FlagAlarmeIntrusion = false; // efface alarme
    FlagAlarmeCable1 = false;
    FlagAlarmeCable2 = false;
    FlagAlarmeCoffret = false;
  }
  Serial.printf("Nala Coffret = %d ,", nalaCoffret);
  Serial.printf("Nala Ped 1 = %d ,", nalaPIR1);
  Serial.printf("Nala Ped 2 = %d\n", nalaPIR2);

  if (gsm) {
    /* verification nombre SMS en attente(raté en lecture directe)
       traitement des sms en memeoire un par un,
       pas de traitement en serie par commande 51, traitement beaucoup trop long */
    nsms = Sim800.getNumSms(); // nombre de SMS en attente (1s)
    Serial.print(F("Sms en attente = ")), Serial.println(nsms);

    if (nsms > 0) {	// nombre de SMS en attente
      // il faut les traiter
      int numsms = Sim800.getIndexSms(); // cherche l'index des sms en mémoire
      Serial.print(F("Numero Sms en attente = ")), Serial.println(numsms);
      traite_sms(numsms);// traitement des SMS en attente
    }
    else if (nsms == 0 && FlagReset) { // on verifie que tous les SMS sont traités avant Reset
      FlagReset = false;
      ResetHard();				//	reset hard
    }
  }
  else if (FlagReset) {
    FlagReset = false;
    ResetHard();				//	reset hard
  }

  envoie_alarme();

  digitalWrite(LED_PIN, 0);
  Alarm.delay(50);
  digitalWrite(LED_PIN, 1);

  Serial.println();
}
//---------------------------------------------------------------------------
void traite_sms(byte slot) {
  // Alarm.disable(loopPrincipale);
  /* il y a 50 slots dispo
  	si slot=51, demande de balayer tous les slots pour verification (pas utilisé trop long)
  	si slot=99, demande depuis liaison serie en test, traiter sans envoyer de sms
  */

  char number[13];													// numero expediteur SMS
  String textesms;													// texte du SMS reçu
  textesms.reserve(140);
  String numero;
  String nom;
  bool smsserveur = false; // true si le sms provient du serveur index=1

  byte i;
  byte j;
  bool sms = true;

  /* Variables pour mode calibration */
  static int tensionmemo = 0;//	memorisation tension batterie lors de la calibration
  int coef = 0; // coeff temporaire
  static byte P = 0; // Pin entrée a utiliser pour calibration
  static byte M = 0; // Mode calibration 1,2,3,4
  static bool FlagCalibration = false;	// Calibration Tension en cours

  if (slot == 99) sms = false;
  if (slot == 51) { // demande de traitement des SMS en attente
    i = 1;
    j = 50;
  }
  else {
    i = slot;
    j = slot;
  }
  for (byte k = i; k <= j; k++) {
    slot = k;
    // /* Retrieve SMS sender address/phone number. */
    if (sms) {
      numero = Sim800.getNumberSms(slot); // recupere le Numero appelant
      nom = Sim800.getNameSms(slot);      // recupere le nom appelant
      textesms = Sim800.readSms(slot);    // recupere le contenu
      textesms = ExtraireSms(textesms);
      if (Sim800.getNumberSms(slot) == Sim800.getPhoneBookNumber(1)) {
        smsserveur = true; // si sms provient du serveur index=1
      }
      if (nom.length() < 1) { // si nom vide, cherche si numero est num de la SIM
        if (numero == Sim800.getNumTel()) {
          nom = F("Moi meme");
        }
      }
      Serial.print(F("Nom appelant = ")), Serial.println(nom);
      Serial.print(F("Numero = ")), Serial.println(numero);
      byte n = Sim800.ListPhoneBook(); // nombre de ligne PhoneBook
      if(numero.length() < 8){ // numero service free
        for (byte Index = 1; Index < n + 1; Index++) { // Balayage des Num Tel dans Phone Book
          if (config.Pos_Pn_PB[Index] == 1) { // Num dans liste restreinte
            String number = Sim800.getPhoneBookNumber(Index);
            char num[13];
            number.toCharArray(num, 13);
            message = textesms;
            EnvoyerSms(num, true);
            EffaceSMS(slot);
            return; // sortir de la procedure traite_sms
          }
        }
      }
    }
    else {
      textesms = String(replybuffer);
      nom = "console";
    }

    if (sms && !(textesms.indexOf(F("MAJHEURE")) == 0)) { // suppression du SMS sauf si MAJHEURE
      EffaceSMS(slot);
    }

    if (!(textesms.indexOf(F("TEL")) == 0 || textesms.indexOf(F("tel")) == 0 || textesms.indexOf(F("Tel")) == 0
        || textesms.indexOf(F("Wifi")) == 0 || textesms.indexOf(F("WIFI")) == 0 || textesms.indexOf(F("wifi")) == 0
        || textesms.indexOf(F("GPRSDATA")) > -1 || textesms.indexOf(F("FTPDATA")) > -1 || textesms.indexOf(F("FTPSERVEUR")) > -1)) {
      textesms.toUpperCase();	// passe tout en Maj sauf si "TEL" ou "WIFI" etc parametres pouvant contenir minuscules
      // textesms.trim();
    }
    textesms.replace(" ", "");// supp tous les espaces
    if (textesms.indexOf("A") == 0 || textesms.indexOf("D") == 0) {
      String temp = Id.substring(6, 10);
      temp.toUpperCase();
      if (textesms.indexOf("A" + temp) == 0) textesms = F("INTRUON");
      if (textesms.indexOf("D" + temp) == 0) textesms = F("INTRUOFF");
    }
    Serial.print(F("textesms  = ")), Serial.println(textesms);

    if ((sms && nom.length() > 0) || !sms) {        // si nom appelant existant dans phone book
      numero.toCharArray(number, numero.length() + 1); // on recupere le numéro
      messageId();
      if (textesms.indexOf(F("TIMEOUTWIFI")) > -1) { // Parametre Arret Wifi
        if (textesms.indexOf(char(61)) == 11) {
          int n = textesms.substring(12, textesms.length()).toInt();
          if (n > 9 && n < 3601) {
            config.timeoutWifi = n;
            sauvConfig();														// sauvegarde en EEPROM
          }
        }
        message += F("TimeOut Wifi (s) = ");
        message += config.timeoutWifi;
        message += fl;
      }
      if (textesms.indexOf(F("WIFIOFF")) > -1) { // Arret Wifi
        message += F("Wifi off");
        message += fl;
        EnvoyerSms(number, sms);
        WifiOff();
      }
      else if (textesms.indexOf(F("Wifi")) == 0) { // demande connexion Wifi
        byte pos1 = textesms.indexOf(char(44));//","
        byte pos2 = textesms.indexOf(char(44), pos1 + 1);
        String ssids = textesms.substring(pos1 + 1, pos2);
        String pwds  = textesms.substring(pos2 + 1, textesms.length());
        char ssid[20];
        char pwd[20];
        ssids.toCharArray(ssid, ssids.length() + 1);
        pwds.toCharArray(pwd, pwds.length() + 1);
        ConnexionWifi(ssid, pwd, number, sms); // message généré par routine
      }
      else if (gsm && textesms.indexOf(F("TEL")) == 0
               || textesms.indexOf(F("Tel")) == 0
               || textesms.indexOf(F("tel")) == 0) { // entrer nouveau num
        bool FlagOK = true;
        byte j = 0;
        String Send	= "AT+CPBW=";// message ecriture dans le phone book
        if (textesms.indexOf(char(61)) == 4) { // TELn= reserver correction/suppression
          int i = textesms.substring(3).toInt();// recupere n° de ligne
          i = i / 1; // important sinon i ne prend pas sa valeur dans les comparaison?
          //Serial.println(i);
          if (i < 1) FlagOK = false;
          Send += i;
          j = 5;
          // on efface la ligne sauf la 1 pour toujours garder au moins un numéro
          if ( (i != 1) && (textesms.indexOf(F("efface")) == 5
                        ||  textesms.indexOf(F("EFFACE")) == 5 )) goto fin_tel;
        }
        else if (textesms.indexOf(char(61)) == 3) { // TEL= nouveau numero
          j = 4;
        }
        else {
          FlagOK = false;
        }
        if (textesms.indexOf("+") == j) {			// debut du num tel +
          if (textesms.indexOf(char(44)) == j + 12) {	// verif si longuer ok
            String numero = textesms.substring(j, j + 12);
            String nom = textesms.substring(j + 13, j + 27);// pas de verif si long<>0?
            Send += F(",\"");
            Send += numero;
            Send += F("\",145,\"");
            Send += nom;
            Send += F("\"");
          }
          else {
            FlagOK = false;
          }
        }
        else {
          FlagOK = false;
        }
fin_tel:
        if (!FlagOK) { // erreur de format
          //Serial.println(F("false"));
          messageId();
          message += F("Commande non reconnue ?");// non reconnu
          message += fl;
          EnvoyerSms(number, sms);					// SMS non reconnu
        }
        else {
          Serial.println(Send);
          if (gsm) {
            Sim800.WritePhoneBook(Send);					//ecriture dans PhoneBook
            Alarm.delay(500);
            Sim800.ModeText(); //pour purger buffer fona
            Alarm.delay(500);
          }
          messageId();
          message += F("Nouveau Num Tel: ");
          message += F("OK");
          message += fl;
          EnvoyerSms(number, sms);
        }
      }
      else if ((gsm && textesms == "LST?") || (textesms == "LST1")) {	//	Liste des Num Tel
        byte n = Sim800.ListPhoneBook(); // nombre de ligne PhoneBook
        for (byte i = 1; i < n + 1; i++) {
          String num = Sim800.getPhoneBookNumber(i);
          // Serial.print(num.length()), Serial.print(" "), Serial.println(num);
          if (num.indexOf("+CPBR:") == 0) { // si existe pas sortir
            Serial.println(F("Failed!"));// next i
            goto fin_i;
          }
          String name = Sim800.getPhoneBookName(i);
          // Serial.println(name);
          message += String(i) + ":";
          message += num;
          message += "," + fl;
          message += name;
          message += fl;
          Serial.println(message);
          if ((i % 3) == 0) {
            EnvoyerSms(number, sms);
            messageId();
          }
        }
fin_i:
        if (message.length() > Id.length() + 20) EnvoyerSms(number, sms);; // SMS final
      }
      else if (textesms.indexOf("ETAT") == 0 || textesms.indexOf("ST") == 0) {// "ETAT? de l'installation"
        generationMessage(0);
        EnvoyerSms(number, sms);
      }
      else if (textesms.indexOf(F("SYS")) > -1) {
        if (gsm) {
          Sim800.getetatSIM();// 1s
          byte n = Sim800.getNetworkStatus();// 1.1s
          String Op = Sim800.getNetworkName();// 1.05s
          if (n == 5) {
            message += F(("rmg, "));// roaming 1.0s
          }
          message += Op + fl;
          read_RSSI();
          int Vbat = Sim800.BattVoltage();
          byte Batp = Sim800.BattPct();
          message += F("Batt GSM : ");
          message += Vbat;
          message += F(" mV, ");
          message += Batp;
          message += F(" %");
          message += fl;
        }
        message += F("Param Sonn = ");
        message += config.Dsonn;
        message += ":";
        message += config.DsonnMax;
        message += ":";
        message += config.Dsonnrepos;
        message += "(s)";
        message += fl;
        message += F("freeRAM=");
        message += String(ESP.getFreeHeap()) + fl ;
        message += F("Ver: ");
        message += ver;
        message += fl;
        message += F("V Batt Sol= ");
        message += String(float(TensionBatterie / 100.0));
        message += F("V, ");
        message += String(BattPBpct(TensionBatterie, 6));
        message += " %";
        message += fl;
        message += F("V USB= ");
        message += (float(VUSB / 1000.0));
        message += "V";
        message += fl;
        EnvoyerSms(number, sms);
      }
      else if (textesms.indexOf(F("ID=")) == 0) {			//	Id= nouvel Id
        String temp = textesms.substring(3);
        if (temp.length() > 0 && temp.length() < 11) {
          Id = "";
          temp.toCharArray(config.Idchar, 11);
          sauvConfig();														// sauvegarde en EEPROM
          Id = String(config.Idchar);
          Id += fl;
        }
        messageId();
        message += F("Nouvel Id");
        message += fl;
        EnvoyerSms(number, sms);
      }
      else if (textesms.indexOf(F("LOG")) == 0) {	// demande log des 5 derniers commandes
        File f = SPIFFS.open(filelog, "r");
        message = F("local log size :");
        message += String(f.size()) + fl;
        f.close();
        for (int i = 0; i < 5; i++) {
          message += String(record[i].dt) + "," + String(record[i].Act) + "," + String(record[i].Name) + fl;
        }
        //Serial.println( message);
        EnvoyerSms(number, sms);
      }
      else if (textesms.indexOf(F("INTRUON")) == 0) {	//	Armement Alarme
        // conserver INTRUON en depannage si ID non conforme
        if (!config.Intru) {
          config.Intru = !config.Intru;
          sauvConfig();											// sauvegarde en EEPROM
          ActiveInterrupt();
          if (!sms) {
            nom = F("console");
          }
          logRecord(nom, "A"); // renseigne le log
          MajLog(nom, "A");
        }
        generationMessage(0);
        EnvoyerSms(number, sms);
      }
      else if (textesms.indexOf(F("INTRUOFF")) == 0) { //	Desarmement
        if (config.Intru) {
          config.Intru = !config.Intru;
          sauvConfig();														// sauvegarde en EEPROM
          DesActiveInterrupt();
          /*	Arret Sonnerie au cas ou? sans envoyer SMS */
          digitalWrite(PinSirene, LOW);	// Arret Sonnerie
          Alarm.disable(TSonn);			// on arrete la tempo sonnerie
          Alarm.disable(TSonnMax);	// on arrete la tempo sonnerie maxi
          FirstSonn = false;
          FlagAlarmeIntrusion = false;
          FlagAlarmeCable1 = false;
          FlagAlarmeCable2 = false;
          FlagAlarmeCoffret = false;
          FlagPIR = false;
          if (!sms) {
            nom = F("console");
          }
          logRecord(nom, "D");				// renseigne le log
          MajLog(nom, "D");
        }
        generationMessage(0);
        EnvoyerSms(number, sms);
      }
      else if (textesms.indexOf(F("SILENCE")) == 0 ) {		//	Alarme Silencieuse
        if (textesms.indexOf(F("ON")) == 7) { //ON
          if (!config.Silence) {
            config.Silence = !config.Silence;
            digitalWrite(PinSirene, LOW);// Arret Sonnerie
            sauvConfig();														// sauvegarde en EEPROM
          }
        }
        if (textesms.indexOf(F("OFF")) == 7) {
          if (config.Silence) {
            config.Silence = !config.Silence;
            sauvConfig();														// sauvegarde en EEPROM
          }
        }
        generationMessage(0);
        EnvoyerSms(number, sms);
      }
      else if (textesms.indexOf(F("ANTICIP")) > -1) { // Anticipation du wakeup
        if (textesms.indexOf(char(61)) == 7) {
          int n = textesms.substring(8, textesms.length()).toInt();
          if (n > 9 && n < 3601) {
            config.anticip = n;
            sauvConfig();														// sauvegarde en EEPROM
          }
        }
        message += F("Anticipation WakeUp (s) = ");
        message += config.anticip;
        message += fl;
        EnvoyerSms(number, sms);
      }
      else if (textesms.indexOf(F("PARAM")) == 0) { // parametre Jour/Nuit
        if (textesms.indexOf(char(61)) == 5) { //char(61) "="
          Sbidon = textesms.substring(6, textesms.length());
          byte pos = Sbidon.indexOf(char(44)); // char(44) ","
          Serial.print(Sbidon), Serial.print(":"), Serial.println(pos);
          int val = Sbidon.substring(0, pos).toInt();
          Serial.println(val);
          if (val > 9 && val < 600) {
            int val2 = Sbidon.substring(pos + 1, Sbidon.length()).toInt();
            if (val2 > 9 && val2 < 600) {
              config.Jour_Nmax = val / 10; // 10 temps boucle acquisition=10s
              config.Nuit_Nmax = val2 / 10;
              Serial.print(F("Jour_Nmax = ")), Serial.print(val);
              Serial.print(F(" Nuit_Nmax = ")), Serial.println(val2);
              sauvConfig();											// sauvegarde en EEPROM
            }
          }
        }
        message += F("Parametres (s)");
        message += fl;
        message += F("Jour : ");
        message += config.Jour_Nmax * 10 + fl;
        message += F("Nuit : ");
        message += config.Nuit_Nmax * 10 + fl;
        message += F("Actuel : ");
        message += Nmax * 10 + fl;
        EnvoyerSms(number, sms);
      }
      else if (textesms.indexOf(F("CAPTEUR")) == 0) {	// Capteurs actif CAPTEUR=1,0,1 (Pedale1,Pedale2,Coffret)
        bool flag = true; // validation du format
        if (textesms.indexOf(char(61)) == 7) { //char(61) "="	liste capteur actif
          byte Num[3];
          Sbidon = textesms.substring(8, 13);
          // Serial.print("Sbidon="),Serial.print(Sbidon),Serial.println(Sbidon.length());
          if (Sbidon.length() == 5) {
            int j = 0;
            for (int i = 0; i < 5; i += 2) {
              if (Sbidon.substring(i, i + 1) == "0" || Sbidon.substring(i, i + 1) == "1") {
                // Serial.print(",="),Serial.println(Sbidon.substring(i+1,i+2));
                // Serial.print("X="),Serial.println(Sbidon.substring(i,i+1));
                Num[j] = Sbidon.substring(i, i + 1).toInt();
                // Serial.print(i),Serial.print(","),Serial.print(j),Serial.print(","),Serial.println(Num[j]);
                j++;
              }
              else {
                Serial.println(F("Format non reconnu"));
                flag = false;// format pas bon
              }
            }
            if (flag) { // sauv configuration
              DesActiveInterrupt(); // desactive tous
              config.Pedale1 = Num[0];
              config.Pedale2 = Num[1];
              config.Coffret = Num[2];
              sauvConfig();											// sauvegarde en EEPROM
              ActiveInterrupt();
            }
          }
        }
        if (flag) {
          message += F("Pedale1 = ");
          if (config.Pedale1) {
            message += 1;
          }
          else {
            message += 0;
          }
          message += fl;
          message += F("Pedale2 = ");
          if (config.Pedale2) {
            message += 1;
          }
          else {
            message += 0;
          }
          message += fl;
          message += F("Coffret = ");
          if (config.Coffret) {
            message += 1;
          }
          else {
            message += 0;
          }
          message += fl;
        }
        else {
          message += F("Format non reconnu");
        }

        EnvoyerSms(number, sms);
      }
      else if (textesms.indexOf(F("TEMPOSORTIE")) == 0) { // Tempo Eclairage Sortie
        if (textesms.indexOf(char(61)) == 11) { // =
          int i = textesms.substring(12).toInt();
          // Serial.print(F("temposortie = ")),Serial.println(i);
          if (i > 0 && i < 121) {
            config.tempoSortie = i;
            sauvConfig();                               // sauvegarde en EEPROM
            Alarm.write(TempoSortie,config.tempoSortie);
            // TempoSortie = Alarm.timerRepeat(config.tempoSortie, Extinction); // tempo extinction a la sortie
            Alarm.disable(TempoSortie);
          }
        }
        message += F("Tempo Sortie Eclairage (s) = ");
        message += config.tempoSortie;
        message += fl;
        EnvoyerSms(number, sms);
      }
      else if (textesms.indexOf(F("TIMEOUTECL")) == 0) { // Timeout extinction secours
        if (textesms.indexOf(char(61)) == 10) { // =
          int i = textesms.substring(11).toInt();
          // Serial.print("Cpt pedale = "),Serial.println(i);
          if (i > 59 && i < 7201) {
            config.timeOutS = i;
            sauvConfig();                               // sauvegarde en EEPROM
            Alarm.disable(TimeOut);
            Alarm.write(TimeOut,config.timeOutS);
            // TimeOut = Alarm.timerRepeat(config.timeOutS, Extinction); // tempo time out extinction
            Alarm.disable(TimeOut);
          }
        }
        message += F("Time Out Eclairage (s) = ");
        message += config.timeOutS;
        message += fl;
        EnvoyerSms(number, sms);
      }
      else if (textesms.indexOf(F("DEBUT")) == 0) {     //	Heure Message Vie/debutJour
        if (textesms.indexOf(char(61)) == 5) {
          long i = atol(textesms.substring(6).c_str()); //	Heure message Vie
          if (i > 0 && i <= 86340) {                    //	ok si entre 0 et 86340(23h59)
            config.DebutJour = i;
            sauvConfig();                               // sauvegarde en EEPROM
            Alarm.disable(DebutJour);
            Alarm.write(DebutJour,config.DebutJour);
            // FinJour = Alarm.alarmRepeat(config.DebutJour, SignalVie);// init tempo
            Alarm.enable(DebutJour);
            AIntru_HeureActuelle();
          }
        }
        message += F("Debut Journee = ");
        message += Hdectohhmm(config.DebutJour);
        message += F("(hh:mm)");
        message += fl;
        EnvoyerSms(number, sms);
      }
      else if (textesms.indexOf(F("SONN")) == 0) {			//	Durée Sonnerie
        if (textesms.indexOf(char(61)) == 4) {
          int x = textesms.indexOf(":");
          int y = textesms.indexOf(":", x + 1);

          int i = atoi(textesms.substring(5, x).c_str());		//	Dsonn Sonnerie
          int j = atoi(textesms.substring(x + 1, y).c_str());//	DsonnMax Sonnerie
          int k = atoi(textesms.substring(y + 1).c_str()); 		//	Dsonnrepos Sonnerie
          //Serial.print(i),Serial.print(char(44)),Serial.print(j),Serial.print(char(44)),Serial.println(k);
          if (i > 5  && i <= 300 &&
              j > i  && j <= 600 &&
              k > 10 && k <= 300) {			//	ok si entre 10 et 300
            config.Dsonn 			= i;
            config.DsonnMax 	= j;
            config.Dsonnrepos = k;
            sauvConfig();															// sauvegarde en EEPROM
            Alarm.write(TSonn,config.Dsonn);
            // TSonn = Alarm.timerRepeat(config.Dsonn, ArretSonnerie);	// tempo durée de la sonnerie
            Alarm.disable(TSonn);

            Alarm.write(TSonnMax,config.DsonnMax);
            // TSonnMax = Alarm.timerRepeat(config.DsonnMax, SonnerieMax); // tempo maximum de sonnerie
            Alarm.disable(TSonnMax);

            Alarm.write(TSonnRepos,config.Dsonnrepos);
            // TSonnRepos = Alarm.timerRepeat(config.Dsonnrepos, ResetSonnerie); // tempo repos apres maxi
            Alarm.disable(TSonnRepos);
          }
        }
        message += F("Param Sonnerie = ");
        message += config.Dsonn;
        message += ":";
        message += config.DsonnMax;
        message += ":";
        message += config.Dsonnrepos;
        message += "(s)";
        message += fl;
        EnvoyerSms(number, sms);
      }
      else if (textesms.indexOf(F("SIRENE")) == 0) {			// Lancement SIRENE
        digitalWrite(PinSirene, HIGH);	// Marche Sonnerie
        Alarm.enable(TSonn);				// lancement tempo
        message += F("Lancement Sirene");
        message += fl;
        message += config.Dsonn;
        message += F("(s)");
        message += fl;
        EnvoyerSms(number, sms);
      }
      else if (textesms.indexOf(F("TIME")) > -1) {
        message += F("Heure Sys = ");
        message += displayTime(0);
        message += fl;
        EnvoyerSms(number, sms);
      }
      else if (textesms.indexOf(F("MAJHEURE")) == 0) {	//	forcer mise a l'heure
        message += F("Mise a l'heure");
        // Sim800.reset(SIMPIN);// lancer SIM800
        if (gsm)MajHeure(Sim800.getTimeSms(slot)); // mise a l'heure du sms
        EffaceSMS(slot);
        if (nom != F("Moi meme")) EnvoyerSms(number, sms);
      }
      else if (gsm && textesms.indexOf(F("IMEI")) > -1) {
        message += F("IMEI = ");
        String m = Sim800.getIMEI();
        message += m + fl;
        EnvoyerSms(number, sms);
      }
      else if (textesms.indexOf(F("FIN")) == 0) {			//	Heure Fin de journée
        if ((textesms.indexOf(char(61))) == 3) {
          long i = atol(textesms.substring(4).c_str()); //	Heure
          if (i > 0 && i <= 86340) {										//	ok si entre 0 et 86340(23h59)
            config.FinJour = i;
            sauvConfig();															// sauvegarde en EEPROM
            Alarm.disable(FinJour);
            Alarm.write(FinJour,config.FinJour);
            // FinJour = Alarm.alarmRepeat(config.FinJour, FinJournee);// init tempo
            Alarm.enable(FinJour);
          }
        }
        message += F("Fin Journee = ");
        message += Hdectohhmm(config.FinJour);
        message += F("(hh:mm)");
        message += fl;
        EnvoyerSms(number, sms);
      }
      else if (textesms.indexOf(F("MOIS")) > -1) { // Calendrier pour un mois
        /* mise a jour calendrier ;format : MOIS=mm,31 fois 0/1
          demande calendrier pour un mois donné ; format : MOIS=mm? */
        bool flag = true; // validation du format
        bool W = true; // true Write, false Read
        int m = 0;
        if (textesms.indexOf("{") == 0) { // json
          DynamicJsonDocument doc(540);
          int f = textesms.lastIndexOf("}");
          // Serial.print("pos }:"),Serial.println(f);
          // Serial.print("json:"),Serial.print(textesms.substring(0,f+1)),Serial.println(".");,1,1,1,1,1,0,0,0,0,0,0]}";
          DeserializationError err = deserializeJson(doc, textesms.substring(0, f + 1));
          if(!err){
            m = doc["MOIS"]; // 12
            JsonArray jour = doc["JOUR"];
            for (int j = 1; j < 32; j++) {
              calendrier[m][j] = jour[j - 1];
            }
            // Serial.print("mois:"),Serial.println(m);
            EnregistreCalendrier(); // Sauvegarde en SPIFFS
            // message += F("Mise a jour calendrier \nmois:");
            // message += m;
            // message += " OK (json)";
          }
          else{
            message += " erreur json ";
            flag = false;
          }
        }
        else { // message normal mois=12,31*0/1
          byte p1 = textesms.indexOf(char(61)); // =
          byte p2 = textesms.indexOf(char(44)); // ,
          if (p2 == 255) {                      // pas de ,
            p2 = textesms.indexOf(char(63));    // ?
            W = false;
          }

          m = textesms.substring(p1 + 1, p2).toInt(); // mois

          // printf("p1=%d,p2=%d\n",p1,p2);
          // Serial.println(textesms.substring(p1+1,p2).toInt());
          // Serial.println(textesms.substring(p2+1,textesms.length()).length());
          if (!(m > 0 && m < 13)) flag = false;
          if (W && flag) { // Write
            if (!(textesms.substring(p2 + 1, textesms.length()).length() == 31)) flag = false; // si longueur = 31(jours)

            for (int i = 1; i < 32; i++) { // verification 0/1
              if (!(textesms.substring(p2 + i, p2 + i + 1) == "0" || textesms.substring(p2 + i, p2 + i + 1) == "1")) {
                flag = false;
              }
            }
            if (flag) {
              // Serial.println(F("mise a jour calendrier"));
              for (int i = 1; i < 32; i++) {
                calendrier[m][i] = textesms.substring(p2 + i, p2 + i + 1).toInt();
                // Serial.print(textesms.substring(p2+i,p2+i+1));
              }
              EnregistreCalendrier(); // Sauvegarde en SPIFFS
              // message += F("Mise a jour calendrier mois:");
              // message += m;
              // message += " OK";
            }
          }
          if(!flag) {
            // printf("flag=%d,W=%d\n",flag,W);
            message += " erreur format ";
          }
        }
        if (flag) { // demande calendrier pour un mois donné
          if (smsserveur || !sms) {
            // si serveur reponse json  {"mois":12,"jour":[1,2,4,5,6 .. 31]}
            DynamicJsonDocument doc(540);
            doc["mois"] = m;
            JsonArray jour = doc.createNestedArray("jour");
            for (int i = 1; i < 32; i++) {
              jour.add(calendrier[m][i]);
            }
            String jsonbidon;
            serializeJson(doc, jsonbidon);
            message += jsonbidon;
            // message +="{\"mois\":" + String(m) + "," +fl;
            // message += "\"jour\":[";
            // for (int i = 1; i < 32 ; i++){
            // message += String(calendrier[m][i]);
            // if (i < 31) message += ",";
            // }
            // message += "]}";
          }
          else {
            message += F("mois = ");
            message += m;
            message += fl;
            for (int i = 1; i < 32 ; i++) {
              message += calendrier[m][i];
              if ((i % 5)  == 0) message += " ";
              if ((i % 10) == 0) message += fl;
            }
          }
        }
        message += fl;
        EnvoyerSms(number, sms);
      }
      else if (textesms == F("CIRCULE")) {
        bool ok = false;
        /* demande passer en mode Circulé pour le jour courant,
        	sans modification calendrier enregistré en SPIFFS */
        if (!(calendrier[month()][day()] ^ flagCircule)) {
          // calendrier[month()][day()] = 1;
          message += F("OK, Circule");
          flagCircule = !flagCircule;
          ok = true;
        }
        else {
          message += F("Jour deja Circule");
        }
        message += fl;
        EnvoyerSms(number, sms);
        if (ok) {
          // if (sms)EffaceSMS(slot);
          action_wakeup_reason(4);
        }
      }
      else if (textesms == F("NONCIRCULE")) {
        bool ok = false;
        /* demande passer en mode nonCirculé pour le jour courant,
          sans modification calendrier enregistré en SPIFFS */
        if (calendrier[month()][day()] ^ flagCircule) {
          // calendrier[month()][day()] = 0;
          message += F("OK, NonCircule");
          flagCircule = !flagCircule;
          ok = true;
        }
        else {
          message += F("Jour deja NonCircule");
        }
        message += fl;
        EnvoyerSms(number, sms);
        if (ok) {
          // if (sms)EffaceSMS(slot);
          action_wakeup_reason(4);
        }
      }
      else if (textesms.indexOf(F("TEMPOWAKEUP")) == 0) { // Tempo wake up
        if ((textesms.indexOf(char(61))) == 11) {
          int i = textesms.substring(12).toInt(); //	durée
          if (i > 59 && i <= 36000) { // 1mn à 10H
            config.RepeatWakeUp = i;
            sauvConfig();															// sauvegarde en EEPROM
          }
        }
        message += F("Tempo repetition Wake up (s)=");
        message += config.RepeatWakeUp;
        EnvoyerSms(number, sms);
      }
      else if (textesms.indexOf(F("TEMPOANALYSE")) == 0) { // Tempo Analyse Alarme apres reveil sur alarme EXT
        if ((textesms.indexOf(char(61))) == 12) {
          int i = textesms.substring(13).toInt(); //	durée
          if (i > 59 && i <= 1800) { // 1mn à 30mn
            config.Tanalyse = i;
            sauvConfig();															// sauvegarde en EEPROM
            Alarm.write(TempoAnalyse,config.Tanalyse);
            // TempoAnalyse = Alarm.timerRepeat(config.Tanalyse, FinAnalyse); // Tempo Analyse Alarme sur interruption
            Alarm.disable(TempoAnalyse);
          }
        }
        message += F("Tempo Analyse apres Wake up (s)=");
        message += config.Tanalyse;
        EnvoyerSms(number, sms);
      }
      else if (textesms.indexOf(F("LST2")) > -1) { //	Liste restreinte	//  =LST2=0,0,0,0,0,0,0,0,0
        bool flag = true; // validation du format
        if (textesms.indexOf(char(61)) == 4) { // "="
          byte Num[10];
          Sbidon = textesms.substring(5, 22);
          // Serial.print("bidon="),Serial.print(Sbidon),Serial.print("="),Serial.println(Sbidon.length());
          if (Sbidon.length() == 17) {
            int j = 1;
            for (int i = 0; i < 17; i += 2) {
              if (i == 16 && (Sbidon.substring(i, i + 1) == "0"	|| Sbidon.substring(i, i + 1) == "1")) {
                Num[j] = Sbidon.substring(i, i + 1).toInt();
              }
              else if ((Sbidon.substring(i + 1, i + 2) == ",") && (Sbidon.substring(i, i + 1) == "0"	|| Sbidon.substring(i, i + 1) == "1")) {
                //Serial.print(",="),Serial.println(bidon.substring(i+1,i+2));
                //Serial.print("X="),Serial.println(bidon.substring(i,i+1));
                Num[j] = Sbidon.substring(i, i + 1).toInt();
                //Serial.print(i),Serial.print(","),Serial.print(j),Serial.print(","),Serial.println(Num[j]);
                j++;
              }
              else {
                Serial.println(F("Format pas reconnu"));
                flag = false;
              }
            }
            if (flag) {
              //Serial.println("copie des num");
              for (int i = 1; i < 10; i++) {
                config.Pos_Pn_PB[i] = Num[i];
              }
              sauvConfig();															// sauvegarde en EEPROM
            }
          }
        }
        message += F("Liste restreinte");
        message += fl;
        for (int i = 1; i < 10; i++) {
          message += config.Pos_Pn_PB[i];
          if ( i < 9) message += char(44); // ,
        }
        EnvoyerSms(number, sms);
      }
      else if (textesms == F("RST")) {               // demande RESET
        message += F("Le systeme va etre relance");  // apres envoie du SMS!
        message += fl;
        FlagReset = true;                            // reset prochaine boucle
        EnvoyerSms(number, sms);
      }
      else if (!sms && textesms.indexOf(F("CALIBRATION=")) == 0) {
        /* 	Mode calibration mesure tension
        		Seulement en mode serie local
        		recoit message "CALIBRATION=.X"
        		entrer mode calibration
        		Selection de la tenssion à calibrer X
        		X = 1 TensionBatterie : PinBattSol : CoeffTension1
        		X = 2 VBatterieProc : PinBattProc : CoeffTension2
        		X = 3 VUSB : PinBattUSB : CoeffTension3
        		X = 4 Tension24 : Pin24V : CoeffTension4
        		effectue mesure tension avec CoeffTensionDefaut retourne et stock resultat
        		recoit message "CALIBRATION=1250" mesure réelle en V*100
        		calcul nouveau coeff = mesure reelle/resultat stocké * CoeffTensionDefaut
        		applique nouveau coeff
        		stock en EEPROM
        		sort du mode calibration

        		variables
        		FlagCalibration true cal en cours, false par defaut
        		Static P pin d'entrée
        		static int tensionmemo memorisation de la premiere tension mesurée en calibration
        		int CoeffTension = CoeffTensionDefaut 7000 par défaut
        */
        Sbidon = textesms.substring(12, 16); // texte apres =
        //Serial.print(F("Sbidon=")),Serial.print(Sbidon),Serial.print(char(44)),Serial.println(Sbidon.length());
        long tension = 0;
        if (Sbidon.substring(0, 1) == "." && Sbidon.length() > 1) { // debut mode cal
          if (Sbidon.substring(1, 2) == "1" ) {
            M = 1;
            P = PinBattSol;
            coef = CoeffTension[0];
          }
          if (Sbidon.substring(1, 2) == "2" ) {
            M = 2;
            P = PinBattProc;
            coef = CoeffTension[1];
          }
          if (Sbidon.substring(1, 2) == "3" ) {
            M = 3;
            P = PinBattUSB;
            coef = CoeffTension[2];
          }
          if (Sbidon.substring(1, 2) == "4" ) {
            Allumage(1); // Allumage
            Alarm.delay(500);
            read_adc(PinBattSol, PinBattProc, PinBattUSB, Pin24V); // lecture des adc
            M = 4;
            P = Pin24V;
            coef = CoeffTension[3];
          }
          // Serial.print("mode = "),Serial.print(M),Serial.println(Sbidon.substring(1,2));
          FlagCalibration = true;

          coef = CoeffTensionDefaut;
          tension = map(adc_mm[M-1] / nSample, 0, 4095, 0, coef);
          // tension = map(moyenneAnalogique(P), 0, 4095, 0, coef);
          // Serial.print("TensionBatterie = "),Serial.println(TensionBatterie);
          tensionmemo = tension;
        }
        else if (FlagCalibration && Sbidon.substring(0, 4).toInt() > 0 && Sbidon.substring(0, 4).toInt() <= 8000) {
          // si Calibration en cours et valeur entre 0 et 5000
          Serial.println(Sbidon.substring(0, 4));
          /* calcul nouveau coeff */
          coef = Sbidon.substring(0, 4).toFloat() / float(tensionmemo) * CoeffTensionDefaut;
          // Serial.print("Coeff Tension = "),Serial.println(coef);
          tension = map(adc_mm[M-1] / nSample, 0, 4095, 0, coef);
          // tension = map(moyenneAnalogique(P), 0, 4095, 0, coef);
          CoeffTension[M - 1] = coef;
          FlagCalibration = false;
          Recordcalib();														// sauvegarde en SPIFFS

          if (M == 4) {
            Extinction(); // eteindre
          }
        }
        else {
          message += F("message non reconnu");
          message += fl;
          FlagCalibration = false;
        }
        message += F("Mode Calib Tension ");
        message += String(M) + fl;
        message += F("TensionMesuree = ");
        message += tension;
        message += fl;
        message += F("Coeff Tension = ");
        message += coef;
        if (M == 1) {
          message += fl;
          message += F("Batterie = ");
          message += String(BattPBpct(tension, 6));
          message += "%";
        }
        message += fl;
        EnvoyerSms(number, sms);
      }
      else if (textesms.indexOf(F("MASTEROFF")) == 0) {
        FlagMasterOff = true;
        Extinction();
        message += F("MasterOFF actif\n");
        message += F("Allumage Impossible");
        message += fl;
        EnvoyerSms(number, sms);
      }
      else if (textesms.indexOf(F("MASTERON")) == 0) {
        FlagMasterOff = false;
        message += F("MasterOFF inactif\n");
        message += F("Allumage possible");
        message += fl;
        EnvoyerSms(number, sms);
      }
      else if (textesms.indexOf(F("ALLUME=")) == 0) {
        // ALLUME forcée
        // ALLUME = 1 à 12 durée en heures, remplace TIMEOUTECL

        int duree = textesms.substring(7,8).toInt();
        if(duree > 0 && duree < 13){
          Alarm.disable(TimeOut);
          Alarm.write(TimeOut,duree * 3600L);
          // TimeOut = Alarm.timerRepeat(duree * 60, Extinction); // tempo time out extinction
          Alarm.disable(TimeOut);

          if (!Allume) {
            Allumage(10);
            message += F("Allumage ");
            message += String(duree);
            message += " Heure(s)";
          }
        }
        generationMessage(0);
        EnvoyerSms(number, sms);
      }
      else if (textesms.indexOf(F("ETEINDRE")) == 0) {
        // if (Allume) {
          Extinction();
        // }
        generationMessage(0);
        EnvoyerSms(number, sms);
      }
      else if (gsm && textesms.indexOf(F("UPLOADLOG")) == 0) {//upload log
        message += F("lancement upload log");
        message += fl;
        MajLog(nom, "upload log");// renseigne log
        Sbidon = String(config.apn);
        Sim800.activateBearerProfile(config.apn); // ouverture GPRS

        Serial.println(F("Starting..."));
        int reply = gprs_upload_function (); // Upload fichier
        Serial.println("The end... Response: " + String(reply));

        if(reply == 0){
          message += F("upload OK");
          SPIFFS.remove(filelog);  // efface fichier log
          MajLog(nom, "");         // nouveau log
          MajLog(nom, F("upload OK"));// renseigne nouveau log
        } else{
          message += F("upload fail");
          MajLog(nom, F("upload fail"));// renseigne log
        }
        Sim800.deactivateBearerProfile(); // fermeture GPRS
        EnvoyerSms(number, sms);
      }
      else if (textesms.indexOf("FTPDATA") > -1) {
        // Parametres FTPDATA=Serveur:User:Pass:port
        // {"FTPDATA":{"serveur":"dd.org","user":"user","pass":"pass",,"port":00}}
        bool erreur = false;
        bool formatsms = false;
        if (textesms.indexOf(":") == 10) { // format json
          DynamicJsonDocument doc(210); //https://arduinojson.org/v6/assistant/
          DeserializationError err = deserializeJson(doc, textesms);
          if (err) {
            erreur = true;
        }
        else {
          JsonObject ftpdata = doc["FTPDATA"];
          strncpy(config.ftpServeur,  ftpdata["serveur"], 26);
          strncpy(config.ftpUser,     ftpdata["user"],    11);
          strncpy(config.ftpPass,     ftpdata["pass"],    16);
          config.ftpPort         =    ftpdata["port"];
          sauvConfig();													// sauvegarde en EEPROM
        }
      }
      else if ((textesms.indexOf(char(61))) == 7) { // format sms
        formatsms = true;
        byte w = textesms.indexOf(":");
        byte x = textesms.indexOf(":", w + 1);
        byte y = textesms.indexOf(":", x + 1);
        byte zz = textesms.length();
        if (textesms.substring(y + 1, zz).toInt() > 0) { // Port > 0
          if ((w - 7) < 25 && (x - w - 1) < 11 && (y - x - 1) < 16) {
            Sbidon = textesms.substring(7, w);
            Sbidon.toCharArray(config.ftpServeur, (Sbidon.length() + 1));
            Sbidon = textesms.substring(w + 1, x);
            Sbidon.toCharArray(config.ftpUser, (Sbidon.length() + 1));
            Sbidon = textesms.substring(x + 1, y);
            Sbidon.toCharArray(config.ftpPass, (Sbidon.length() + 1));
            config.ftpPort = textesms.substring(y + 1, zz).toInt();
            sauvConfig();													// sauvegarde en EEPROM
          }
          else {
            erreur = true;
          }
        } else {
          erreur = true;
        }
        }
        if (!erreur) {
          if (formatsms) {
            message += "Sera pris en compte au prochain demarrage\nOu envoyer RST maintenant";
            message += fl;
            message += F("Parametres FTP :");
            message += fl;
            message += "Serveur:" + String(config.ftpServeur) + fl;
            message += "User:"    + String(config.ftpUser) + fl;
            message += "Pass:"    + String(config.ftpPass) + fl;
            message += "Port:"    + String(config.ftpPort) + fl;
          }
          else {
            DynamicJsonDocument doc(210);
            JsonObject FTPDATA = doc.createNestedObject("FTPDATA");
            FTPDATA["serveur"] = config.ftpServeur;
            FTPDATA["user"]    = config.ftpUser;
            FTPDATA["pass"]    = config.ftpPass;
            FTPDATA["port"]    = config.ftpPort;
            Sbidon = "";
            serializeJson(doc, Sbidon);
            message += Sbidon;
            message += fl;
          }
        }
        else {
          message += "Erreur format";
          message += fl;
        }
        EnvoyerSms(number, sms);
      }
      else if (textesms.indexOf("FTPSERVEUR") == 0) { // Serveur FTP
        // case sensitive
        // FTPSERVEUR=xyz.org
        if (textesms.indexOf(char(61)) == 10) {
          Sbidon = textesms.substring(11);
          Serial.print("ftpserveur:"),Serial.print(Sbidon);
          Serial.print(" ,"), Serial.println(Sbidon.length());
          Sbidon.toCharArray(config.ftpServeur, (Sbidon.length() + 1));
          sauvConfig();
        }
        message += F("FTPserveur =");
        message += String(config.ftpServeur);
        message += F("\n au prochain demarrage");
        EnvoyerSms(number, sms);
      }
      else if (textesms.indexOf(F("GPRSDATA")) > -1) {
        // Parametres GPRSDATA = "APN":"user":"pass"
        // GPRSDATA="sl2sfr":"":""
        // {"GPRSDATA":{"apn":"sl2sfr","user":"","pass":""}}
        bool erreur = false;
        bool formatsms = false;
        if (textesms.indexOf(":") == 11) { // format json
          DynamicJsonDocument doc(120);
          DeserializationError err = deserializeJson(doc, textesms);
          if (err) {
            erreur = true;
          }
          else {
            JsonObject gprsdata = doc["GPRSDATA"];
            strncpy(config.apn, gprsdata["apn"], 11);
            strncpy(config.gprsUser, gprsdata["user"], 11);
            strncpy(config.gprsPass, gprsdata["pass"], 11);
            // Serial.print("apn length:"),Serial.println(strlen(gprsdata["apn"]));
            // Serial.print("apn:"),Serial.println(config.apn);
            // Serial.print("user:"),Serial.println(config.gprsUser);
            // Serial.print("pass:"),Serial.println(config.gprsPass);
            sauvConfig();													// sauvegarde en EEPROM
          }
        }
        else if ((textesms.indexOf(char(61))) == 8) { // format sms
          formatsms = true;
          byte cpt = 0;
          byte i = 9;
          do { // compte nombre de " doit etre =6
            i = textesms.indexOf('"', i + 1);
            cpt ++;
          } while (i <= textesms.length());
          Serial.print("nombre de \" :"), Serial.println(cpt);
          if (cpt == 6) {
            byte x = textesms.indexOf(':');
            byte y = textesms.indexOf(':', x + 1);
            byte z = textesms.lastIndexOf('"');
            // Serial.printf("%d:%d:%d\n",x,y,z);
            // Serial.printf("%d:%d:%d\n", x -1 - 10, y-1 - x-1-1, z - y-1-1);
            if ((x - 11) < 11 && (y - x - 3) < 11 && (z - y - 2) < 11) { // verification longueur des variables
              Sbidon = textesms.substring(10, x - 1);
              Sbidon.toCharArray(config.apn, (Sbidon.length() + 1));
              Sbidon = textesms.substring(x + 1 + 1 , y - 1);
              Sbidon.toCharArray(config.gprsUser, (Sbidon.length() + 1));
              Sbidon = textesms.substring(y + 1 + 1, z);
              Sbidon.toCharArray(config.gprsPass, (Sbidon.length() + 1));

              // Serial.print("apn:"),Serial.println(config.apn);
              // Serial.print("user:"),Serial.println(config.gprsUser);
              // Serial.print("pass:"),Serial.println(config.gprsPass);

              sauvConfig();													// sauvegarde en EEPROM
            }
            else {
              erreur = true;
            }
          }
          else {
            erreur = true;
          }
        }
        if (!erreur) {
          if (formatsms) {
            message += "Sera pris en compte au prochain demarrage\nOu envoyer RST maintenant" + fl;
            message += "Parametres GPRS \"apn\":\"user\":\"pass\"";
            message += fl + "\"";
            message += String(config.apn);
            message += "\":\"";
            message += String(config.gprsUser);
            message += "\":\"";
            message += String(config.gprsPass);
            message += "\"" + fl;
          }
          else {
            DynamicJsonDocument doc(120);
            JsonObject gprsdata = doc.createNestedObject("GPRSDATA");
            gprsdata["apn"]  = config.apn;
            gprsdata["user"] = config.gprsUser;
            gprsdata["pass"] = config.gprsPass;
            Sbidon = "";
            serializeJson(doc, Sbidon);
            message += Sbidon;
            message += fl;
          }
        }
        else {
          message += "Erreur format";
          message += fl;
        }
        EnvoyerSms(number, sms);
      }
      else if (textesms == "VIDELOG"){
        SPIFFS.remove(filelog);
        FileLogOnce = false;
        message += "Effacement fichier log";
        EnvoyerSms(number, sms);
      }
      else if (textesms.indexOf(F("SENDAT")) == 0){
        // envoie commande AT au SIM800
        // attention DANGEREUX pas de verification!
        if (textesms.indexOf(char(61)) == 6) {
          String CdeAT = textesms.substring(7, textesms.length());
          String reply = sendAT(CdeAT,"OK","ERROR",1000);
          // Serial.print("reponse: "),Serial.println(reply);
          message += String(reply);          
          EnvoyerSms(number, sms);
        }
      }
      //**************************************
      else {
        message += F("message non reconnu !");
        message += fl;
        if (nom != F("Moi meme")) EnvoyerSms(number, sms);
      }
    }
    else {
      Serial.print(F("Appelant non reconnu ! "));
    }
  }
  // Alarm.enable(loopPrincipale);
}
//---------------------------------------------------------------------------
void envoie_alarme() {
  /* determine si un SMS appartition/disparition Alarme doit etre envoyé */
  bool SendEtat = false;

  if (FlagAlarme24V != FlagLastAlarme24V) {
    SendEtat = true;
    MajLog(F("Auto"), F("Alarme24V"));
    FlagLastAlarme24V = FlagAlarme24V;
  }
  if (FlagAlarmeTension != FlagLastAlarmeTension) {
    SendEtat = true;
    MajLog(F("Auto"), F("AlarmeTension"));
    FlagLastAlarmeTension = FlagAlarmeTension;
  }
  if (FlagAlarmeIntrusion != FlagLastAlarmeIntrusion) {
    SendEtat = true;
    MajLog(F("Auto"), F("AlarmeIntrusion"));
    FlagLastAlarmeIntrusion = FlagAlarmeIntrusion;
  }
  if (SendEtat) { 						// si envoie Etat demandé
    envoieGroupeSMS(0, 0);			// envoie groupé
    SendEtat = false;					// efface demande
  }
}
//---------------------------------------------------------------------------
void envoieGroupeSMS(byte grp, bool m) {
  if (gsm) {
    /* m=0 message normal/finanalyse
      si grp = 0,
      envoie un SMS à tous les numero existant (9 max) du Phone Book
      SAUF ceux de la liste restreinte
      si grp = 1,
      envoie un SMS à tous les numero existant (9 max) du Phone Book
      de la liste restreinte config.Pos_Pn_PB[x]=1			*/

    byte n = Sim800.ListPhoneBook(); // nombre de ligne PhoneBook
    // Serial.print(F("Nombre de ligne PB=")),Serial.println(n);
    for (byte Index = 1; Index < n + 1; Index++) { // Balayage des Num Tel dans Phone Book
      if ((grp == 0 && config.Pos_Pn_PB[Index] == 0) || (grp == 1 && config.Pos_Pn_PB[Index] == 1)) {
        String number = Sim800.getPhoneBookNumber(Index);
        generationMessage(m);
        char num[13];
        number.toCharArray(num, 13);
        EnvoyerSms(num, true);
      }
    }
  }
}
//---------------------------------------------------------------------------
void generationMessage(bool n) {
  // n = 0 message normal
  // n = 1 message fin analyse
  messageId();
  if (FlagAlarmeTension || FlagLastAlarmeTension || FlagAlarmeIntrusion || FlagAlarme24V) {
    message += F("--KO--------KO--");
  }
  else {
    message += F("-------OK-------");
  }
  message += fl;
  if (config.Intru && FlagAlarmeIntrusion) {
    message += F("Alarme !") ;
    if (FlagAlarmeCable1) {
      message += F(" Pedale 1");
    }
    if (FlagAlarmeCable2) {
      message += F(" Pedale 2");
    }
    if (FlagAlarmeCoffret) {
      message += F(" Coffret");
    }
    message += fl;
  }
  else if (config.Intru) {
    message += F("Alarme Active ");
    if (config.Pedale1) {
      message += "1";
    }
    else {
      message += "0";
    }
    if (config.Pedale2) {
      message += "1";
    }
    else {
      message += "0";
    }
    if (config.Coffret) {
      message += "1";
    }
    else {
      message += "0";
    }
    message += fl;
  }
  else {
    message += F("Alarme Arrete");
    message += fl;
  }
  // if (config.Silence) {
  //   message += F("Alarme Silencieuse ON");
  //   message += fl;
  // }
  // else
  // {
  //   message += F("Alarme Silencieuse OFF");
  //   message += fl;
  // }
  message += F("Batterie : ");
  if (!FlagAlarmeTension) {
    message += F("OK, ");
    message += String(BattPBpct(TensionBatterie, 6));
    message += "%" + fl;
  }
  else {
    message += F("Alarme, ");
    message += String(BattPBpct(TensionBatterie, 6));
    message += "%";
    message += fl;
    message += F("V USB =");
    message += String(float(VUSB / 1000.0)) + fl;
  }
  if (Allume) {
    for (int i = 0; i < 5 ; i++) {
      read_adc(PinBattSol, PinBattProc, PinBattUSB, Pin24V);
      Alarm.delay(1);
    }
    char bid[8];// Allume : 24.26 V
    sprintf(bid, "%.2lf V", float(Tension24) / 100);
    message += F("Allume : ");
    message += String(bid);
    message += fl;
  }
  if (FlagAlarme24V) {
    message += F("Alarme 24V = ");
    message += String(float(Tension24 / 100.0)) + "V" + fl;
  }
  if ((calendrier[month()][day()] ^ flagCircule)) {
    message += F("Jour Circule");
  }
  else {
    message += F("Jour Non Circule");
  }
  message += fl;
  message += F("Nbr Allumage = ");
  message += String(CptAllumage);
  message += fl ;
  if (n) {
    message += F("Fin Analyse");
    message += fl;
  }
  // Serial.println(message);
}
//---------------------------------------------------------------------------
void EnvoyerSms(char *num, bool sms) {

  if (sms && gsm) { // envoie sms
    message.toCharArray(replybuffer, message.length() + 1);
    bool OK = Sim800.sendSms(num, replybuffer);
    if (OK){
      Serial.print("send sms OK : ");
      Serial.println(num);
    }
  }
  Serial.print (F("Message (long) = ")), Serial.println(message.length());
  Serial.println(message);
}
//---------------------------------------------------------------------------
void read_RSSI() {	// lire valeur RSSI et remplir message
  if (gsm) {
    int r;
    byte n = Sim800.getRSSI();
    // Serial.print(F("RSSI = ")); Serial.print(n); Serial.print(F(": "));
    if (n == 0) r = -115;
    if (n == 1) r = -111;
    if (n == 31) r = -52;
    if ((n >= 2) && (n <= 30)) {
      r = map(n, 2, 30, -110, -54);
    }
    message += F("RSSI=");
    message += String(n);
    message += ", ";
    message += String(r);
    message += F("dBm");
    message += fl;
  }
}
//---------------------------------------------------------------------------
void MajHeure(String smsdate) {
  if (gsm) {
    /*parametrage du SIM800 a faire une fois
      AT+CLTS? si retourne 0
      AT+CLTS=1
      AT+CENG=3
      AT&W pour sauvegarder ce parametre
      si AT+CCLK? pas OK
      avec Fonatest passer en GPRS 'G', envoyer 'Y' la sync doit se faire, couper GPRS 'g'
      't' ou AT+CCLK? doit donner la date et heure réseau
      format date retourné par Fona "yy/MM/dd,hh:mm:ss±zz",
      +CCLK: "14/08/08,02:25:43-16" -16= décalage GMT en n*1/4heures(-4) */
    if (smsdate.length() > 1) { // si smsdate present mise a l'heure forcé
      Sim800.SetTime(smsdate);
    }
    static bool First = true;
    int ecart;
    Serial.print(F("Mise a l'heure reguliere !, "));
    // setTime(10,10,0,1,1,18);
    int Nday, Nmonth, Nyear, Nminute, Nsecond, Nhour;
    Sim800.RTCtime(&Nday, &Nmonth, &Nyear, &Nhour, &Nminute, &Nsecond);


    printf("%s %02d/%02d/%d %02d:%02d:%02d\n", "MajH1", Nday, Nmonth, Nyear, Nhour, Nminute, Nsecond);
    long debut = millis();
    if (First || Nyear < 17) {
      while (Nyear < 17) {
        Sim800.RTCtime(&Nday, &Nmonth, &Nyear, &Nhour, &Nminute, &Nsecond);
        printf("%s %02d/%02d/%d %02d:%02d:%02d\n", "MajH2", Nday, Nmonth, Nyear, Nhour, Nminute, Nsecond);
        Alarm.delay(1000);
        // if (millis() - debut > 10000) {// supprimé risque de deconnexion reseau plus de redemarage
        // Sim800.setPhoneFunctionality(0);
        // Alarm.delay(1000);
        // Sim800.setPhoneFunctionality(1);
        // Alarm.delay(1000);
        // }
        if (millis() - debut > 15000) {
          Serial.println(F("Impossible de mettre à l'heure !"));
          //on s'envoie à soi même un SMS "MAJHEURE"
          // message = F("MAJHEURE");
          // char numchar[13];
          // String numstring = Sim800.getNumTel();
          // numstring.toCharArray(numchar, 13);
          // EnvoyerSms(numchar, true);
          // break;

          // Mise à l'heure par défaut V2-12
          Sim800.SetTime("22/08/01,08:00:00+08"); // 01/08 jour toujours circule
          Nyear   = 22;
          Nmonth  = 8;
          Nday    = 1;
          Nhour   = 8;
          Nminute = 0;
          Nsecond = 0;
          break;
        }
      }
      setTime(Nhour, Nminute, Nsecond, Nday, Nmonth, Nyear);
      First = false;
    }
    else {
      //  calcul décalage entre H sys et H reseau en s
      ecart = (Nhour - hour()) * 3600;
      ecart += (Nminute - minute()) * 60;
      ecart += Nsecond - second();
      // ecart += 10;
      Serial.print(F("Ecart s= ")), Serial.println(ecart);
      if (abs(ecart) > 5) {
        ArretSonnerie();	// Arret Sonnerie propre
        Alarm.disable(loopPrincipale);
        Alarm.disable(TempoAnalyse);
        Alarm.disable(TempoSortie);
        Alarm.disable(TimeOut);
        Alarm.disable(DebutJour);
        Alarm.disable(FinJour);
        Alarm.disable(TSonnRepos);

        setTime(Nhour, Nminute, Nsecond, Nday, Nmonth, Nyear);

        Alarm.enable(loopPrincipale);
        Alarm.enable(TempoAnalyse);
        Alarm.enable(TempoSortie);
        Alarm.enable(TimeOut);
        Alarm.enable(DebutJour);
        Alarm.enable(FinJour);
      }
    }
  }
  displayTime(0);
  AIntru_HeureActuelle();
}
//---------------------------------------------------------------------------
void ActivationSonnerie() {
  // Sonnerie PN
  if (!SonnMax) {									// pas atteint Temps sonnerie maxi
    if (!config.Silence) digitalWrite(PinSirene, HIGH);// Marche Sonnerie sauf Silence
    Alarm.enable(TSonn);
    if (!FirstSonn) {							// premiere sonnerie on lance Tempo Max
      Alarm.enable(TSonnMax);
      FirstSonn = true;
    }
  }
}
//---------------------------------------------------------------------------
void ArretSonnerie() {
  Serial.print(F("Arret Sonnerie : "));
  Serial.println(Alarm.getTriggeredAlarmId());
  digitalWrite(PinSirene, LOW);// Arret Sonnerie
  Alarm.disable(TSonn);	// on arrete la tempo sonnerie
  Alarm.disable(TSonnMax);// on arrete la tempo sonnerie maxi
  FirstSonn = false;
  FlagAlarmeIntrusion = false;
  FlagAlarmeCable1 = false;
  FlagAlarmeCable2 = false;
  FlagAlarmeCoffret = false;
  FlagPIR = false;
}
//---------------------------------------------------------------------------
void SonnerieMax() {
  Serial.print(F("Fin periode Max Sonnerie : "));
  Serial.println(Alarm.getTriggeredAlarmId());
  // fin de la tempo temps de sonnerie maxi
  Alarm.enable(TSonnRepos);// on lance la tempo repos sonnerie
  Alarm.disable(TSonnMax);// on arrete la tempo sonnerie maxi
  Alarm.disable(TSonn);	// on arrete la tempo sonnerie
  digitalWrite(PinSirene, LOW);// Arret Sonnerie
  FirstSonn = false;///
  SonnMax = true;				// interdit nouveau lancement sonnerie
  // avant fin tempo repos sonnerie
}
//---------------------------------------------------------------------------
void ResetSonnerie() {
  Serial.print(F("Fin periode inhibition sonnerie : "));
  Serial.println(Alarm.getTriggeredAlarmId());
  // fin de la tempo repos apres temps sonnerie maxi
  Alarm.disable(TSonnRepos);// on arrete la tempo repos
  SonnMax = false;
  ArretSonnerie();
}
//---------------------------------------------------------------------------
void FinAnalyse() {
  /* retour etat normal apres Alarme sur Interruption */
  Alarm.disable(TempoAnalyse);
  WupAlarme = false;
}
//---------------------------------------------------------------------------
long DureeSleep(long Htarget) { // Htarget Heure de reveil visée
  /* calcul durée entre maintenant et Htarget*/
  long SleepTime = 0;
  long Heureactuelle = HActuelledec();
  if (Heureactuelle < Htarget) {
    SleepTime = Htarget - Heureactuelle;
  }
  else {
    if (Heureactuelle < 86400) { // < 24h00
      SleepTime = (86400 - Heureactuelle) + Htarget;
    }
  }
  return SleepTime;
}
//---------------------------------------------------------------------------
long HActuelledec() {
  long Heureactuelle = hour() * 60; // calcul en 4 lignes sinon bug!
  Heureactuelle += minute();
  Heureactuelle  = Heureactuelle * 60;
  Heureactuelle += second(); // en secondes
  return Heureactuelle;
}
//---------------------------------------------------------------------------
void SignalVie() {
  Serial.println(F("Signal vie"));
  if (gsm) {
    MajHeure("");
    envoieGroupeSMS(0, 0);
    Sim800.delAllSms();// au cas ou, efface tous les SMS envoyé/reçu
  }
  CptAllumage = 0;
  action_wakeup_reason(4);
}
//---------------------------------------------------------------------------
void sauvConfig() { // sauve configuration en EEPROM
  EEPROM.begin(512);
  EEPROM.put(confign, config);
  EEPROM.commit();
  EEPROM.end();
}
//---------------------------------------------------------------------------
String displayTime(byte n) {
  // n = 0 ; dd/mm/yyyy hh:mm:ss
  // n = 1 ; yyyy-mm-dd hh:mm:ss
  char bid[20];
  if (n == 0) {
    sprintf(bid, "%02d/%02d/%4d %02d:%02d:%02d", day(), month(), year(), hour(), minute(), second());
  }
  else {
    sprintf(bid, "%4d-%02d-%02d %02d:%02d:%02d", year(), month(), day(), hour(), minute(), second());
  }
  return String(bid);
}
//---------------------------------------------------------------------------
void logRecord(String nom, String action) { // renseigne log et enregistre EEPROM
  static int index = 0;
  String temp;
  if (month() < 10) {
    temp =  "0" + String(month());
  }
  else {
    temp = String(month());
  }
  if (day() < 10 ) {
    temp += "0" + String(day());
  }
  else {
    temp += String(day());
  }
  if (hour() < 10) {
    temp += "-0" + String(hour());
  }
  else {
    temp += "-" + String(hour());
  }
  if (minute() < 10) {
    temp += "0" + String(minute());
  }
  else {
    temp += String(minute());
  }
  temp  .toCharArray(record[index].dt, 10);
  nom   .toCharArray(record[index].Name, 15);
  action.toCharArray(record[index].Act, 2);

  EEPROM.begin(512);
  EEPROM.put(recordn, record);// ecriture des valeurs par defaut
  EEPROM.commit();
  EEPROM.end();
  if (index < 4) {
    index ++;
  }
  else {
    index = 0;
  }
}
//---------------------------------------------------------------------------
void listDir(fs::FS &fs, const char * dirname, uint8_t levels) {
  Serial.printf("Listing directory: %s\r\n", dirname);

  File root = fs.open(dirname);
  if (!root) {
    Serial.println(F("- failed to open directory"));
    return;
  }
  if (!root.isDirectory()) {
    Serial.println(F(" - not a directory"));
    return;
  }

  File file = root.openNextFile();
  while (file) {
    if (file.isDirectory()) {
      Serial.print(F("  DIR : "));
      Serial.println(file.name());
      if (levels) {
        listDir(fs, file.name(), levels - 1);
      }
    } else {
      Serial.print(F("  FILE: "));
      Serial.print(file.name());
      Serial.print(F("\tSIZE: "));
      Serial.println(file.size());
    }
    file = root.openNextFile();
  }
  file.close();
}
//---------------------------------------------------------------------------
void readFile(fs::FS &fs, const char * path) {
  Serial.printf("Reading file: %s\r\n", path);

  File file = fs.open(path);
  if (!file || file.isDirectory()) {
    Serial.println(F("- failed to open file for reading"));
    return;
  }
  String buf = "";
  int i = 0;
  // Serial.println("- read from file:");
  while (file.available()) {
    int inchar = file.read();
    if (isDigit(inchar)) {
      buf += char(inchar);
      i ++;
    }
  }
  int m = 0;
  int j = 0;
  for (int i = 0; i < 372; i++) { // 12mois de 31 j =372
    j = 1 + (i % 31);
    if (j == 1) m ++;
    calendrier[m][j] = buf.substring(i, i + 1).toInt();
  }
}
//---------------------------------------------------------------------------
void appendFile(fs::FS &fs, const char * path, const char * message) {
  // Serial.printf("Appending to file: %s\r\n", path);

  File file = fs.open(path, FILE_APPEND);
  if (!file) {
    // Serial.println("- failed to open file for appending");
    return;
  }
  if (file.print(message)) {
    // Serial.println("- message appended");
  } else {
    // Serial.println("- append failed");
  }
}
//---------------------------------------------------------------------------
void MajLog(String Id, String Raison) { // mise à jour fichier log en SPIFFS
  if(SPIFFS.exists(filelog)){
    /* verification de la taille du fichier */
    File f = SPIFFS.open(filelog, "r");
    Serial.print(F("Taille fichier log = ")), Serial.println(f.size());
    // Serial.print(Id),Serial.print(","),Serial.println(Raison);
    if (f.size() > 150000 && !FileLogOnce) {
      /* si trop grand on efface */
      FileLogOnce = true;
      messageId();
      message += F("Fichier log presque plein\n");
      message += String(f.size());
      message += F("\nFichier sera efface a 300000");
      if (gsm) {
        String number = Sim800.getPhoneBookNumber(1); // envoyé au premier num seulement
        char num[13];
        number.toCharArray(num, 13);
        EnvoyerSms(num, true);
      }
    }
    else if (f.size() > 300000 && FileLogOnce) { // 292Ko 75000 lignes
      messageId();
      message += F("Fichier log plein\n");
      message += String(f.size());
      message += F("\nFichier efface");
      if (gsm) {
        String number = Sim800.getPhoneBookNumber(1); // envoyé au premier num seulement
        char num[13];
        number.toCharArray(num, 13);
        EnvoyerSms(num, true);
      }
      SPIFFS.remove(filelog);
      FileLogOnce = false;
    }
    f.close();
    /* preparation de la ligne */
    char Cbidon[101]; // 100 char maxi
    sprintf(Cbidon, "%02d/%02d/%4d %02d:%02d:%02d", day(), month(), year(), hour(), minute(), second());
    Id = ";" + Id + ";";
    Raison += "\n";
    strcat(Cbidon, Id.c_str());
    strcat(Cbidon, Raison.c_str());
    Serial.println(Cbidon);
    appendFile(SPIFFS, filelog, Cbidon);
  }
  else{
    char Cbidon[101]; // 100 char maxi
    sprintf(Cbidon, "%02d/%02d/%4d %02d:%02d:%02d;", day(), month(), year(), hour(), minute(), second());
    strcat(Cbidon,config.Idchar);
    strcat(Cbidon,fl.c_str());
    appendFile(SPIFFS, filelog, Cbidon);
    Serial.print("nouveau fichier log:"),Serial.println(Cbidon);
  }
}
//---------------------------------------------------------------------------
void EnregistreCalendrier() { // remplace le nouveau calendrier

  SPIFFS.remove(filecalendrier);
  Sbidon = "";
  char bid[63];
  for (int m = 1; m < 13; m++) {
    for (int j = 1; j < 32; j++) {
      Sbidon += calendrier[m][j];
      if (j < 31)Sbidon += char(59); // ;
    }
    Serial.println(Sbidon);
    Sbidon += fl;
    Sbidon.toCharArray(bid, 63);
    appendFile(SPIFFS, filecalendrier, bid);
    Sbidon = "";
  }

}
//---------------------------------------------------------------------------
void OuvrirCalendrier() {

  // this opens the file "f.txt" in read-mode
  listDir(SPIFFS, "/", 0);
  bool f = SPIFFS.exists(filecalendrier);
  // Serial.println(f);
  File f0 = SPIFFS.open(filecalendrier, "r");

  if (!f || f0.size() == 0) {
    Serial.println(F("File doesn't exist yet. Creating it")); // creation calendrier defaut
    char bid[63];
    Sbidon = "";
    for (int m = 1; m < 13; m++) {
      for (int j = 1; j < 32; j++) {
        if (m == 1 || m == 2 || m == 3 || m == 11 || m == 12) {
          Sbidon += "0;";
        }
        else {
          Sbidon += "1;";
        }
      }
      Serial.println(Sbidon);
      Sbidon += fl;
      Sbidon.toCharArray(bid, 63);
      appendFile(SPIFFS, filecalendrier, bid);
      Sbidon = "";
    }
  }
  readFile(SPIFFS, filecalendrier);

  for (int m = 1; m < 13; m++) {
    for (int j = 1; j < 32; j++) {
      Serial.print(calendrier[m][j]), Serial.print(char(44));
    }
    Serial.println();
  }
  listDir(SPIFFS, "/", 0);

}
//---------------------------------------------------------------------------
void FinJournee() {
  // fin de journée retour deep sleep
  Extinction();
  jour = false;
  flagCircule = false;
  FirstWakeup = true;
  Serial.println(F("Fin de journee retour sleep"));
  TIME_TO_SLEEP = DureeSleep(config.DebutJour - config.anticip);// xx mn avant
  Sbidon  = F("FinJour ");
  Sbidon += Hdectohhmm(TIME_TO_SLEEP);
  MajLog(F("Auto"), Sbidon);
  Sbidon  = F("nbr allum =");
  Sbidon += CptAllumage;
  MajLog(F("Auto"), Sbidon);
  DebutSleep();
}
//---------------------------------------------------------------------------
void PrintEEPROM() {
  Serial.print(F("Version = "))                 , Serial.println(ver);
  Serial.print(F("ID = "))                      , Serial.println(config.Idchar);
  Serial.print(F("magic = "))                   , Serial.println(config.magic);
  Serial.print(F("Alarme = "))                  , Serial.println(config.Intru);
  Serial.print(F("Debut Jour = "))              , Serial.println(config.DebutJour);
  Serial.print(F("Fin jour = "))                , Serial.println(config.FinJour);
  Serial.print(F("T anticipation Wakeup = "))   , Serial.println(config.anticip);
  Serial.print(F("Tempo repetition Wake up (s)= ")), Serial.println(config.RepeatWakeUp);
  Serial.print(F("Tempo Analyse apres Wake up (s)= ")) , Serial.println(config.Tanalyse);
  Serial.print(F("TimeOut Alarme Jour (s)= "))  , Serial.println(config.Jour_Nmax * 10);
  Serial.print(F("TimeOut Alarme Nuit (s)= "))  , Serial.println(config.Nuit_Nmax * 10);
  Serial.print(F("Tempo Sortie (s)= "))         , Serial.println(config.tempoSortie);
  Serial.print(F("Time Out Eclairage (s)= "))   , Serial.println(config.timeOutS);
  Serial.print(F("Time Out Wifi (s)= "))        , Serial.println(config.timeoutWifi);
  Serial.print(F("Alarme sur Pedale 1 = "))     , Serial.println(config.Pedale1);
  Serial.print(F("Alarme sur Pedale 2 = "))     , Serial.println(config.Pedale2);
  Serial.print(F("Alarme sur Coffret = "))      , Serial.println(config.Coffret);
  Serial.print(F("Liste Restreinte = "));
  for (int i = 1; i < 10; i++) {
    Serial.print(config.Pos_Pn_PB[i]);
    if (i == 9) {
      Serial.println();
    }
    else {
      Serial.print(F(","));
    }
  }
  Serial.print(F("GPRS APN = ")), Serial.println(config.apn);
  Serial.print(F("GPRS user = ")), Serial.println(config.gprsUser);
  Serial.print(F("GPRS pass = ")), Serial.println(config.gprsPass);
  Serial.print(F("ftp serveur = ")), Serial.println(config.ftpServeur);
  Serial.print(F("ftp port = ")), Serial.println(config.ftpPort);
  Serial.print(F("ftp user = ")), Serial.println(config.ftpUser);
  Serial.print(F("ftp pass = ")), Serial.println(config.ftpPass);
  Serial.print(F("GPRS APN = ")), Serial.println(config.apn);
  Serial.print(F("GPRS user = ")), Serial.println(config.gprsUser);
  Serial.print(F("GPRS pass = ")), Serial.println(config.gprsPass);
  Serial.print(F("ftp serveur = ")), Serial.println(config.ftpServeur);
  Serial.print(F("ftp port = ")), Serial.println(config.ftpPort);
  Serial.print(F("ftp user = ")), Serial.println(config.ftpUser);
  Serial.print(F("ftp pass = ")), Serial.println(config.ftpPass);
}
//---------------------------------------------------------------------------
void Extinction() {
  Allumage(0);
  Alarm.disable(TempoSortie);
  Alarm.disable(TimeOut);
  Alarm.write(TimeOut,config.timeOutS);// retablir timeout memorisé
  // TimeOut = Alarm.timerRepeat(config.timeOutS, Extinction);// retablir timeout memorisé
  Alarm.disable(TimeOut);
}
//---------------------------------------------------------------------------
void Allumage(byte n) {
  /*
  	n=0  extinction auto
  	n=1  Commande depuis pedale 1
  	n=2  Commande depuis pedale 2
    n=10 Commande allumage forcé
  */
  static bool AllumageForcee = false;
  if(n==10) AllumageForcee = true;
  if(n==0) AllumageForcee = false;
  static byte Al1 = 0;
  static byte Al2 = 0;

  // correction extinction intempestive si cde seconde pedale arrive juste apres premiere
  // a tester
  static unsigned long t = millis();
  if((Al1 == 1 && n == 2) || (Al2 == 1 && n == 1)){
    // allumage depuis pedale, armement du timer
    if(millis()- t < 5000){
      // si armement consecutif < 5s on sort
      Serial.println("Protection pedale <5s");
      return;
    }
  }
  t = millis();

  if (jour || n == 10) {
    byte Cd1 = 0;
    byte Cd2 = 0;
    switch (n) {
      case 0:
        Al1 = 0;
        Al2 = 0;
        break;
      case 1:
        Cd1 = 1;
        break;
      case 2:
        Cd2 = 1;
        break;
    }

    // Serial.print(F("Sub Allumage avec n = ")),Serial.print(n);
    // Serial.print(F(" Al1,Al2 = ")),Serial.print(Al1),Serial.print(char(44)),Serial.println(Al2);

    if (!Allume && n!=0) {	// si pas Allumé
      Serial.println(F("Allumage"));
      if (!FlagMasterOff) digitalWrite(PinEclairage, HIGH);
      Allume = true;
      CptAllumage ++;
      if (n == 1)Al1 = 1;
      if (n == 2)Al2 = 1;
      Alarm.enable(TimeOut);
      Sbidon  = F("Allumage ");
      Sbidon += n;
      MajLog(F("Auto"), Sbidon);
    }
    else {	// si Allumé
      if(!AllumageForcee){
        if (n == 0) {
          digitalWrite(PinEclairage, LOW);
          Allume = false;
          Sbidon  = F("Extinction ");
          Sbidon += n;
          MajLog(F("Auto"), Sbidon);
        }
        else if (Al1 == Cd2 || Al2 == Cd1) {
          Serial.print(F("Extinction dans (s) ")), Serial.println(config.tempoSortie);
          Alarm.disable(TempoSortie);
          Alarm.enable(TempoSortie);
          Serial.print(F("Nombre Allumage = ")), Serial.println(CptAllumage);
        }
      }
    }
  }
}
//---------------------------------------------------------------------------
void ConnexionWifi(char* ssid, char* pwd, char* number, bool sms) {

  messageId();
  Serial.print(F("connexion Wifi:")), Serial.print(ssid), Serial.print(char(44)), Serial.println(pwd);
  String ip;
  WiFi.begin(ssid, pwd);
  // WiFi.mode(WIFI_STA);
  byte timeout = 0;
  bool error = false;

  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.print(".");
    timeout ++;
    if (timeout > 60) {
      error = true;
      break;
    }
  }
  if (!error) {
    Serial.println();
    Serial.println(F("WiFi connected"));
    Serial.print(F("IP address: "));
    ip = WiFi.localIP().toString();
    Serial.println(ip);
    ArduinoOTA.begin();

    server.on("/",         HomePage);
    server.on("/download", File_Download);
    server.on("/upload",   File_Upload);
    server.on("/fupload",  HTTP_POST, []() {
      server.send(200);
    }, handleFileUpload);
    server.on("/delete",   File_Delete);
    server.on("/dir",      SPIFFS_dir);
    server.on("/cal",      CalendarPage);
    server.on("/Tel_list", Tel_listPage);
    server.on("/timeremaining", handleTime); // renvoie temps restant sur demande
    server.on("/datetime", handleDateTime); // renvoie Date et Heure
    server.on("/wifioff",  WifiOff);
    ///////////////////////////// End of Request commands
    server.begin();
    Serial.println(F("HTTP server started"));

    message += F("Connexion Wifi : ");
    message += fl;
    message += String(ip);
    message += fl;
    message += String(WiFi.RSSI());
    message += F(" dBm");
    message += fl;
    message += F("TimeOut Wifi ");
    message += config.timeoutWifi;
    message += " s";
  }
  else {
    message += F("Connexion Wifi impossible");
  }
  EnvoyerSms(number, sms);

  // if (sms) { // suppression du SMS
  //   /* Obligatoire ici si non bouclage au redemarrage apres timeoutwifi
  //     ou OTA sms demande Wifi toujours present */
  //   EffaceSMS(slot);
  // }
  debut = millis();
  if (!error) {
    /* boucle permettant de faire une mise à jour OTA et serveur, avec un timeout en cas de blocage */
    unsigned long timeout = millis();
    while (millis() - timeout < config.timeoutWifi * 1000) {
      // if(WiFi.status() != WL_CONNECTED) break; // wifi a été coupé on sort
      ArduinoOTA.handle();
      server.handleClient(); // Listen for client connections
      delay(1);
    }
    WifiOff();
  }
}
//---------------------------------------------------------------------------
void WifiOff() {
  Serial.println(F("Wifi off"));
  WiFi.disconnect(true);
  WiFi.mode(WIFI_OFF);
  WiFi.mode(WIFI_MODE_NULL);
  btStop();
  Alarm.delay(100);
  ResetHard();
}
//---------------------------------------------------------------------------
void ResetHard() {
  // GPIO13 to RS reset hard
  pinMode(PinReset, OUTPUT);
  digitalWrite(PinReset, LOW);
}
//---------------------------------------------------------------------------
String ExtraireSms(String msgbrut) { //Extraction du contenu du SMS

  int pos[10];									// SMS jusqu'a 5 lignes
  int i = 0;
  for (i = 0; i < 10; i++) {
    if (i == 0) {
      pos[i] = msgbrut.indexOf("\n");
    }
    else {
      pos[i] = msgbrut.indexOf("\n", pos[i - 1] + 1);
    }
    // Serial.print(i),Serial.print(" pos = "),Serial.println(pos[i]);
    if (pos[i] == -1) {
      i --;
      break;
    }
  }

  String message = msgbrut.substring(pos[1] + 1, pos[i - 1] - 1);
  // Serial.print("message extrait = "),Serial.println(message);
  message.replace("\n", "|");				// remplacement des sauts de lignes par |
  message = message.substring(0, message.length() - 2);
  // Serial.print("message extrait sans \n= "),Serial.println(message);

  return message;
}
//---------------------------------------------------------------------------
int moyenneAnalogique(int Pin) {	// calcul moyenne 10 mesures consécutives
  int moyenne = 0;
  for (int j = 0; j < 10; j++) {
    // Alarm.delay(1);
    moyenne += analogRead(Pin);
  }
  moyenne /= 10;
  return moyenne;
}
//---------------------------------------------------------------------------
void OuvrirFichierCalibration() { // Lecture fichier calibration

  if (SPIFFS.exists(filecalibration)) {
    File f = SPIFFS.open(filecalibration, "r");
    for (int i = 0; i < 4; i++) { //Read
      String s = f.readStringUntil('\n');
      CoeffTension[i] = s.toFloat();
    }
    f.close();
  }
  else {
    Serial.print(F("Creating Data File:")), Serial.println(filecalibration); // valeur par defaut
    CoeffTension[0] = CoeffTensionDefaut;
    CoeffTension[1] = CoeffTensionDefaut;
    CoeffTension[2] = CoeffTensionDefaut;
    CoeffTension[3] = CoeffTensionDefaut;
    Recordcalib();
  }
  Serial.print(F("Coeff T Batterie = ")), Serial.print(CoeffTension[0]);
  Serial.print(F(" Coeff T Proc = "))	  , Serial.print(CoeffTension[1]);
  Serial.print(F(" Coeff T VUSB = "))		, Serial.print(CoeffTension[2]);
  Serial.print(F(" Coeff T 24V = "))		, Serial.println(CoeffTension[3]);

}
//---------------------------------------------------------------------------
void Recordcalib() { // enregistrer fichier calibration en SPIFFS

  // Serial.print(F("Coeff T Batterie = ")),Serial.println(CoeffTension1);
  // Serial.print(F("Coeff T Proc = "))	  ,Serial.println(CoeffTension2);
  // Serial.print(F("Coeff T VUSB = "))		,Serial.println(CoeffTension3);
  File f = SPIFFS.open(filecalibration, "w");
  f.println(CoeffTension[0]);
  f.println(CoeffTension[1]);
  f.println(CoeffTension[2]);
  f.println(CoeffTension[3]);
  f.close();

}
//---------------------------------------------------------------------------
String Hdectohhmm(long Hdec) {
  String hhmm;
  if (int(Hdec / 3600) < 10) hhmm = "0";
  hhmm += int(Hdec / 3600);
  hhmm += ":";
  if (int((Hdec % 3600) / 60) < 10) hhmm += "0";
  hhmm += int((Hdec % 3600) / 60);
  hhmm += ":";
  if (int((Hdec % 3600) % 60) < 10) hhmm += "0";
  hhmm += int((Hdec % 3600) % 60);
  return hhmm;
}
//---------------------------------------------------------------------------
void DesActiveInterrupt() {

  if (config.Pedale1) {
    //detachInterrupt(digitalPinToInterrupt(PinPedale1));
  }
  if (config.Pedale2) {
    //detachInterrupt(digitalPinToInterrupt(PinPedale2));
  }
  if (config.Coffret) {
    detachInterrupt(digitalPinToInterrupt(PinCoffret));
  }
}
//---------------------------------------------------------------------------
void ActiveInterrupt() {

  // if (config.Pedale1) {
    attachInterrupt(digitalPinToInterrupt(PinPedale1), handleInterruptP1, RISING);
  // }
  // if (config.Pedale2) {
    attachInterrupt(digitalPinToInterrupt(PinPedale2), handleInterruptP2, RISING);
  // }
  if (config.Coffret) {
    attachInterrupt(digitalPinToInterrupt(PinCoffret), handleInterruptPo, RISING);
  }
}
//---------------------------------------------------------------------------
void AIntru_HeureActuelle() {

  long Heureactuelle = HActuelledec();

  if (config.FinJour > config.DebutJour) {
    if ((Heureactuelle > config.FinJour && Heureactuelle > config.DebutJour)
        || (Heureactuelle < config.FinJour && Heureactuelle < config.DebutJour)) {
      // Nuit
      IntruD();
    }
    else {	// Jour
      IntruF();
    }
  }
  else {
    if (Heureactuelle > config.FinJour && Heureactuelle < config.DebutJour) {
      // Nuit
      IntruD();
    }
    else {	// Jour
      IntruF();
    }
  }
}
//---------------------------------------------------------------------------
void IntruF() { // Charge parametre Alarme Intrusion Jour
  Nmax = config.Jour_Nmax;
  jour = true;
  // Serial.println(F("Jour"));
}
//---------------------------------------------------------------------------
void IntruD() { // Charge parametre Alarme Intrusion Nuit
  Nmax = config.Nuit_Nmax;
  jour = false;
  // Serial.println(F("Nuit"));
}
//---------------------------------------------------------------------------
void DebutSleep() {
  ArretSonnerie();
  if (Allume)Extinction();
  // selection du pin mask en fonction des capteurs actif
  const uint64_t ext_wakeup_pin_1_mask = 1ULL << PinPedale1;
  const uint64_t ext_wakeup_pin_2_mask = 1ULL << PinPedale2;
  const uint64_t ext_wakeup_pin_3_mask = 1ULL << PinCoffret;
  if (config.Intru) {
    if (config.Coffret && config.Pedale1 && config.Pedale2) {
      esp_sleep_enable_ext1_wakeup(ext_wakeup_pin_1_mask  | ext_wakeup_pin_2_mask | ext_wakeup_pin_3_mask, ESP_EXT1_WAKEUP_ANY_HIGH);
    } else if (config.Coffret && config.Pedale1) {
      esp_sleep_enable_ext1_wakeup(ext_wakeup_pin_1_mask  | ext_wakeup_pin_3_mask, ESP_EXT1_WAKEUP_ANY_HIGH);
    } else if (config.Coffret && config.Pedale2) {
      esp_sleep_enable_ext1_wakeup(ext_wakeup_pin_2_mask | ext_wakeup_pin_3_mask, ESP_EXT1_WAKEUP_ANY_HIGH);
    } else if (config.Pedale1 && config.Pedale2) {
      esp_sleep_enable_ext1_wakeup(ext_wakeup_pin_1_mask  | ext_wakeup_pin_2_mask, ESP_EXT1_WAKEUP_ANY_HIGH);
    } else if (config.Coffret) {
      esp_sleep_enable_ext1_wakeup(ext_wakeup_pin_3_mask, ESP_EXT1_WAKEUP_ANY_HIGH);
    } else if (config.Pedale1) {
      esp_sleep_enable_ext1_wakeup(ext_wakeup_pin_1_mask, ESP_EXT1_WAKEUP_ANY_HIGH);
    } else if (config.Pedale2) {
      esp_sleep_enable_ext1_wakeup(ext_wakeup_pin_2_mask, ESP_EXT1_WAKEUP_ANY_HIGH);
    }
  }
  else {
    /* pas de wakeup sur pin */
    Serial.println(F("pas de pin selectionne pour wakeup!"));
  }


  esp_sleep_enable_timer_wakeup(TIME_TO_SLEEP * uS_TO_S_FACTOR);
  Serial.print(F("Setup ESP32 to sleep for "));
  print_uint64_t(TIME_TO_SLEEP);
  Serial.print(F("s ;"));
  Serial.println(Hdectohhmm(TIME_TO_SLEEP));
  Serial.flush();
  //Go to sleep now
  Serial.println(F("Going to sleep now"));

  byte i = 0;
  if (gsm) {
    while (!Sim800.sleep()) {
      Alarm.delay(100);
      if (i++ > 10) break;
    }
  }
  // delay(1000);
  Serial.flush();
  esp_deep_sleep_start();
  delay(100);

  Serial.println(F("This will never be printed"));
  Serial.flush();

}
//---------------------------------------------------------------------------
void action_wakeup_reason(byte wr) { // action en fonction du wake up
  Serial.print(F("Wakeup :")), Serial.print(wr);
  Serial.print(F(", jour :")), Serial.print(jour);
  Serial.print(F(" ,Calendrier :")), Serial.print(calendrier[month()][day()]);
  Serial.print(F(" ,flagCircule :")), Serial.println(flagCircule);
  byte pin = 0;
  Serial.println(F("***********************************"));
  if (wr == 99 || wr == 32 || wr == 33 || wr == 34) {
    pin = wr;
    wr = 3;
  }
  if (wr == 0)wr = 4; // demarrage normal, decision idem timer

  switch (wr) {
    case 2: break; // ne rien faire ESP_SLEEP_WAKEUP_EXT0

    case 3: // ESP_SLEEP_WAKEUP_EXT1

      /* declenchement externe pendant deep sleep
      	si nuit ou jour noncirculé
      	on reste en fonctionnement pendant TempoAnalyse
      	avant retour deep sleep*/
      WupAlarme = true;
      LastWupAlarme = true;
      Alarm.enable(TempoAnalyse); // debut tempo analyse ->fonctionnement normal
      Sbidon = F("Externe Debut ");
      Sbidon += String(pin);
      MajLog(F("Alarme"), Sbidon);
      // }
      break;

    case 4: // SP_SLEEP_WAKEUP_TIMER
      /* jour noncirculé retour deep sleep pour RepeatWakeUp 1H00
        verifier si wake up arrive avant fin journée marge 1mn*/
      if (FirstWakeup) { // premier wake up du jour avant DebutJour
        // SignalVie();
        // ne rien faire, attendre DebutJour
        FirstWakeup = false;
        if (HActuelledec() > config.DebutJour) {
          // premier lancement en journée
          SignalVie();
        }
        break;
      }
      if ((calendrier[month()][day()] ^ flagCircule) && jour) { // jour circulé & jour
        // Nmax = config.Jour_Nmax; // parametre jour
        Sbidon = F("Jour circule ou demande circulation");
        Serial.println(Sbidon);
        MajLog(F("Auto"), Sbidon);
      }
      else { // non circulé
        Sbidon = F("Jour noncircule");
        Serial.println(Sbidon);
        MajLog(F("Auto"), Sbidon);
        // Nmax = config.Nuit_Nmax; // parametre nuit
        calculTimeSleep();
        if (TIME_TO_SLEEP <= config.anticip) { // on continue sans sleep
          Sbidon = F("on continue sans sleep");
          Serial.println(Sbidon);
          MajLog(F("Auto"), Sbidon);
        }
        else {
          DebutSleep();
        }
      }
      break;

    case 5: break;  // ne rien faire ESP_SLEEP_WAKEUP_TOUCHPAD
    case 6: break;  // ne rien faire ESP_SLEEP_WAKEUP_ULP
      // default: break; // demarrage normal
  }
}
//---------------------------------------------------------------------------
void calculTimeSleep() {

  AIntru_HeureActuelle(); // determine si jour/nuit

  if (jour && (HActuelledec() + config.RepeatWakeUp) > config.FinJour) {
    if (HActuelledec() > (config.FinJour - config.anticip)) {
      /* eviter de reporter 24H si on est à moins de anticip de FinJour */
      TIME_TO_SLEEP = 1; // si 1 pas de sleep
    }
    else {
      TIME_TO_SLEEP = DureeSleep(config.FinJour - config.anticip);
      Serial.print(F("time sleep calcul 1 : ")), print_uint64_t(TIME_TO_SLEEP);
      Serial.println("");
    }
  }
  else if (!jour) {
    if (HActuelledec() < (config.DebutJour - config.anticip)) {
      TIME_TO_SLEEP = DureeSleep(config.DebutJour - config.anticip);
      Serial.print(F("time sleep calcul 2 : ")), print_uint64_t(TIME_TO_SLEEP);
      Serial.println("");
    }
    else if (HActuelledec() < 86400) {
      TIME_TO_SLEEP = (86400 - HActuelledec()) + config.DebutJour - config.anticip;
      Serial.print(F("time sleep calcul 2bis : ")), print_uint64_t(TIME_TO_SLEEP);
      Serial.println("");
    }
  }
  else {
    TIME_TO_SLEEP = config.RepeatWakeUp;
    Serial.print(F("time sleep calcul 3 : ")), print_uint64_t(TIME_TO_SLEEP);
    Serial.println("");
  }

  /* Garde fou si TIME_TO_SLEEP > 20H00 c'est une erreur, on impose 1H00 */
  if (TIME_TO_SLEEP > 72000) {
    TIME_TO_SLEEP = 3600;
    Sbidon = F("jour ");
    Sbidon += jour;
    Sbidon = F(", Calendrier ");
    Sbidon += calendrier[month()][day()];
    Sbidon = F(", flagCirc ");
    Sbidon += flagCircule;
    MajLog(F("Auto"), Sbidon);
    Sbidon = F("Attention erreur Sleep>20H00 ");
    Sbidon += Hdectohhmm(TIME_TO_SLEEP);
    MajLog(F("Auto"), Sbidon);
  }

  Sbidon = F("lance timer : ");
  Sbidon += Hdectohhmm(TIME_TO_SLEEP);
  MajLog(F("Auto"), Sbidon);
}
//---------------------------------------------------------------------------
int get_wakeup_reason() {
  esp_sleep_wakeup_cause_t wakeup_reason;

  wakeup_reason = esp_sleep_get_wakeup_cause();
  uint64_t wakeup_pin_mask;
  switch (wakeup_reason) {
    case ESP_SLEEP_WAKEUP_EXT0  : return ESP_SLEEP_WAKEUP_EXT0; // 2
    case ESP_SLEEP_WAKEUP_EXT1: //{// 3
      wakeup_pin_mask = esp_sleep_get_ext1_wakeup_status();
      if (wakeup_pin_mask != 0) {
        int pin = __builtin_ffsll(wakeup_pin_mask) - 1;
        Serial.print(F("Wake up from GPIO ")); Serial.println(String(pin));
        return pin; // pin
      } else {
        Serial.println(F(" Wake up from GPIO ?"));
        return 99; // 99
      }
      break;
    // }
    case ESP_SLEEP_WAKEUP_TIMER    : return ESP_SLEEP_WAKEUP_TIMER; // 4
    case ESP_SLEEP_WAKEUP_TOUCHPAD : return ESP_SLEEP_WAKEUP_TOUCHPAD; // 5
    case ESP_SLEEP_WAKEUP_ULP      : return ESP_SLEEP_WAKEUP_ULP; // 6
    default : return 0; // Serial.println("Wakeup was not caused by deep sleep"); break;// demarrage normal
  }
  Serial.flush();
}

//---------------------------------------------------------------------------
void EffaceSMS(int s) {
  bool err;
  byte n = 0;
  do {
    err = Sim800.delSms(s);
    n ++;
    Serial.print(F("resultat del Sms "));	Serial.println(err);
    if (n > 10) { // on efface tous si echec
      err = Sim800.delAllSms();
      Serial.print(F("resultat delall Sms "));	Serial.println(err);
      break;
    }
  } while (!err);
}
//---------------------------------------------------------------------------
void print_uint64_t(uint64_t num) {

  char rev[128];
  char *p = rev + 1;

  while (num > 0) {
    *p++ = '0' + ( num % 10);
    num /= 10;
  }
  p--;
  /*Print the number which is now in reverse*/
  while (p > rev) {
    Serial.print(*p--);
  }
}
//---------------------------------------------------------------------------
void init_adc_mm(void) {
  //initialisation des tableaux
  /* valeur par defaut facultative,
  	permet d'avoir une moyenne proche
  	du resulat plus rapidement
  	val defaut = valdefaut*nSample */
  unsigned int ini_adc1 = 0;// val defaut adc 1
  unsigned int ini_adc2 = 0;// val defaut adc 2
  unsigned int ini_adc3 = 0;// val defaut adc 3
  unsigned int ini_adc4 = 0;// val defaut adc 4
  for (int plus_ancien = 0; plus_ancien < nSample; plus_ancien++) {
    adc_hist[0][plus_ancien] = ini_adc1;
    adc_hist[1][plus_ancien] = ini_adc2;
    adc_hist[2][plus_ancien] = ini_adc3;
    adc_hist[3][plus_ancien] = ini_adc4;
  }
  //on commencera à stocker à cet offset
  adc_mm[0] = ini_adc1;
  adc_mm[1] = ini_adc2;
  adc_mm[2] = ini_adc3;
  adc_mm[3] = ini_adc4;
}
//---------------------------------------------------------------------------
void read_adc(int pin1, int pin2, int pin3, int pin4) {
  // http://www.f4grx.net/algo-comment-calculer-une-moyenne-glissante-sur-un-microcontroleur-a-faibles-ressources/
  static int plus_ancien = 0;
  //acquisition
  int sample[4];
  for (byte i = 0; i < 4; i++) {
    if (i == 0)sample[i] = moyenneAnalogique(pin1);
    if (i == 1)sample[i] = moyenneAnalogique(pin2);
    if (i == 2)sample[i] = moyenneAnalogique(pin3);
    if (i == 3)sample[i] = moyenneAnalogique(pin4);

    //calcul MoyenneMobile
    adc_mm[i] = adc_mm[i] + sample[i] - adc_hist[i][plus_ancien];

    //cette plus ancienne valeur n'est plus utile, on y stocke la plus récente
    adc_hist[i][plus_ancien] = sample[i];
  }
  plus_ancien ++;
  if (plus_ancien == nSample) { //gestion du buffer circulaire
    plus_ancien = 0;
  }
}
//--------------------------------------------------------------------------------//
void messageId() {
  message  = Id;
  message += displayTime(0);
  message += fl;
}
//---------------------------------------------------------------------------
void HomePage() {
  SendHTML_Header();
  webpage += F("<h3 class='rcorners_m'>Parametres</h3><br>");
  webpage += F("<table align='center'>");
  webpage += F("<tr>");
  webpage += F("<td>Version</td>");
  webpage += F("<td>");	webpage += ver;	webpage += F("</td>");
  webpage += F("</tr>");

  webpage += F("<tr>");
  webpage += F("<td>Id</td>");
  webpage += F("<td>");	webpage += String(config.Idchar);	webpage += F("</td>");
  webpage += F("</tr>");

  webpage += F("<tr>");
  webpage += F("<td>Debut Jour</td>");
  webpage += F("<td>");	webpage += Hdectohhmm(config.DebutJour);	webpage += F("</td>");
  webpage += F("</tr>");

  webpage += F("<tr>");
  webpage += F("<td>Anticipation WakeUp (s)</td>");
  webpage += F("<td>");	webpage += String(config.anticip);	webpage += F("</td>");
  webpage += F("</tr>");

  webpage += F("<tr>");
  webpage += F("<td>Fin Jour</td>");
  webpage += F("<td>");	webpage += Hdectohhmm(config.FinJour);	webpage += F("</td>");
  webpage += F("</tr>");

  webpage += F("<tr>");
  webpage += F("<td>Nmax Jour (s)</td>");
  webpage += F("<td>");	webpage += String(config.Jour_Nmax * 10);	webpage += F("</td>");
  webpage += F("</tr>");

  webpage += F("<tr>");
  webpage += F("<td>Nmax Nuit (s)</td>");
  webpage += F("<td>");	webpage += String(config.Nuit_Nmax * 10);	webpage += F("</td>");
  webpage += F("</tr>");
  webpage += F("<tr>");
  webpage += F("<td>Tempo Analyse apr&egrave;s Wake up (s)</td>");
  webpage += F("<td>");	webpage += String(config.Tanalyse);	webpage += F("</td>");
  webpage += F("</tr>");

  webpage += F("<tr>");
  webpage += F("<td>Tempo r&eacute;p&eacute;tition Wake up Jour Circul&eacute; (s)</td>");
  webpage += F("<td>");	webpage += String(config.RepeatWakeUp);	webpage += F("</td>");
  webpage += F("</tr>");

  webpage += F("<tr>");
  webpage += F("<td>Tempo Sortie (s)</td>");
  webpage += F("<td>");	webpage += String(config.tempoSortie);	webpage += F("</td>");
  webpage += F("</tr>");

  webpage += F("<tr>");
  webpage += F("<td>TimeOut Eclairage (s)</td>");
  webpage += F("<td>");	webpage += String(config.timeOutS);	webpage += F("</td>");
  webpage += F("</tr>");

  webpage += F("<tr>");
  webpage += F("<td>TimeOut Wifi (s)</td>");
  webpage += F("<td>");	webpage += String(config.timeoutWifi);	webpage += F("</td>");
  webpage += F("</tr>");

  webpage += F("<tr>");
  webpage += F("<td>Alarme</td>");
  webpage += F("<td>");
  if (config.Intru) {
    webpage += F("Active");
  } else {
    webpage += F("Inactive");
  } webpage += F("</td>");
  webpage += F("</tr>");

  webpage += F("<tr>");
  webpage += F("<td>Silence</td>");
  webpage += F("<td>");
  if (config.Silence) {
    webpage += F("ON");
  } else {
    webpage += F("OFF");
  } webpage += F("</td>");
  webpage += F("</tr>");

  webpage += F("<tr>");
  webpage += F("<td>Alarme sur Pedale 1</td>");
  webpage += F("<td>");
  if (config.Pedale1) {
    webpage += F("Active");
  } else {
    webpage += F("Inactive");
  } webpage += F("</td>");
  webpage += F("</tr>");

  webpage += F("<tr>");
  webpage += F("<td>Alarme sur Pedale 2</td>");
  webpage += F("<td>");
  if (config.Pedale2) {
    webpage += F("Active");
  } else {
    webpage += F("Inactive");
  } webpage += F("</td>");
  webpage += F("</tr>");

  webpage += F("<tr>");
  webpage += F("<td>Alarme sur Coffret</td>");
  webpage += F("<td>");
  if (config.Coffret) {
    webpage += F("Active");
  } else {
    webpage += F("Inactive");
  } webpage += F("</td>");
  webpage += F("</tr>");

  webpage += F("<tr>");
  webpage += F("<td>Liste Restreinte</td>");
  webpage += F("<td>");
  for (int i = 1; i < 10; i++) {
    webpage += String(config.Pos_Pn_PB[i]);
    if (i < 9) {
      webpage += (F(","));
    }
  }
  webpage += F("</td>");
  webpage += F("</tr>");

  webpage += F("<tr>");
  webpage += F("<td>GPRS APN</td>");
  webpage += F("<td>");	webpage += String(config.apn);	webpage += F("</td>");
  webpage += F("</tr>");

  webpage += F("<tr>");
  webpage += F("<td>GPRS user</td>");
  webpage += F("<td>");	webpage += String(config.gprsUser);	webpage += F("</td>");
  webpage += F("</tr>");

  webpage += F("<tr>");
  webpage += F("<td>GPRS pass</td>");
  webpage += F("<td>");	webpage += String(config.gprsPass);	webpage += F("</td>");
  webpage += F("</tr>");

  webpage += F("<tr>");
  webpage += F("<td>ftp Serveur</td>");
  webpage += F("<td>");	webpage += String(config.ftpServeur);	webpage += F("</td>");
  webpage += F("</tr>");

  webpage += F("<tr>");
  webpage += F("<td>ftp Port</td>");
  webpage += F("<td>");	webpage += String(config.ftpPort);	webpage += F("</td>");
  webpage += F("</tr>");

  webpage += F("<tr>");
  webpage += F("<td>ftp User</td>");
  webpage += F("<td>");	webpage += String(config.ftpUser);	webpage += F("</td>");
  webpage += F("</tr>");

  webpage += F("<tr>");
  webpage += F("<td>ftp Pass</td>");
  webpage += F("<td>");	webpage += String(config.ftpPass);	webpage += F("</td>");
  webpage += F("</tr>");

  webpage += F("</table><br>");

  webpage += F("<a href='/download'><button>Download</button></a>");
  webpage += F("<a href='/upload'><button>Upload</button></a>");
  webpage += F("<a href='/delete'><button>Delete</button></a>");
  webpage += F("<a href='/dir'><button>Directory</button></a>");
  webpage += F("<a href='/Tel_list'><button>Tel_list</button></a>");
  webpage += F("<a href='/cal'><button>Calendar</button></a>");
  webpage += F("<a href='/wifioff'><button>Wifi Off</button></a>");
  append_page_footer();
  SendHTML_Content();
  SendHTML_Stop(); // Stop is needed because no content length was sent
}
//---------------------------------------------------------------------------
void Tel_listPage() {
  SendHTML_Header();
  webpage += F("<h3 class='rcorners_m'>Liste des num&eacute;ros t&eacute;l&eacute;phone</h3><br>");
  webpage += F("<table align='center'>");
  webpage += F("<tr>");
  webpage += F("<th> Nom </th>");
  webpage += F("<th> Num&eacute;ro </th>");
  webpage += F("<th> Liste restreinte </th>");
  webpage += F("</tr>");
  if (gsm){
    byte n = Sim800.ListPhoneBook(); // nombre de ligne PhoneBook
    for (byte i = 1; i < n + 1; i++) {
      String num = Sim800.getPhoneBookNumber(i);
      // Serial.print(num.length()), Serial.print(" "), Serial.println(num);
      if (num.indexOf("+CPBR:") == 0) { // si existe pas sortir
        Serial.println(F("Failed!"));// next i
        goto fin_liste;
      }
      String name = Sim800.getPhoneBookName(i);
      // Serial.println(name);
      webpage += F("<tr>");
      webpage += F("<td>"); webpage += name; webpage += F("</td>");
      webpage += F("<td>"); webpage += num ; webpage += F("</td>");
      webpage += F("<td>"); webpage += String(config.Pos_Pn_PB[i]); webpage += F("</td>");
      webpage += F("</tr>");
    }
  }
fin_liste:


  webpage += F("</table><br>");
  append_page_footer();
  SendHTML_Content();
  SendHTML_Stop(); // Stop is needed because no content length was sent
}
//---------------------------------------------------------------------------
void CalendarPage() {
  SendHTML_Header();
  webpage += F("<h3 class='rcorners_m'>Calendrier</h3><br>");
  webpage += F("<table align='center'>");

  for (int m = 1; m < 13; m ++) {
    webpage += F("<tr>");
    webpage += F("<td>"); webpage += Mois[m]; webpage += F("</td>");
    for (int j = 1; j < 32; j++) {
      webpage += F("<td>");	webpage += calendrier[m][j];	webpage += F("</td>");
      if (j % 5 == 0)webpage += F("<td> </td>");
    }
    webpage += F("</tr>");
  }

  webpage += F("</table><br>");
  append_page_footer();
  SendHTML_Content();
  SendHTML_Stop(); // Stop is needed because no content length was sent
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
void File_Download() { // This gets called twice, the first pass selects the input, the second pass then processes the command line arguments
  if (server.args() > 0 ) { // Arguments were received
    if (server.hasArg("download")) DownloadFile(server.arg(0));
  }
  else SelectInput("Enter filename to download", "download", "download");
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
void DownloadFile(String filename) {
  if (SPIFFS_present) {
    File download = SPIFFS.open("/" + filename,  "r");
    if (download) {
      server.sendHeader("Content-Type", "text/text");
      server.sendHeader("Content-Disposition", "attachment; filename=" + filename);
      server.sendHeader("Connection", "close");
      server.streamFile(download, "application/octet-stream");
      download.close();
    } else ReportFileNotPresent("download");
  } else ReportSPIFFSNotPresent();
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
void File_Upload() {
  append_page_header();
  webpage += F("<h3>Select File to Upload</h3>");
  webpage += F("<FORM action='/fupload' method='post' enctype='multipart/form-data'>");
  webpage += F("<input class='buttons' style='width:40%' type='file' name='fupload' id = 'fupload' value=''><br>");
  webpage += F("<br><button class='buttons' style='width:10%' type='submit'>Upload File</button><br>");
  webpage += F("<a href='/'>[Back]</a><br><br>");
  append_page_footer();
  server.send(200, "text/html", webpage);
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
void handleFileUpload() { // upload a new file to the Filing system
  HTTPUpload& uploadfile = server.upload(); // See https://github.com/esp8266/Arduino/tree/master/libraries/ESP8266WebServer/srcv
  // For further information on 'status' structure, there are other reasons such as a failed transfer that could be used
  if (uploadfile.status == UPLOAD_FILE_START)
  {
    String filename = uploadfile.filename;
    if (!filename.startsWith("/")) filename = "/" + filename;
    Serial.print(F("Upload File Name: ")); Serial.println(filename);
    SPIFFS.remove(filename);                  // Remove a previous version, otherwise data is appended the file again
    UploadFile = SPIFFS.open(filename, "w");  // Open the file for writing in SPIFFS (create it, if doesn't exist)
  }
  else if (uploadfile.status == UPLOAD_FILE_WRITE)
  {
    if (UploadFile) UploadFile.write(uploadfile.buf, uploadfile.currentSize); // Write the received bytes to the file
  }
  else if (uploadfile.status == UPLOAD_FILE_END)
  {
    if (UploadFile)         // If the file was successfully created
    {
      UploadFile.close();   // Close the file again
      Serial.print(F("Upload Size: ")); Serial.println(uploadfile.totalSize);
      webpage = "";
      append_page_header();
      webpage += F("<h3>File was successfully uploaded</h3>");
      webpage += F("<h2>Uploaded File Name: "); webpage += uploadfile.filename + "</h2>";
      webpage += F("<h2>File Size: "); webpage += file_size(uploadfile.totalSize) + "</h2><br>";
      append_page_footer();
      server.send(200, "text/html", webpage);
      OuvrirCalendrier();
    }
    else
    {
      ReportCouldNotCreateFile("upload");
    }
  }
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
void SPIFFS_dir() {
  if (SPIFFS_present) {
    File root = SPIFFS.open("/");
    if (root) {
      root.rewindDirectory();
      SendHTML_Header();
      webpage += F("<h3 class='rcorners_m'>SPIFFS Contents</h3><br>");
      webpage += F("<table align='center'>");
      webpage += F("<tr><th>Name/Type</th><th style='width:20%'>Type File/Dir</th><th>File Size</th></tr>");
      printDirectory("/", 0);
      webpage += F("</table>");
      SendHTML_Content();
      root.close();
    }
    else
    {
      SendHTML_Header();
      webpage += F("<h3>No Files Found</h3>");
    }
    append_page_footer();
    SendHTML_Content();
    SendHTML_Stop();   // Stop is needed because no content length was sent
  } else ReportSPIFFSNotPresent();
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
void printDirectory(const char * dirname, uint8_t levels) {
  File root = SPIFFS.open(dirname);
  if (!root) {
    return;
  }
  if (!root.isDirectory()) {
    return;
  }
  File file = root.openNextFile();
  while (file) {
    if (webpage.length() > 1000) {
      SendHTML_Content();
    }
    if (file.isDirectory()) {
      webpage += "<tr><td>" + String(file.isDirectory() ? "Dir" : "File") + "</td><td>" + String(file.name()) + "</td><td></td></tr>";
      printDirectory(file.name(), levels - 1);
    }
    else
    {
      webpage += "<tr><td>" + String(file.name()) + "</td>";
      webpage += "<td>" + String(file.isDirectory() ? "Dir" : "File") + "</td>";
      webpage += "<td>" + file_size(file.size()) + "</td></tr>";
    }
    file = root.openNextFile();
  }
  file.close();
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
void File_Delete() {
  if (server.args() > 0 ) { // Arguments were received
    if (server.hasArg("delete")) SPIFFS_file_delete(server.arg(0));
  }
  else SelectInput("Select a File to Delete", "delete", "delete");
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
void SPIFFS_file_delete(String filename) { // Delete the file
  if (SPIFFS_present) {
    SendHTML_Header();
    File dataFile = SPIFFS.open("/" + filename, "r"); // Now read data from SPIFFS Card
    if (dataFile)
    {
      if (SPIFFS.remove("/" + filename)) {
        Serial.println(F("File deleted successfully"));
        webpage += "<h3>File '" + filename + "' has been erased</h3>";
        webpage += F("<a href='/delete'>[Back]</a><br><br>");
      }
      else
      {
        webpage += F("<h3>File was not deleted - error</h3>");
        webpage += F("<a href='delete'>[Back]</a><br><br>");
      }
    } else ReportFileNotPresent("delete");
    append_page_footer();
    SendHTML_Content();
    SendHTML_Stop();
  } else ReportSPIFFSNotPresent();
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
void SendHTML_Header() {
  server.sendHeader("Cache-Control", "no-cache, no-store, must-revalidate");
  server.sendHeader("Pragma", "no-cache");
  server.sendHeader("Expires", "-1");
  server.setContentLength(CONTENT_LENGTH_UNKNOWN);
  server.send(200, "text/html", ""); // Empty content inhibits Content-length header so we have to close the socket ourselves.
  append_page_header();
  server.sendContent(webpage);
  webpage = "";
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
void SendHTML_Content() {
  server.sendContent(webpage);
  webpage = "";
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
void SendHTML_Stop() {
  server.sendContent("");
  server.client().stop(); // Stop is needed because no content length was sent
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
void SelectInput(String heading1, String command, String arg_calling_name) {
  SendHTML_Header();
  webpage += F("<h3>"); webpage += heading1 + "</h3>";
  webpage += F("<FORM action='/"); webpage += command + "' method='post'>"; // Must match the calling argument e.g. '/chart' calls '/chart' after selection but with arguments!
  webpage += F("<input type='text' name='"); webpage += arg_calling_name; webpage += F("' value=''><br>");
  webpage += F("<type='submit' name='"); webpage += arg_calling_name; webpage += F("' value=''><br><br>");
  webpage += F("<a href='/'>[Back]</a><br><br>");
  append_page_footer();
  SendHTML_Content();
  SendHTML_Stop();
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
void ReportSPIFFSNotPresent() {
  SendHTML_Header();
  webpage += F("<h3>No SPIFFS Card present</h3>");
  webpage += F("<a href='/'>[Back]</a><br><br>");
  append_page_footer();
  SendHTML_Content();
  SendHTML_Stop();
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
void ReportFileNotPresent(String target) {
  SendHTML_Header();
  webpage += F("<h3>File does not exist</h3>");
  webpage += F("<a href='/"); webpage += target + "'>[Back]</a><br><br>";
  append_page_footer();
  SendHTML_Content();
  SendHTML_Stop();
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
void ReportCouldNotCreateFile(String target) {
  SendHTML_Header();
  webpage += F("<h3>Could Not Create Uploaded File (write-protected?)</h3>");
  webpage += F("<a href='/"); webpage += target + "'>[Back]</a><br><br>";
  append_page_footer();
  SendHTML_Content();
  SendHTML_Stop();
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
String file_size(int bytes) {
  String fsize = "";
  if (bytes < 1024)                      fsize = String(bytes) + " B";
  else if (bytes < (1024 * 1024))        fsize = String(bytes / 1024.0, 3) + " KB";
  else if (bytes < (1024 * 1024 * 1024)) fsize = String(bytes / 1024.0 / 1024.0, 3) + " MB";
  else                                   fsize = String(bytes / 1024.0 / 1024.0 / 1024.0, 3) + " GB";
  return fsize;
}
//---------------------------------------------------------------------------
void handleTime() { // getion temps restant page web
  char time_str[9];
  const uint32_t millis_in_day    = 1000 * 60 * 60 * 24;
  const uint32_t millis_in_hour   = 1000 * 60 * 60;
  const uint32_t millis_in_minute = 1000 * 60;

  static unsigned long t0 = 0;
  if (millis() - debut > config.timeoutWifi * 1000) debut = millis(); // securité evite t<0
  t0 = debut + (config.timeoutWifi * 1000) - millis();
  // Serial.print(debut),Serial.print("|"),Serial.println(t0);

  uint8_t days     = t0 / (millis_in_day);
  uint8_t hours    = (t0 - (days * millis_in_day)) / millis_in_hour;
  uint8_t minutes  = (t0 - (days * millis_in_day) - (hours * millis_in_hour)) / millis_in_minute;
  uint8_t secondes = (t0 - (days * millis_in_day) - ((hours * millis_in_hour)) / millis_in_minute) / 1000 % 60;
  sprintf(time_str, "%02d:%02d:%02d", hours, minutes, secondes);
  // Serial.println(time_str);
  server.send(200, "text/plane", String(time_str)); //Send Time value only to client ajax request
}
//---------------------------------------------------------------------------
void handleDateTime() { // getion Date et heure page web
  char time_str[20];
  sprintf(time_str, "%02d/%02d/%4d %02d:%02d:%02d", day(), month(), year(), hour(), minute(), second());
  server.send(200, "text/plane", String(time_str)); //Send Time value only to client ajax request
}
//---------------------------------------------------------------------------
byte gprs_upload_function (){
  // https://forum.arduino.cc/index.php?topic=376911.15
  int buffer_space = 1000;
  UploadFile = SPIFFS.open(filelog, "r");
  byte reply = 1;
  int i = 0;
  // ne fonctionne pas dans tous les cas ex roaming
  // while (i < 10 && reply == 1){ //Try 10 times...
    // reply = sendATcommand("AT+CREG?","+CREG: 0,1","ERROR", 1000);
    // i++;
    // delay(1000);
  // }
  reply = 0;
if (reply == 0){ // ouverture GPRS
// reply = sendATcommand("AT+SAPBR=3,1,\"CONTYPE\",\"GPRS\"","OK","ERROR", 1000);
if (reply == 0){
// reply = sendATcommand("AT+SAPBR=3,1,\"APN\",\"sl2sfr\"", "OK", "ERROR", 1000);//Replace with your APN
if (reply == 0){
//reply = sendATcommand("AT+SAPBR=3,1,\"USER\",\"entelpcs\"", "OK", "ERROR", 1000);
if (reply == 0){
//reply = sendATcommand("AT+SAPBR=3,1,\"PWD\",\"entelpcs\"", "OK", "ERROR", 1000);
if (reply == 0){
reply = 2;
i = 0;
while (i < 3 && reply == 2){ //Try 3 times...
  reply = sendATcommand("AT+SAPBR=1,1", "OK", "ERROR", 10000);
  if (reply == 2){
    sendATcommand("AT+SAPBR=0,1", "OK", "ERROR", 10000);
  }
  i++;
}
if (reply == 0){
reply = sendATcommand("AT+SAPBR=2,1", "OK", "ERROR", 1000);
if (reply == 0){
reply = sendATcommand("AT+FTPCID=1", "OK", "ERROR", 1000);
if (reply == 0){
reply = sendATcommand("AT+FTPSERV=\"" + String(config.ftpServeur) + "\"", "OK", "ERROR", 1000);//replace ftp.sample.com with your server address
if (reply == 0){
reply = sendATcommand("AT+FTPPORT="+ String(config.ftpPort), "OK", "ERROR", 1000);
if (reply == 0){
reply = sendATcommand("AT+FTPUN=\"" + String(config.ftpUser) + "\"", "OK", "ERROR", 1000);//Replace 1234@sample.com with your username
if (reply == 0){
reply = sendATcommand("AT+FTPPW=\"" + String(config.ftpPass) + "\"", "OK", "ERROR", 1000);//Replace 12345 with your password
if (reply == 0){
reply = sendATcommand("AT+FTPPUTNAME=\"" + String(filelog) + "\"", "OK", "ERROR", 1000);
if (reply == 0){
  reply = sendATcommand("AT+FTPPUTPATH=\"/" + String(config.Idchar) + "/\"", "OK", "ERROR", 1000);// repertoire "/Id/"
if (reply == 0){
  unsigned int ptime = millis();
  reply = sendATcommand("AT+FTPPUT=1", "+FTPPUT: 1,1", "+FTPPUT: 1,6", 60000);
  Serial.println("Time: " + String(millis() - ptime));
  if (reply == 0){
    if (UploadFile) {
      int i = 0;
      unsigned int ptime = millis();
      long archivosize = UploadFile.size();
      while (UploadFile.available()) {
        while(archivosize >= buffer_space){
          reply = sendATcommand("AT+FTPPUT=2," + String(buffer_space), "+FTPPUT: 2,1", "OK", 3000);
            if (reply == 0) { //This loop checks for positive reply to upload bytes and in case or error it retries to upload
              Serial.println("Remaining Characters: " + String(UploadFile.available()));
              for(int d = 0; d < buffer_space; d++){
                Serial2.write(UploadFile.read());
                archivosize -= 1;
              }
            }
            else {
              Serial.println("Error while sending data:");
              reply = 1;
            }
        }
        if (sendATcommand("AT+FTPPUT=2," + String(archivosize), "+FTPPUT: 2," + String(archivosize), "ERROR", 10000) == 0) {
          for(int t = 0; t < archivosize; t++){
            Serial2.write(UploadFile.read());
          }
        }
      }
    UploadFile.close();
    Serial.println("Time: " + String(millis() - ptime));
    }
  }
}
}
}
}
}
}
}
}
}
}
}
}
}
}
  sendATcommand("AT+SAPBR=0,1", "OK", "ERROR", 10000); // fermeture GPRS
return reply;
}
//---------------------------------------------------------------------------
byte sendATcommand(String ATcommand, String answer1, String answer2, unsigned int timeout){
  byte reply = 1;
  String content = "";
  char character;

  //Clean the modem input buffer
  while(Serial2.available()>0) Serial2.read();

  //Send the atcommand to the modem
  Serial2.println(ATcommand);
  delay(100);
  unsigned int timeprevious = millis();
  while((reply == 1) && ((millis() - timeprevious) < timeout)){
    while(Serial2.available()>0) {
      character = Serial2.read();
      content.concat(character);
      Serial.print(character);
      delay(10);
    }
    //Stop reading conditions
    if (content.indexOf(answer1) != -1){
      reply = 0;
    }else if(content.indexOf(answer2) != -1){
      reply = 2;
    }else{
      //Nothing to do...
    }
  }
  return reply;
}
//---------------------------------------------------------------------------
String sendAT(String ATcommand, String answer1, String answer2, unsigned int timeout){
  byte reply = 1;
  String content = "";
  char character;

  //Clean the modem input buffer
  while(Serial2.available()>0) Serial2.read();

  //Send the atcommand to the modem
  Serial2.println(ATcommand);
  delay(100);
  unsigned int timeprevious = millis();
  while((reply == 1) && ((millis() - timeprevious) < timeout)){
    while(Serial2.available()>0) {
      character = Serial2.read();
      content.concat(character);
      Serial.print(character);
      delay(10);
    }
  }
  // Serial.print("reponse: "),Serial.println(content);
  // Serial.println("fin reponse");
  return content;
}
/* --------------------  test local serial seulement ----------------------*/
void recvOneChar() {

  char   receivedChar;
  static String serialmessage = "";
  static bool   newData = false;

  if (Serial.available() > 0) {
    receivedChar = Serial.read();
    if (receivedChar != 10 && receivedChar != 13) {
      serialmessage += receivedChar;
    }
    else {
      newData = true;
      return;
    }
  }
  if (newData == true) {
    Serial.println(serialmessage);
    interpretemessage(serialmessage);
    newData = false;
    serialmessage = "";
  }
}

void interpretemessage(String demande) {
  String bidons;
  //demande.toUpperCase();
  if (demande.indexOf(char(61)) == 0) {
    bidons = demande.substring(1);
    bidons.toCharArray(replybuffer, bidons.length() + 1);
    traite_sms(99);//	traitement SMS en mode test local
  }
}
