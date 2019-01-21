/* Ph CORBEL 12/2018
Gestion eclairage tunnel
Alimentation sur panneaux solaires

mode deep sleep 
reveille tout les matin 06h55
reception des SMS en attente
apres 5 min de fonctionnement (ex: 2mn pour reception/suppression 8 SMS, 4mn 14SMS)
envoie sms signal vie 
analyse calendrier sauvegardé en SPIFFS
si jour circulé 
on continue normalement
en fin de journée retour sleep jusqu'a 06h55
pas d'option wake up à une heure donnée
il faut donc calculer une durée de sleep(quid precision RTC)
si non circulé, 
retour SIM800 et ESP32 en sleep reveil toute les heures
au reveil attendre au moins 30s pour que les SMS arrivent,
quand plus de SMS et traitement retour sleep 1H00

mode normal
SIM800 en reception
entrees pedale 1 et 2 sur interruption ESP32
allumage /extinction avec time out de 1 heure

Si coupure cable(equiv pedale enfoncée en permanence)
declenchement Alarme

Surveillance Batterie solaire

	Librairie TimeAlarms.h modifiée
	#define dtNBR_ALARMS 10 (ne fonctionne pas avec 9)   6 à l'origine nombre d'alarmes RAM*11 max is 255
	
to do

debug a faire 
temps mini pedale enfoncée
alarme cable/porte
page web
message ST a remanier

accelerer lecture SMS si grande qte au lancement, 
ne pas attendre next Acquisition 4mn 14SMS > 3mn 14SMS??? 
perdu lescture des Interrupts -- a revoir --


Compilation LOLIN D32,default,80MHz
987002 75%, 46924 14%

 */
 
#include <credentials_home.h>

#include <Sim800l.h>              //my SIM800 modifié
#include <Time.h>
#include <TimeAlarms.h>	
#include <sys/time.h>
#include <WiFi.h>
#include <EEPROM.h>               // variable en EEPROM
#include <SPIFFS.h>
#include <ArduinoOTA.h>
#include <WiFiClient.h>
#include <WebServer.h>
#include <FS.h>
#include <SPI.h>

String  webpage = "";
#define ServerVersion "1.0"
bool    SPIFFS_present = false;
#include "CSS.h"               // pageweb

#define PinBattProc		35   // liaison interne carte Lolin32 adc
#define PinBattSol		39   // Batterie générale 12V adc VN
#define PinBattUSB		36   // V USB 5V adc VP 36, 25 ADC2 pas utilisable avec Wifi 
#define PinPedale1		32   // Entrée Pedale1 Wake up EXT1
#define PinPedale2		33   // Entrée Pedale2 Wake up EXT1
#define PinPorte   		34   // Entrée Porte Coffret Wake up EXT1 
#define PinEclairage 	21   // Sortie Commande eclairage
#define PinSirene			0    // Sortie Commande Sirene
#define PIN						1234 // Code PIN carte SIM
#define RX_PIN				16   // TX Sim800
#define TX_PIN				17   // RX Sim800

#define BUTTON_PIN_BITMASK 0x700000000 // 32,33,34 Interruption Wake up
#define uS_TO_S_FACTOR 1000000  /* Conversion factor for micro seconds to seconds */

WebServer server(80);

long TIME_TO_SLEEP = 15;        /* Time ESP32 will go to sleep (in seconds) */
unsigned long debut = millis(); // pour decompteur temps wifi
byte calendrier[13][32]; // tableau calendrier ligne 0 et jour 0 non utilisé, 12*31
char filecalendrier[13]  = "/filecal.csv";  // fichier en SPIFFS contenant le calendrier de circulation
char filecalibration[11] = "/coeff.txt";    // fichier en SPIFFS contenant le calendrier de calibration
char filelog[9]          = "/log.txt";      // fichier en SPIFFS contenant le log

const String soft	= "ESP32_Tunnel.ino.d32"; // nom du soft
String	ver       = "V1-1";
int Magique       = 2341;

String Sbidon 		= "";
String message;
String bufferrcpt;
String fl = "\n";                   //	saut de ligne SMS
String Id ;                         // 	Id du materiel sera lu dans EEPROM
char    SIM800InBuffer[64];         //	for notifications from the SIM800
char replybuffer[255];              // 	Buffer de reponse SIM800
volatile int IRQ_Cpt_PDL1  = 0;
volatile int IRQ_Cpt_PDL2  = 0;
volatile int IRQ_Cpt_Porte = 0;
volatile unsigned long rebond1 = 0;		//	antirebond IRQ	
volatile unsigned long rebond2 = 0;
byte confign = 0;					// Num enregistrement EEPROM
bool Allume = false;
bool FlagAlarmeTension       = false; // Alarme tension Batterie
bool FlagLastAlarmeTension   = false;
bool FlagAlarmeIntrusion     = false; // Alarme Defaut Cable detectée
bool FlagAlarmeCable1        = false; // Alamre Cable Pedale1
bool FlagAlarmeCable2        = false; // Alamre Cable Pedale2
bool FlagAlarmePorte         = false; // Alamre Porte Coffret
bool FlagLastAlarmeIntrusion = false;
bool FirstSonn = false;				// Premier appel sonnerie
bool SonnMax   = false;				// temps de sonnerie maxi atteint
bool FlagReset = false;
bool jour      = false;				// jour = true, nuit = false
int  Nmax      = 0;						// comptage alarme cable avant alarme different Jour/Nuit
byte DbounceTime = 20;				// antirebond
int CoeffTension[3];          // Coeff calibration Tension
int CoeffTensionDefaut = 7000;// Coefficient par defaut

RTC_DATA_ATTR int CptAllumage = 0; // Nombre Allumage par jour en memoire RTC
RTC_DATA_ATTR bool WupAlarme  = false; // declenchement alarme externe
bool LastWupAlarme            = false;

int   slot = 0;            			 //this will be the slot number of the SMS
char   receivedChar;
bool   newData = false;
String demande;
long   TensionBatterie  = 0; // Tension Batterie solaire
long   VBatterieProc    = 0; // Tension Batterie Processeur
long   VUSB             = 0; // Tension USB
String catalog[2][10]; // liste des fichiers en SPIFFS nom/taille 10 lignes max

File UploadFile;

typedef struct											// declaration structure  pour les log
{
	char 		dt[10];									//	DateTime 0610-1702 9+1
	char 		Act[2];									//	Action A/D/S/s 1+1 
	char 		Name[15];								//	14 car
} champ;
champ record[5];

byte recordn = 100;							// Num enregistrement EEPROM

struct  config_t 								// Structure configuration sauvée en EEPROM
{
  int 		magic				;					// num magique
  long    Ala_Vie 		;					// Heure message Vie, 7h matin en seconde = 7*60*60
	long    FinJour 		;					// Heure fin jour, 20h matin en seconde = 20*60*60
	long    RepeatWakeUp ; 				// Periodicité WakeUp Jour non circulé
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
	bool    Porte;                // Alarme Porte Active
	long    Jour_Nmax;            // Comptage Alarme Jour
	long    Nuit_Nmax;            // Comptage Alarme Nuit
	char 		Idchar[11];						// Id
} ;
config_t config;


AlarmId loopPrincipale;	// boucle principale
AlarmId Svie;						// tempo Signal de Vie et MajHeure
AlarmId TempoAnalyse;		// tempo analyse alarme suite Interruption
AlarmId TempoSortie;		// Temporisation eclairage a la sortie
AlarmId TimeOut;				// TimeOut Allumage
AlarmId FinJour;				// Fin de journée retour deep sleep
AlarmId TSonn;					// 4 tempo durée de la sonnerie
AlarmId TSonnMax;				// 5 tempo maximum de sonnerie
AlarmId TSonnRepos;			// 6 tempo repos apres maxi

portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;

HardwareSerial *SIM800Serial = &Serial2;// liaison serie FONA SIM800
Sim800l Sim800l;  											// to declare the library

//---------------------------------------------------------------------------
	void IRAM_ATTR handleInterruptP1() { // Pedale 1
		
		portENTER_CRITICAL_ISR(&mux);
		if(xTaskGetTickCount() - rebond1 > DbounceTime){
			IRQ_Cpt_PDL1++;
			rebond1 = xTaskGetTickCount(); // equiv millis()
		}
		portEXIT_CRITICAL_ISR(&mux);

	}
	void IRAM_ATTR handleInterruptP2() { // Pedale 2
	
		portENTER_CRITICAL_ISR(&mux);
		if(xTaskGetTickCount() - rebond2 > DbounceTime){
			IRQ_Cpt_PDL2++;
			rebond2 = xTaskGetTickCount(); // equiv millis()
		}
		portEXIT_CRITICAL_ISR(&mux);

	}
	void IRAM_ATTR handleInterruptPo() { // Porte
		portENTER_CRITICAL_ISR(&mux);
		IRQ_Cpt_Porte++;
		portEXIT_CRITICAL_ISR(&mux);
	}
//---------------------------------------------------------------------------

void setup() {
	message.reserve(140);
	
  Serial.begin(115200);
	Serial.println();
	Serial.println(F("lancement SIM800"));
  SIM800Serial->begin(9600); // 4800
	Sim800l.begin();
	
	pinMode(PinEclairage,OUTPUT);
  pinMode(PinPedale1  ,INPUT_PULLUP);
	pinMode(PinPedale2  ,INPUT_PULLUP);
	pinMode(PinPorte    ,INPUT_PULLUP);
	pinMode(PinSirene   ,OUTPUT);
	digitalWrite(PinEclairage, LOW);
	digitalWrite(PinSirene, LOW);
	adcAttachPin(PinBattProc);
	adcAttachPin(PinBattSol);
	adcAttachPin(PinBattUSB);
	
		/* Lecture configuration en EEPROM	 */
  EEPROM.begin(512);
	EEPROM.get(recordn, record); // Lecture des log
	EEPROM.get(confign, config); // lecture config
	Alarm.delay(500);
  if (config.magic != Magique) {
    /* verification numero magique si different
				erreur lecture EEPROM ou carte vierge
    		on charge les valeurs par défaut
		*/
		Serial.println(F("Nouvelle Configuration !"));
		config.magic         = Magique;
		config.Ala_Vie       = 7*60*60;
		config.FinJour       = 20*60*60;
		config.RepeatWakeUp  = 60*60;
		config.Tanalyse      = 10*60;
		config.Intru         = true;
		config.Silence       = false;
		config.Dsonn         = 60;
		config.DsonnMax      = 90;
		config.Dsonnrepos    = 120;
		config.tempoSortie   = 10;
		config.timeOutS      = 60;// 3600
		config.timeoutWifi   = 10*60;
		config.Pedale1       = true;
		config.Pedale2       = true;
		config.Porte         = true;
		config.Jour_Nmax     = 3*60/10; // 3mn /10 temps de boucle Acquisition
		config.Nuit_Nmax     = 30/10;   // 30s /10 temps de boucle Acquisition
		String temp          = "TPCF_Canal";// TPCF_TCnls
    temp.toCharArray(config.Idchar, 11);
		EEPROM.put(confign,config);
		// valeur par defaut des record (log)
		for (int i = 0; i<5 ;i++){
			temp = "";
			temp.toCharArray(record[i].dt,10);		
			temp.toCharArray(record[i].Act,2);		
			temp.toCharArray(record[i].Name,15);
		}
		EEPROM.put(recordn, record);// ecriture des valeurs par defaut
	}
	EEPROM.end();
	PrintEEPROM();
	Id  = String(config.Idchar);
  Id += fl;
	
	ArduinoOTA.setHostname("ESP32_Tunnel");
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
  else{
    Serial.println(F("SPIFFS initialised... file access enabled..."));
    SPIFFS_present = true; 
  }
	
	OuvrirCalendrier();					// ouvre calendrier circulation en SPIFFS
	OuvrirFichierCalibration(); // ouvre fichier calibration en SPIFFS
// Serial.print(F("temps =")),Serial.println(millis());
	Sim800l.reset(PIN);					// lancer SIM800	
	// Sim800l.getRSSI();
	// Alarm.delay(1000);
// Serial.print(F("temps =")),Serial.println(millis());
	MajHeure();
// Serial.print(F("temps =")),Serial.println(millis());
  // OneH = Alarm.timerRepeat(3600,test);
	
	loopPrincipale = Alarm.timerRepeat(10, Acquisition); // boucle principale 10s
  Alarm.enable(loopPrincipale);

	TempoAnalyse = Alarm.timerRepeat(config.Tanalyse, FinAnalyse); // Tempo Analyse Alarme sur interruption
	Alarm.disable(TempoAnalyse);
	
	TempoSortie = Alarm.timerRepeat(config.tempoSortie, Extinction); // tempo extinction a la sortie
	Alarm.disable(TempoSortie);
	
	TimeOut = Alarm.timerRepeat(config.timeOutS, Extinction); // tempo time out extinction
	Alarm.disable(TimeOut);
	
	Svie = Alarm.alarmRepeat(config.Ala_Vie, SignalVie); // chaque jour
  Alarm.enable(Svie);
	
	FinJour = Alarm.alarmRepeat(config.FinJour, FinJournee); // Fin de journée retour deep sleep
  Alarm.enable(FinJour);
	
	TSonn = Alarm.timerRepeat(config.Dsonn, ArretSonnerie);	// tempo durée de la sonnerie
  Alarm.disable(TSonn);

  TSonnMax = Alarm.timerRepeat(config.DsonnMax, SonnerieMax); // tempo maximum de sonnerie
  Alarm.disable(TSonnMax);

  TSonnRepos = Alarm.timerRepeat(config.Dsonnrepos, ResetSonnerie); // tempo repos apres maxi
  Alarm.disable(TSonnRepos);
	
	ActiveInterrupt();
	
Serial.print(F("temps =")),Serial.println(millis());
}
//---------------------------------------------------------------------------
void loop() {
	
	recvOneChar();
	showNewData();
	
	if(rebond1 > millis()) rebond1 = millis();
	if(rebond2 > millis()) rebond2 = millis();
	
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
		if ((bufferrcpt.indexOf(F("RING"))) >-1) {	// RING, Ca sonne
      Sim800l.hangoffCall();									// on raccroche
    }
		/* Scan the notification string for an SMS received notification.
    If it's an SMS message, we'll get the slot number in 'slot' */
    if (1 == sscanf(SIM800InBuffer, "+CMTI: \"SM\",%d", &slot)) {
      traite_sms(slot);
    } 
  }
	
	if(IRQ_Cpt_Porte > 0){ // Alarme porte
		if(config.Intru && config.Porte){
			FlagAlarmePorte = true;
			portENTER_CRITICAL(&mux);
			if(IRQ_Cpt_Porte > 0)IRQ_Cpt_Porte = 0;
			portEXIT_CRITICAL(&mux);			
			Acquisition();
		}
	}
	
	if(IRQ_Cpt_PDL1 > 0 || IRQ_Cpt_PDL2 > 0){ 
		Serial.print(F("Interruption : "));
		Serial.print(IRQ_Cpt_PDL1);
		Serial.print(" ");
		Serial.println(IRQ_Cpt_PDL2);
	}
	if(IRQ_Cpt_PDL1 > 0){
		Allumage(1);
		portENTER_CRITICAL(&mux);
		IRQ_Cpt_PDL1 = 0;
		portEXIT_CRITICAL(&mux);
	}
	if(IRQ_Cpt_PDL2 > 0){
		Allumage(2);
		portENTER_CRITICAL(&mux);
		IRQ_Cpt_PDL2 = 0;
		portEXIT_CRITICAL(&mux);
	}

	ArduinoOTA.handle();
	Alarm.delay(1);
	
}	//fin loop
//---------------------------------------------------------------------------
void Acquisition(){	
	static int8_t nsms;
	static int cpt = 0; // compte le nombre de passage boucle
	static bool firstdecision = false;
	
	if(cpt > 3 && nsms == 0 && !firstdecision){ 
	/* une seule fois au demarrage attendre au moins 40s et plus de sms en attente */
		action_wakeup_reason();
		firstdecision = true;
	}
	cpt ++;
	
	if(LastWupAlarme != WupAlarme && nsms == 0){ // fin de la tempo analyse retour sleep
		LastWupAlarme = false;
		WupAlarme     = false;
		if(!jour){ // retour sleep jusqu'a 3mn avant AlaVie			
			TIME_TO_SLEEP = DureeSleep(config.Ala_Vie - 3*60);// 3mn avant
			Sbidon = F("Externe ");
			Sbidon += Hdectohhmm(TIME_TO_SLEEP);
			MajLog(F("Auto"),Sbidon);
			DebutSleep();
		}		
	}
	
	
	if(CoeffTension[0] == 0 || CoeffTension[1] == 0 || CoeffTension[2] == 0){
		OuvrirFichierCalibration(); // patch relecture des coeff perdu
	}
	
	if(!Sim800l.getetatSIM())Sim800l.reset(PIN);// verification SIM
	Serial.print(displayTime(0));
	Serial.print(F(" Freemem = ")),Serial.println(ESP.getFreeHeap());
	static byte nalaTension = 0;
	static byte nRetourTension = 0;
	TensionBatterie = map(moyenneAnalogique(PinBattSol) , 0, 4095, 0, CoeffTension[0]);
	VBatterieProc   = map(moyenneAnalogique(PinBattProc), 0, 4095, 0, CoeffTension[1]);
	VUSB            = map(moyenneAnalogique(PinBattUSB) , 0, 4095, 0, CoeffTension[2]);
	
	if(Battpct(TensionBatterie) < 25 || VUSB < 4000){ // || VUSB > 6000
		nalaTension ++;
    if (nalaTension == 4) {
      FlagAlarmeTension = true;
      nalaTension = 0;
    }
	}
	else if (Battpct(TensionBatterie) > 80 && VUSB > 4800) { //  && VUSB < 5400	//hysteresis et tempo sur Alarme Batterie
    nRetourTension ++;
		if(nRetourTension == 4){
			FlagAlarmeTension = false;			
			nRetourTension =0;
			nalaTension = 0;
		}
  }
  else {
    if (nalaTension > 0)nalaTension--;		//	efface progressivement le compteur
  }

	message = F("Batt Solaire = ");
	message += float(TensionBatterie/100.0);
	message += "V, ";
	message += String(Battpct(TensionBatterie));
	message += "%";
	message += F(", Batt Proc = ");
	message +=(String(VBatterieProc) + "mV");
	message +=(F(", V USB = "));
	message +=(float(VUSB/1000.0));
	message +=("V");
	message += fl;
	Serial.print(message);
	
	// alarme cable a terminer
	
	static byte nalaPIR1 = 0;
	static byte nalaPIR2 = 0;
	static byte nalaPorte = 0;
	if (config.Intru) { 
		// gestion des capteurs coupé ou en alarme permanente
		// verif sur plusieurs passages consecutifs
		if (digitalRead(PinPorte) && config.Porte){
			nalaPorte ++;		
			if(nalaPorte > 1){
				FlagAlarmeIntrusion = true;
				FlagAlarmePorte = true;
				nalaPorte = 0;
			}
		}
		else{
			if (nalaPorte > 0) nalaPorte --;		//	efface progressivement le compteur
		}
		
		if (digitalRead(PinPedale1) && config.Pedale1){
			nalaPIR1 ++;		
			if(nalaPIR1 > Nmax){
				FlagAlarmeIntrusion = true;
				FlagAlarmeCable1 = true;
				nalaPIR1 = 0;
			}
		}
		else{
			if (nalaPIR1 > 0) nalaPIR1 --;		//	efface progressivement le compteur
		}
		
		if (digitalRead(PinPedale2) && config.Pedale2){
			nalaPIR2 ++;		
			if(nalaPIR2 > Nmax){
				FlagAlarmeIntrusion = true;
				FlagAlarmeCable2 = true;
				nalaPIR2 = 0;
			}
		}
		else{
			if (nalaPIR2 > 0) nalaPIR2 --;		//	efface progressivement le compteur
		}
		
		Serial.print(F("Pedale 1:")),Serial.print(nalaPIR1);
		Serial.print(F(" Pedale 2:")),Serial.print(nalaPIR2);
		Serial.print(F(" Flag Porte:")),Serial.println(FlagAlarmePorte);
		
		if(FlagAlarmeIntrusion || FlagAlarmePorte){
			ActivationSonnerie();		// activation Sonnerie
			if(FlagAlarmePorte){
				Serial.println(F("Alarme Porte"));
			}
			else if(FlagAlarmeCable1 || FlagAlarmeCable2){
				Serial.println(F("Alarme Cable"));
			}
		}
	}
	else{
		FlagAlarmeIntrusion = false; // efface alarme 
		FlagAlarmeCable1 = false;
		FlagAlarmeCable2 = false;
		FlagAlarmePorte = false;
	}		
	// Serial.printf("Nala Porte = %d ,",nalaPorte);
	// Serial.printf("Nala Ped 1 = %d ,",nalaPIR1);
	// Serial.printf("Nala Ped 2 = %d\n",nalaPIR2);
	
	
	/* verification nombre SMS en attente(raté en lecture directe)
		 traitement des sms en memeoire un par un, 
		 pas de traitement en serie par commande 51, traitement beaucoup trop long */ 
  nsms = Sim800l.getNumSms(); // nombre de SMS en attente (1s)
  Serial.print(F("Sms en attente = ")), Serial.println(nsms);

  if(nsms > 0) {	// nombre de SMS en attente
    // il faut les traiter
		int numsms = Sim800l.getIndexSms(); // cherche l'index des sms en mémoire
    traite_sms(numsms);// traitement des SMS en attente
  }
	else if (nsms == 0 && FlagReset) { // on verifie que tous les SMS sont traités avant Reset
    FlagReset = false;
    ESP.restart();				//	reset soft
  }

	envoie_alarme();
	
	digitalWrite(LED_PIN,0);
	Alarm.delay(50);
	digitalWrite(LED_PIN,1);
	
	Serial.println();
	
}
//---------------------------------------------------------------------------
void test(){
	// pour test seulement	
	// static int cpt = 0;
	// cpt ++;
	// if(cpt == 3)Sim800l.dateNet();
	// if(cpt > 360){ // toute les heures
		// Sim800l.ModeText();
		// Sim800l.dateNet();
		MajHeure();
		// cpt = 0;
		EnvoyerSms(myTel, true);
	// }
}
//---------------------------------------------------------------------------
void traite_sms(byte slot){
	// Alarm.disable(loopPrincipale);
	/* il y a 50 slots dispo
		si slot=51, demande de balayer tous les slots pour verification
		si slot=99, demande depuis liaison serie en test, traiter sans envoyer de sms	
	*/
		
	char number[13];													// numero expediteur SMS
	bool error;
	String textesms;													// texte du SMS reçu
	textesms.reserve(140);
	String numero;
	String nom;
	
	byte i;
  byte j;
	bool sms = true;
	
	/* Variables pour mode calibration */
	static int tensionmemo = 0;//	memorisation tension batterie lors de la calibration
	int coef = 0; // coeff temporaire
	static byte P = 0; // Pin entrée a utiliser pour calibration
	static byte M = 0; // Mode calibration 1,2,3
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
    if(sms){
			numero = Sim800l.getNumberSms(slot); 	// recupere le Numero appelant	
			nom = Sim800l.getNameSms(slot);			// recupere le nom appelant
			textesms = Sim800l.readSms(slot);		// recupere le contenu	
			textesms = ExtraireSms(textesms);
			Serial.print(F("Nom appelant = ")),Serial.println(nom);
			Serial.print(F("Numero = ")),Serial.println(numero);		
		}
		else{
			textesms = String(replybuffer);
		}
		
		if (!(textesms.indexOf(F("TEL")) == 0 || textesms.indexOf(F("tel")) == 0 || textesms.indexOf(F("Tel")) == 0
				|| textesms.indexOf(F("Wifi")) == 0 || textesms.indexOf(F("WIFI")) == 0 || textesms.indexOf(F("wifi")) == 0)) {
			textesms.toUpperCase();	// passe tout en Maj sauf si "TEL" ou "WIFI" parametres pouvant contenir minuscules
			// textesms.trim();
    }
		textesms.replace(" ", "");// supp tous les espaces
		Serial.print(F("textesms  = ")),Serial.println(textesms);
		
		if((sms && nom.length() > 0) || !sms){          // si nom appelant existant dans phone book
			numero.toCharArray(number,numero.length()+1); // on recupere le numéro
			message = Id;
			if(textesms.indexOf(F("TIMEOUTWIFI"))>-1){ // Parametre Arret Wifi
				if(textesms.substring(11,12) == "="){
					int n = textesms.substring(12,textesms.length()).toInt();
					if(n > 9 && n < 3601){
						config.timeoutWifi = n;
						sauvConfig();														// sauvegarde en EEPROM
					}
				}
				message += F("TimeOut Wifi (s) = ");
				message += config.timeoutWifi;
				message += fl;
			}
			if(textesms.indexOf(F("WIFIOFF"))>-1){ // Arret Wifi
				WifiOff();
				message += F("Wifi off");
				message += fl;
				EnvoyerSms(number, true);
			}
			else if(textesms.indexOf(F("Wifi"))== 0){ // demande connexion Wifi
				byte pos1 = textesms.indexOf(char(44));//","
				byte pos2 = textesms.indexOf(char(44), pos1 + 1);
				String ssids = textesms.substring(pos1 + 1,pos2);
				String pwds  = textesms.substring(pos2 + 1,textesms.length());
				char ssid[20];
				char pwd[20];
				ssids.toCharArray(ssid,ssids.length()+1);
				pwds.toCharArray(pwd,pwds.length()+1);
				ConnexionWifi(ssid,pwd,number,sms);// message généré par routine
			}
			else if (textesms.indexOf(F("TEL")) == 0
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
                         || textesms.indexOf(F("EFFACE")) == 5 )) goto fin_tel;
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
          message = Id ;
          message += F("Commande non reconnue ?");// non reconnu
					message += fl;
          EnvoyerSms(number, sms);					// SMS non reconnu
        }
        else {
					Serial.println(Send);
					Sim800l.WritePhoneBook(Send);					//ecriture dans PhoneBook
          Alarm.delay(500);
					Sim800l.ModeText(); //pour purger buffer fona
          Alarm.delay(500);
          message = Id;
          message += F("Nouveau Num Tel: ");
          message += F("OK");
					message += fl;
          EnvoyerSms(number, sms);
        }
      }
			else if (textesms == F("LST?")) {	//	Liste des Num Tel
				byte n =Sim800l.ListPhoneBook();// nombre de ligne PhoneBook
				for (byte i = 1; i < n+1; i++) {
					String num = Sim800l.getPhoneBookNumber(i);
					Serial.print(num.length()),Serial.print(" "),Serial.println(num);
          if (num.indexOf("+CPBR:") == 0) { // si existe pas sortir
            Serial.println(F("Failed!"));// next i
            goto fin_i;
          }
					String name = Sim800l.getPhoneBookName(i);
					Serial.println(name);
          message += String(i) + ":";
          message += num;
          message += "," + fl;
          message += name;
          message += fl;
					Serial.println(message);
          if ((i % 3) == 0) {
            EnvoyerSms(number, sms);
            message = Id;
          }
        }
fin_i:
        if (message.length() > Id.length()) EnvoyerSms(number, sms);; // SMS final
      }
			else if (textesms.indexOf(F("ETAT")) == 0 || textesms.indexOf(F("ST")) == 0) {// "ETAT? de l'installation"
        generationMessage();
        EnvoyerSms(number, sms);
      }
			else if(textesms.indexOf(F("SYS"))>-1){
				Sim800l.getetatSIM();// 1s
				byte n = Sim800l.getNetworkStatus();// 1.1s
				String Op = Sim800l.getNetworkName();// 1.05s
				if(n == 5){
					message += F(("rmg, "));// roaming 1.0s
				}
				message += Op + fl;		
				read_RSSI();			
				int Vbat = Sim800l.BattVoltage();
				byte Batp = Sim800l.BattPct();
				message += F("Batt GSM : ");
				message += Vbat;
				message += F(" mV, ");			
				message += Batp;
				message += F(" %");
				message += fl;
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
				message += String(float(TensionBatterie/100.0));
				message += F("V, ");
				message += String(Battpct(TensionBatterie));
				message += " %";
				message += fl;
				message += F("V USB= ");
				message += (float(VUSB/1000.0));
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
        message = Id;
        message += F("Nouvel Id");
				message += fl;
        EnvoyerSms(number, sms);
			}
			else if (textesms.indexOf(F("LOG")) == 0){	// demande log des 5 derniers commandes
				message = "";
				for (int i = 0; i < 5; i++){
					message += String(record[i].dt) + "," + String(record[i].Act) + "," + String(record[i].Name) + fl;
				}
				//Serial.println( message);
				EnvoyerSms(number, sms);
			}
			else if (textesms.indexOf(F("INTRUON")) == 0 
						|| textesms.indexOf(("A"+Id.substring(6,10))) == 0) {	//	Armement Alarme
				// conserver INTRUON en depannage si ID non conforme
				if (!config.Intru) {
					config.Intru = !config.Intru;
					sauvConfig();											// sauvegarde en EEPROM
					ActiveInterrupt();
					if(!sms){															//V2-14
						nom = F("console");
						// Sbidon.toCharArray(nom,8);//	si commande locale
					}
					logRecord(nom,"A"); // V2-14 renseigne le log
					MajLog(nom,"A");
				}
				generationMessage();
				EnvoyerSms(number, sms);
      }
			else if(textesms.indexOf(F("INTRUOFF")) == 0 
					 || textesms.indexOf(("D"+Id.substring(6,10))) == 0){ //	Desarmement
				if(config.Intru) {
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
					FlagAlarmePorte = false;
					// FlagPIR = false;
					if(!sms){															//V2-14
						nom = F("console");
						// Sbidon.toCharArray(nom,8);//	si commande locale
					}
					logRecord(nom,"D");				// V2-14 renseigne le log
					MajLog(nom, "D");
				}
				generationMessage();
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
				generationMessage();
				EnvoyerSms(number, sms);				
      }
			else if(textesms.indexOf(F("PARAM")) == 0){ // parametre Jour/Nuit
				if (textesms.indexOf(char(61))== 5) { //char(61) "="	liste capteur actif
					Sbidon=textesms.substring(6,textesms.length());
					byte pos = Sbidon.indexOf(char(44)); // char(44) ","
					Serial.print(Sbidon),Serial.print(":"),Serial.println(pos);
					int val = Sbidon.substring(0,pos).toInt();
					Serial.println(val);
					if(val > 9 && val < 600){
						int val2 = Sbidon.substring(pos+1,Sbidon.length()).toInt();
						if(val2 > 9 && val2 < 600){
							config.Jour_Nmax = val/10; // 10 temps boucle acquisition=10s
							config.Nuit_Nmax = val2/10;
							Serial.print(F("Jour_Nmax = ")),Serial.print(val);
							Serial.print(F(" Nuit_Nmax = ")),Serial.println(val2);
							sauvConfig();											// sauvegarde en EEPROM
						}
					}
				}
				message += F("Parametres (s)");
				message += fl;
				message += F("Jour : ");
				message += config.Jour_Nmax*10 + fl;
				message += F("Nuit : ");
				message += config.Nuit_Nmax*10 + fl;
				EnvoyerSms(number, sms);
			}
			else if(textesms.indexOf(F("CAPTEUR")) == 0){	// Capteurs actif CAPTEUR=1,0,1 (Pedale1,Pedale2,Porte)
				bool flag = true; // validation du format
				if (textesms.indexOf(char(61))== 7) { //char(61) "="	liste capteur actif
					byte Num[3];
					Sbidon=textesms.substring(8,13);
					// Serial.print("Sbidon="),Serial.print(Sbidon),Serial.println(Sbidon.length());
					if (Sbidon.length() == 5){
						int j=0;
						for (int i = 0;i < 5; i +=2){
							if(Sbidon.substring(i,i+1) == "0" || Sbidon.substring(i,i+1) == "1"){
								// Serial.print(",="),Serial.println(Sbidon.substring(i+1,i+2));
								// Serial.print("X="),Serial.println(Sbidon.substring(i,i+1));
								Num[j] = Sbidon.substring(i,i+1).toInt();
								// Serial.print(i),Serial.print(","),Serial.print(j),Serial.print(","),Serial.println(Num[j]);
								j++;
							}
							else{									
								Serial.println(F("Format non reconnu"));
								flag = false;// format pas bon		
							}						
						}
						if(flag){ // sauv configuration
							DesActiveInterrupt();
							config.Pedale1 = Num[0];
							config.Pedale2 = Num[1];
							config.Porte   = Num[2];
							sauvConfig();											// sauvegarde en EEPROM
							ActiveInterrupt();
						}
					}					
				}
				if(flag){
					message += F("Pedale1 = ");
					if(config.Pedale1){
						message += 1;
					}
					else{
						message += 0;
					}
					message += fl;
					message += F("Pedale2 = ");
					if(config.Pedale2){
						message += 1;
					}
					else{
						message += 0;
					}
					message += fl;
					message += F("Porte = ");
					if(config.Porte){
						message += 1;
					}
					else{
						message += 0;
					}
					message += fl;
				}
				else{
					message += F("Format non reconnu");
				}
				
				EnvoyerSms(number, sms);
			}
			else if(textesms.indexOf(F("TEMPOSORTIE")) == 0){// Tempo Eclairage Sortie
				if (textesms.indexOf(char(61)) == 11){ // =
					int i = textesms.substring(11).toInt();
					// Serial.print("Cpt pedale = "),Serial.println(i);
					if(i > 0 && i < 121){
						config.tempoSortie = i;
						sauvConfig();                               // sauvegarde en EEPROM
					}
				}
				message += F("Tempo Sortie Eclairage (s) = ");
				message += config.tempoSortie;
				message += fl;
				EnvoyerSms(number, sms);				
			}
			else if(textesms.indexOf(F("TIMEOUTECL")) == 0){// Timeout extinction secours
				if (textesms.indexOf(char(61)) == 10){ // =
					int i = textesms.substring(11).toInt();
					// Serial.print("Cpt pedale = "),Serial.println(i);
					if(i > 59 && i < 7201){
						config.timeOutS = i;
						sauvConfig();                               // sauvegarde en EEPROM
					}
				}
				message += F("Time Out Eclairage (s) = ");
				message += config.timeOutS;
				message += fl;
				EnvoyerSms(number, sms);				
			}
			else if (textesms.indexOf(F("VIE")) == 0) {       //	Heure Message Vie
				if (textesms.indexOf(char(61)) == 3) {
					long i = atol(textesms.substring(4).c_str()); //	Heure message Vie
					if (i > 0 && i <= 86340) {                    //	ok si entre 0 et 86340(23h59)
						config.Ala_Vie = i;
						sauvConfig();                               // sauvegarde en EEPROM
						Svie = Alarm.alarmRepeat(config.Ala_Vie, SignalVie);// init tempo
					}
				}
				message += F("Heure Vie = ");
				message += Hdectohhmm(config.Ala_Vie);
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
			else if(textesms.indexOf(F("SIRENE")) == 0){			// Lancement SIRENE
				digitalWrite(PinSirene, HIGH);	// Marche Sonnerie
				Alarm.enable(TSonn);				// lancement tempo
				message += F("Lancement Sirene");
				message += fl;
				message += config.Dsonn;
				message += F("(s)");
				message += fl;			
				EnvoyerSms(number, sms);
			}
			else if(textesms.indexOf(F("TIME"))>-1){
				message += F("Heure Sys = ");
				message += displayTime(0);
				message += fl;
				EnvoyerSms(number, sms);
			}			
			else if (textesms.indexOf(F("MAJHEURE")) == 0) {	//	forcer mise a l'heure V2-19
				message += F("Mise a l'heure");
				Sim800l.reset(PIN);// lancer SIM800	
				MajHeure();		// mise a l'heure
				EnvoyerSms(number, sms);
			}			
			else if(textesms.indexOf(F("IMEI"))>-1){
				message += F("IMEI = ");
				String m = Sim800l.getIMEI();
				message += m + fl;
				EnvoyerSms(number, sms);
			}
			else if (textesms.indexOf(F("FIN")) == 0) {			//	Heure Fin de journée
				if ((textesms.indexOf(char(61))) == 3) {
					long i = atol(textesms.substring(4).c_str()); //	Heure 
					if (i > 0 && i <= 86340) {										//	ok si entre 0 et 86340(23h59)
						config.FinJour = i;
						sauvConfig();															// sauvegarde en EEPROM
						FinJour = Alarm.alarmRepeat(config.FinJour, FinJournee);// init tempo
					}
				}
				message += F("Fin Journee = ");
				message += Hdectohhmm(config.FinJour);
				message += F("(hh:mm)");
				message += fl;
				EnvoyerSms(number, sms);
      }
			else if(textesms.indexOf(F("MOIS")) == 0){ // Calendrier pour un mois
				/* mise a jour calendrier ;format : MOIS=mm,31 fois 0/1 
				demande calendrier pour un mois donné ; format : MOIS=mm? */
				bool flag = true; // validation du format
				bool W = true; // true Write, false Read
				
				byte p1 = textesms.indexOf(char(61)); // =
				byte p2 = textesms.indexOf(char(44)); // ,
				if(p2 == 255){                        // pas de ,
					p2 = textesms.indexOf(char(63));    // ?
					W = false;
				}
				
				byte m = textesms.substring(p1+1,p2).toInt(); // mois
				
				// printf("p1=%d,p2=%d\n",p1,p2);
				// Serial.println(textesms.substring(p1+1,p2).toInt());
				// Serial.println(textesms.substring(p2+1,textesms.length()).length());
				if(!(m > 0 && m < 13)) flag = false;
				if(W && flag){ // Write					
					if(!(textesms.substring(p2+1,textesms.length()).length() == 31)) flag = false; // si longueur = 31(jours)
					
					for(int i = 1; i < 32; i++){ // verification 0/1
						if(!(textesms.substring(p2+i,p2+i+1) == "0" || textesms.substring(p2+i,p2+i+1) == "1")){
							flag = false;
						}
					}
					if(flag){
						// Serial.println(F("mise a jour calendrier"));
						for(int i = 1; i < 32; i++){
							calendrier[m][i] = textesms.substring(p2+i,p2+i+1).toInt();
							// Serial.print(textesms.substring(p2+i,p2+i+1));
						}
						EnregistreCalendrier(); // Sauvegarde en SPIFFS						
						message += F("Mise a jour calendrier OK");
					}
				}
				else if(flag){ // ? demande calendrier pour un mois donné
					message += F("mois = ");
					message += m;
					message += fl;
					for(int i = 1; i < 32 ; i++){
						message += calendrier[m][i];
						if((i%5)  == 0) message += " ";
						if((i%10) == 0) message += fl;
					}				
				}
				if(!flag){
					message += F("Format non reconnu !");
				}
				message += fl;
				EnvoyerSms(number, sms);
			}
			else if (textesms == F("CIRCULE")) { 
			/* demande passer en mode Circulé pour le jour courant, 
				sans modification calendrier enregistré en SPIFFS */
				if(calendrier[month()][day()] == 0){
					calendrier[month()][day()] = 1;
					message += F("OK, Circule");
				}
				else{
					message += F("Jour deja Circule");
				}
				message += fl;
				EnvoyerSms(number, sms);
			}
			else if (textesms == F("NONCIRCULE")) { 
			/* demande passer en mode nonCirculé pour le jour courant, 
				sans modification calendrier enregistré en SPIFFS */
				if(calendrier[month()][day()] == 1){
					calendrier[month()][day()] = 0;
					message += F("OK, NonCircule");
				}
				else{
					message += F("Jour deja NonCircule");
				}
				message += fl;
				EnvoyerSms(number, sms);
			}
			else if (textesms == F("RST")) {               // demande RESET
        message += F("Le systeme va etre relance");  // apres envoie du SMS!
				message += fl;
        FlagReset = true;                            // reset prochaine boucle
				EnvoyerSms(number, sms);
      }
			else if (!sms && textesms.indexOf(F("CALIBRATION=")) == 0){	
				/* 	Mode calibration mesure tension
						Seulement en mode serie local
						recoit message "CALIBRATION=.X"
						entrer mode calibration
						Selection de la tenssion à calibrer X
						X = 1 TensionBatterie : PinBattSol : CoeffTension1
						X = 2 VBatterieProc : PinBattProc : CoeffTension2
						X = 3 VUSB : PinBattUSB : CoeffTension3
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
						int CoeffTension = CoeffTensionDefaut 6600 par défaut
				*/
				Sbidon = textesms.substring(12,16);// texte apres =
				//Serial.print(F("Sbidon=")),Serial.print(Sbidon),Serial.print(char(44)),Serial.println(Sbidon.length());
				long tension = 0;
				if(Sbidon.substring(0,1) == "." && Sbidon.length() > 1){// debut mode cal					
					if(Sbidon.substring(1,2) == "1" ){M = 1; P = PinBattSol; coef = CoeffTension[0];}
					if(Sbidon.substring(1,2) == "2" ){M = 2; P = PinBattProc; coef = CoeffTension[1];}
					if(Sbidon.substring(1,2) == "3" ){M = 3; P = PinBattUSB; coef = CoeffTension[2];}
					// Serial.print("mode = "),Serial.print(M),Serial.println(Sbidon.substring(1,2));
					FlagCalibration = true;
					
					coef = CoeffTensionDefaut;
					tension = map(moyenneAnalogique(P), 0,4095,0,coef);
					// Serial.print("TensionBatterie = "),Serial.println(TensionBatterie);
					tensionmemo = tension;
				}
				else if(FlagCalibration && Sbidon.substring(0,4).toInt() > 0 && Sbidon.substring(0,4).toInt() <=8000){
					// si Calibration en cours et valeur entre 0 et 5000
					Serial.println(Sbidon.substring(0,4));
					/* calcul nouveau coeff */
					coef = Sbidon.substring(0,4).toFloat()/float(tensionmemo)*CoeffTensionDefaut;
					// Serial.print("Coeff Tension = "),Serial.println(CoeffTension);
					tension = map(moyenneAnalogique(P), 0,4095,0,coef);
					CoeffTension[M-1] = coef;
					// switch(M){
						// case 1:
							// CoeffTension[0] = coef;
							// break;
						// case 2:
							// CoeffTension[1] = coef;
							// break;
						// case 3:
							// CoeffTension[2] = coef;
							// Serial.print("CoeffTension3 = "),Serial.print(CoeffTension[2]);//tracer changement valeur
							// break;
					// }
					// Serial.print("TensionBatterie = "),Serial.println(TensionBatterie);
					FlagCalibration = false;
					Recordcalib();														// sauvegarde en SPIFFS	
				}
				else{
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
				if(M == 1){
					message += fl;
					message += F("Batterie = ");
					message += String(Battpct(tension));
					message += "%";
				}
				message += fl;
				EnvoyerSms(number, sms);
			}
			else{
				message += F("message non reconnu !");
				message += fl;
				EnvoyerSms(number, sms);
			}
		}
		else {
      Serial.print(F("Appelant non reconnu ! "));
    }
		if(sms){ // suppression du SMS
			error = Serial.println(Sim800l.delSms(slot));
			Serial.print(F("resultat del Sms ")),Serial.println(error);
		}
	}
	// Alarm.enable(loopPrincipale);
}
//---------------------------------------------------------------------------
void envoie_alarme() {
  /* determine si un SMS appartition/disparition Alarme doit etre envoyé */
  bool SendEtat = false;
	
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
    envoieGroupeSMS(0);				// envoie groupé
    SendEtat = false;					// efface demande
  }
}
//---------------------------------------------------------------------------
void envoieGroupeSMS(byte grp) {
  /* si grp = 0,
    envoie un SMS à tous les numero existant (9 max) du Phone Book
    si grp = 1,
    envoie un SMS à tous les numero existant (9 max) du Phone Book
    de la liste restreinte config.Pos_Pn_PB[x]=1			*/
		
	byte n =Sim800l.ListPhoneBook();// nombre de ligne PhoneBook
  for (byte Index = 1; Index < n+1; Index++) {		// Balayage des Num Tel Autorisés=dans Phone Book
    // if (!fona.getPhoneBookNumber(Index, Telbuff, 13)) { // lire Phone Book
      // Serial.print(Index), Serial.println(F("fin Phone Book!"));
      // break;
    // }
    /* Serial.print(F("Num :  ")), Serial.println(Telbuff);
    if (String(Telbuff).length() > 0)	{	// Numero Tel existant/non vide
      if (grp == 1) {	// grp = 1 message liste restreinte
        if (config.Pos_Pn_PB[Index] == 1) {          
					generationMessage();
					message += F("lancement");
          sendSMSReply(Telbuff,true);
        }
      }
      else {	// grp = 0, message à tous */
				String number = Sim800l.getPhoneBookNumber(Index);
        generationMessage();
        // sendSMSReply(Telbuff,true);
				char num[13];
				number.toCharArray(num,13);
				EnvoyerSms(num, true);
      //}
    //}
  }
}
//---------------------------------------------------------------------------
void generationMessage() {
	message = Id;
	if(FlagAlarmeTension || FlagLastAlarmeTension || FlagAlarmeIntrusion){
		message += F("--KO--------KO--");
	}
	else {
    message += F("-------OK-------");
  }
	message+= fl;
	message += F("Batterie : ");			//"Alarme Batterie : "
  if (FlagAlarmeTension) {
    message += F("Alarme, ");
  }
  else {
    message += F("OK, ");
  }
	message += String(Battpct(TensionBatterie));
	message += "%";
	message += fl;
	message += F("V USB =");
	message += String(float(VUSB/1000.0)) + fl;
	message+= F("Nbr Allumage = ");
	message+= String(CptAllumage);
	message += fl ;
	
	if (config.Intru && FlagAlarmeIntrusion) {
    message += F("-- Alarme !--") ;
		message += fl;
		if(FlagAlarmeCable1){
			message += F("Pedale 1");
			message += fl;
		}
		if(FlagAlarmeCable2){
			message += F("Pedale 2");
			message += fl;
		}
		if(FlagAlarmePorte){
			message += F("Porte");
			message += fl;
		}
	}
	if (config.Intru) {
		message += F("Alarme Active ");// ajouter capteur actif futur V2-20
		message += fl;
	}
	else {
		message += F("Alarme Arrete");
		message += fl;
	}
	if (config.Silence) {
		message += F("Alarme Silencieuse ON");
		message += fl;
	}
	else
	{
		message += F("Alarme Silencieuse OFF");
		message += fl;
	}
	Serial.println(message);
}
//---------------------------------------------------------------------------
void EnvoyerSms(char *num, bool sms){
	
	if(sms){ // envoie sms
		message.toCharArray(replybuffer,message.length()+1);
		bool error = Sim800l.sendSms(num,replybuffer);
		Serial.print(F("resultat sms ")),Serial.println(error);
	}
	Serial.print (F("Message (long) = ")), Serial.println(message.length());
	Serial.println(message);
}
//---------------------------------------------------------------------------
void read_RSSI() {	// lire valeur RSSI et remplir message
  int r;
  byte n = Sim800l.getRSSI();
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
//---------------------------------------------------------------------------
void MajHeure(){
	
	static bool First = true;
	int ecart;
	Serial.print(F("Mise a l'heure reguliere !, "));
	// setTime(10,10,0,1,1,18);
	int Nday,Nmonth,Nyear,Nminute,Nsecond,Nhour;
	Sim800l.RTCtime(&Nday,&Nmonth,&Nyear,&Nhour,&Nminute,&Nsecond);
	
	
	printf("%s %02d/%02d/%d %02d:%02d:%02d\n","MajH1",Nday,Nmonth,Nyear,Nhour,Nminute,Nsecond);
	long debut = millis();
	if(First || Nyear < 17){
		while(Nyear < 17){
			Sim800l.RTCtime(&Nday,&Nmonth,&Nyear,&Nhour,&Nminute,&Nsecond);
			printf("%s %02d/%02d/%d %02d:%02d:%02d\n","MajH2",Nday,Nmonth,Nyear,Nhour,Nminute,Nsecond);
			Alarm.delay(1000);
			if(millis() - debut > 10000) {
				Sim800l.setPhoneFunctionality(0);
				Alarm.delay(1000);
				Sim800l.setPhoneFunctionality(1);
				Alarm.delay(1000);
			}
		}
		setTime(Nhour,Nminute,Nsecond,Nday,Nmonth,Nyear);
		First = false;
	}
	else{
		//  calcul décalage entre H sys et H reseau en s	
		ecart = (Nhour - hour()) * 3600;
		ecart += (Nminute - minute()) * 60;
		ecart += Nsecond - second();
		// ecart += 10;
		Serial.print(F("Ecart s= ")), Serial.println(ecart);
		if(abs(ecart) > 5){
			ArretSonnerie();	// Arret Sonnerie propre
			Alarm.disable(loopPrincipale);
			Alarm.disable(TSonnRepos);
			Alarm.disable(Svie);
			Alarm.disable(TempoSortie);
			Alarm.disable(TimeOut);
			Alarm.disable(FinJour);
			Alarm.disable(TempoAnalyse);
			
			setTime(Nhour,Nminute,Nsecond,Nday,Nmonth,Nyear);
			
			Alarm.enable(loopPrincipale);
			Alarm.enable(Svie);
			Alarm.enable(TempoSortie);
			Alarm.enable(TimeOut);
			Alarm.enable(FinJour);			
			Alarm.enable(TempoAnalyse);
			
		}		
	}
	displayTime(0);
	AIntru_HeureActuelle();
	
	/* test */
	// char dateheure[20];
	// sprintf(dateheure,"%02d/%02d/%d %02d:%02d:%02d",Nday,Nmonth,Nyear,Nhour,Nminute,Nsecond);
	// message = Id;
	// message += ecart;
	// message += fl;
	// message += String(dateheure);
	// message += fl;
	// message += displayTime(0);
	// message += fl;
	// Serial.println(message);
	
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
  Serial.print(F("Fin tempo Sonnerie : "));
  Serial.println(Alarm.getTriggeredAlarmId());
  digitalWrite(PinSirene, LOW);// Arret Sonnerie
  Alarm.disable(TSonn);	// on arrete la tempo sonnerie
  Alarm.disable(TSonnMax);// on arrete la tempo sonnerie maxi
  FirstSonn = false;
  FlagAlarmeIntrusion = false;
	FlagAlarmeCable1 = false;
	FlagAlarmeCable2 = false;
	FlagAlarmePorte = false;
  // FlagPIR = false;
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
void FinAnalyse(){
	/* retour etat normal apres Alarme sur Interruption */
	Alarm.disable(TempoAnalyse);	
	WupAlarme = false;
}
//---------------------------------------------------------------------------
long DureeSleep(long Htarget){ // Htarget Heure de reveil visée
	/* calcul durée entre maintenant et
	heure Vie-5mn, 5 mn(Tlancement)*/
	long SleepTime = 0;
	long Heureactuelle = HActuelledec();
	if(Heureactuelle < Htarget){
		SleepTime = Htarget - Heureactuelle;
	}
	else{
		if(Heureactuelle < 86400){// < 24h00
			SleepTime = (86400 - Heureactuelle) + Htarget;
		}
	}
	return SleepTime;
}
//---------------------------------------------------------------------------
long HActuelledec(){
	long Heureactuelle = hour()*60;// calcul en 4 lignes sinon bug!
	Heureactuelle += minute();
	Heureactuelle  = Heureactuelle*60;
	Heureactuelle += second(); // en secondes
	return Heureactuelle;
}
//---------------------------------------------------------------------------
void SignalVie(){
	Serial.println(F("Signal vie"));
	MajHeure();
	envoieGroupeSMS(0);
	Sim800l.delAllSms();// au cas ou, efface tous les SMS envoyé/reçu
	CptAllumage = 0;
	
}
//---------------------------------------------------------------------------
void sauvConfig() { // sauve configuration en EEPROM
	EEPROM.begin(512);
	EEPROM.put(confign,config);
	EEPROM.end();
}
//---------------------------------------------------------------------------
String displayTime(byte n) {
	// n = 0 ; dd/mm/yyyy hh:mm:ss
	// n = 1 ; yyyy-mm-dd hh:mm:ss
	char bid[20];
	if(n == 0){
		sprintf(bid,"%02d/%02d/%4d %02d:%02d:%02d",day(),month(),year(),hour(),minute(),second());
	}
	else{
		sprintf(bid,"%4d-%02d-%02d %02d:%02d:%02d",year(),month(),day(),hour(),minute(),second());
	}
	return String(bid);
}
//---------------------------------------------------------------------------
void logRecord(String nom,String action){ // renseigne log et enregistre EEPROM
	static int index = 0;
	String temp;
	if (month()<10){
		temp =  "0" + String(month());
	}
	else {
		// temp =  "0";//V2-18
		temp = String(month());//V2-18
	}
	if (day() < 10 ){
		temp += "0" + String(day());
	}
	else {
		temp += String(day());
	}
	if (hour() < 10){
		temp += "-0"+ String(hour());
	}
	else {
		temp += "-"+ String(hour());
	}
	if (minute() < 10){
		temp += "0" + String(minute());
	}
	else {
		temp += String(minute());
	}
	temp  .toCharArray(record[index].dt,10);
	nom   .toCharArray(record[index].Name,15);
	action.toCharArray(record[index].Act,2);
	
	// int longueur = EEPROM_writeAnything(recordn, record);// enregistre en EEPROM
	EEPROM.begin(512);
	EEPROM.put(recordn, record);// ecriture des valeurs par defaut
	EEPROM.end();
	if(index < 4){
		index ++;
	}
	else{
		index = 0;
	}
}
//---------------------------------------------------------------------------
void listDir(fs::FS &fs, const char * dirname, uint8_t levels){
	Serial.printf("Listing directory: %s\r\n", dirname);

	File root = fs.open(dirname);
	if(!root){
			Serial.println(F("- failed to open directory"));
			return;
	}
	if(!root.isDirectory()){
			Serial.println(F(" - not a directory"));
			return;
	}

	File file = root.openNextFile();
	while(file){
		if(file.isDirectory()){
			Serial.print(F("  DIR : "));
			Serial.println(file.name());
			if(levels){
				listDir(fs, file.name(), levels -1);
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
void readFile(fs::FS &fs, const char * path){
	Serial.printf("Reading file: %s\r\n", path);

	File file = fs.open(path);
	if(!file || file.isDirectory()){
		Serial.println(F("- failed to open file for reading"));
		return;
	}
	String buf="";
	int i = 0;
	// Serial.println("- read from file:");
	while(file.available()){
		int inchar = file.read();
		if(isDigit(inchar)){
			buf +=char(inchar);
			i ++;
		}
	}
	int m = 0;
	int j = 0;
	for(int i = 0; i < 372; i++){// 12mois de 31 j =372
		j = 1 + (i%31);
		if(j == 1) m ++;
		calendrier[m][j]= buf.substring(i,i+1).toInt();
	}
}
//---------------------------------------------------------------------------
void appendFile(fs::FS &fs, const char * path, const char * message){
	// Serial.printf("Appending to file: %s\r\n", path);

	File file = fs.open(path, FILE_APPEND);
	if(!file){
		// Serial.println("- failed to open file for appending");
		return;
	}
	if(file.print(message)){
		// Serial.println("- message appended");
	} else {
		// Serial.println("- append failed");
	}
}
//---------------------------------------------------------------------------
void MajLog(String Id,String Raison){ // mise à jour fichier log en SPIFFS
	/* verification de la taille du fichier */
	File f = SPIFFS.open(filelog, "r");
	Serial.print(F("Taille fichier log = ")),Serial.println(f.size());
	// Serial.print(Id),Serial.print(","),Serial.println(Raison);
	static bool once = false;
	if(f.size() > 150000 && !once){
		/* si trop grand on efface */
		once = true;
		message = Id;
		message += F("Fichier log presque plein\n");
		message += String(f.size());
		message += F("\nFichier sera efface à 300000");
		EnvoyerSms(myTel, true);
	}
	else if(f.size() > 300000 && once){ // 292Ko 75000 lignes
		message = Id;
		message += F("Fichier log plein\n");
		message += String(f.size());
		message += F("\nFichier efface");
		EnvoyerSms(myTel, true);
		SPIFFS.remove(filelog);
		once = false;
	}
	f.close();
	/* preparation de la ligne */
	char Cbidon[46]; //19 + 2 + 14 + 10 + 1
	sprintf(Cbidon,"%02d/%02d/%4d %02d:%02d:%02d",day(),month(),year(),hour(),minute(),second());	
	Id = ";" + Id + ";";
	Raison += "\n";
	strcat(Cbidon,Id.c_str());
	strcat(Cbidon,Raison.c_str());
	Serial.print(Cbidon);
	appendFile(SPIFFS, filelog, Cbidon);
}
//---------------------------------------------------------------------------
void EnregistreCalendrier(){ // remplace le nouveau calendrier
	
	SPIFFS.remove(filecalendrier);
	Sbidon = "";
	char bid[63];
	for(int m = 1; m < 13; m++){
		for(int j = 1; j < 32; j++){
			Sbidon += calendrier[m][j];
			if(j < 31)Sbidon += char(59); // ;
		}
		Serial.println(Sbidon);
		Sbidon += fl;
		Sbidon.toCharArray(bid,63);
		appendFile(SPIFFS, filecalendrier, bid);
		Sbidon = "";
	}
	
}
//---------------------------------------------------------------------------
void OuvrirCalendrier(){
	
	// this opens the file "f.txt" in read-mode
	listDir(SPIFFS, "/", 0);
	bool f = SPIFFS.exists(filecalendrier);
	// Serial.println(f);
	File f0 = SPIFFS.open(filecalendrier, "r");

	if (!f || f0.size() == 0){
		Serial.println(F("File doesn't exist yet. Creating it")); // creation calendrier defaut
		char bid[63];
		Sbidon="";
		for(int m = 1; m < 13;m++){
			for(int j = 1; j < 32; j++){
				if(m == 1 || m == 2 || m == 3 || m == 11 || m == 12){
					Sbidon += "0;";
				}
				else{
					Sbidon += "1;";
				}
			}
			Serial.println(Sbidon);
			Sbidon += fl;
			Sbidon.toCharArray(bid,63);
			appendFile(SPIFFS, filecalendrier, bid);
			Sbidon = "";
		}
		/* f = SPIFFS.exists(filecalendrier);
		if (!f) {
			// Serial.println("file creation failed");
		// }else{Serial.println("file creation OK");}
		} else {
		Serial.println(F("Read file"));
		// we could open the file
		} */
	}
	readFile(SPIFFS, filecalendrier);
	
	for(int m = 1; m < 13;m++){
		for(int j = 1; j < 32; j++){
			Serial.print(calendrier[m][j]),Serial.print(char(44));
		}
		Serial.println();
	}
	listDir(SPIFFS, "/", 0);

}
//---------------------------------------------------------------------------
void FinJournee(){
	// fin de journée retour deep sleep
	jour = false;
	Serial.println(F("Fin de journée retour sleep"));
	TIME_TO_SLEEP = DureeSleep(config.Ala_Vie - 3*60);// 3mn avant
	Sbidon = F("FinJour ");
	Sbidon += Hdectohhmm(TIME_TO_SLEEP);
	MajLog(F("Auto"),Sbidon);
	DebutSleep();
}
//---------------------------------------------------------------------------
void PrintEEPROM(){
	Serial.print(F("Version = "))									,Serial.println(ver);
	Serial.print(F("ID = "))											,Serial.println(config.Idchar);
	Serial.print(F("magic = "))										,Serial.println(config.magic);
	Serial.print(F("Alarme = "))									,Serial.println(config.Intru);
	Serial.print(F("Ala_Vie = "))									,Serial.println(config.Ala_Vie);
	Serial.print(F("Fin jour = "))								,Serial.println(config.FinJour);
	Serial.print(F("Interval reveil JCirc (s)= ")),Serial.println(config.RepeatWakeUp);
	Serial.print(F("Tempo Analyse Alarme (s)= ")) ,Serial.println(config.Tanalyse);
	Serial.print(F("TimeOut Alarme Jour (s)= "))  ,Serial.println(config.Jour_Nmax);
	Serial.print(F("TimeOut Alarme Nuit (s)= "))	,Serial.println(config.Nuit_Nmax);
	Serial.print(F("Tempo Sortie (s)= "))				  ,Serial.println(config.tempoSortie);
	Serial.print(F("Time Out Eclairage (s)= "))   ,Serial.println(config.timeOutS);
	Serial.print(F("Time Out Wifi (s)= "))				,Serial.println(config.timeoutWifi);
	Serial.print(F("Alarme sur Pedale 1 = ")) 	  ,Serial.println(config.Pedale1);
	Serial.print(F("Alarme sur Pedale 2 = ")) 	  ,Serial.println(config.Pedale2);
	Serial.print(F("Alarme sur Porte = ")) 	      ,Serial.println(config.Porte);
}
//---------------------------------------------------------------------------
void Extinction(){
	Allumage(0);
	Alarm.disable(TempoSortie);
	Alarm.disable(TimeOut);
	MajLog(F("Auto"),F("Extinction"));
}
//---------------------------------------------------------------------------
void Allumage(byte n){	
	/* 
		n=0 extinction auto
		n=1 Commande depuis pedale 1
		n=2 Commande depuis pedale 2 
	*/
	
	static byte Al1 = 0;
	static byte Al2 = 0;
	byte Cd1 = 0;
	byte Cd2 = 0;
	switch(n){
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
	
	if(!Allume){	// si pas Allumé
		Serial.println(F("                   Allumage"));
		digitalWrite(PinEclairage, HIGH);
		Allume = true;
		CptAllumage ++;
		if(n==1)Al1=1;
		if(n==2)Al2=1;
		Alarm.enable(TimeOut);
		MajLog(F("Auto"),F("Allumage"));
	}
	else{	// si Allumé
		if(n == 0){
			digitalWrite(PinEclairage, LOW);
			Allume = false;					
		}
		else if(Al1 == Cd2 || Al2 == Cd1){			
			Serial.print(F("                   Extinction dans (s) ")),Serial.println(config.tempoSortie);
			Alarm.enable(TempoSortie);
			Serial.print(F("Nombre Allumage = ")),Serial.println(CptAllumage);
		}
	}
}
//---------------------------------------------------------------------------
void ConnexionWifi(char* ssid,char* pwd, char* number, bool sms){
	
	Serial.print(F("connexion Wifi:")),Serial.print(ssid),Serial.print(char(44)),Serial.println(pwd);
	String ip;
	WiFi.begin(ssid, pwd);
	// WiFi.mode(WIFI_STA);
	byte timeout = 0;
	bool error = false;
	
	while (WiFi.status() != WL_CONNECTED) {
		delay(1000);
		Serial.print(".");
		timeout ++;
		if(timeout > 60){
			error = true;
			break;
		}
	}
	Serial.println();
	Serial.println(F("WiFi connected"));
	Serial.print(F("IP address: "));
	ip = WiFi.localIP().toString();
	Serial.println(ip);
	ArduinoOTA.begin();
	
	server.on("/",         HomePage);
  server.on("/download", File_Download);
  server.on("/upload",   File_Upload);
  server.on("/fupload",  HTTP_POST,[](){ server.send(200);}, handleFileUpload);
	server.on("/delete",   File_Delete);
  server.on("/dir",      SPIFFS_dir);
	server.on("/timeremaining", handleTime); // renvoie temps restant sur demande
	server.on("/wifioff",  WifiOff);
  ///////////////////////////// End of Request commands
  server.begin();
  Serial.println(F("HTTP server started"));
	
	message = Id;
	if(!error){
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
	else{
		message+= F("Connexion Wifi impossible");
	}
	EnvoyerSms(number, sms);
	
	if(sms){ // suppression du SMS
		/* Obligatoire ici si non bouclage au redemarrage apres timeoutwifi
		ou OTA sms demande Wifi toujours present */
		// bool err = Sim800l.delSms(slot);
		// Serial.print(F("resultat del Sms "));
		// Serial.println(err);
		// if(!err){ // 2eme essai si echec
			// err = Sim800l.delAllSms();
			// Serial.print(F("resultat del Sms "));
			// Serial.println(err);
		// }
		bool err;
		byte n = 0;
		do {
			err = Sim800l.delSms(slot);
			n ++;
			Serial.print(F("resultat del Sms "));	Serial.println(err);
			if(n > 10){ // on efface tous si echec
				Sim800l.delAllSms();
				break;
			}
		} while(!err);
		
	}
	debut = millis();
	if(!error){
		/* boucle permettant de faire une mise à jour OTA et serveur, avec un timeout en cas de blocage */
		unsigned long timeout = millis();
		while(millis() - timeout < config.timeoutWifi*1000){
			// if(WiFi.status() != WL_CONNECTED) break; // wifi a été coupé on sort			
			ArduinoOTA.handle();
			server.handleClient(); // Listen for client connections
			delay(1);
		}
		WifiOff();
	}
}
//---------------------------------------------------------------------------
void WifiOff(){
	Serial.println(F("Wifi off"));
	WiFi.disconnect(true);
	WiFi.mode(WIFI_OFF);
	btStop();
	Alarm.delay(100);
	ESP.restart();
}
//---------------------------------------------------------------------------
String ExtraireSms(String msgbrut){ //Extraction du contenu du SMS
	
	int pos[10];									// SMS jusqu'a 5 lignes
	int i = 0;
	for(i = 0;i < 10; i++){
		if(i == 0){
			pos[i] = msgbrut.indexOf("\n");
		}
		else{
			pos[i]= msgbrut.indexOf("\n", pos[i-1]+1);
		}		
		// Serial.print(i),Serial.print(" pos = "),Serial.println(pos[i]);
		if(pos[i] == -1){
			i --;
			break;
		}
	}
	
	String message = msgbrut.substring(pos[1]+1,pos[i-1]-1);
	// Serial.print("message extrait = "),Serial.println(message);
	message.replace("\n", "|");				// remplacement des sauts de lignes par |
	message = message.substring(0,message.length()-2);
	// Serial.print("message extrait sans \n= "),Serial.println(message);
	
	return message;
}
//---------------------------------------------------------------------------
int moyenneAnalogique(int Pin){	// calcul moyenne 10 mesures consécutives 
	int moyenne = 0;
  for (int j = 0; j < 10; j++) {
    Alarm.delay(10);
		moyenne += analogRead(Pin);
  }
  moyenne /= 10;
	return moyenne;
}
//---------------------------------------------------------------------------
int Battpct(long vbat){
	// retourn etat batterie en %
	int EtatBat = 0;
	if (vbat > 1260) {
		EtatBat = 100;
	}
	else if (vbat > 1255) {
		EtatBat = 95;
	}
	else if (vbat > 1250) {
		EtatBat = 90;
	}
	else if (vbat > 1246) {
		EtatBat = 85;
	}
	else if (vbat > 1242) {
		EtatBat = 80;
	}
	else if (vbat > 1237) {
		EtatBat = 75;
	}
	else if (vbat > 1232) {
		EtatBat = 70;
	}
	else if (vbat > 1226) {
		EtatBat = 65;
	}
	else if (vbat > 1220) {
		EtatBat = 60;
	}
	else if (vbat > 1213) {
		EtatBat = 55;
	}
	else if (vbat > 1206) {
		EtatBat = 50;
	}
	else if (vbat > 1198) {
		EtatBat = 45;
	}
	else if (vbat > 1190) {
		EtatBat = 40;
	}
	else if (vbat > 1183) {
		EtatBat = 35;
	}
	else if (vbat > 1175) {
		EtatBat = 30;
	}
	else if (vbat > 1167) {
		EtatBat = 25;
	}
	else if (vbat > 1158) {
		EtatBat = 20;	
	}
	else if (vbat > 1145) {
		EtatBat = 15;
	}
	else if (vbat > 1131) {
		EtatBat = 10;
	}
	else if (vbat > 1100) {
		EtatBat = 5;
	}
	else if (vbat <= 1050) {
		EtatBat = 0;
	}
	return EtatBat;
}
//---------------------------------------------------------------------------
void OuvrirFichierCalibration(){ // Lecture fichier calibration
	
	if(SPIFFS.exists(filecalibration)){
		File f = SPIFFS.open(filecalibration, "r");
		for(int i = 0;i < 3;i++){ //Read
			String s = f.readStringUntil('\n');
			CoeffTension[i] = s.toFloat();
			// Serial.print(i),Serial.print(" "),Serial.println(s);
			// if(i==0)CoeffTension[0] = s.toFloat();
			// if(i==1)CoeffTension[1] = s.toFloat();
			// if(i==2){
				// CoeffTension[2] = s.toFloat();
				// Serial.print("CoeffTension3 = "),Serial.print(CoeffTension[2]);//tracer changement valeur
			// }
		}
		f.close();
	}
	else{
		Serial.print(F("Creating Data File:")),Serial.println(filecalibration);// valeur par defaut
		CoeffTension[0] = 6600;
		CoeffTension[1] = 6600;
		CoeffTension[2] = 6600;
		Recordcalib();
	}
	Serial.print(F("Coeff T Batterie = ")),Serial.print(CoeffTension[0]);
	Serial.print(F(" Coeff T Proc = "))	  ,Serial.print(CoeffTension[1]);
	Serial.print(F(" Coeff T VUSB = "))		,Serial.println(CoeffTension[2]);
	
}
//---------------------------------------------------------------------------
void Recordcalib(){ // enregistrer fichier calibration en SPIFFS
	
	// Serial.print(F("Coeff T Batterie = ")),Serial.println(CoeffTension1);
	// Serial.print(F("Coeff T Proc = "))	  ,Serial.println(CoeffTension2);
	// Serial.print(F("Coeff T VUSB = "))		,Serial.println(CoeffTension3);
	File f = SPIFFS.open(filecalibration,"w");
	f.println(CoeffTension[0]);
	f.println(CoeffTension[1]);
	f.println(CoeffTension[2]);
	f.close();
	
}
//---------------------------------------------------------------------------
String Hdectohhmm(long Hdec){
	String hhmm;
	if(int(Hdec / 3600) < 10) hhmm = "0";
	hhmm += int(Hdec / 3600);
	hhmm += ":";
	if(int((Hdec % 3600) / 60) < 10) hhmm += "0";
	hhmm += int((Hdec % 3600) / 60);
	hhmm += ":";
	if(int((Hdec % 3600) % 60) <10); hhmm += "0";
	hhmm += int((Hdec % 3600) % 60);
	return hhmm;
}
//---------------------------------------------------------------------------
void DesActiveInterrupt(){
	
	if(config.Pedale1){
		detachInterrupt(digitalPinToInterrupt(PinPedale1));
	}
	if(config.Pedale2){
		detachInterrupt(digitalPinToInterrupt(PinPedale2));
	}
	if(config.Porte){
		detachInterrupt(digitalPinToInterrupt(PinPorte));
	}
}
//---------------------------------------------------------------------------
void ActiveInterrupt(){
	
	if(config.Pedale1){
		attachInterrupt(digitalPinToInterrupt(PinPedale1), handleInterruptP1, RISING);
	}
	if(config.Pedale2){
		attachInterrupt(digitalPinToInterrupt(PinPedale2), handleInterruptP2, RISING);
	}
	if(config.Porte){
		attachInterrupt(digitalPinToInterrupt(PinPorte)  , handleInterruptPo, RISING);
	}
}
//---------------------------------------------------------------------------
void AIntru_HeureActuelle(){
	
	long Heureactuelle =HActuelledec();
	
	if(config.FinJour > config.Ala_Vie){
		if((Heureactuelle > config.FinJour && Heureactuelle > config.Ala_Vie)
		 ||(Heureactuelle < config.FinJour && Heureactuelle < config.Ala_Vie)){
			// Nuit
			IntruD();
		}
		else{	// Jour
			IntruF();
		}
	}
	else{
		if(Heureactuelle > config.FinJour && Heureactuelle < config.Ala_Vie){
		 // Nuit
			IntruD();
		}
		else{	// Jour
			IntruF();
		}
	}
}	
//---------------------------------------------------------------------------
void IntruF(){// Charge parametre Alarme Intrusion Jour
	Nmax = config.Jour_Nmax;
	jour = true;
	Serial.println(F("Jour"));
}
//---------------------------------------------------------------------------
void IntruD(){// Charge parametre Alarme Intrusion Nuit
	Nmax = config.Nuit_Nmax;
	jour = false;
	Serial.println(F("Nuit"));
}
//---------------------------------------------------------------------------
void DebutSleep(){
	 //If you were to use ext1, you would use it like
  esp_sleep_enable_ext1_wakeup(BUTTON_PIN_BITMASK,ESP_EXT1_WAKEUP_ANY_HIGH);
	
	esp_sleep_enable_timer_wakeup(TIME_TO_SLEEP * uS_TO_S_FACTOR);
  Serial.print(F("Setup ESP32 to sleep for "));
	Serial.println(Hdectohhmm(TIME_TO_SLEEP));
	Serial.flush();
  //Go to sleep now
  Serial.println(F("Going to sleep now"));
	
	Sim800l.sleep();
	Serial.flush();
  esp_deep_sleep_start();
	
  Serial.println(F("This will never be printed"));
	Serial.flush();
	
}
//---------------------------------------------------------------------------
void action_wakeup_reason(){ // action en fonction du wake up
	
	esp_sleep_wakeup_cause_t wakeup_reason;
  wakeup_reason = esp_sleep_get_wakeup_cause();
	uint64_t wakeup_pin_mask = esp_sleep_get_ext1_wakeup_status();
	
  switch(wakeup_reason){
		case ESP_SLEEP_WAKEUP_EXT0 : break; // ne rien faire
			
		case ESP_SLEEP_WAKEUP_EXT1 :
			
			if (wakeup_pin_mask != 0) {
				int pin = __builtin_ffsll(wakeup_pin_mask)-1;
				Serial.print(F("Wake up from GPIO ")); Serial.print(String(pin));
			} else {
				Serial.println(F(" Wake up from GPIO ?"));					
			}
			/* declenchement externe pendant deep sleep
				si nuit ou jour noncirculé
				on reste en fonctionnement pendant TempoAnalyse 
				avant retour deep sleep*/
			if(!jour ||(jour && calendrier[month()][day()] == 0)){
				WupAlarme = true;
				LastWupAlarme = true;
				Alarm.enable(TempoAnalyse); // debut tempo analyse ->fonctionnement normal
				Sbidon = F("Externe ");
				Sbidon += String(wakeup_reason);
				MajLog(F("Auto"),Sbidon);
			}
			break;
			
		case ESP_SLEEP_WAKEUP_TIMER :
			/* jour noncirculé retour deep sleep pour RepeatWakeUp 1H00 
			verifier si wake up arrive avant fin journée marge 3mn*/
			if(calendrier[month()][day()] == 0){
				if(HActuelledec() < config.FinJour - config.RepeatWakeUp - 180){
					TIME_TO_SLEEP = config.RepeatWakeUp; 
				}
				else{
					TIME_TO_SLEEP = DureeSleep(config.FinJour - 180);
				}
				Sbidon = F("lance timer 1H ");
				Sbidon += Hdectohhmm(TIME_TO_SLEEP);
				MajLog(F("Auto"),Sbidon);
				DebutSleep();
			}
			else{ // jour circulé
				/*  ne rien faire  */
			}
			break;
			
    case ESP_SLEEP_WAKEUP_TOUCHPAD : break; // ne rien faire
    case ESP_SLEEP_WAKEUP_ULP : break; // ne rien faire
    default: break; // demarrage normal	
	}
	Serial.flush();
}
//---------------------------------------------------------------------------
void HomePage(){
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
	webpage += F("<tr>");
	webpage += F("<td>Debut Jour</td>");
	webpage += F("<td>");	webpage += Hdectohhmm(config.Ala_Vie);	webpage += F("</td>");
	webpage += F("</tr>");
	webpage += F("<tr>");
	webpage += F("<td>Fin Jour</td>");
	webpage += F("<td>");	webpage += Hdectohhmm(config.FinJour);	webpage += F("</td>");
	webpage += F("</tr>");
	webpage += F("<tr>");
	webpage += F("<td>Nmax Jour (s)</td>");
	webpage += F("<td>");	webpage += String(config.Jour_Nmax*10);	webpage += F("</td>");
	webpage += F("</tr>");
	webpage += F("<tr>");
	webpage += F("<td>Nmax Nuit (s)</td>");
	webpage += F("<td>");	webpage += String(config.Nuit_Nmax*10);	webpage += F("</td>");
	webpage += F("</tr>");
	webpage += F("<tr>");
	webpage += F("<td>Tempo Analyse Alarme (s)</td>");
	webpage += F("<td>");	webpage += String(config.Tanalyse);	webpage += F("</td>");
	webpage += F("</tr>");
	webpage += F("<tr>");
	webpage += F("<td>Interval Reveil Jour Circul&eacute; (s)</td>");
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
	if(config.Intru){webpage += F("Active");} else {webpage += F("Inactive");} webpage += F("</td>");
	webpage += F("</tr>");
	webpage += F("<tr>");
	webpage += F("<td>Silence</td>");
	webpage += F("<td>");	
	if(config.Silence){webpage += F("ON");} else {webpage += F("OFF");} webpage += F("</td>");
	webpage += F("</tr>");
	webpage += F("<tr>");
	webpage += F("<td>Alarme sur Pedale 1</td>");
	webpage += F("<td>");	
	if(config.Pedale1){webpage += F("Active");} else {webpage += F("Inactive");} webpage += F("</td>");
	webpage += F("</tr>");
	webpage += F("<tr>");
	webpage += F("<td>Alarme sur Pedale 2</td>");
	webpage += F("<td>");	
	if(config.Pedale2){webpage += F("Active");} else {webpage += F("Inactive");} webpage += F("</td>");
	webpage += F("</tr>");
	webpage += F("<tr>");
	webpage += F("<td>Alarme sur Porte</td>");
	webpage += F("<td>");	
	if(config.Porte){webpage += F("Active");} else {webpage += F("Inactive");} webpage += F("</td>");
	webpage += F("</tr>");
	
	
	webpage += F("</table><br>");
	
  webpage += F("<a href='/download'><button>Download</button></a>");
  webpage += F("<a href='/upload'><button>Upload</button></a>");
  webpage += F("<a href='/delete'><button>Delete</button></a>");
  webpage += F("<a href='/dir'><button>Directory</button></a>");
	webpage += F("<a href='/wifioff'><button>Wifi Off</button></a>");
  append_page_footer();
  SendHTML_Content();
  SendHTML_Stop(); // Stop is needed because no content length was sent
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
void File_Download(){ // This gets called twice, the first pass selects the input, the second pass then processes the command line arguments
  if (server.args() > 0 ) { // Arguments were received
    if (server.hasArg("download")) DownloadFile(server.arg(0));
  }
  else SelectInput("Enter filename to download","download","download");
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
void DownloadFile(String filename){
  if (SPIFFS_present) { 
    File download = SPIFFS.open("/"+filename,  "r");
    if (download) {
      server.sendHeader("Content-Type", "text/text");
      server.sendHeader("Content-Disposition", "attachment; filename="+filename);
      server.sendHeader("Connection", "close");
      server.streamFile(download, "application/octet-stream");
      download.close();
    } else ReportFileNotPresent("download"); 
  } else ReportSPIFFSNotPresent();
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
void File_Upload(){
  append_page_header();
  webpage += F("<h3>Select File to Upload</h3>"); 
  webpage += F("<FORM action='/fupload' method='post' enctype='multipart/form-data'>");
  webpage += F("<input class='buttons' style='width:40%' type='file' name='fupload' id = 'fupload' value=''><br>");
  webpage += F("<br><button class='buttons' style='width:10%' type='submit'>Upload File</button><br>");
  webpage += F("<a href='/'>[Back]</a><br><br>");
  append_page_footer();
  server.send(200, "text/html",webpage);
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
void handleFileUpload(){ // upload a new file to the Filing system
  HTTPUpload& uploadfile = server.upload(); // See https://github.com/esp8266/Arduino/tree/master/libraries/ESP8266WebServer/srcv
                                            // For further information on 'status' structure, there are other reasons such as a failed transfer that could be used
  if(uploadfile.status == UPLOAD_FILE_START)
  {
    String filename = uploadfile.filename;
    if(!filename.startsWith("/")) filename = "/"+filename;
    Serial.print(F("Upload File Name: ")); Serial.println(filename);
    SPIFFS.remove(filename);                  // Remove a previous version, otherwise data is appended the file again
    UploadFile = SPIFFS.open(filename, "w");  // Open the file for writing in SPIFFS (create it, if doesn't exist)
  }
  else if (uploadfile.status == UPLOAD_FILE_WRITE)
  {
    if(UploadFile) UploadFile.write(uploadfile.buf, uploadfile.currentSize); // Write the received bytes to the file
  } 
  else if (uploadfile.status == UPLOAD_FILE_END)
  {
    if(UploadFile)          // If the file was successfully created
    {                                    
      UploadFile.close();   // Close the file again
      Serial.print(F("Upload Size: ")); Serial.println(uploadfile.totalSize);
      webpage = "";
      append_page_header();
      webpage += F("<h3>File was successfully uploaded</h3>"); 
      webpage += F("<h2>Uploaded File Name: "); webpage += uploadfile.filename+"</h2>";
      webpage += F("<h2>File Size: "); webpage += file_size(uploadfile.totalSize) + "</h2><br>"; 
      append_page_footer();
      server.send(200,"text/html",webpage);
    } 
    else
    {
      ReportCouldNotCreateFile("upload");
    }
  }
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
void SPIFFS_dir(){ 
  if (SPIFFS_present) { 
    File root = SPIFFS.open("/");
    if (root) {
      root.rewindDirectory();
      SendHTML_Header();
      webpage += F("<h3 class='rcorners_m'>SPIFFS Contents</h3><br>");
      webpage += F("<table align='center'>");
      webpage += F("<tr><th>Name/Type</th><th style='width:20%'>Type File/Dir</th><th>File Size</th></tr>");
      printDirectory("/",0);
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
void printDirectory(const char * dirname, uint8_t levels){
  File root = SPIFFS.open(dirname);
  if(!root){
    return;
  }
  if(!root.isDirectory()){
    return;
  }
  File file = root.openNextFile();
  while(file){
    if (webpage.length() > 1000) {
      SendHTML_Content();
    }
    if(file.isDirectory()){
      webpage += "<tr><td>"+String(file.isDirectory()?"Dir":"File")+"</td><td>"+String(file.name())+"</td><td></td></tr>";
      printDirectory(file.name(), levels-1);
    }
    else
    {
      webpage += "<tr><td>"+String(file.name())+"</td>";
      webpage += "<td>"+String(file.isDirectory()?"Dir":"File")+"</td>";
      int bytes = file.size();
      String fsize = "";
      if (bytes < 1024)                     fsize = String(bytes)+" B";
      else if(bytes < (1024 * 1024))        fsize = String(bytes/1024.0,3)+" KB";
      else if(bytes < (1024 * 1024 * 1024)) fsize = String(bytes/1024.0/1024.0,3)+" MB";
      else                                  fsize = String(bytes/1024.0/1024.0/1024.0,3)+" GB";
      webpage += "<td>"+fsize+"</td></tr>";
    }
    file = root.openNextFile();
  }
  file.close();
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
void File_Delete(){
  if (server.args() > 0 ) { // Arguments were received
    if (server.hasArg("delete")) SPIFFS_file_delete(server.arg(0));
  }
  else SelectInput("Select a File to Delete","delete","delete");
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
void SPIFFS_file_delete(String filename) { // Delete the file 
  if (SPIFFS_present) { 
    SendHTML_Header();
    File dataFile = SPIFFS.open("/"+filename, "r"); // Now read data from SPIFFS Card 
    if (dataFile)
    {
      if (SPIFFS.remove("/"+filename)) {
        Serial.println(F("File deleted successfully"));
        webpage += "<h3>File '"+filename+"' has been erased</h3>"; 
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
void SendHTML_Header(){
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
void SendHTML_Content(){
  server.sendContent(webpage);
  webpage = "";
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
void SendHTML_Stop(){
  server.sendContent("");
  server.client().stop(); // Stop is needed because no content length was sent
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
void SelectInput(String heading1, String command, String arg_calling_name){
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
void ReportSPIFFSNotPresent(){
  SendHTML_Header();
  webpage += F("<h3>No SPIFFS Card present</h3>"); 
  webpage += F("<a href='/'>[Back]</a><br><br>");
  append_page_footer();
  SendHTML_Content();
  SendHTML_Stop();
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
void ReportFileNotPresent(String target){
  SendHTML_Header();
  webpage += F("<h3>File does not exist</h3>"); 
  webpage += F("<a href='/"); webpage += target + "'>[Back]</a><br><br>";
  append_page_footer();
  SendHTML_Content();
  SendHTML_Stop();
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
void ReportCouldNotCreateFile(String target){
  SendHTML_Header();
  webpage += F("<h3>Could Not Create Uploaded File (write-protected?)</h3>"); 
  webpage += F("<a href='/"); webpage += target + "'>[Back]</a><br><br>";
  append_page_footer();
  SendHTML_Content();
  SendHTML_Stop();
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
String file_size(int bytes){
  String fsize = "";
  if (bytes < 1024)                 fsize = String(bytes)+" B";
  else if(bytes < (1024*1024))      fsize = String(bytes/1024.0,3)+" KB";
  else if(bytes < (1024*1024*1024)) fsize = String(bytes/1024.0/1024.0,3)+" MB";
  else                              fsize = String(bytes/1024.0/1024.0/1024.0,3)+" GB";
  return fsize;
}
//---------------------------------------------------------------------------
void handleTime(){
	char time_str[9];
	const uint32_t millis_in_day    = 1000 * 60 * 60 * 24;
	const uint32_t millis_in_hour   = 1000 * 60 * 60;
	const uint32_t millis_in_minute = 1000 * 60;
	
	static unsigned long t0 = 0;
	if(millis() - debut > config.timeoutWifi*1000) debut = millis();// securité evite t<0
	t0= debut + (config.timeoutWifi*1000) - millis();
	// Serial.print(debut),Serial.print("|"),Serial.println(t0);
	
	uint8_t days     = t0 / (millis_in_day);
	uint8_t hours    = (t0 - (days * millis_in_day)) / millis_in_hour;
	uint8_t minutes  = (t0 - (days * millis_in_day) - (hours * millis_in_hour)) / millis_in_minute;
	uint8_t secondes = (t0 - (days * millis_in_day) - ((hours * millis_in_hour)) / millis_in_minute)/1000 %60;
	sprintf(time_str, "%02d:%02d:%02d", hours, minutes, secondes);
	// Serial.println(time_str); 
 server.send(200, "text/plane", String(time_str)); //Send Time value only to client ajax request
}

/* --------------------  test local serial seulement ----------------------*/
void recvOneChar() {
  if (Serial.available() > 0) {
    receivedChar = Serial.read();
    demande += receivedChar;
    if (receivedChar == 10) {
      newData = true;
    }
  }
}

void showNewData() {
  if (newData == true) {
    Serial.println(demande);
    interpretemessage();
    newData = false;
  }
}
void interpretemessage() {
  String bidons;
  //demande.toUpperCase();
  if (demande.indexOf(char(61)) == 0) {
    bidons = demande.substring(1); //(demande.indexOf(char(61))+1);
    int lon0 = bidons.length();
    bidons.toCharArray(replybuffer, lon0 - 1);// len-1 pour supprimer lf
    //Serial.print(lon0),Serial.print(char(44)),Serial.print(bidons),Serial.print(char(44)),Serial.println(replybuffer);
    traite_sms(99);//	traitement SMS en mode test local
  }

  demande = "";
}
//---------------------------------------------------------------------------
