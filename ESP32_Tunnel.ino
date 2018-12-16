/* Ph CORBEL 12/2018
Gestion eclairage tunnel
Alimentation sur panneaux solaires

mode deep sleep 
reveille tout les matin 06h55
reception des SMS en attente
apres 5 min de fonctionnement
envoie sms signal vie 
analyse calendrier sauvegardé en SPIFFS
si jour circulé 
on continue normalement
en fin de journée retour sleep jusqu'a 06h55
pas d'option wake up à une heure donnée
il faut donc calculer une durée de sleep(quid precision RTC)
si non circulé, 
retour SIM800 et ESP32 en sleep pour 23h55

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

debug a faire alarme cable/porte

armer interrupt apres lancement

 */
 
#include <credentials_home.h>

#include <Sim800l.h>              //my SIM800 modifié
#include <Time.h>
#include <TimeAlarms.h>	
#include <sys/time.h>
#include <WiFi.h>
#include <EEPROM.h>               // variable en EEPROM
#include "SPIFFS.h"
#include <ArduinoOTA.h>
// #include "coeff.h"							// coefficient mesure tension

#define PinBattProc		35   // liaison interne carte Lolin32 adc
#define PinBattSol		34   // Batterie générale 12V adc
#define PinBattUSB		36   // V USB 5V adc VP
#define PinPedale1		32   // Entrée Pedale1
#define PinPedale2		33   // Entrée Pedale2
#define PinPorte   		39   // Entrée Porte Coffret VN
#define PinEclairage	21   // Sortie Commande eclairage
#define PinSirene			0    // Sortie Commande Sirene
#define PIN						1234 // Code PIN carte SIM

byte calendrier[13][32];
char filecalendrier[13]  = "/filecal.csv";  // fichier en SPIFFS contenant le calendrier de circulation
char filecalibration[12] = "/coeff.txt";    // fichier en SPIFFS contenant le calendrier de calibration
const String soft	= "ESP32_Tunnel.ino.d32"; // nom du soft
String	ver       = "V1-1";
int Magique       = 2345;
String message;
String bufferrcpt;
String fl = "\n";                   //	saut de ligne SMS
String Id ;                         // 	Id du materiel sera lu dans EEPROM
char    SIM800InBuffer[64];         //	for notifications from the SIM800
char replybuffer[255];              // 	Buffer de reponse SIM800
volatile int IRQ_Cpt_PDL1 = 0;
volatile int IRQ_Cpt_PDL2 = 0;
int Cpt_PDL1 = 0;
int Cpt_PDL2 = 0;
volatile unsigned long rebond1 = 0;		//	antirebond IRQ	
volatile unsigned long rebond2 = 0;
byte confign = 0;					// Num enregistrement EEPROM
bool Allume = false;
bool FlagAlarmeTension       = false;// Alarme tension Batterie
bool FlagLastAlarmeTension   = false;
bool FlagAlarmeIntrusion     = false;// Alarme Defaut Cable detectée
bool FlagAlarmeCable1        = false; // Alamre Cable Pedale1
bool FlagAlarmeCable2        = false; // Alamre Cable Pedale2
bool FlagAlarmePorte         = false; // Alamre Porte Coffret
bool FlagLastAlarmeIntrusion = false;
bool FirstSonn = false;				// Premier appel sonnerie
bool SonnMax   = false;				// temps de sonnerie maxi atteint
bool FlagReset = false;
// byte CptAlarme1 = 0;
// byte CptAlarme2 = 0;
int     CoeffTension1;				// Coeff calibration Tension Batterie
int     CoeffTension2;				// Coeff calibration Tension Proc
int     CoeffTension3;				// Coeff calibration Tension USB
int  CoeffTensionDefaut = 7000;// Coefficient par defaut

RTC_DATA_ATTR int CptAllumage = 0; // Nombre Allumage par jour en memoire RTC
byte slot = 0;            			//this will be the slot number of the SMS
char 		receivedChar;
bool newData = false;
String 	demande;
long TensionBatterie  = 0; // Tension Batterie solaire
long VBatterieProc    = 0; // Tension Batterie Processeur
long VUSB             = 0; // Tension USB

typedef struct											// declaration structure  pour les log
{
	char 		dt[10];									//	DateTime 0610-1702 9+1
	char 		Act[2];									//	Action A/D/S/s 1+1 
	char 		Name[15];								//	14 car
} champ;
champ record[5];

byte recordn = 100;							// Num enregistrement EEPROM

struct  config_t 									// Structure configuration sauvée en EEPROM
{
  int 		magic				;					// num magique
  long  	Ala_Vie 		;					// Heure message Vie, 7h matin en seconde = 7*60*60
	long  	FinJour 		;					// Heure fin jour, 20h matin en seconde = 20*60*60
	int     Tlancement  ;         // Temps lancement, prise decision circulé/noncirculé
  int			tempoSortie ;					// tempo eclairage apres sorties(s)
  int			timeOutS	 	;					// tempo time out eclairage (s)
	int 		tempPDL 		;					// tempo entre n coups pedale(ms)
	int			Cpt_PDL			;					// Nombre de coup de pedale pour declencher 1 à n
	int			timeoutWifi ;					// tempo coupure Wifi si pas de mise a jour (s)
	int 		Dsonn 	;							// Durée Sonnerie
  int 		DsonnMax;							// Durée Max Sonnerie
  int 		Dsonnrepos;						// Durée repos Sonnerie
  bool    Intru   ;							// Alarme Intrusion active
  bool    Silence ;							// Mode Silencieux = true false par defaut
	bool    Pedale1;              // Alarme Pedale1 Active
	bool    Pedale2;              // Alarme Pedale2 Active
	bool    Porte;                // Alarme Porte Active
	char 		Idchar[11];						// Id
} ;
config_t config;

AlarmId OneH;
AlarmId loopPrincipale;	// boucle principlae
AlarmId Svie;						// tempo Signal de Vie et MajHeure
AlarmId FirstMessage;		// Premier message analyse jour circulé O/N
AlarmId TempoSortie;		// Temporisation eclairage a la sortie
AlarmId TimeOut;				// TimeOut Allumage
AlarmId FinJour;				// Fin de journée retour deep sleep
AlarmId TSonn;					// 4 tempo durée de la sonnerie
AlarmId TSonnMax;				// 5 tempo maximum de sonnerie
AlarmId TSonnRepos;			// 6 tempo repos apres maxi

portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;

HardwareSerial *fonaSerial = &Serial2;// liaison serie FONA SIM800
Sim800l Sim800l;  											//to declare the library

//---------------------------------------------------------------------------
	void IRAM_ATTR handleInterruptP1() {
		if (millis() - rebond1 > 100){
			portENTER_CRITICAL_ISR(&mux);
			IRQ_Cpt_PDL1++;
			portEXIT_CRITICAL_ISR(&mux);
			rebond1 = millis();
		}
	}
	void IRAM_ATTR handleInterruptP2() {
		if (millis() - rebond2 > 100){
			portENTER_CRITICAL_ISR(&mux);
			IRQ_Cpt_PDL2++;
			portEXIT_CRITICAL_ISR(&mux);
			rebond2 = millis();
		}
	}
//---------------------------------------------------------------------------

void setup() {
	message.reserve(140);
	
  Serial.begin(115200);
	Serial.println();
	Serial.println(F("lancement SIM800"));
  fonaSerial->begin(9600); // 4800
	Sim800l.begin();
	
	pinMode(PinEclairage,OUTPUT);
  pinMode(PinPedale1  ,INPUT_PULLUP);
	pinMode(PinPedale2  ,INPUT_PULLUP);
	pinMode(PinPorte    ,INPUT_PULLUP);
	pinMode(PinSirene   ,OUTPUT);
	digitalWrite(PinEclairage, LOW);
	digitalWrite(PinSirene, LOW);
	
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
		config.Tlancement    = 5*60;
		config.Intru         = false;
		config.Silence       = true;
		config.Dsonn         = 60;
		config.DsonnMax      = 90;
		config.Dsonnrepos    = 120;
		config.tempoSortie   = 10;
		config.timeOutS      = 60;// 3600
		config.tempPDL       = 3000;
		config.Cpt_PDL       = 1;
		config.timeoutWifi   = 10*60;
		config.Pedale1       = true;
		config.Pedale2       = true;
		config.Porte         = true;
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
			Serial.println("Start updating " + type);
		})
		.onEnd([]() {
			Serial.println(F("End"));
		})
		.onProgress([](unsigned int progress, unsigned int total) {
			Serial.printf("Progress: %u%%\r", (progress / (total / 100)));
		})
		.onError([](ota_error_t error) {
			Serial.printf("Error[%u]: ", error);
			if 			(error == OTA_AUTH_ERROR) 		Serial.println("Auth Failed");
			else if (error == OTA_BEGIN_ERROR) 		Serial.println("Begin Failed");
			else if (error == OTA_CONNECT_ERROR) 	Serial.println("Connect Failed");
			else if (error == OTA_RECEIVE_ERROR) 	Serial.println("Receive Failed");
			else if (error == OTA_END_ERROR) 			Serial.println("End Failed");
		});
	
	OuvrirCalendrier();					// ouvre calendrier circulation en SPIFFS
	OuvrirFichierCalibration(); // ouvre fichier calibration en SPIFFS
	Sim800l.reset(PIN);// lancer SIM800	
	Sim800l.getRSSI();
	Alarm.delay(1000);
	
	MajHeure();
	
  OneH = Alarm.timerRepeat(3600,test);
	
	loopPrincipale = Alarm.timerRepeat(10, Acquisition); // boucle principale 15s
  Alarm.enable(loopPrincipale);

	FirstMessage = Alarm.timerOnce(config.Tlancement, OnceOnly); // appeler une fois apres 5min type=0
	Alarm.enable(FirstMessage);
	
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
	
	attachInterrupt(digitalPinToInterrupt(PinPedale1), handleInterruptP1, RISING);
	attachInterrupt(digitalPinToInterrupt(PinPedale2), handleInterruptP2, RISING);
	
}
//---------------------------------------------------------------------------
void loop() {
	
	static unsigned long T01 = 0;
	static unsigned long T02 = 0;
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
	
	if(IRQ_Cpt_PDL1 > 0 || IRQ_Cpt_PDL2 > 0){ 
		Serial.print(F("Interruption : "));
		Serial.print(IRQ_Cpt_PDL1);
		Serial.print(" ");
		Serial.println(IRQ_Cpt_PDL2);
		
		if(IRQ_Cpt_PDL1 > 0){
			T01 = millis();
			Serial.print(F("pedale1=")),Serial.println(Cpt_PDL1);
			Cpt_PDL1 ++;
			if(Cpt_PDL1 > config.Cpt_PDL - 1){
				Cpt_PDL1 = 0;
				Allumage(1);
			}
		}
		if(IRQ_Cpt_PDL2 > 0){
			T02 = millis();
			Serial.print(F("pedale2=")),Serial.println(Cpt_PDL2);
			Cpt_PDL2 ++;
			if(Cpt_PDL2 > config.Cpt_PDL - 1){
				Cpt_PDL2 = 0;
				Allumage(2);
			}
		}
		
		portENTER_CRITICAL(&mux);
		if(IRQ_Cpt_PDL1 > 0)IRQ_Cpt_PDL1 = 0;
		portEXIT_CRITICAL(&mux);
	
		portENTER_CRITICAL(&mux);
		if(IRQ_Cpt_PDL2 > 0)IRQ_Cpt_PDL2 = 0;
		portEXIT_CRITICAL(&mux);	
	}
	if(Cpt_PDL1 >0 && (millis() - T01 > config.tempPDL)) Cpt_PDL1 = 0;//timeout pedale
	if(Cpt_PDL2 >0 && (millis() - T02 > config.tempPDL)) Cpt_PDL2 = 0;//timeout pedale
	
	ArduinoOTA.handle();
	Alarm.delay(1);
	
}	//fin loop
//---------------------------------------------------------------------------
void Acquisition(){	
	static byte CptAlarmeCable = 0;
	
	if(!Sim800l.getetatSIM())Sim800l.reset(PIN);// verification SIM
	Serial.print(displayTime(0));

	static byte nalaTension = 0;
	static byte nRetourTension = 0;
	TensionBatterie  = map(moyenneAnalogique(PinBattSol), 0, 4095, 0, CoeffTension1);
	VBatterieProc = map(moyenneAnalogique(PinBattProc), 0, 4095, 0, CoeffTension2);
	VUSB  = map(moyenneAnalogique(PinBattUSB), 0, 4095, 0, CoeffTension3);
	if(Battpct(TensionBatterie) < 25 || VUSB < 4000 || VUSB > 6000){
		nalaTension ++;
    if (nalaTension == 4) {
      FlagAlarmeTension = true;
      nalaTension = 0;
    }
	}
	else if (TensionBatterie > 75 && VUSB > 4800 && VUSB < 5400) {	//hysteresis et tempo sur Alarme Batterie
    nRetourTension ++;
		if(nRetourTension == 4){
			FlagAlarmeTension = false;			
			nRetourTension =0;
		}
  }
  else {
    if (nalaTension > 0)nalaTension--;		//	efface progressivement le compteur
  }

	message = F(", Batt Solaire = ");
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
		// verif sur 3 passages consecutifs
		if (!digitalRead(PinPorte) && config.Porte){
			nalaPorte ++;		
			if(nalaPorte > 1){
				// CptAlarme1 = 1;
				// FausseAlarme1 = 1000;//V2-11
				FlagAlarmeIntrusion = true;
				FlagAlarmePorte = true;
				nalaPorte = 0;
			}
		}
		else{
			if (nalaPorte > 0) nalaPorte --;		//	efface progressivement le compteur
		}
		if (!digitalRead(PinPedale1) && config.Pedale1){
			nalaPIR1 ++;		
			if(nalaPIR1 > 3){
				// CptAlarme1 = 1;
				// FausseAlarme1 = 1000;//V2-11
				FlagAlarmeIntrusion = true;
				FlagAlarmeCable1 = true;
				nalaPIR1 = 0;
			}
		}
		else{
			if (nalaPIR1 > 0) nalaPIR1 --;		//	efface progressivement le compteur
		}
		if (!digitalRead(PinPedale2) && config.Pedale2){
			nalaPIR2 ++;		
			if(nalaPIR2 > 3){
				// CptAlarme2 = 1;
				// FausseAlarme2 = 1000;//V2-11
				FlagAlarmeIntrusion = true;
				FlagAlarmeCable2 = true;
				nalaPIR2 = 0;
			}
		}
		else{
			if (nalaPIR2 > 0) nalaPIR2 --;		//	efface progressivement le compteur
		}
		Serial.print("Pedale 1 enfonce"),Serial.println(nalaPIR1);
		Serial.print("Pedale 2 enfonce"),Serial.println(nalaPIR2);
		if(FlagAlarmeIntrusion){
			ActivationSonnerie();		// activation Sonnerie
			Serial.println(F("Alarme Cable/Porte"));
		}
	}
	else{
		FlagAlarmeIntrusion = false;// efface alarme 
		FlagAlarmeCable1 = false;
		FlagAlarmeCable2 = false;
		FlagAlarmePorte = false;
		// CptAlarme1 = 0;
		// CptAlarme2 = 0;
	}		
	Serial.printf("Nala Porte = %d ,",nalaPorte);
	Serial.printf("Nala Ped 1 = %d ,",nalaPIR1);
	Serial.printf("Nala Ped 2 = %d\n",nalaPIR2);
	
	
	/* verification nombre SMS en attente(raté en lecture directe)
		 traitement des sms en memeoire un par un, 
		 pas de traitement en serie par commande 51, traitement beaucoup trop long */
		 
  int8_t smsnum = Sim800l.getNumSms(); // nombre de SMS en attente
  Serial.print(F("Sms en attente = ")), Serial.println (smsnum);

  if (smsnum > 0) {	// nombre de SMS en attente
    // il faut les traiter
		int numsms = Sim800l.getIndexSms(); // cherche l'index des sms en mémoire
    traite_sms(numsms);// traitement des SMS en attente
  }
  else if (smsnum == 0 && FlagReset) { // on verifie que tous les SMS sont traités avant Reset
    FlagReset = false;
    ESP.restart();				//	reset soft
  }

	envoie_alarme();
	
	digitalWrite(LED_PIN,0);
	Alarm.delay(50);
	digitalWrite(LED_PIN,1);
	
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
	
	char text[50];
	char rep2[50];
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
		Serial.print("textesms  = "),Serial.println(textesms);
		
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
				Serial.println("Wifi off");
				WiFi.disconnect(true);
				WiFi.mode(WIFI_OFF);
				btStop();
				Alarm.delay(100);
				// WiFi.forceSleepBegin();
				// esp_wifi_stop();
				message += F("Wifi off");
				message += fl;
				EnvoyerSms(number, true);
			}
			else if(textesms.indexOf(F("Wifi"))== 0){ // demande connexion Wifi
				byte pos1 = textesms.indexOf(char(44));
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
				message += String(float(TensionBatterie/100.0)) + ",";
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
					// AllumeCapteur();									// allumage des capteurs selon parametres
					// V1-12 on attache les interruptions
					// if (config.PirActif[0]) attachInterrupt(digitalPinToInterrupt(Ip_PIR1), IRQ_PIR1, RISING);
					// if (config.PirActif[1]) attachInterrupt(digitalPinToInterrupt(Ip_PIR2), IRQ_PIR2, RISING);
					// if (config.PirActif[2]) attachInterrupt(digitalPinToInterrupt(Ip_PIR3), IRQ_PIR3, RISING);
					// if (config.PirActif[3]) attachInterrupt(digitalPinToInterrupt(Ip_PIR4), IRQ_PIR4, RISING);
					if(!sms){															//V2-14
						nom = F("console");
						// bidon.toCharArray(nom,8);//	si commande locale
					}
					logRecord(nom,"A"); // V2-14 renseigne le log
				}
				generationMessage();
				EnvoyerSms(number, sms);
      }
			else if(textesms.indexOf(F("INTRUOFF")) == 0 
					 || textesms.indexOf(("D"+Id.substring(6,10))) == 0){ //	Desarmement
				if(config.Intru) {
					config.Intru = !config.Intru;
					sauvConfig();														// sauvegarde en EEPROM
					/*	Arret Sonnerie au cas ou? sans envoyer SMS */
					digitalWrite(PinSirene, LOW);	// Arret Sonnerie
					Alarm.disable(TSonn);			// on arrete la tempo sonnerie
					Alarm.disable(TSonnMax);	// on arrete la tempo sonnerie maxi
					// V1-12 on detache les interruptions
					// if (config.PirActif[0]) detachInterrupt(digitalPinToInterrupt(Ip_PIR1));
					// if (config.PirActif[1]) detachInterrupt(digitalPinToInterrupt(Ip_PIR2));
					// if (config.PirActif[2]) detachInterrupt(digitalPinToInterrupt(Ip_PIR3));
					// if (config.PirActif[3]) detachInterrupt(digitalPinToInterrupt(Ip_PIR4));
					// digitalWrite(Op_PIR1, LOW); // on etteint les capteurs PIR TX et RX
					// digitalWrite(Op_PIR2, LOW);
					// digitalWrite(Op_PIR3, LOW);
					// digitalWrite(Op_PIR4, LOW);
					FirstSonn = false;
					FlagAlarmeIntrusion = false;
					FlagAlarmeCable1 = false;
					FlagAlarmeCable2 = false;
					FlagAlarmePorte = false;
					// FlagPIR = false;
					if(!sms){															//V2-14
						nom = F("console");
						// bidon.toCharArray(nom,8);//	si commande locale
					}
					logRecord(nom,"D");				// V2-14 renseigne le log
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
			else if(textesms.indexOf(F("CAPTEUR")) == 0){	// Capteurs actif CAPTEUR=1,0,1 (Pedale1,Pedale2,Porte)
				bool flag = true; // validation du format
				if (textesms.indexOf(char(61))== 7) { //char(61) "="	liste capteur actif
					byte Num[3];
					String bidon=textesms.substring(8,13);
					Serial.print("bidon="),Serial.print(bidon),Serial.println(bidon.length());
					if (bidon.length() == 5){
						int j=0;
						for (int i = 0;i < 5; i +=2){
							if(bidon.substring(i,i+1) == "0" || bidon.substring(i,i+1) == "1"){
								// Serial.print(",="),Serial.println(bidon.substring(i+1,i+2));
								// Serial.print("X="),Serial.println(bidon.substring(i,i+1));
								Num[j] = bidon.substring(i,i+1).toInt();
								// Serial.print(i),Serial.print(","),Serial.print(j),Serial.print(","),Serial.println(Num[j]);
								j++;
							}
							else{									
								Serial.println(F("Format non reconnu"));
								flag = false;// format pas bon		
							}						
						}
						if(flag){ // sauv configuration
							config.Pedale1 = Num[0];
							config.Pedale2 = Num[1];
							config.Porte   = Num[2];
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
			else if(textesms.indexOf(F("CPTPEDALE")) == 0){// compteur pedale avant armement
				if (textesms.indexOf(char(61)) == 9){ // =
					int i = textesms.substring(10).toInt();
					// Serial.print("Cpt pedale = "),Serial.println(i);
					if(i > 0 && i < 10){
						config.Cpt_PDL = i;
						sauvConfig();                               // sauvegarde en EEPROM
					}
				}
				message += F("Compteur Pedale = ");
				message += config.Cpt_PDL;
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
				message += int(config.Ala_Vie / 3600);
				message += ":";
				message += int((config.Ala_Vie % 3600) / 60);
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
				message += int(config.FinJour / 3600);
				message += ":";
				message += int((config.FinJour % 3600) / 60);
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
						message += F("Mise à jour calendrier OK");
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
				String bidon = textesms.substring(12,16);// texte apres =
				//Serial.print(F("bidon=")),Serial.print(bidon),Serial.print(char(44)),Serial.println(bidon.length());
				long tension = 0;
				if(bidon.substring(0,1) == "." && bidon.length() > 1){// debut mode cal					
					if(bidon.substring(1,2) == "1" ){M = 1; P = PinBattSol; coef = CoeffTension1;}
					if(bidon.substring(1,2) == "2" ){M = 2; P = PinBattProc; coef = CoeffTension2;}
					if(bidon.substring(1,2) == "3" ){M = 3; P = PinBattUSB; coef = CoeffTension3;}
					Serial.print("mode = "),Serial.print(M),Serial.println(bidon.substring(1,2));
					FlagCalibration = true;
					
					coef = CoeffTensionDefaut;
					tension = map(moyenneAnalogique(P), 0,4095,0,coef);
					// Serial.print("TensionBatterie = "),Serial.println(TensionBatterie);
					tensionmemo = tension;
				}
				else if(FlagCalibration && bidon.substring(0,4).toInt() > 0 && bidon.substring(0,4).toInt() <=8000){
					// si Calibration en cours et valeur entre 0 et 5000
					Serial.println(bidon.substring(0,4));
					/* calcul nouveau coeff */
					coef = bidon.substring(0,4).toFloat()/float(tensionmemo)*CoeffTensionDefaut;
					// Serial.print("Coeff Tension = "),Serial.println(CoeffTension);
					tension = map(moyenneAnalogique(P), 0,4095,0,coef);
					switch(M){
						case 1:
							CoeffTension1 = coef;
							break;
						case 2:
							CoeffTension2 = coef;
							break;
						case 3:
							CoeffTension3 = coef;
							break;
					}
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
    FlagLastAlarmeTension = FlagAlarmeTension;
  }
  if (FlagAlarmeIntrusion != FlagLastAlarmeIntrusion) {
    SendEtat = true;
    FlagLastAlarmeIntrusion = FlagAlarmeIntrusion;
  }
  if (SendEtat) { 							// si envoie Etat demandé
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
    message += F("-- Cable coupe !--") ;		// Intrusion !
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
		Serial.print("resultat sms "),Serial.println(error);
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
			Alarm.disable(loopPrincipale);
			Alarm.disable(Svie);
			Alarm.disable(TempoSortie);
			Alarm.disable(TimeOut);
			Alarm.disable(FinJour);	
			
			setTime(Nhour,Nminute,Nsecond,Nday,Nmonth,Nyear);
			
			Alarm.enable(loopPrincipale);
			Alarm.enable(TempoSortie);
			Alarm.enable(TimeOut);
			Alarm.enable(FinJour);
			Alarm.enable(Svie);
		}		
	}
	displayTime(0);
	
	/* test */
	char dateheure[20];
	sprintf(dateheure,"%02d/%02d/%d %02d:%02d:%02d",Nday,Nmonth,Nyear,Nhour,Nminute,Nsecond);
	message = Id;
	message += ecart;
	message += fl;
	message += String(dateheure);
	message += fl;
	message += displayTime(0);
	message += fl;
	Serial.println(message);
	
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
void OnceOnly(){	
	// Analyse si jour circulé lancement normal
	// lecture calendrier
	OuvrirCalendrier(); // ouvre calendrier circulation en SPIFFS
	if(calendrier[month()][day()] == 0){
		// si non retour, deep sleep SIM800 et ESP32
		Serial.println(F("Jour non circulé")); // on entre en sleep pour 23h55
		
		// Sim800l.sleep();
		// on calcul la durée de sleep = t maintenant jusqu'a Svie
		Serial.print(F("Duree sleep = ")),Serial.println(DureeSleep());
		
		// ESP sleeptime;
	}
	Serial.print(F("Duree sleep = ")),Serial.println(DureeSleep());
	Serial.println(F("Jour circulé")); // on continue normalement
}
//---------------------------------------------------------------------------
long DureeSleep(){
	/* calcul durée entre maintenant et
	heure Vie-5mn, 5 mn(Tlancement)*/
	long SleepTime = 0;
	long Heureactuelle = hour()*60;// calcul en 4 lignes sinon bug!
	Heureactuelle += minute();
	Heureactuelle  = Heureactuelle*60;
	Heureactuelle += second(); // en secondes
	if(Heureactuelle < config.Ala_Vie){
		SleepTime = config.Ala_Vie - Heureactuelle;
	}
	else{
		if(Heureactuelle < 86400){// < 24h00
			SleepTime = (86400 - Heureactuelle) + config.Ala_Vie;
		}
	}
	return SleepTime - config.Tlancement;
}
//---------------------------------------------------------------------------
void SignalVie(){
	Serial.println(F("Signal vie"));
	MajHeure();
	CptAllumage = 0;
	envoieGroupeSMS(0);
	Sim800l.delAllSms();// au cas ou, efface tous les SMS envoyé/reçu
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
void deleteFile(fs::FS &fs, const char * path){
	// Serial.printf("Deleting file: %s\r\n", path);
	if(fs.remove(path)){
		// Serial.println("- file deleted");
	} else {
		// Serial.println("- delete failed");
	}
}
//---------------------------------------------------------------------------
void EnregistreCalendrier(){ // remplace le nouveau calendrier
	bool result = SPIFFS.begin();
	deleteFile(SPIFFS,filecalendrier);
	String bidon="";
	char bid[63];
	for(int m = 1; m < 13;m++){
		for(int j = 1; j < 32; j++){
			bidon += calendrier[m][j];
		}
		Serial.println(bidon);
		bidon += fl;
		bidon.toCharArray(bid,63);
		appendFile(SPIFFS, filecalendrier, bid);
		bidon = "";
	}
	SPIFFS.end();
}
//---------------------------------------------------------------------------
void OuvrirCalendrier(){
	// always use this to "mount" the filesystem
	bool result = SPIFFS.begin();
	Serial.println("SPIFFS opened: " + result);

	// this opens the file "f.txt" in read-mode
	listDir(SPIFFS, "/", 0);
	bool f = SPIFFS.exists(filecalendrier);
	Serial.println(f);

	if (!f) {
		Serial.println(F("File doesn't exist yet. Creating it")); // creation calendrier defaut
		char bid[63];
		String bidon="";
		for(int m = 1; m < 13;m++){
			for(int j = 1; j < 32; j++){
				if(m == 1 || m == 2 || m == 3 || m == 11 || m == 12){
					bidon += "0;";
				}
				else{
					bidon += "1;";
				}
			}
			Serial.println(bidon);
			bidon += fl;
			bidon.toCharArray(bid,63);
			appendFile(SPIFFS, filecalendrier, bid);
			bidon = "";
		}
		f = SPIFFS.exists(filecalendrier);
		if (!f) {
			// Serial.println("file creation failed");
		// }else{Serial.println("file creation OK");}
		} else {
		Serial.println(F("Read file"));
		// we could open the file
		}
	}
	readFile(SPIFFS, filecalendrier);
	for(int m = 1; m < 13;m++){
		for(int j = 1; j < 32; j++){
			Serial.print(calendrier[m][j]),Serial.print(char(44));
		}
		Serial.println();
	}
	listDir(SPIFFS, "/", 0);
	
	SPIFFS.end();
}
//---------------------------------------------------------------------------
void FinJournee(){
	// fin de journée retour deep sleep
	Serial.print(F("Fin de journée retour sleep a terminer"));
	Serial.print(F("Durée sleep = ")),Serial.println(DureeSleep());
}
//---------------------------------------------------------------------------
void PrintEEPROM(){
	Serial.print(F("Version = "))									,Serial.println(ver);
	Serial.print(F("ID = "))											,Serial.println(config.Idchar);
	Serial.print(F("magic = "))										,Serial.println(config.magic);
	Serial.print(F("Ala_Vie = "))									,Serial.println(config.Ala_Vie);
	Serial.print(F("Fin jour = "))								,Serial.println(config.FinJour);
	Serial.print(F("Lancement (s) = "))						,Serial.println(config.Tlancement);
	Serial.print(F("Comptage Pedale = "))       	,Serial.println(config.Cpt_PDL);
	Serial.print(F("Tempo Pedale (ms) = "))				,Serial.println(config.tempPDL);
	Serial.print(F("Tempo Sortie (s) = "))				,Serial.println(config.tempoSortie);
	Serial.print(F("Time Out Eclairage (s) = "))	,Serial.println(config.timeOutS);
	Serial.print(F("Time Out Wifi (s) = "))				,Serial.println(config.timeoutWifi);
	Serial.print(F("Pedale 1 Alarme Active = ")) 	,Serial.println(config.Pedale1);
	Serial.print(F("Pedale 2 Alarme Active = ")) 	,Serial.println(config.Pedale2);
	Serial.print(F("Porte Alarme Active = ")) 	  ,Serial.println(config.Porte);
}
//---------------------------------------------------------------------------
void Extinction(){
	Allumage(0);
	Alarm.disable(TempoSortie);
	Alarm.disable(TimeOut);
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
	
	Serial.print(F("Sub Allumage avec n = ")),Serial.print(n);
	Serial.print(F(" Al1,Al2 = ")),Serial.print(Al1),Serial.print(char(44)),Serial.println(Al2);
	
	if(!Allume){	// si pas Allumé
		Serial.println(F("                   Allumage"));
		digitalWrite(PinEclairage, HIGH);
		Allume = true;
		CptAllumage ++;
		if(n==1)Al1=1;
		if(n==2)Al2=1;
		Alarm.enable(TimeOut);
	}
	else{	// si Allumé
		if(n == 0){
			digitalWrite(PinEclairage, LOW);
			Allume = false;					
		}
		else if(Al1 == Cd2 || Al2 == Cd1){			
			Serial.print(F("                   Extinction dans (s) ")),Serial.println(config.tempoSortie);
			Alarm.enable(TempoSortie);
			Serial.println(CptAllumage);
		}
	}
}
//---------------------------------------------------------------------------
void ConnexionWifi(char* ssid,char* pwd, char* number, bool sms){
	
	Serial.print(F("connexion Wifi:")),Serial.print(ssid),Serial.print(char(44)),Serial.println(pwd);
	String ip;
	WiFi.begin(ssid, pwd);
	WiFi.mode(WIFI_STA);
	byte timeout = 0;
	bool error = false;
	while (WiFi.status() != WL_CONNECTED) {
		Alarm.delay(1000);
		Serial.print(".");
		timeout ++;
		if(timeout > 60){
			error = true;
			break;
		}
	}
	Serial.println();
	Serial.println(F("WiFi connected"));
	Serial.println(F("IP address: "));
	ip = WiFi.localIP().toString();
	Serial.println(ip);
	ArduinoOTA.begin();
	
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
		Serial.print(F("resultat del Sms "));
		Serial.println(Sim800l.delSms(slot));
	}
	
	if(!error){
		/* boucle permettant de faire une mise à jour OTA, avec un timeout en cas de blocage */
		unsigned long debut = millis();
		while(millis() - debut < config.timeoutWifi*1000){
			Alarm.delay(1);
			ArduinoOTA.handle();
		}
		Serial.println("Wifi off");
		WiFi.disconnect(true);
		WiFi.mode(WIFI_OFF);
		btStop();
		Alarm.delay(100);
	}
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
	bool result = SPIFFS.begin();
	if(SPIFFS.exists(filecalibration)){
		File f = SPIFFS.open(filecalibration, "r");
		for(int i = 0;i < 3;i++){ //Read
			String s = f.readStringUntil('\n');
			// Serial.print(i),Serial.print(" "),Serial.println(s);
			if(i==0)CoeffTension1 = s.toFloat();
			if(i==1)CoeffTension2 = s.toFloat();
			if(i==2)CoeffTension3 = s.toFloat();
		}
		f.close();
	}
	else{
		Serial.print(F("Creating Data File:")),Serial.println(filecalibration);// valeur par defaut
		CoeffTension1 = 6600;
		CoeffTension2 = 6600;
		CoeffTension3 = 6600;
		Recordcalib();
	}
	Serial.print(F("Coeff T Batterie = ")),Serial.println(CoeffTension1);
	Serial.print(F("Coeff T Proc = "))	  ,Serial.println(CoeffTension2);
	Serial.print(F("Coeff T VUSB = "))		,Serial.println(CoeffTension3);
	SPIFFS.end();
}
//---------------------------------------------------------------------------
void Recordcalib(){ // enregistrer fichier calibration en SPIFFS
	bool result = SPIFFS.begin();
	// Serial.print(F("Coeff T Batterie = ")),Serial.println(CoeffTension1);
	// Serial.print(F("Coeff T Proc = "))	  ,Serial.println(CoeffTension2);
	// Serial.print(F("Coeff T VUSB = "))		,Serial.println(CoeffTension3);
	File f = SPIFFS.open(filecalibration,"w");
	f.println(CoeffTension1);
	f.println(CoeffTension2);
	f.println(CoeffTension3);
	f.close();
	SPIFFS.end();
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
