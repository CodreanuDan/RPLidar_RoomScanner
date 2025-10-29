#include <SoftwareSerial.h>

// Pini Arduino Uno pentru LIDAR (SoftwareSerial)
#define LIDAR_RX_PIN 10  // Pinul D10 (conectat la TX-ul LIDAR-ului)
#define LIDAR_TX_PIN 11  // Pinul D11 (conectat la RX-ul LIDAR-ului)

// Pin PWM pentru controlul motorului LIDAR
#define MOTOR_PWM_PIN 9  

// Instanta SoftwareSerial pentru LIDAR
SoftwareSerial LidarSerial(LIDAR_RX_PIN, LIDAR_TX_PIN);

// Constante pentru comunicarea LIDAR
const unsigned char CMD_STOP[2] = {0xA5, 0x25};
const unsigned char CMD_SCAN_STANDARD[2] = {0xA5, 0x20};
const unsigned char CMD_GET_HEALTH[2] = {0xA5, 0x52}; 

// Răspunsul așteptat la A5 20 (Descriptor de Răspuns)
const unsigned char SCAN_DESCRIPTOR[7] = {0xA5, 0x5A, 0x05, 0x00, 0x00, 0x40, 0x81};

// Constante pentru controlul timpului și vitezei
const unsigned long SCAN_DURATION_MS = 5000; // 5 secunde
const int MOTOR_SPEED_PWM = 200; // Viteza motorului (din 255)

// Variabile de stare
bool is_scanning = false; 
unsigned long scan_start_time = 0; 

// Declaratii functii
void setMotorSpeed(int pwm_value);
void sendLidarCommand(const unsigned char *cmd, size_t len, const char* name);
void clearLidarBuffer(); 
bool waitForDescriptor();
bool waitForHealthResponse();
void startStandardScan();
void stopScan();

// Funcție pentru a curăța bufferul SoftwareSerial agresiv
void clearLidarBuffer() {
    Serial.print("Curatare buffer LidarSerial...");
    while (LidarSerial.available()) {
        LidarSerial.read();
    }
    Serial.println("OK.");
    delay(20); 
}

/***************************************************
 * Functia de Setup (Initializare)
 ***************************************************/
void setup() {
    Serial.begin(115200);
    delay(100); 
    Serial.println("\n--- RPLIDAR CONTROL (Arduino Uno) ---");
    Serial.println("ATENTIE: Durata scanarii este 5 secunde. Mod agresiv de golire a bufferului serial.");

    LidarSerial.begin(115200);
    Serial.println("Lidar Serial (D10/D11) Initiat @ 115200bps.");

    pinMode(MOTOR_PWM_PIN, OUTPUT);
    setMotorSpeed(0); 
    
    Serial.println("------------------------------------------");
    Serial.println("Asteptati comanda 'start' (cu Newline) pentru a incepe masuratoarea...");
    Serial.println("------------------------------------------");
}

/***************************************************
 * Functia principala de Loop (Bucla)
 ***************************************************/
void loop() {
    // 1. GESTIONARE COMANDA 'START' DE LA PC
    if (Serial.available()) {
        String command = Serial.readStringUntil('\n');
        command.trim();
        command.toLowerCase();

        if (command == "start" && !is_scanning) {
            startStandardScan();
        } else if (command == "stop") {
            stopScan();
        } else if (command == "start" && is_scanning) {
            Serial.println("Masuratoarea este deja activa.");
        }
    }

    // 2. GESTIONARE FLUX DE DATE DE LA LIDAR (Afisare HEX in timp real)
    if (is_scanning && LidarSerial.available()) {
        uint8_t rx_byte = LidarSerial.read();
        
        // Tipărire rapidă
        if (rx_byte < 0x10) {
            Serial.print("0"); 
        }
        Serial.print(rx_byte, HEX);
        Serial.print(" ");
    }

    // 3. LOGICA DE TIMER (Controlul celor 5 secunde)
    if (is_scanning) {
        if (millis() - scan_start_time >= SCAN_DURATION_MS) {
            stopScan();
        }
    }
}

/***************************************************
 * Functii de Control si Debugging
 ***************************************************/

void setMotorSpeed(int pwm_value) {
    analogWrite(MOTOR_PWM_PIN, pwm_value);
    if (pwm_value > 0) {
        Serial.print("✅ Motor PORNIT la PWM: "); 
        Serial.println(pwm_value);
    } else {
        Serial.println("🛑 Motor OPRIT.");
    }
}

void sendLidarCommand(const unsigned char *cmd, size_t len, const char* name) {
    
    Serial.print("-> Trimitere ");
    Serial.print(name);
    Serial.print(": ");
    
    if (cmd[0] < 0x10) Serial.print("0"); 
    Serial.print(cmd[0], HEX);
    Serial.print(" ");
    
    if (cmd[1] < 0x10) Serial.print("0"); 
    Serial.print(cmd[1], HEX);
    Serial.println(); 
    
    LidarSerial.write(cmd, len);
    delay(5);
}

// Așteaptă și validează răspunsul Get Health (10 octeti)
bool waitForHealthResponse() {
    Serial.print("-> Astept raspuns Get Health (10 octeti): ");
    
    unsigned long timeout = millis() + 500;
    
    while(millis() < timeout && LidarSerial.available() < 10) {}
    
    if (LidarSerial.available() >= 10) {
        uint8_t buffer[10];
        LidarSerial.readBytes((char*)buffer, 10);
        
        Serial.println(); 
        Serial.print("   Primit: ");
        
        for (int i = 0; i < 10; i++) {
            if (buffer[i] < 0x10) Serial.print("0"); 
            Serial.print(buffer[i], HEX);
            Serial.print(" ");
        }
        
        if (buffer[0] == 0xA5 && buffer[1] == 0x5A) {
            Serial.println("\n✅ Raspuns de tip Descriptor primit (A5 5A)!");
            Serial.print("   STATUS: ");
            Serial.print(buffer[7] == 0x00 ? "GOOD" : "ERROR");
            Serial.print(", ErrorCode: ");
            if (buffer[8] < 0x10) Serial.print("0"); 
            Serial.print(buffer[8], HEX);
            if (buffer[9] < 0x10) Serial.print("0"); 
            Serial.println(buffer[9], HEX);
            return true;
        } else {
            Serial.println("\n❌ Antet raspuns (A5 5A) INVAILD.");
            return false;
        }
    }
    
    Serial.println("\n❌ Timeout. Nu s-a primit raspuns de 10 octeti de la LIDAR.");
    return false;
}

// Așteaptă și validează răspunsul descriptor de 7 octeți (pentru SCAN)
bool waitForDescriptor() {
    
    Serial.print("-> Astept raspuns descriptor SCAN (7 octeti: A5 5A 05 00 00 40 81): ");
    
    unsigned long timeout = millis() + 200; 
    
    while(millis() < timeout && LidarSerial.available() < 7) {}
    
    if (LidarSerial.available() >= 7) {
        uint8_t buffer[7];
        LidarSerial.readBytes((char*)buffer, 7); 
        
        Serial.println(); 
        
        Serial.print("   Primit: "); 
        for (int i = 0; i < 7; i++) {
            if (buffer[i] < 0x10) Serial.print("0"); 
            Serial.print(buffer[i], HEX);
            Serial.print(" ");
        }
        
        bool match = true;
        for (int i = 0; i < 7; i++) {
            if (buffer[i] != SCAN_DESCRIPTOR[i]) {
                match = false;
            }
        }
        
        if (match) {
            Serial.println("\n✅ Raspuns Descriptor SCAN VALID primit!");
            return true;
        } else {
            Serial.println("\n❌ Raspuns Descriptor SCAN INVAILD (sau date neasteptate).");
            return false;
        }
    }
    
    Serial.println("\n❌ Timeout. Nu s-a primit descriptorul SCAN de 7 octeti de la LIDAR.");
    return false;
}


// Secventa de pornire a scanării standard (FARA RESET)
void startStandardScan() {
    Serial.println("\n--- INITIERE MASURATOARE ---");

    setMotorSpeed(0); 
    delay(50);
    
    // 1. Oprim SCAN-ul precedent (siguranță)
    sendLidarCommand(CMD_STOP, 2, "STOP"); 
    
    // Curățăm bufferul SoftwareSerial înainte de GET HEALTH
    clearLidarBuffer();

    // 2. Test GET HEALTH
    sendLidarCommand(CMD_GET_HEALTH, 2, "GET HEALTH"); 
    if (!waitForHealthResponse()) {
        Serial.println("⛔ EROARE CRITICĂ: LIDAR-ul nu răspunde. Verificati cablarea D10/D11.");
        setMotorSpeed(0);
        return; 
    }
    
    // 3. PORNIM MOTORUL
    setMotorSpeed(MOTOR_SPEED_PWM);
    delay(500); // Așteptăm 0.5s să atingă viteza

    // Curățăm bufferul SoftwareSerial înainte de SCAN
    clearLidarBuffer();
    
    // 4. START SCAN Standard (A5 20)
    sendLidarCommand(CMD_SCAN_STANDARD, 2, "SCAN Standard"); 
    
    // 5. Așteptăm Descriptorul de Răspuns (7 octeți)
    if (!waitForDescriptor()) {
        Serial.println("ATENTIE: Descriptorul a fost ratat. Citind datele timp de 5s...");
        
        // NOU: Golim bufferul și începem cu o pagină curată fluxul de scanare
        clearLidarBuffer();
    }
    
    // 6. Setăm starea de scanare și timpul de start
    is_scanning = true;
    scan_start_time = millis(); 
    
    Serial.println("------------------------------------------");
    Serial.print(" FLUX DE DATE START (");
    Serial.print(SCAN_DURATION_MS / 1000);
    Serial.println("S) ");
    Serial.println("------------------------------------------");
}

// Secventa de oprire a scanării
void stopScan() {
    Serial.println("\n------------------------------------------");
    Serial.print("🛑 ");
    Serial.print(SCAN_DURATION_MS / 1000);
    Serial.println(" SECUNDE EXPIRATE. OPRIRE SCANARE");
    
    // 1. STOP Scan
    sendLidarCommand(CMD_STOP, 2, "STOP"); 

    // 2. Oprim motorul
    setMotorSpeed(0);

    // 3. Citim octeții rămași din buffer (dacă există)
    while (LidarSerial.available()) {
        uint8_t rx_byte = LidarSerial.read();
        if (rx_byte < 0x10) Serial.print("0"); 
        Serial.print(rx_byte, HEX);
        Serial.print(" ");
    }
    
    // 4. Actualizăm starea
    is_scanning = false;
    Serial.println("\n------------------------------------------");
    Serial.println("Scanare oprită. Tastati 'start' din nou.");
}