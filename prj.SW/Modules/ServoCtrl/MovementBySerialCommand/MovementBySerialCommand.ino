#include <ESP32Servo.h>

Servo myServo;

#define SERVO_PIN 5

// --- Variabile ---
String command = "";
bool isRotating = false;
bool reverseDirection = false;
int currentAngle = 0;
unsigned long lastStepTime = 0;

// 1 sec = 5 grade/s -> delay intre pasi, aici o sa modificam la 1 secunda/pas
const unsigned long STEP_INTERVAL = 1000; 
const int STEP_SIZE = 1; 

void setup() 
{
    Serial.begin(115200);
    myServo.attach(SERVO_PIN);
    myServo.write(0);
    Serial.println("Servo ready. Commands: start, reverse, stop");
}

void loop() 
{
    // --- Cititre serial/buton/aplicatie ---
    while (Serial.available()) 
    {
        char c = Serial.read();
        if (c == '\n' || c == '\r') 
        {
            command.trim();

            if (command == "start") 
            {
                isRotating = true;
                reverseDirection = false;
                Serial.println("Rotating forward (5°/s)...");
            } 
            else if (command == "reverse") 
            {
                isRotating = true;
                reverseDirection = true;
                Serial.println("Rotating backward (5°/s)...");
            } 
            else if (command == "stop") 
            {
                isRotating = false;
                Serial.println("Stopped.");
            }

            command = "";
        } 
        else 
        {
            command += c;
        }
    }

    // --- Control miscare la interval de timp cu cate x grade ---
    if (isRotating && millis() - lastStepTime >= STEP_INTERVAL) 
    {
        lastStepTime = millis();

        if (!reverseDirection) 
        {
            currentAngle += STEP_SIZE;
            if (currentAngle >= 180) 
            {
                currentAngle = 180;
                isRotating = false;
                Serial.println("Reached 180° (end).");
            }
        } 
        else 
        {
            currentAngle -= STEP_SIZE;
            if (currentAngle <= 0) 
            {
                currentAngle = 0;
                isRotating = false;
                Serial.println("Reached 0° (start).");
            }
        }

        myServo.write(currentAngle);
        Serial.print("Angle: ");
        Serial.println(currentAngle);
    }
}
