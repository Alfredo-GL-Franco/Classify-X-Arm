#include <Servo.h>

Servo s1, s2, s3, s4, s5;
String inputString = "";
boolean stringComplete = false;

void setup() {
    Serial.begin(9600);
    
    // Asignar servos a pines
    s1.attach(A0);
    s2.attach(A1);
    s3.attach(A2);
    s4.attach(A3);
    s5.attach(A4);
  // garra.attach(A5);

}

void loop() {
    if (stringComplete) {
        // Procesar la entrada recibida
        int servoNum = inputString.substring(1, 2).toInt();
        float angle = inputString.substring(3).toFloat();

        // Mapear el ángulo (0-360) a valores de servo (0-180)
        angle = constrain(angle, 0, 180);
        

        // Mover el servo correspondiente
        switch (servoNum) {
            case 1: s1.write(angle); break;
            case 2: s2.write(angle); break;
            case 3: s3.write(angle); break;
            case 4: s4.write(angle); break;
            case 5: s5.write(angle); break;
        }

        // Reiniciar la cadena de entrada
        inputString = "";
        stringComplete = false;
    }
    
  }


// Función para recibir datos seriales
void serialEvent() {
    while (Serial.available()) {
        char inChar = (char)Serial.read();
        if (inChar == '\n') {
            stringComplete = true;
        } else {
            inputString += inChar;
        }
    }
}
