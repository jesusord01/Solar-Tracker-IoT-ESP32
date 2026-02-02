/* Sketch para Panel Solar con Monitoreo y Arduino IoT Cloud
*/

#include "thingProperties.h"
#include <LiquidCrystal_I2C.h>
#include <ESP32Servo.h>

// ==========================================
// CONFIGURACIÓN DE PINES Y VARIABLES
// ==========================================

// --- Pines LDR (Entradas Analógicas) ---
const int LDR_TOP_LEFT = 34;     // GPIO34
const int LDR_TOP_RIGHT = 35;    // GPIO35
const int LDR_BOTTOM_LEFT = 32;  // GPIO32
const int LDR_BOTTOM_RIGHT = 33; // GPIO33
const int LIGHT_THRESHOLD = 10;  // Umbral de diferencia de luz

// --- Servos ---
Servo servo_horizontal;
Servo servo_vertical;
const int PIN_SERVO_V = 18;      // GPIO18
const int PIN_SERVO_H = 19;      // GPIO19

// Límites y Posición Inicial de Servos
int pos_sh = 90;
int pos_sv = 90;
const int UPPER_LIMIT_POS = 160;
const int LOWER_LIMIT_POS = 20;

// --- Sensor Corriente ACS712 ---
const int CURRENT_SENSOR = 36;   // GPIO36 (VP)
const float SENSIBILITY = 0.185; // 0.185 V/A para modelo 5A
const int CURRENT_SAMPLES = 100; // Muestras para promedio
const float ZERO_POINT_VOLTAGE = 2.5; // Punto medio teórico (VCC/2)

// --- Sensor de Batería (Divisor de Voltaje) ---
const int PIN_BATERIA = 39;      // GPIO39 (VN)
float R1 = 100000.0;             // 100k a Positivo
float R2 = 100000.0;             // 100k a GND
float voltajeReferencia = 3.3;   // Voltaje de referencia del ESP32

// --- LCD I2C ---
LiquidCrystal_I2C lcd(0x27, 16, 2);

// --- Variables de Tiempo ---
unsigned long lastTime = 0;
unsigned long threshold = 2000; // Actualizar LCD y Nube cada 2 segundos

// ==========================================
// SETUP
// ==========================================
void setup() {
  // Initialize serial and wait for port to open:
  Serial.begin(9600); 
  delay(1500); 

  // --- 1. Inicialización Arduino IoT Cloud ---
  initProperties(); // Defined in thingProperties.h
  ArduinoCloud.begin(ArduinoIoTPreferredConnection);
  
  setDebugMessageLevel(2);
  ArduinoCloud.printDebugInfo();

  // --- 2. Configuración Hardware (ADC, LCD, Servos) ---
  
  // CONFIGURACIÓN CRÍTICA DEL ADC
  analogReadResolution(12);       // 12 bits = 0 a 4095
  analogSetAttenuation(ADC_11db); // Permite leer hasta ~3.3V

  // Inicializar LCD
  lcd.init();
  lcd.backlight();
  lcd.setCursor(0, 0);
  lcd.print("Conectando...");
  lcd.setCursor(0, 1);
  lcd.print("IoT Cloud...");

  // Inicializar Servos
  servo_vertical.attach(PIN_SERVO_V);
  servo_horizontal.attach(PIN_SERVO_H);
  
  // Posición inicial
  servo_vertical.write(pos_sv);
  servo_horizontal.write(pos_sh);
  
  // Esperar un momento a que la conexión se estabilice
  delay(2000); 
  lcd.clear();
  lcd.print("Sistema Online");
  delay(1000);
  lcd.clear();
}

// ==========================================
// LOOP PRINCIPAL
// ==========================================
void loop() {
  // 1. Actualizar conexión con la Nube (CRÍTICO)
  ArduinoCloud.update();

  // 2. Lógica del Seguidor Solar (Siempre activa)
  int ldr_tl = analogRead(LDR_TOP_LEFT);
  int ldr_tr = analogRead(LDR_TOP_RIGHT);
  int ldr_bl = analogRead(LDR_BOTTOM_LEFT);
  int ldr_br = analogRead(LDR_BOTTOM_RIGHT);

  int avg_top = (ldr_tl + ldr_tr) / 2;
  int avg_bottom = (ldr_bl + ldr_br) / 2;
  int avg_left = (ldr_tl + ldr_bl) / 2;
  int avg_right = (ldr_tr + ldr_br) / 2;

  moveSolarTracker(avg_top, avg_bottom, avg_left, avg_right);

  // 3. Medición, Pantalla y Actualización de Variables Nube (Cada 2 segundos)
  if ((millis() - lastTime) > threshold) {
    lastTime = millis();

    // --- A. Medir Corriente ---
    float current_measured = medirCorriente(CURRENT_SENSOR, SENSIBILITY, CURRENT_SAMPLES);

    // --- B. Medir Batería ---
    long sumaBat = 0;
    for(int i = 0; i < 30; i++){
      sumaBat += analogRead(PIN_BATERIA);
      delay(2);
    }
    float adcValorBat = sumaBat / 30.0;
    
    float voltajePin = (adcValorBat * voltajeReferencia) / 4095.0;
    float voltajeBateriaCalculado = voltajePin * ((R1 + R2) / R2);
    
    int porcentaje = map(voltajeBateriaCalculado * 100, 300, 420, 0, 100);
    if (porcentaje > 100) porcentaje = 100;
    if (porcentaje < 0) porcentaje = 0;

    // --- C. Actualizar Variables de Arduino IoT Cloud ---
    // Asignamos los valores leídos a las variables de la nube
    corriente = current_measured; 
    voltaje = voltajeBateriaCalculado;

    // --- D. Mostrar en LCD ---
    lcd.clear();
    
    // Fila 0: Corriente
    lcd.setCursor(0, 0);
    lcd.print("I:");
    lcd.print(current_measured, 2); 
    lcd.print("A Cloud"); // Indicador visual de que envía datos

    // Fila 1: Batería
    lcd.setCursor(0, 1);
    lcd.print("Bat:");
    lcd.print(voltajeBateriaCalculado, 1);   
    lcd.print("V ");
    lcd.print(porcentaje);
    lcd.print("%");
  }

  delay(30); 
}

// ==========================================
// FUNCIONES AUXILIARES
// ==========================================

void moveSolarTracker(int avg_top, int avg_bottom, int avg_left, int avg_right) {
  // Movimiento Vertical
  if (abs(avg_top - avg_bottom) > LIGHT_THRESHOLD) {
    if ((avg_top > avg_bottom) && (pos_sv < UPPER_LIMIT_POS)) {
      pos_sv++;
      servo_vertical.write(pos_sv);
    } else if ((avg_bottom > avg_top) && (pos_sv > LOWER_LIMIT_POS)) {
      pos_sv--;
      servo_vertical.write(pos_sv);
    }
  }

  // Movimiento Horizontal
  if (abs(avg_left - avg_right) > LIGHT_THRESHOLD) {
    if ((avg_left > avg_right) && (pos_sh < UPPER_LIMIT_POS)) {
      pos_sh++;
      servo_horizontal.write(pos_sh);
    } else if ((avg_right > avg_left) && (pos_sh > LOWER_LIMIT_POS)) {
      pos_sh--;
      servo_horizontal.write(pos_sh);
    }
  }
}

float medirCorriente(int pin, float sensitivity, int samples) {
  float voltageAcc = 0;
  
  for (int i = 0; i < samples; i++) {
    int adcValue = analogRead(pin);
    float voltage = adcValue * (3.3 / 4095.0);
    voltageAcc += voltage;
    delayMicroseconds(200); 
  }

  float avgVoltage = voltageAcc / samples;
  float current = -1 * ((avgVoltage - ZERO_POINT_VOLTAGE) / sensitivity);

  if (abs(current) < 0.05) {
    current = 0.0;
  }

  return current;
}