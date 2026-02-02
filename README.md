# Smart Dual-Axis Solar Tracker con IoT (ESP32)

Este proyecto es un prototipo de **Eficiencia Energética** que optimiza la captación de luz solar mediante un sistema mecatrónico de seguimiento automático (Dual-Axis) y permite la supervisión remota de la generación de energía.

El sistema busca la posición de mayor incidencia de luz utilizando sensores LDR y reporta métricas de consumo y estado de batería a la nube en tiempo real.

!PanelSolar.jpg](PanelSolar.jpg)

## Características Principales

* **Seguimiento Solar Activo:** Algoritmo de control que compara la incidencia de luz en 4 cuadrantes (LDRs) para mover dos servomotores (Azimut y Elevación), aumentando la eficiencia del panel.
* **Telemetría IoT (Arduino Cloud):** Visualización remota de variables críticas:
    * Corriente generada (Amperios).
    * Voltaje de la batería.
    * Porcentaje de carga.
* **Programación No Bloqueante:** Implementación de temporizadores con `millis()` para asegurar que la comunicación con la nube no interrumpa el movimiento fluido de los motores.
* **Monitoreo Local:** Pantalla LCD I2C para visualización de datos "in-situ".

## Tecnologías y Hardware

* **Microcontrolador:** ESP32 (Elegido por su conectividad WiFi y doble núcleo).
* **Sensores:**
    * 4x LDR (Fotorresistencias) en configuración de puente.
    * ACS712 (Sensor de Corriente Hall).
    * Divisor de Tensión (Sensor de voltaje).
* **Actuadores:** 2x Servomotores SG90/MG995.
* **Plataforma IoT:** Arduino IoT Cloud.

## Diagrama de Funcionamiento

El código opera en dos bucles paralelos:
1.  **Bucle de Control (Tiempo Real):** Lee los sensores de luz y ajusta la posición de los servos constantemente.
2.  **Bucle de Telemetría (Intervalo 2s):** Muestrea el voltaje y corriente, promedia las lecturas para reducir ruido eléctrico y envía los paquetes de datos a la nube.

## Snippet de Código (Lógica de Muestreo)

El sistema utiliza un promedio de 100 muestras para obtener una lectura de corriente estable, filtrando el ruido del sensor ACS712:

Desarrollado por Edin Jesus Ordoñez Diaz - Ingeniería Mecatrónica UTP
```cpp
float medirCorriente(int pin, float sensitivity, int samples) {
  float voltageAcc = 0;
  for (int i = 0; i < samples; i++) {
    // Conversión ADC a Voltaje (ESP32 3.3V / 12-bit)
    float voltage = analogRead(pin) * (3.3 / 4095.0);
    voltageAcc += voltage;
  }
  // Cálculo final con sensibilidad del sensor
  return -1 * ((avgVoltage - ZERO_POINT_VOLTAGE) / sensitivity);
}

