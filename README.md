# 🔥 Sistema de Control de Temperatura con PID

Proyecto de control automático de temperatura usando un microcontrolador ESP32 con **algoritmo PID**, interfaz LCD y comunicación SPI para termopar. Ideal para aplicaciones de hornos, soldadoras o estaciones de reflow.

[![Licencia MIT](https://img.shields.io/badge/Licencia-MIT-green.svg)](https://opensource.org/licenses/MIT)

---

## 🌟 Características Principales
- **Control PID ajustable** (`Kp=350, Ki=5, Kd=1.8`)  
- **Interfaz de usuario**:
  - LCD 16x2 via I2C (dirección `0x27`)  
  - Botones para ajustar setpoint (±5°C)  
- **Hardware crítico**:
  - Sensor de temperatura MAX6675 (termopar tipo K)  
  - Detección de **cruce por cero** para control preciso de triac  
  - Salida PWM para elemento calefactor (SSR o triac)  
- **Multitarea real** usando `uasyncio`  

---

## 🛠️ Tecnologías Utilizadas
| Componente               | Especificaciones                         |
|--------------------------|------------------------------------------|
| **Microcontrolador**     | Raspberry Pi Pico/ESP32                  |
| **Comunicación**         | SPI (MAX6675), I2C (LCD)                 |
| **Sensor de Temperatura**| MAX6675 (Rango: 0-1024°C, Res: 0.25°C)   |
| **Control de Potencia**  | Optoacoplador MOC3041 + Triac BTA16      |
| **Librerías**            | `uasyncio`, `machine`                    |

---

## 📐 Diagrama de Conexiones
```plaintext
Pines Microcontrolador:
- MAX6675: SCK(4), MOSI(2), MISO(18), CS(5)
- LCD I2C: SDA(17), SCL(16)
- Botones: Incremento(22), Decremento(21)
- Zero-Cross: Pin(19)
- Salida Calefactor: Pin(23)
