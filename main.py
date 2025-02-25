from machine import Pin, I2C, Timer, SPI
from time import sleep_us, ticks_us, ticks_ms
import uasyncio as asyncio

# --- Clase I2cLcd ---
class I2cLcd:
    def __init__(self, i2c, i2c_addr, num_lines, num_columns):
        self.i2c = i2c
        self.i2c_addr = i2c_addr
        self.num_lines = num_lines
        self.num_columns = num_columns
        self._backlight = 0x08  # Luz de fondo encendida por defecto
        sleep_us(20000)
        self._write_init_nibble(0x30)
        sleep_us(5000)
        self._write_init_nibble(0x30)
        sleep_us(1000)
        self._write_init_nibble(0x30)
        sleep_us(1000)
        self._write_init_nibble(0x20)  # Modo de 4 bits
        self._write_command(0x28)  # 2 líneas, matriz de 5x8 puntos
        self._write_command(0x0C)  # Display ON, cursor OFF
        self._write_command(0x06)  # Entrada automática
        self.clear()

    def _write_init_nibble(self, nibble):
        self.i2c.writeto(self.i2c_addr, bytes([nibble | self._backlight | 0x04]))
        self.i2c.writeto(self.i2c_addr, bytes([nibble | self._backlight]))

    def _write_command(self, cmd):
        self._write_byte(cmd, 0)

    def _write_data(self, data):
        self._write_byte(data, 1)

    def _write_byte(self, byte, mode):
        high = (byte & 0xF0) | self._backlight | (0x01 if mode else 0)
        low = ((byte << 4) & 0xF0) | self._backlight | (0x01 if mode else 0)
        self.i2c.writeto(self.i2c_addr, bytes([high | 0x04]))
        self.i2c.writeto(self.i2c_addr, bytes([high]))
        self.i2c.writeto(self.i2c_addr, bytes([low | 0x04]))
        self.i2c.writeto(self.i2c_addr, bytes([low]))

    def clear(self):
        self._write_command(0x01)
        sleep_us(2000)

    def move_to(self, col, row):
        addr = col + 0x40 * row if row > 0 else col
        self._write_command(0x80 | addr)

    def putstr(self, string):
        for char in string:
            self._write_data(ord(char))


# --- Configuración de Pines ---
firing_pin = Pin(23, Pin.OUT)
increase_pin = Pin(22, Pin.IN, Pin.PULL_UP)
decrease_pin = Pin(21, Pin.IN, Pin.PULL_UP)
zero_cross = Pin(19, Pin.IN, Pin.PULL_UP)

# Configuración del termopar MAX6675
spi = SPI(1, baudrate=500000, polarity=0, phase=0, sck=Pin(4), mosi=Pin(2), miso=Pin(18))
thermoCS = Pin(5, Pin.OUT)
thermoCS.on()

# Configuración I2C para LCD
i2c = I2C(1, scl=Pin(16), sda=Pin(17), freq=100000)
lcd = I2cLcd(i2c, 0x27, 2, 16)

# --- Variables Globales ---
setpoint = 100
real_temperature = 25  # Valor inicial
kp, ki, kd = 350, 5, 1.8  # Constantes PID
PID_p, PID_i, PID_d = 0, 0, 0
PID_value = 0
maximum_firing_delay = 7400  # En microsegundos
minimum_firing_delay = 2000  # En microsegundos
last_temp_read_time = ticks_ms()
debounce_delay = 200  # milisegundos
last_increase_time, last_decrease_time = 0, 0
previous_error = 0
previous_time = ticks_us()


# --- Funciones para Sensor y Control ---
def read_temperature():
    """Lee la temperatura del termopar MAX6675."""
    global real_temperature
    thermoCS.off()
    sleep_us(10)
    raw = spi.read(2)
    thermoCS.on()
    value = (raw[0] << 8) | raw[1]
    if value & 0x4:
        real_temperature = None
    else:
        value >>= 3
        real_temperature = value * 0.25


def control_temperature():
    """Calcula el valor del PID para regular la temperatura."""
    global PID_value, real_temperature, PID_p, PID_i, PID_d, previous_error, previous_time
    if real_temperature is None:
        return

    current_time = ticks_us()
    elapsed_time = (current_time - previous_time) / 1_000_000
    previous_time = current_time

    error = setpoint - real_temperature

    # Cálculo de términos PID
    PID_p = kp * error
    PID_i += ki * error * elapsed_time
    PID_d = kd * (error - previous_error) / elapsed_time if elapsed_time > 0 else 0

    # Limitar PID_i para evitar acumulación excesiva
    PID_i = min(max(PID_i, -500), 500)

    # Calcular salida PID y limitarla
    PID_value = int(max(0, min(PID_p + PID_i + PID_d, maximum_firing_delay)))
    previous_error = error


def zero_cross_handler(pin):
    """Manejador de interrupción para el cruce por cero."""
    delay = maximum_firing_delay - PID_value
    delay = max(minimum_firing_delay, delay)
    Timer(1).init(mode=Timer.ONE_SHOT, period=delay // 1000, callback=lambda t: firing())


def firing():
    """Activa el disparo del elemento calefactor."""
    firing_pin.on()
    sleep_us(100)
    firing_pin.off()


# --- Funciones para Interfaz y Botones ---
async def update_lcd():
    """Actualiza los valores mostrados en el LCD."""
    while True:
        lcd.clear()
        lcd.move_to(0, 0)
        lcd.putstr(f"Set: {setpoint}C")
        lcd.move_to(0, 1)
        if real_temperature is not None:
            lcd.putstr(f"Real: {real_temperature:.1f}C")
        else:
            lcd.putstr("Sensor Error")
        await asyncio.sleep(1)


def handle_buttons():
    """Maneja el incremento y decremento del setpoint con debounce."""
    global setpoint, last_increase_time, last_decrease_time
    current_time = ticks_ms()
    if increase_pin.value() == 0 and current_time - last_increase_time > debounce_delay:
        setpoint += 5
        last_increase_time = current_time
    if decrease_pin.value() == 0 and current_time - last_decrease_time > debounce_delay:
        setpoint = max(25, setpoint - 5)
        last_decrease_time = current_time


# --- Bucle Principal ---
async def control_loop():
    """Bucle principal que actualiza el control PID y lee la temperatura."""
    global last_temp_read_time
    while True:
        current_time = ticks_ms()
        handle_buttons()
        if current_time - last_temp_read_time >= 250:
            last_temp_read_time = current_time
            read_temperature()
            control_temperature()
        await asyncio.sleep(0.05)


async def main():
    """Ejecuta el sistema."""
    zero_cross.irq(trigger=Pin.IRQ_RISING, handler=zero_cross_handler)
    asyncio.create_task(update_lcd())
    await control_loop()


# --- Ejecución ---
try:
    asyncio.run(main())
except KeyboardInterrupt:
    print("Programa interrumpido")
