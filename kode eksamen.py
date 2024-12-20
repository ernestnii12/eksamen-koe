from machine import Pin, I2C, UART, ADC
from time import sleep, ticks_ms
from bme680_i2c import BME680_I2C
from gps_simple import GPS_SIMPLE
from gpio_lcd import GpioLcd
from mpu6050 import MPU6050
from neopixel import NeoPixel
from uthingsboard.client import TBDeviceMqttClient
import secrets
import gc

# CONFIGURATION
# I2C
i2c_port = 0
i2c = I2C(i2c_port)

# Initialize Sensors
bme = BME680_I2C(i2c=i2c)
imu = MPU6050(i2c)

# GPS
gps_port = 2
gps_speed = 9600
uart = UART(gps_port, gps_speed)
gps = GPS_SIMPLE(uart)

# LCD
lcd = GpioLcd(rs_pin=Pin(27), enable_pin=Pin(25),
              d4_pin=Pin(33), d5_pin=Pin(32), d6_pin=Pin(21), d7_pin=Pin(22),
              num_lines=4, num_columns=20)

# NeoPixel
n = 16
p = 12
np = NeoPixel(Pin(p, Pin.OUT), n)

# ADC for Battery
potmeter_adc = ADC(Pin(36))
potmeter_adc.atten(ADC.ATTN_11DB)  # Full range 3.3V

# Battery Settings
adc1 = 2479  # ADC for 6.0V
U1 = 6.0
adc2 = 3470  # ADC for 8.4V
U2 = 8.4
battery_capacity = 1800

# Slope and intercept
a = (U1 - U2) / (adc1 - adc2)
b = U2 - a * adc2

# ThingsBoard Client Setup
client = TBDeviceMqttClient(secrets.SERVER_IP_ADDRESS, access_token=secrets.ACCESS_TOKEN)
client.connect()
print("Connected to ThingsBoard.")

# Helper Functions
def batt_voltage(adc_v):
    return a * adc_v + b

def batt_percentage_direct(u_batt):
    return (u_batt / 8.4) * 100

def predict_weather(temperature, humidity, pressure):
    if temperature <= 0 and humidity > 80:
        return "Forvent sne!"
    elif temperature > 0 and humidity > 70:
        return "Forvent regn!"
    return "Stabilt vejr"

def check_pressure_drop(current_pressure, previous_pressure):
    if previous_pressure is None:
        return None
    return "Mulig storm!" if (previous_pressure - current_pressure) > 5 else None

def set_color(r, g, b):
    for i in range(n):
        np[i] = (r, g, b)
    np.write()

def send_to_thingsboard(lat, lon, temperature, humidity, pressure, battery_percent, remaining_time, weather_prediction, speed):
    telemetry = {
        'latitude': lat,
        'longitude': lon,
        'temperature': temperature,
        'humidity': humidity,
        'pressure': pressure,
        'battery': battery_percent,
        'remaining_time': remaining_time,
        'weather': weather_prediction,
        'speed': speed
    }
    client.send_telemetry(telemetry)

# MAIN LOOP VARIABLES
previous_pressure = None
start_time = ticks_ms()
threshold_milliseconds = 3000
display_state = 0
last_sent_time = ticks_ms()  # For tracking when to send data to ThingsBoard

# MAIN LOOP
while True:
    try:
        # Read battery voltage
        adc_value = potmeter_adc.read()
        voltage = batt_voltage(adc_value)
        battery_percent = batt_percentage_direct(voltage)
        remaining_capacity = (battery_percent / 100) * battery_capacity
        remaining_time = remaining_capacity / 200

        # BME680 data
        temperature = round(bme.temperature)
        humidity = round(bme.humidity)
        pressure = round(bme.pressure)

        # GPS data
        speed = 0  # Default speed if GPS data not available
        if gps.receive_nmea_data():
            lat = gps.get_latitude()
            lon = gps.get_longitude()
            speed = gps.get_speed()

        # MPU6050 movement
        values = imu.get_values()
        if values["acceleration z"] < -10000:
            set_color(100, 0, 0)  # Red color
        else:
            set_color(0, 0, 0)

        # Weather and storm detection
        storm_warning = check_pressure_drop(pressure, previous_pressure)
        weather_prediction = predict_weather(temperature, humidity, pressure)
        previous_pressure = pressure

        # LCD Display Logic
        if ticks_ms() - start_time > threshold_milliseconds:
            start_time = ticks_ms()
            display_state = (display_state + 1) % 7
            lcd.clear()

            if display_state == 0:
                lcd.putstr(f"Battery: {int(battery_percent)}%")
            elif display_state == 1:
                lcd.putstr(f"Time left: {remaining_time:.1f}h")
            elif display_state == 2:
                lcd.putstr(f"Breddegrad: {lat:.5f}")
            elif display_state == 3:
                lcd.putstr(f"Laengdegrad : {lon:.5f}")
            elif display_state == 4:
                lcd.putstr(f"Temp: {temperature}C")
            elif display_state == 5:
                lcd.putstr(f"Vejr: {weather_prediction}")
            elif display_state == 6:
                lcd.putstr(f"Fart: {speed:.1f} km/t")

        # Only send data to ThingsBoard every 10 seconds
        if ticks_ms() - last_sent_time >= 100000:
            send_to_thingsboard(lat, lon, temperature, humidity, pressure, battery_percent, remaining_time, weather_prediction, speed)
            last_sent_time = ticks_ms()  # Update the last sent time

        # Debug Output
        print(f"Voltage: {voltage:.2f}V | Battery: {battery_percent:.1f}%")
        print(f"Temp: {temperature}C | Humidity: {humidity}% | Pressure: {pressure}hPa")
        print(f"Speed: {speed:.1f} km/t")
        if storm_warning:
            print(storm_warning)
        print("Weather: " + weather_prediction)
        print("------------------")

    except OSError as e:
        print("Sensor read error.")

    sleep(1)  # Continue checking sensors every second