from micropython import const
import ubluetooth as bluetooth
import time
from machine import Pin, PWM
from ble_advertising import advertising_payload

# BLE constants
_IRQ_CENTRAL_CONNECT = const(1)
_IRQ_CENTRAL_DISCONNECT = const(2)
_IRQ_GATTS_WRITE = const(3)

_UART_UUID = bluetooth.UUID("6E400001-B5A3-F393-E0A9-E50E24DCCA9E")
_UART_TX = bluetooth.UUID("6E400003-B5A3-F393-E0A9-E50E24DCCA9E")
_UART_RX = bluetooth.UUID("6E400002-B5A3-F393-E0A9-E50E24DCCA9E")

_UART_SERVICE = (
    _UART_UUID,
    (
        (_UART_TX, bluetooth.FLAG_NOTIFY,),
        (_UART_RX, bluetooth.FLAG_WRITE,),
    ),
)

# ✅ Servo setup on GPIO4 (A4 pin on ESP32-C3)
servo = PWM(Pin(4), freq=50)

def set_servo_angle(angle):
    min_duty = 26
    max_duty = 128
    duty = int((angle / 180) * (max_duty - min_duty) + min_duty)
    servo.duty(duty)

class BLEUART:
    def __init__(self, ble, name="ESP-D"):
        self._ble = ble
        self._ble.active(True)
        self._ble.irq(self._irq)

        ((self._tx_handle, self._rx_handle),) = self._ble.gatts_register_services((_UART_SERVICE,))
        self._connections = set()

        payload = advertising_payload(name=name, services=[_UART_UUID])
        self._ble.gap_advertise(100_000, adv_data=payload)
        print("Advertising as:", name)

        self.last_detected_time = 0
        self.servo_active = False
        self.servo_timeout = 3  # seconds after last detection

    def _irq(self, event, data):
        if event == _IRQ_CENTRAL_CONNECT:
            conn_handle, _, _ = data
            print("Device connected")
            self._connections.add(conn_handle)

        elif event == _IRQ_CENTRAL_DISCONNECT:
            conn_handle, _, _ = data
            print("Device disconnected")
            self._connections.remove(conn_handle)
            self._ble.gap_advertise(100_000, adv_data=advertising_payload(name="ESP-D", services=[_UART_UUID]))

        elif event == _IRQ_GATTS_WRITE:
            conn_handle, attr_handle = data
            value = self._ble.gatts_read(self._rx_handle)
            message = value.decode().strip().lower()
            print("Received:", message)

            if message in ["leaf blight", "curly leaf blight"]:
                self.last_detected_time = time.time()
                if not self.servo_active:
                    print("Activating servo: 90°")
                    set_servo_angle(90)
                    self.servo_active = True

    def check_servo_timeout(self):
        if self.servo_active and (time.time() - self.last_detected_time) > self.servo_timeout:
            print("No detection, returning to 0°")
            set_servo_angle(0)
            self.servo_active = False

    def send(self, data):
        for conn_handle in self._connections:
            self._ble.gatts_notify(conn_handle, self._tx_handle, data)

# BLE init
ble = bluetooth.BLE()
uart = BLEUART(ble)

# Main loop
while True:
    uart.check_servo_timeout()
    time.sleep(0.5)
