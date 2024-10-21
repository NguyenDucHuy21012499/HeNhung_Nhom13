import time  
import adafruit_dht  
import board  
from RPLCD.i2c import CharLCD  
import RPi.GPIO as GPIO  

# Giải phóng các GPIO nếu chương trình bị gián đoạn trước đó
GPIO.cleanup()

# Khởi tạo cảm biến DHT11  
dht_device = adafruit_dht.DHT11(board.D4)  

# Khởi tạo LCD với địa chỉ I2C  
lcd = CharLCD('PCF8574', 0x27)  # Địa chỉ có thể thay đổi  

# Định nghĩa GPIO chân  
LED_PIN = 18  
BUZZER_PIN = 23  
BUTTON_PIN = 24  
LED_SYSTEM_PIN = 27  # LED hệ thống  
SERVO1_PIN = 15  # Servo1  
SERVO2_PIN = 17  # Servo2  

# Thiết lập GPIO  
GPIO.setmode(GPIO.BCM)  
GPIO.setup(LED_PIN, GPIO.OUT)  
GPIO.setup(BUZZER_PIN, GPIO.OUT)  
GPIO.setup(LED_SYSTEM_PIN, GPIO.OUT)

# Thiết lập PWM cho Servo  
GPIO.setup(SERVO1_PIN, GPIO.OUT)
GPIO.setup(SERVO2_PIN, GPIO.OUT)

servo1 = GPIO.PWM(SERVO1_PIN, 50)  # 50Hz tần số PWM
servo2 = GPIO.PWM(SERVO2_PIN, 50)  # 50Hz tần số PWM

# Thiết lập GPIO cho nút nhấn  
GPIO.setup(BUTTON_PIN, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)

# Biến trạng thái  
system_active = False  
last_button_state = GPIO.LOW  # Trạng thái trước của nút nhấn
debounce_time = 0.2  # Thời gian debounce
last_button_time = 0  # Thời gian nút nhấn trước đó

last_sensor_time = 0  # Thời gian đọc cảm biến gần nhất
sensor_interval = 1  # Đọc cảm biến mỗi giây

# Hàm để hiển thị dữ liệu lên LCD  
def display_data(temperature_c, humidity):  
    lcd.clear()  
    lcd.write_string("Temp: {:.1f} C".format(temperature_c))  # Dòng đầu  
    lcd.cursor_pos = (1, 0)  # Đặt con trỏ ở đầu dòng thứ hai  
    lcd.write_string("Humidity: {}%".format(humidity))  # Dòng thứ hai  

# Hàm để kiểm tra nhiệt độ và độ ẩm  
def check_conditions(temperature_c, humidity):  
    if humidity > 85 or temperature_c > 30:  
        # Nếu độ ẩm > 85% hoặc nhiệt độ > 30°C  
        GPIO.output(LED_PIN, GPIO.HIGH)  
        GPIO.output(BUZZER_PIN, GPIO.HIGH)  

        # Điều chỉnh servo1 nếu nhiệt độ > 30°C
        if temperature_c > 30:
            servo1.ChangeDutyCycle(10)  # Quay nhanh hơn

        # Điều chỉnh servo2 nếu độ ẩm > 85%
        if humidity > 85:
            servo2.ChangeDutyCycle(10)  # Quay nhanh hơn
    else:  
        # Nếu không, tắt LED và buzzer, và cho servo quay chậm lại
        GPIO.output(LED_PIN, GPIO.LOW)  
        GPIO.output(BUZZER_PIN, GPIO.LOW)  
        servo1.ChangeDutyCycle(7.5)  # Quay chậm lại
        servo2.ChangeDutyCycle(7.5)  # Quay chậm lại

# Hàm để tắt hệ thống  
def deactivate_system():  
    global system_active  
    system_active = False  
    GPIO.output(LED_PIN, GPIO.LOW)  
    GPIO.output(BUZZER_PIN, GPIO.LOW)
    GPIO.output(LED_SYSTEM_PIN, GPIO.LOW)  # Tắt LED hệ thống
    
    # Dừng servo khi hệ thống tắt
    servo1.ChangeDutyCycle(0)  # Dừng hẳn servo1
    servo2.ChangeDutyCycle(0)  # Dừng hẳn servo2
    
    lcd.clear()  
    print("System deactivated")  

# Hàm để khởi động lại servo khi hệ thống bật
def activate_system():
    global system_active
    system_active = True
    GPIO.output(LED_SYSTEM_PIN, GPIO.HIGH)  # Bật LED hệ thống
    
    # Khởi động lại servo khi hệ thống bật
    servo1.start(7.5)  # Trung lập ban đầu
    servo2.start(7.5)  # Trung lập ban đầu
    
    print("System activated")

# Vòng lặp chính  
try:  
    while True:  
        # Kiểm tra trạng thái nút nhấn
        button_state = GPIO.input(BUTTON_PIN)
        
        if button_state != last_button_state and time.time() - last_button_time > debounce_time:
            if button_state == GPIO.HIGH:
                # Đổi trạng thái hệ thống khi nhấn nút
                if system_active:
                    deactivate_system()  # Gọi hàm để tắt hệ thống  
                else:
                    activate_system()  # Gọi hàm để bật hệ thống  
            
            # Cập nhật trạng thái nút nhấn trước đó và thời gian
            last_button_state = button_state
            last_button_time = time.time()

        # Kiểm tra cảm biến mỗi giây mà không chặn vòng lặp
        if system_active and time.time() - last_sensor_time > sensor_interval:
            try:  
                # Đọc nhiệt độ và độ ẩm  
                temperature_c = dht_device.temperature  
                humidity = dht_device.humidity  
                
                # Hiển thị dữ liệu lên LCD  
                display_data(temperature_c, humidity)  
                
                # Kiểm tra điều kiện để bật/tắt LED, buzzer và điều chỉnh servo  
                check_conditions(temperature_c, humidity)  
                
                # In ra console (nếu cần)  
                print(f"Temp: {temperature_c:.1f} C, Humidity: {humidity}%")  

            except RuntimeError as error:  
                # In lỗi ra console nhưng không dừng chương trình  
                print(error.args[0])  
            
            # Cập nhật thời gian lần cuối đọc cảm biến
            last_sensor_time = time.time()

        # Đợi một chút để giảm tải CPU  
        time.sleep(0.01)  # 10ms

except KeyboardInterrupt:  
    # Tắt tất cả khi dừng chương trình  
    deactivate_system()  
    GPIO.cleanup()  
    print("Chương trình đã dừng.")
