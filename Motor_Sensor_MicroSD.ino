// Подключение библиотек
#include <SD.h>             // Библиотека для работы с картридером
#include <SPI.h>            // Библиотека для передачи данных по SPI
#include <Servo.h>          // Библиотека для работы с моторами
#include <SimpleFOC.h>      // Библиотека для работы с энкодерами
 
// ------------------------------------------------------------------------------------------------------------------------------------------------------------------------
// Макросы
// Определяем пины
#define PIN_MOTOR_LIDAR   5   // Пин, который подает сигнал на мотор, отвечающий за лидар
#define PIN_MOTOR_GEAR    6   // Пин, который подает сигнал на мотор, отвечающий за шестеренку
#define PIN_SENSOR_LIDAR  3   // Пин, который читает ШИМ с энкодера, отвечающего за лидар
#define PIN_SENSOR_GEAR   2   // Пин, который читает ШИМ с энкодера, отвечающего за шестеренку
#define PIN_SD            10  // Пин, который отвечает за  картридер
#define PIN_BUTTON        2   // Аналоговый пин А2, к котороу подключена кнопка

/* Модуль SD карты подключен в SPI по стандартной схеме:
  MOSI - пин 11
  MISO - пин 12
  CLK  - пин 13 */

/* Для связи со второй Arduino используется Serial соединение:
  RX -> TX
  TX -> RX 

  TX - пин 1
  RX - пин 2 */

// Остальные макросы
#define MAX_PWM 1100       // Максимальное значение ШИМ в микросекундах
#define MIN_PWM 800         // Минимальное значени ШИМ в микросекундах
#define MIN_PULSE 4         // Минимальная длина импульса в микросекундах
#define MAX_PULSE 904       // Максимальная длина импульса в микросекундах

//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
// Создаем объекты                                                                         
Servo Motor_Lidar;                                                                            // Мотор, отвечающий за лидар
Servo Motor_Gear;                                                                             // Мотор, отвечающий за шестеренку              
MagneticSensorPWM Sensor_Lidar = MagneticSensorPWM(PIN_SENSOR_LIDAR, MIN_PULSE, MAX_PULSE);   // Энкодер, отвечающий за лидар
MagneticSensorPWM Sensor_Gear = MagneticSensorPWM(PIN_SENSOR_GEAR, MIN_PULSE, MAX_PULSE);     // Энкодер, отвечающий за шестеренку

// Объявляем и инициализируем переменные
float angle_Lidar, angle_Gear;
int dist;
int i = 0, n = 10;
File file;
String result;

// ------------------------------------------------------------------------------------------------------------------------------------------------------------------------
// Основное тело программы
void setup() 
{
  // Настраиваем передачу между Arduino через Serial со скоростью 115200 бод
  Serial.begin(115200);
  while (!Serial) {}

  pinMode(LED_BUILTIN, OUTPUT); 
  
  // Инициализация картридера и открытие файла для записи
  SD.begin(PIN_SD);
  SD.remove("test.txt");
  file = SD.open("test.txt", FILE_WRITE);

  // Инициализация энкодеров
  Sensor_Lidar.init();
  Sensor_Gear.init();

  // Вводим энкодеры в неблокирующий режим
  Sensor_Lidar.enableInterrupt(doPWM_Lidar);
  Sensor_Gear.enableInterrupt(doPWM_Gear);

  // Инициализация моторов
  Motor_Lidar.attach(PIN_MOTOR_LIDAR);
  Motor_Gear.attach(PIN_MOTOR_GEAR);

  // Калибровка моторов
  calibration_motor();

  // Запуск моторов
  start_motor();

  while (Serial.available() > 0) Serial.read();
  
}

void loop() 
{
  delay(2000);
  Serial.write(1);
  
  while (i < n) 
  {
    // Задаем скорость моторов
    Motor_Lidar.writeMicroseconds(1100);
    Motor_Gear.writeMicroseconds(990);

    // Обновляем энкодеры
    Sensor_Lidar.update();
    Sensor_Gear.update();

    // Прерывание при нажатии на кнопку
    if (analogRead(PIN_BUTTON) > 200) 
    {
      // Вычисляем углы
      angle_Lidar = Sensor_Lidar.getAngle();
      angle_Gear = Sensor_Gear.getAngle();

      // Считываем расстояние с другой Arduino
      while(!Serial.available()) {}
      dist = Serial.read();
      if(dist > 0)
      {
        digitalWrite(LED_BUILTIN, HIGH);
      
        result = String(dist) + " " + String(angle_Lidar * (180.0 / 3.1414)) + " " + String(angle_Gear * (15.0 / 35.0) * (180.0 / 3.1414)) + "\n";
    
        file.write(result.c_str());
        i++;
      }
      delay(200);     
    }  

    digitalWrite(LED_BUILTIN, LOW);
  } 

  // Закрываем файл для записи
  file.close();

  // Останавливаем моторы
  stop_motor();
  while(1) {}
}

//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
// Функции
// Калибрровки мотора
void calibration_motor()
{
  Motor_Lidar.writeMicroseconds(MAX_PWM);
  delay(2000);
  Motor_Lidar.writeMicroseconds(MIN_PWM);
  delay(2000);
  
  Motor_Gear.writeMicroseconds(MAX_PWM);
  delay(2000);
  Motor_Gear.writeMicroseconds(MIN_PWM);
  delay(2000);
}

// Перевод энкодера, отвечающего за лидар в неблокирующий режим
void doPWM_Lidar()
{
  Sensor_Lidar.handlePWM();
}

// Перевод энкодера, отвечающего за шестеренку в неблокирующий режим
void doPWM_Gear()
{
  Sensor_Gear.handlePWM();
}

// Старт моторов (чтобы преодалеть трение)
void start_motor()
{
  Motor_Lidar.writeMicroseconds(1050);
  Motor_Gear.writeMicroseconds(1050);
  delay(1000);
}

// Остановка моторов
void stop_motor()
{
  Motor_Lidar.writeMicroseconds(0);
  Motor_Gear.writeMicroseconds(0);
  delay(1000);
}
