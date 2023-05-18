// Подключение библиотек
#include <TFLidar.h>        // Библиотека для работы с лидаром
#include <SoftwareSerial.h> // Библиотека для работы программного Serial

// ------------------------------------------------------------------------------------------------------------------------------------------------------------------------
// Макросы
// Определяем пины
#define PIN_TX_SERIAL 7   // Пин, к которому подключен TX-порт лидара
#define PIN_RX_SERIAL 8   // Пин, к которому подключен RX-порт лидара
#define PIN_BUTTON    2   // Аналоговый пин А2, к котороу подключена кнопка

/* Для связи со второй Arduino используется Serial соединение:
  RX -> TX
  TX -> RX 

  TX - пин 1
  RX - пин 2 */

//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
// Создаем объекты                                                                         
TFLidar Lidar;                                                               // Лидар    
SoftwareSerial Serial_Lidar = SoftwareSerial(PIN_TX_SERIAL, PIN_RX_SERIAL);  // Serial-порт для связи с лидаром

// Объявляем и инициализируем переменные
int dist;

// ------------------------------------------------------------------------------------------------------------------------------------------------------------------------
// Основное тело программы
void setup() 
{
  // Настраиваем передачу между Arduino через Serial со скоростью 115200 бод
  Serial.begin(115200);
  while (!Serial) {}

  // Настраиваем общение с лидаром со скоростью 115200 бод
  Serial_Lidar.begin(115200);
  Lidar.begin(&Serial_Lidar);

  pinMode(LED_BUILTIN, OUTPUT); 
}

void loop() 
{
  
  Lidar.getData(dist);

  // Прерывание при нажатии на кнопку
  if (analogRead(PIN_BUTTON) > 200) 
  {
    digitalWrite(LED_BUILTIN, HIGH);
    Serial.write(dist);
    delay(300);  
  }
  digitalWrite(LED_BUILTIN, LOW);
}

//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
// Функции
// Вычисление расстояния
int get_distance() 
{
  int dist;                     //actual distance measurements of LiDAR
  int strength;                 //signal strength of LiDAR
  int check;                    //save check value
  int i;
  int uart[9];                  //save data measured by LiDAR
  const int HEADER = 0x59;      //frame header of data package

  while (!Serial_Lidar.available()) {}

  // Check if serial port has data input
  if (Serial_Lidar.available())                
  {
    // Assess data package frame header 0x59
    if (Serial_Lidar.read() == HEADER)        
    {
      uart[0] = HEADER;
       // Assess data package frame header 0x59
      if (Serial_Lidar.read() == HEADER)     
      {
        uart[1] = HEADER;
        // Save data in array
        for (i = 2; i < 9; i++)         
        {
          uart[i] = Serial_Lidar.read();
        }
        check = uart[0] + uart[1] + uart[2] + uart[3] + uart[4] + uart[5] + uart[6] + uart[7];
        // Verify the received data as per protocol
        if (1 || uart[8] == (check & 0xff))        
        {
          // Calculate distance value
          dist = uart[2] + uart[3] * 256; 

          // Calculate signal strength value
          strength = uart[4] + uart[5] * 256; 

          while (Serial_Lidar.available()) Serial_Lidar.read();
          return dist;
        }
      }
    }
  }
}