//подключение используемых файлов и библиотек
#include <OneWire.h>
#include <DallasTemperature.h>
#include <LiquidCrystal.h>
#include <PID_v1.h>
// инициализирование пинов 
#define ONE_WIRE_BUS A1  //инициализируем пин для приема данных с датчика температуры
const int LEDR1 = 3;  //пин для красного цвета (датчик температуры)
const int LEDG1 = 4;  //пин для зеленого цвета (датчик температуры)
const int LEDB1 = 5;  //пин для синего цвета   (датчик температуры) 

const int PhotoR = A0;  //инициализируем пин для приема данных с Фоторезистора 
const int LedG = 8;    //Пин для зеленого цвета LED  (Фоторезистор) 
const int LedB = 7;    //Пин для синего цвета   LED  (Фоторезистор) 
const int LedR = 6;    //Пин для красного цвета LED  (Фоторезистор) 

//Значения, полученные с фоторезистора
float LightLevelCur = -1;   //переменная для хранения текущего (t) значения
//изначально она инициализированна как -1 чтобsы в случае не работы фоторезистора можно было сразу понять в чем проблема
float LightLevelLast = -1;  //переменная для хранения предыдущего (t-1) значения с фоторезистора 

//Пороговые значения
const int THR_Blue = 300;    //Максимальное значение сопротивления фоторезистора до которого будет гореть голубой LED
const int THR_Green = 700;   //Максимальное значение сопротивления фоторезистора до которого будет гореть зеленый LED
                             //если значение сопротивления фоторезистора больше 700 - будет гореть красный LED
const float colorRed = 25;   //Минимальное значение температуры(по цельсию) при котором будет гореть красный LED.
const float colorGreen = 23; //Минимальное значение температуры(по цельсию) при котором будет гореть зеленый LED.
                             //Если значение температуры меньше минимального значения для того чтобы горел зеленый LED -будет гореть голубой LED.

//Интервал значений, полученных с фоторезистора
const int INTERVAL = 20;// если LightLevelCur больше или меньше чем LightLevelLast на INTERVAL - то что-то делаем, иначе бездействуем.


// Устанавливаем пин One Wire для взаимодействия с датчиком температуры (или устройствами с данной библиотеки)
OneWire oneWire(ONE_WIRE_BUS);

// Передаем элемент One Wire 
DallasTemperature sensors(&oneWire);

// Массив для передачи адресса устройства(в данном случае датчика температуры)
DeviceAddress insideThermometer;

//Инициализируем пины для работы с устройством
const int LCD_RS = 2; 
const int LCD_E  = 9;
const int LCD_D4 = 10;
const int LCD_D5 = 11;
const int LCD_D6 = 12;
const int LCD_D7 = 13;
LiquidCrystal lcd(LCD_RS, LCD_E, LCD_D4, LCD_D5, LCD_D6, LCD_D7);
double NeedTempPoint;
double OutputValue;
double CurrentTemp;
double kp = 10, ki = 0.5, kd = 0;
PID myPID(&CurrentTemp, &OutputValue, &NeedTempPoint, kp, ki , kd,  DIRECT);

void PIDRegFunc(){//функция управления охлаждением/нагреванием
  CurrentTemp = sensors.getTempC(insideThermometer);//считываем температуру с датчика
  myPID.Compute();//получаем значение выхода
  if (NeedTempPoint < CurrentTemp){//если нужный уровень температуры меньше чем текущий уровень температуры 
    myPID.SetControllerDirection(REVERSE);//изменяем направление вычислений
        Serial.print("Turn on the air conditioner. ");//включаем кондиционер(вывод в консоль)
        Serial.print("Current power: ");//выводим строку
        Serial.println(OutputValue);//выводим мощность с которой будет работать кондиционер
        Serial.print("\n");
  }else if(NeedTempPoint > CurrentTemp){//если нужная нам температура больше текущая
    myPID.SetControllerDirection(DIRECT);//меняем направление расчетов
        Serial.print("Turn on the heater."); //включаем обогреватель(вывод в консоль)
        Serial.print("Current power: ");//вывод в консоль строки
        Serial.println(OutputValue);//вывод значени мощности
                Serial.print("\n");
  }else {//иначе
    Serial.print("The temperature in the order.");//температура в порядке
            Serial.print("\n");
  }
}


float GetVatueResit(){
  return (((1023-analogRead(PhotoR))*5)/(1023*0.1))/10;
}

void light_Level() {
  LightLevelCur = analogRead(PhotoR);                     //Считывание значения с фоторезистора
  if (abs(LightLevelCur - LightLevelLast) > INTERVAL) {   //Проверка изменения полученного значения на указанный интервал
    if (LightLevelCur < THR_Blue) {                       //Сравнение значения с порогом для синего цвета
      digitalWrite(LedR, LOW);                            //если заходим внутрь этого if, то светится только голубой LED
      digitalWrite(LedG, LOW);
      digitalWrite(LedB, HIGH);
    } else if (LightLevelCur < THR_Green) {               //Сравнение значения с порогом для зеленого цвета
      digitalWrite(LedR, LOW);                            //если заходим внутрь этого if, то светится только Зеленый LED
      digitalWrite(LedG, HIGH); 
      digitalWrite(LedB, LOW);
    } else {                                              //Иначе зажикаем красный
      digitalWrite(LedR, HIGH);                           //если заходим внутрь этого if, то светится только Красный LED
      digitalWrite(LedG, LOW);
      digitalWrite(LedB, LOW);
    }
    lcd.setCursor(7,1);                                   //Устанавливаем курсор на дисплее. Вторая строка, 8 позиция в этой строке
    lcd.print(GetVatueResit());                             //Выводим текущий уровень освещенности на дисплей
    lcd.setCursor(11,1);                                  //Устанавливаем курсор на дисплее. Вторая строка, 12 позиция в этой строке
    lcd.print("(KOm)");                                    //Выводим единицы измерения фоторезистора (Ом)
  }
  LightLevelLast = LightLevelCur;                         //Переопределение предыдущего значения
    Serial.print("Current Resist value: ");                 //Выводим надпись в консоль, а затем строкой ниже выводим значение сопротивления фототранзистора
    Serial.print(GetVatueResit());                        //Вывод полученного значения в консоль
    Serial.println("(KOm)");
    Serial.print("Current Light Level(ADC): ");
    Serial.println(LightLevelCur);
}


void setup(void)//стандартная функция для установки портов на запись/чтение
{
  // start serial port
  Serial.begin(9600);
  pinMode(LedR, OUTPUT);  //Настройка пина на вывод для LED(Фоторезистор)
  pinMode(LedG, OUTPUT);  //Настройка пина на вывод для LED(Фоторезистор)
  pinMode(LedB, OUTPUT);  //Настройка пина на вывод для LED(Фоторезистор)
  pinMode(LEDR1, OUTPUT); //Настройка пина на вывод для LED(температура)
  pinMode(LEDG1, OUTPUT); //Настройка пина на вывод для LED(температура)
  pinMode(LEDB1, OUTPUT); //Настройка пина на вывод для LED(температура)

  lcd.begin(16,2);        //Программируем дисплей, 16 - количество элементво в одной строке, 2 - количество строк
  lcd.print("  Temp:");   //Печатаем на первой строке статическое "Temp : "
  lcd.setCursor(0,1);     //устанавливаем курсор на ту позицию которая нам дальше нужна, а именно 0 элемент в 2й строке.
  lcd.print("PhotoR:");  //Печатаем в этой позиции "Light : "

  // Запуск библиотеки
  sensors.begin();
  // Если не найден адресс нашего устройства - выводим на ошибку
  if (!sensors.getAddress(insideThermometer, 0))
    Serial.println("Unable to find address for Device 0");
 myPID.SetOutputLimits(0, 100);//устанавливаем ограничение мощности(минимум - 0, максимум - 100)
 CurrentTemp = sensors.getTempC(insideThermometer);//первоначальное считывание температуры для пид регулятора
 NeedTempPoint = 24;//уровень температуры нужный нам
 myPID.SetMode(AUTOMATIC);//запустить пид регулятор
}



//Функция для вывода температуры на концоль и дисплей
void printTemperature(DeviceAddress deviceAddress)
{
  float tempC = sensors.getTempC(deviceAddress); // получаем текущую температуру
  Serial.print(tempC);//Печатаем текущую температуру на консоль
  Serial.print("(C)");//печатем знак "(С)" на консоль
  
  lcd.setCursor(7,0); //Устанавливаем курсор на 7й элемент 1й строки на дисплее
  lcd.print(tempC);   //выводим температуру на дисплей
  lcd.setCursor(13,0 );//устанавливаем курсор на 13й элемент 1й строки на дисплее
  lcd.print("(C)"); //печатаем знак "(С)"
}

// Главная функция для вывода информации о устройстве(датчике температуры)
void printData(DeviceAddress deviceAddress)
{
  Serial.print("Current temperature: ");
  printTemperature(deviceAddress);
  Serial.println();
}
//Функция для контроля LED принадлежащих датчику температуры
void LEDControl(DeviceAddress deviceAddress){
  float tempC = sensors.getTempC(deviceAddress);//получаем текущее значение температуры
  if(tempC < colorGreen){         //если текущая температура меньше чем температура для того чтобы зажечь зеленый LED
    digitalWrite(LEDB1, HIGH);    //то зажигаем голубой LED
    digitalWrite(LEDG1, LOW);
    digitalWrite(LEDR1, LOW);
  }else if(tempC < colorRed){     //если текущая температура меньше чем температура для того чтобы зажечь красный LED
      digitalWrite(LEDB1, LOW);
      digitalWrite(LEDG1, HIGH);  //то зажигаем зеленый LED
      digitalWrite(LEDR1, LOW);
    }else {                       //если не проходят предыдущие проверки
      digitalWrite(LEDB1, LOW);
      digitalWrite(LEDG1, LOW);
      digitalWrite(LEDR1, HIGH); //зажигаем красный LED
    }
  }

//главный цикл, без него прошивка не будет ничего делать
void loop(void)
{ 
  LEDControl(insideThermometer);
  sensors.requestTemperatures();
  printData(insideThermometer);
  PIDRegFunc();
  light_Level();
  delay(300);
 
}

