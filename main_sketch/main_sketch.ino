#include <Adafruit_PWMServoDriver.h>    // Подключаем библиотеку Adafruit_PWMServoDriver
#include <GyverHub.h>
#include <EEPROM.h>

GyverHub hub("MyDevices", "ESP8266", "");  // имя сети, имя устройства, иконка

Adafruit_PWMServoDriver right = Adafruit_PWMServoDriver(0x42);    //Create an object of board 1
Adafruit_PWMServoDriver left = Adafruit_PWMServoDriver(0x41);    //Create an object of board 2 (A0 Address Jumper)
#define SERVOMIN  90                // Минимальная длительность импульса для сервопривода
#define SERVOMAX  500                  // Максимальная длина импульса для сервопривода

#define a_const 36   // константа:  плеча А
#define b_const 46   // константа: l плеча В
#define c_const 85   // константа: l плеча C
#define const_angle 17.5 // угол между осью плеча B перпендикуляром к A

int corrections[] = {-45, 0, 45, 45, 0, -45};
//номера ног 0 .... 5

// Это все константы механики. Они не меняются, обусловлены конструкцией.
// ИЗ КАДА НЕ ТРОГАЙ 

gh::Flag right_h, front, left_h, stop, read_e, write_e, hex, quad;
uint16_t up_dist, down_dist, distation, step_distation, pos;
String resim = "";
String moving_res = "";
uint16_t timing = 1000;

int counter;
int counter1 = 0;
uint32_t tmr1 = millis();

float positions[6][4]{
  {0, 0, 0, 0}, // нога 0 (х, у) нач (х, у) кон
  {0, 0, 0, 0},
  {0, 0, 0, 0},
  {0, 0, 0, 0},
  {0, 0, 0, 0},
  {0, 0, 0, 0}, // нога 5 (х, у) нач (х, у) кон
};

void build(gh::Builder& b) {
  b.Text("режим хотьбы").rows(1);
  b.beginRow();
  b.Button().label("квадропод").attach(&quad);
  b.Button().label("гексапод").attach(&hex);
  b.endRow(); 
  b.Text("направление").rows(1);
  b.beginRow();
  b.Button().label("право").attach(&right_h);
  b.Button().label("вперед").attach(&front);
  b.Button().label("лево").attach(&left_h);
  b.endRow(); 
  b.Button().label("стоп").attach(&stop).size(450);  
  b.Slider(&pos).label("поворот").range(0, 360, 10);
  b.Text("Параметры шага").rows(1);
  b.beginRow();
  b.Slider_("up_dist", &up_dist).label("верхняя точка").range(0, 100, 5);
  b.Slider_("down_dist", &down_dist).label("нижняя точка").range(0, 100, 5);
  b.endRow();
  b.beginRow();
  b.Slider_("distation", &distation).label("отводит на").range(20, 120, 10);
  b.Slider_("step_distation", &step_distation).label("шагает на").range(0, 50, 5);
  b.endRow();
  b.Slider_("timing", &timing).label("задержка").range(100, 1000, 50);
  b.Text("eeprom").rows(1);
  b.beginRow();
  b.Button().label("записать").attach(&write_e);  
  b.Button().label("считать").attach(&read_e);  
  b.endRow();
}

void setup() {
  Serial.begin(9600);
  //-----------------------------------------
  right.begin();             //Start each board
  left.begin();
  right.setOscillatorFrequency(27000000);    
  left.setOscillatorFrequency(27000000);
  right.setPWMFreq(50);         
  left.setPWMFreq(50);
  //-----------------------------------------
  WiFi.mode(WIFI_STA);
  WiFi.begin("Penibord_2G", "StrA97!B16");
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  hub.onBuild(build); // подключаем билдер
  hub.begin();        // запускаем систему
  //-----------------------------------------
  EEPROM.begin(1000);   // для esp8266/esp32

  up_dist = EEPROM.read(10);
  down_dist = EEPROM.read(12);
  distation = EEPROM.read(14);
  step_distation = EEPROM.read(16);
}

void loop() {  
  ep_tick();
  hub.tick();

  if(right_h) resim = "r"; 
  if(left_h) resim = "l"; 
  if(front) resim = "f"; 
  if(stop) resim = "s"; moving_res = "s";

  if(resim == "r"){
    rotation(distation, step_distation, 0);

  }else if(resim == "l"){
    rotation(distation, step_distation, 1);
  }else if(resim == "f"){
    angle_moving(pos, distation, step_distation);
  }else if(resim == "s"){
    for(int i = 0; i < 6; i++){
      positions[i][0] = 100;
      positions[i][1] = 0;
      positions[i][2] = 100;
      positions[i][3] = 0;
    }
  }

  if(hex) moving_res = "h";
  if(quad) moving_res = "q";

  if(moving_res == "h"){
    Serial.println("H");
    hexapod(timing, -up_dist, -down_dist);
  }else if(moving_res == "q"){
    Serial.println("Q");
    quadropod(timing, -up_dist, -down_dist);
  }
}

void rotation(int angle_dist, int l_step, int direction){ // 0 - right 1 -left
  int angles[] = {45, 90, 135, 135, 90, 45};
  for(int i = 0; i < 6; i++){
    int angle = angles[i];
    float l_diag = sqrt(sq(angle_dist) + sq(l_step));  //ищем длинну диагонали
    float omicron = degrees(acos((sq(angle_dist) + sq(l_diag) - sq(l_step)) / (2 * l_diag * angle_dist))); // ищем угол между angle_dist и l_diag
    float epsilon1 = angle - omicron;
    float kappa1 = 90 - epsilon1;               //ищем углы для треугольника обр Х1 и У1
    float Y1 = sin(radians(kappa1)) * l_diag;
    float X1 = sin(radians(epsilon1)) * l_diag; //ищем координаты

    float epsilon2 = angle + omicron;
    float kappa2 = 90 - epsilon2;               //ищем углы для треугольника обр Х2 и У2
    float Y2 = sin(radians(kappa2)) * l_diag;
    float X2 = sin(radians(epsilon2)) * l_diag; //ищем координаты

    if(direction == 0){
      positions[i][0] = X1;
      positions[i][1] = Y1;
      positions[i][2] = X2;
      positions[i][3] = Y2;
    }else{
      positions[i][0] = X2;
      positions[i][1] = Y2;
      positions[i][2] = X1;
      positions[i][3] = Y1;
    }
  }
}

void angle_moving(float move_angle, int l_dist, int l_step){
  //нужно посчитать координаты х у для этого рассмотрим треугольник с углом move_angle 
  //и сторонами l_step, l_perp и l_diff. Ищем стороны l_perp и l_diff
  float l_perp = sin(radians(move_angle)) * l_step;
  float l_diff = sin(radians(90 - move_angle)) * l_step;

  for(int i = 0; i < 6; i++){             // X Y здесь записаны позиции для движения ног
    positions[i][0] = l_dist - l_diff;
    positions[i][1] = l_perp;
    positions[i][2] = l_dist + l_diff;
    positions[i][3] = -l_perp;
  }
}

void hexapod(int period, int l_up, int l_down){
  if(millis() - tmr1 >= period){
    tmr1 = millis();
    counter += 1;
  }
  switch(counter){
    case 1:
      for(int i = 0; i < 5; i = i + 2) move_to(positions[i][0], positions[i][1], l_down, i); 
      for(int i = 1; i < 6; i = i + 2) move_to(positions[i][2], positions[i][3], l_up, i);
      break;
    case 2:
      for(int i = 0; i < 5; i = i + 2) move_to(positions[i][2], positions[i][3], l_down, i);
      for(int i = 1; i < 6; i = i + 2) move_to(positions[i][0], positions[i][1], l_up, i);
      break;
    case 3:
      for(int i = 0; i < 5; i = i + 2) move_to(positions[i][2], positions[i][3], l_up, i);
      for(int i = 1; i < 6; i = i + 2) move_to(positions[i][0], positions[i][1], l_down, i);
      break;
    case 4:
      for(int i = 0; i < 5; i = i + 2) move_to(positions[i][0], positions[i][1], l_up, i);
      for(int i = 1; i < 6; i = i + 2) move_to(positions[i][2], positions[i][3], l_down, i);
      counter = 0;
      break;
  }
}

void quadropod(int period, int up, int down){
  int up_down[6][6]{ // down - 0; up -1
    {down, down, down, down, up, up}, //1
    {up, up, down, down, down, down}, //2
    {down, down, up, up, down, down}, //3    
    {down, down, down, down, up, up}, //1
    {down, down, up, up, down, down}, //3
    {up, up, down, down, down, down}, //2
  };

  int pos_num[6][6]{ // st - 0, mid - 1, fin - 2
    {0, 1, 1, 2, 2, 0}, //1
    {2, 0, 0, 1, 1, 2}, //2
    {1, 2, 2, 0, 0, 1}, //3    
    {0, 1, 1, 2, 2, 0}, //1
    {1, 2, 2, 0, 0, 1}, //3
    {2, 0, 0, 1, 1, 2}, //2

  };

  if(millis() - tmr1 >= period){
    tmr1 = millis();
    counter += 1;
    if(counter == 6) counter = 0;
    Serial.println("tick");
  }
  int the_pos[2][6];

  for(int i = 0; i < 6; i++){
    if(pos_num[i][counter] == 0){
      the_pos[0][i] = positions[i][0];
      the_pos[1][i] = positions[i][1];
    }else if(pos_num[i][counter] == 1){
      the_pos[0][i] = middle(positions[i][0], positions[i][2]);
      the_pos[1][i] = middle(positions[i][1], positions[i][3]);
    }else if(pos_num[i][counter] == 2){
      the_pos[0][i] = positions[i][2];
      the_pos[1][i] = positions[i][3];
    }
  }

  for(int i = 0; i < 6; i++){
    move_to(the_pos[0][i], the_pos[1][i], up_down[i][counter], i);
  }
}

int middle(int start, int stop){
  int result = (start + stop) / 2;
  return result;
}

void move_to(int x, int y, int z, int leg_num){ 
  //сейчас будем считать углы серво ног при координатах x y z и гомере ноги
  // -----------1-----------
  // смотри тетрадь стр. 17
  float q = sqrt(sq(x) + sq(y));
  float l = sqrt(sq(x) + sq(15 - y));
  float gamma = degrees(acos((sq(15) + sq(q) - sq(l)) / (30 * q)));
  if(x < 0) gamma = 360 - gamma;
  gamma = 180 - gamma;
  int S1 = gamma + corrections[leg_num];
  // -----------2-----------
  // смотри тетрадь стр. 17 - 18
  float d = sqrt(sq(q - a_const) + sq(z));
  int h = 15;
  float j = sqrt(sq(q - a_const) + sq(15 - z));
  float beta = degrees(acos((sq(h) + sq(d) - sq(j)) / (2 * h * d)));
  if(q < a_const) beta = 360 - beta;
  float omega = degrees(acos((sq(b_const) + sq(d) - sq(c_const)) / (2 * b_const * d)));
  int S2 = beta - (omega + const_angle);
  S2 = 180 - S2;
  // -----------3-----------
  // смотри тетрадь стр. 18 - 19
  float alpha = degrees(acos((sq(b_const) + sq(c_const) - sq(d)) / (2 * b_const * c_const)));
  int S3 = alpha - const_angle;
  int degrees[] = {S1, S2, S3};

  //------------------------------------------------------------------------------------
  if(leg_num < 3){
    for(int i = 0; i < 3; i++){
      right.setPWM(leg_num * 3 + i, 0, map(degrees[i], 0, 180, SERVOMIN, SERVOMAX));
    }
  }
  if(leg_num >= 3){
    for(int i = 0; i < 3; i++){
      left.setPWM((leg_num - 3) * 3 + i, 0, map(degrees[i], 180, 0, SERVOMIN, SERVOMAX));
    }
  }
}

void ep_tick(){
    if(read_e){
    hub.update("up_dist").value(EEPROM.read(10));
    hub.update("down_dist").value(EEPROM.read(12));
    hub.update("distation").value(EEPROM.read(14));
    hub.update("step_distation").value(EEPROM.read(16));
  }
  if(write_e){
    EEPROM.put(10, up_dist);
    EEPROM.put(12, down_dist);
    EEPROM.put(14, distation);
    EEPROM.put(16, step_distation);
    EEPROM.commit();
  }
}