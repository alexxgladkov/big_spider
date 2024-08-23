#include <Adafruit_PWMServoDriver.h>    // Подключаем библиотеку Adafruit_PWMServoDriver
#include <GyverHub.h>
#include <EEPROM.h>

GyverHub hub("MyDevices", "ESP8266", "");  // имя сети, имя устройства, иконка

Adafruit_PWMServoDriver right = Adafruit_PWMServoDriver(0x42);
Adafruit_PWMServoDriver left = Adafruit_PWMServoDriver(0x41);
//-----------------------------------------------------------------------------------------
// библиотеки

#define SERVOMIN  90                // Минимальная длительность импульса для сервопривода
#define SERVOMAX  500               // Максимальная длина импульса для сервопривода

#define a_const 34       // константа:  плеча А
#define b_const 39       // константа: l плеча В
#define c_const 77       // константа: l плеча C
#define const_angle 18.5 // угол между осью плеча B перпендикуляром к A

#define dist_1 165
#define dist_2 125
#define dist_3 90

int corrections[] = {-45, 0, 45, 45, 0, -45}; // поправки для углов серво первого плеча ног
//-----------------------------------------------------------------------------------------
// Это все константы механики. Они не меняются, обусловлены конструкцией

gh::Flag right_h, front, left_h, stop, read_e, write_e, hex, quad, wave;
uint16_t up_dist, down_dist, distation, step_distation, pos;   
uint16_t alpha_sld = 20;
uint16_t beta_sld = 20;
String resim = "";
String moving_res = "";
uint16_t timing = 1000;
//-----------------------------------------------------------------------------------------
// переменные для управления со смартфона

int per = 50;
int counter_hex = 0;
int counter_quad = 0;
uint32_t tmr1 = millis();
uint32_t tmr2 = millis();
uint32_t tmr3 = millis();
//-----------------------------------------------------------------------------------------
// таймеры для работы прошивки

float positions[6][4]{
  {0, 0, 0, 0}, // нога 0 (х, у) нач (х, у) кон
  {0, 0, 0, 0},
  {0, 0, 0, 0},
  {0, 0, 0, 0},
  {0, 0, 0, 0},
  {0, 0, 0, 0}, // нога 5 (х, у) нач (х, у) кон
};

float last_angle[3][6] =  {
  {90, 90, 90, 90, 90, 90},
  {180, 180, 180, 180, 180, 180},
  {0, 0, 0, 0, 0, 0},
};
//-----------------------------------------------------------------------------------------
// массивы для хранения положений ног

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
  b.Slider(&pos).label("поворот").range(0, 360, 10);
  b.Button().label("стоп").attach(&stop).size(450); 
  b.Text("Параметры шага").rows(1);
  b.beginRow();
  b.Slider_("up_dist", &up_dist).label("верхняя точка").range(0, 100, 5);
  b.Slider_("down_dist", &down_dist).label("нижняя точка").range(0, 100, 5);
  b.endRow();
  b.beginRow();
  b.Slider_("distation", &distation).label("отводит на").range(20, 120, 10);
  b.Slider_("step_distation", &step_distation).label("шагает на").range(0, 50, 5);
  b.endRow();
  b.Text("eeprom").rows(1);
  b.beginRow();
  b.Button().label("записать").attach(&write_e);  
  b.Button().label("считать").attach(&read_e);  
  b.endRow();
  b.Input(&per);
}
//-----------------------------------------------------------------------------------------
// конструируем интерфейс

void setup() {
  Serial.begin(9600);
  //-----------------Serial------------------
  right.begin();             
  left.begin();
  right.setOscillatorFrequency(27000000);    
  left.setOscillatorFrequency(27000000);
  right.setPWMFreq(50);         
  left.setPWMFreq(50);
  //------------------Servo------------------
  WiFi.mode(WIFI_STA);
  WiFi.begin("Penibord_2G", "StrA97!B16");
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  hub.onBuild(build); // подключаем билдер
  hub.begin();        // запускаем систему
  //---------------Приложение----------------
  EEPROM.begin(1000);   // для esp8266/esp32
  up_dist = EEPROM.read(10);
  down_dist = EEPROM.read(12);
  distation = EEPROM.read(14);
  step_distation = EEPROM.read(16);
  //------------------память-----------------
}

void loop() {  
  ep_tick();
  hub.tick();

  if(right_h) resim = "r"; 
  if(left_h) resim = "l"; 
  if(front) resim = "f"; 
  if(stop) resim = "s";

  if(resim == "r"){
    rotation(distation, step_distation, 0);
  }else if(resim == "l"){
    rotation(distation, step_distation, 1);
  }else if(resim == "f"){
    angle_moving(pos, distation, step_distation);
  }

  if(hex) moving_res = "h";
  if(quad) moving_res = "q";

  if(moving_res == "h"){
    hexapod(timing, -up_dist, -down_dist);
  }else if(moving_res == "q"){
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
      if(i < 3){
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
    }if(direction == 1){
      if(i >= 3){
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
}

void angle_moving(float move_angle, int l_dist, int l_step){
  //нужно посчитать координаты х у для этого рассмотрим треугольник с углом move_angle 
  //и сторонами l_step, l_perp и l_diff. Ищем стороны l_perp и l_diff
  float l_perp = sin(radians(move_angle)) * l_step;
  float l_diff = sin(radians(90 - move_angle)) * l_step;

  for(int i = 0; i < 6; i++){             // X Y здесь записаны позиции для движения ног
    positions[i][0] = l_dist - l_diff;
    positions[i][2] = l_dist + l_diff;
    if(i == 0 || i == 5){
      positions[i][3] = -l_perp + l_dist;
      positions[i][1] = l_perp + l_dist; 
    }if(i == 1 || i == 4){
      positions[i][3] = -l_perp;
      positions[i][1] = l_perp; 
    }if(i == 2 || i == 3){
      positions[i][3] = -l_perp - l_dist;
      positions[i][1] = l_perp - l_dist; 
    }
  }
}

void hexapod(int period, int up, int down){
  int the_pos_hex[3][6];// 3 координаты 6 ног
  float pos_hex[6][4] = {
    {1, 0.5, 0.5, 0},
    {0.5, 0, 1, 0.5},
    {1, 0.5, 0.5, 0},
    {0.5, 0, 1, 0.5},
    {1, 0.5, 0.5, 0},
    {0.5, 0, 1, 0.5},    
  };

  int takt_hex[4] = {3, 1, 3, 1};
  int lenth_hex = 0;
  for(int i = 0; i < 4; i++) lenth_hex += takt_hex[i];
  float full_hex[6][lenth_hex];
  int count_hex = 0;

  for(int i = 0; i < 6; i++){
    for(int j = 0; j < 4; j++){
      if(takt_hex[j] == 1){
        full_hex[i][count_hex] = pos_hex[i][j];
        count_hex += 1;
      }else if(takt_hex[j] == 3){
        if(pos_hex[i][j] == 1){
          full_hex[i][count_hex] = 1;
          full_hex[i][count_hex + 1] = 2;
          full_hex[i][count_hex + 2] = 3;
        }if(pos_hex[i][j] != 1){
          full_hex[i][count_hex] = pos_hex[i][j];
          full_hex[i][count_hex + 1] = pos_hex[i][j];
          full_hex[i][count_hex + 2] = pos_hex[i][j];
        }
        count_hex += 3;
      }
    }
    count_hex = 0;
  }

  if(millis() - tmr2 >= period){
    tmr2 = millis();
    counter_hex += 1; 
    if(counter_hex == lenth_hex){
      counter_hex = 0;
    } 
  }

  for(int i = 0; i < 6; i++){
    if(full_hex[i][counter_hex] == 0){
      the_pos_hex[0][i] = positions[i][2];
      the_pos_hex[1][i] = positions[i][3];
      the_pos_hex[2][i] = down;
    }else if(full_hex[i][counter_hex] == 1){
      the_pos_hex[0][i] = positions[i][2];
      the_pos_hex[1][i] = positions[i][3];
      the_pos_hex[2][i] = up;
    }else if(full_hex[i][counter_hex] == 2){
      the_pos_hex[0][i] = positions[i][0];
      the_pos_hex[1][i] = positions[i][1];
      the_pos_hex[2][i] = up;
    }else if(full_hex[i][counter_hex] == 3){
      the_pos_hex[0][i] = positions[i][0];
      the_pos_hex[1][i] = positions[i][1];
      the_pos_hex[2][i] = down;
    }else{
      the_pos_hex[0][i] = positions[i][0] + ((positions[i][2] - positions[i][0]) * full_hex[i][counter_hex]);
      the_pos_hex[1][i] = positions[i][1] + ((positions[i][3] - positions[i][1]) * full_hex[i][counter_hex]);
      the_pos_hex[2][i] = down;
    }
  }

  for(int i = 0; i < 6; i++){
    move_to(the_pos_hex[0][i], the_pos_hex[1][i], the_pos_hex[2][i], i);
  }
}

void quadropod(int period, int up, int down){
  int the_pos_quad[3][6];// 3 координаты 6 ног
  float pos_quad[6][6] = {
    {1,    0.33, 0.33, 0.66, 0.66, 0},
    {0.66, 0,    1,    0.33, 0.33, 0.66},
    {0.33, 0.66, 0.66, 0,    1,    0.33},
    {1,    0.33, 0.33, 0.66, 0.66, 0},
    {0.33, 0.66, 0.66, 0,    1,    0.33},
    {0.66, 0,    1,    0.33, 0.33, 0.66},
  };

  int takt_quad[6] = {3, 1, 3, 1, 3, 1};
  int lenth_quad = 0;
  for(int i = 0; i < 6; i++) lenth_quad += takt_quad[i];
  float full_quad[6][lenth_quad];
  int count_quad = 0;

  for(int i = 0; i < 6; i++){
    for(int j = 0; j < 6; j++){
      if(takt_quad[j] == 1){
        full_quad[i][count_quad] = pos_quad[i][j];
        count_quad += 1;
      }else if(takt_quad[j] == 3){
        if(pos_quad[i][j] == 1){
          full_quad[i][count_quad] = 1;
          full_quad[i][count_quad + 1] = 2;
          full_quad[i][count_quad + 2] = 3;
        }if(pos_quad[i][j] != 1){
          full_quad[i][count_quad] = pos_quad[i][j];
          full_quad[i][count_quad + 1] = pos_quad[i][j];
          full_quad[i][count_quad + 2] = pos_quad[i][j];
        }
        count_quad += 3;
      }
    }
    count_quad = 0;
  }

  if(millis() - tmr1 >= period){
    tmr1 = millis();
    counter_quad += 1; 
    if(counter_quad == lenth_quad){
      counter_quad = 0;
    } 
  }

  for(int i = 0; i < 6; i++){
    if(full_quad[i][counter_quad] == 0){
      the_pos_quad[0][i] = positions[i][2];
      the_pos_quad[1][i] = positions[i][3];
      the_pos_quad[2][i] = down;
    }else if(full_quad[i][counter_quad] == 1){
      the_pos_quad[0][i] = positions[i][2];
      the_pos_quad[1][i] = positions[i][3];
      the_pos_quad[2][i] = up;
    }else if(full_quad[i][counter_quad] == 2){
      the_pos_quad[0][i] = positions[i][0];
      the_pos_quad[1][i] = positions[i][1];
      the_pos_quad[2][i] = up;
    }else if(full_quad[i][counter_quad] == 3){
      the_pos_quad[0][i] = positions[i][0];
      the_pos_quad[1][i] = positions[i][1];
      the_pos_quad[2][i] = down;
    }else{
      the_pos_quad[0][i] = positions[i][0] + ((positions[i][2] - positions[i][0]) * full_quad[i][counter_quad]);
      the_pos_quad[1][i] = positions[i][1] + ((positions[i][3] - positions[i][1]) * full_quad[i][counter_quad]);
      the_pos_quad[2][i] = down;
    }
  }

  for(int i = 0; i < 6; i++){
    move_to(the_pos_quad[0][i], the_pos_quad[1][i], the_pos_quad[2][i], i);
  }
}

void angleing(int basic_pos, int alph, int bet, int dist_x){
  int x_abs = dist_x / sqrt(2);
  //--------------------1---------------------
  int dist_a1 = tan(radians(alph)) * ((dist_1 / 2) + (x_abs));
  int dist_a2 = tan(radians(alph)) * ((dist_2 / 2) + dist_x);
  //--------------------2---------------------
  int dist_b = tan(radians(bet) * (dist_3 + (dist_x / sqrt(2))));
  //--------------------3---------------------
  int loc_pos[3][6]{
    {x_abs, dist_x, x_abs, x_abs, dist_x, x_abs},
    {x_abs, 0, -x_abs, -x_abs, 0, x_abs},
    {dist_a1 + dist_b, dist_a1, dist_b, -dist_a1 - dist_b, -dist_a1, -dist_a1 + dist_b},
  };
  for(int i = 0; i < 6; i++) loc_pos[3][i] += basic_pos;
  for(int i = 0; i < 6; i++){
    move_to(loc_pos[0][i], loc_pos[1][i], loc_pos[2][i], i);
  }
}

void stand_up(int distation_, int up_dist_, int down_dist_){
  int root_dist = distation_ / sqrt(2);
  int loc_pos[2][6]{
    {root_dist, distation_, root_dist, root_dist, distation_, root_dist},
    {root_dist, 0, root_dist, root_dist, 0, root_dist},
  };

  for(int i = up_dist_; i > down_dist_; i--){
    for(int j = 0; j < 6; j++){
      move_to(loc_pos[0][j], loc_pos[1][j], i, j);
    }
  }
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
      if(millis() - tmr3 >= per){
        if(abs(last_angle[i][leg_num] - degrees[i]) > 0){
          int edenitsa = (last_angle[i][leg_num] - degrees[i]) / abs(last_angle[i][leg_num] - degrees[i]);
          last_angle[i][leg_num] -= edenitsa;
          right.setPWM(leg_num * 3 + i, 0, map(last_angle[i][leg_num], 0, 180, SERVOMIN, SERVOMAX));
        }
      }
      right.setPWM(leg_num * 3 + i, 0, map(degrees[i], 0, 180, SERVOMIN, SERVOMAX));
    }
  }if(leg_num >= 3){
    for(int i = 0; i < 3; i++){
      if(millis() - tmr3 >= per){
        if(abs(last_angle[i][leg_num] - degrees[i]) > 0){
          int edenitsa = (last_angle[i][leg_num] - degrees[i]) / abs(last_angle[i][leg_num] - degrees[i]);
          last_angle[i][leg_num] -= edenitsa;
          left.setPWM((leg_num - 3) * 3 + i, 0, map(last_angle[i][leg_num], 180, 0, SERVOMIN, SERVOMAX));
        }
      }
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