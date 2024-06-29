#include <Servo.h>
#define a 36   // константа:  плеча А
#define b 46   // константа: l плеча В
#define c 85   // константа: l плеча C
#define q 17.5 // угол между осью плеча B перпендикуляром к A

#define y_lenth1 90
#define x_lenth1 77.5
#define x_lenth2 57.5

// Это все константы механики. Они не меняются, обусловлены конструкцией.
// ИЗ КАДА НЕ ТРОГАЙ 
Servo myservos[18];
int counter;
uint32_t tmr1 = millis();

float positions[6][4]{
  {0, 0, 0, 0}, // нога 0 (х, у) нач (х, у) кон
  {0, 0, 0, 0},
  {0, 0, 0, 0},
  {0, 0, 0, 0},
  {0, 0, 0, 0},
  {0, 0, 0, 0}, // нога 5 (х, у) нач (х, у) кон
};

void setup() {
  Serial.begin(9600);
  for(int i = 0; i < 18; i++) myservos[i].attach(i + 2);
  angle_moving(135, 80, 30, -40, -50, 1000);
  hexapod();
}

void loop() {  
  
}

void angle_moving(float move_angle, int l_dist, int l_step, int l_up, int l_down, int period){
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

void hexapod(){
  if(millis() - tmr1 >= period){
    tmr1 = millis();
    counter += 1;
  }
  switch(counter){
    case 1:
      for(int i = 0; i < 6; i = i + 2) move_to(positions[i][0], positions[i][1], l_down, i);
      for(int i = 1; i < 6; i = i + 2) move_to(positions[i][2], positions[i][3], l_up, i);
      break;
    case 2:
      for(int i = 0; i < 6; i = i + 2) move_to(positions[i][2], positions[i][3], l_down, i);
      for(int i = 1; i < 6; i = i + 2) move_to(positions[i][0], positions[i][1], l_up, i);
      break;
    case 3:
      for(int i = 0; i < 6; i = i + 2) move_to(positions[i][2], positions[i][3], l_up, i);
      for(int i = 1; i < 6; i = i + 2) move_to(positions[i][0], positions[i][1], l_down, i);
      break;
    case 4:
      for(int i = 0; i < 6; i = i + 2) move_to(positions[i][0], positions[i][1], l_up, i);
      for(int i = 1; i < 6; i = i + 2) move_to(positions[i][2], positions[i][3], l_down, i);
      counter = 0;
      break;
  }
}

void move_to(float x, float y, float z, int leg_num){ //координаты ноги в пространстве xyz, и номер серво значение которого он вернет
  // на виде сверху найдем три точки: начало координат (0,0), точка ноги (x, y), доп точка (0, 5)
  // обозначим стороны этого треугольника за e, f, g. f против искомого угла
  int e = 5;                             // сторона между началом координат и доп точкой
  float g = sqrt(sq(x) + sq(y));         // по факту x - 0 и y - 0 но эт не важно
  float f = sqrt(sq(x - 0) + sq(y - 5)); //ищем длинну g и f
  float gamma; 
  if(x == y) gamma = 45;
  else  gamma = degrees(acos((sq(e) + sq(g) - sq(f)) / (2 * e * g)));//угол поворота ноги в суставе 1
  // Здесь мы определили угол поворота первого сустава, дальше определим для второго. 
  // g посути мы теперь используем как х для вида сбоку ноги, а z как y, но использовать мы будем g и z 
  
  float d;      // l от первого сустава до кончика ноги
  float alpha_; // угол треугольника против стороны c
  float alpha;  // угол треугольника против стороны c
  float beta_;  // угол между перпендикуляром к Ох и отр. d
  float beta;   //угол поворота servo2 (вторая серва на ноге)
  float omega;  // угол между d и b

  d = sqrt(sq(g - a) + sq(z));                                           //находим длинну отрезка между координатами сустава 2 и кончика ноги
  alpha_ = degrees(acos((sq(b) + sq(c) - sq(d)) / (2 * b * c)));         // alpha это угол между 2 и третьей фалангой
  beta_ = degrees(acos((sq(d) + sq(z) - sq(g - a)) / (2 * d * abs(z)))); // beta  это угол между d и z
  omega = degrees(acos((sq(b) + sq(d) - sq(c)) / (2 * b * d)));
  beta = 180 - beta_ - omega - q; // вычисляются значения серво 2 и 3
  alpha = alpha_ - q;

  int leg_poz[] = {gamma, beta, alpha};
  if(leg_num > 2) leg_poz[0] = 180 - leg_poz[0];
  /*
  Serial.println(String(leg_poz[0]) + "             1");
  Serial.println(String(leg_poz[1]) + "             2");
  Serial.println(String(leg_poz[2]) + "             3");
  */
  for(int i = 0; i < 3; i++) myservos[leg_num * 3 + i].write(leg_poz[i]);

  //у нас есть три угла, тут надо записать их в сервы и по сути одной этой функцией мы выполняем все движения ног
}

void calculate(float z, float x){
  float g_min;
  float g_max;
  if(z >= 0){
    g_min = sqrt(sq(c) - sq(43.7 - z)) + 50;
  }else if(-z >= b){
    g_min = 15;
  }else{
    g_min = sqrt(sq(43) - sq(z)) + 36;
  }
  g_max = sqrt(17161 - sq(z)) + 36;
  //посчитали ограничения g
  float y_max = sqrt(sq(g_max) - sq(x));
  Serial.print("x  ");
  Serial.print(g_min);
  Serial.print(" / ");
  Serial.print(g_max);
  Serial.print("   y  ");
  Serial.print(y_max);
  Serial.print(" / ");
  Serial.print(-y_max);
  Serial.println("");
}