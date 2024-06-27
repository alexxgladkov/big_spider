#include "ServoDriverSmooth.h"
#define a 36   // константа:  плеча А
#define b 46   // константа: l плеча В
#define c 85   // константа: l плеча C
#define q 17.5 // угол между осью плеча B перпендикуляром к A
// Это все константы механики. Они не меняются, обусловлены конструкцией.
// ИЗ КАДА НЕ ТРОГАЙ 
ServoDriverSmooth legs[18];

void setup() {
  Serial.begin(9600);
  for(int i = 0; i < 19; i++) legs[i].attach(i);
}

void loop() {  
  for(int i = 0; i < 19; i++) legs[i].tick();
}

void count(float x, float y, float z, int serv){ //координаты ноги в пространстве xyz, и номер серво значение которого он вернет
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

  Serial.print(alpha);
  Serial.print("   ");
  Serial.print(beta);
  Serial.print("   ");
  Serial.print(gamma);
  Serial.println("");

  if(serv == 1) return(gamma);
  if(serv == 2) return(beta);
  if(serv == 3) return(alpha);
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