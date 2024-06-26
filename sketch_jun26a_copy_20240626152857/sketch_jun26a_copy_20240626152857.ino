#define a 36   // константа:  плеча А
#define b 46   // константа: l плеча В
#define c 85   // константа: l плеча C
#define q 17.5 // угол между осью плеча B перпендикуляром к A
// Это все константы механики. Они не меняются, обусловлены конструкцией.
// ИЗ КАДА НЕ ТРОГАЙ 

float start[] = {0 + a, 0};    // тут система x; y
float pos[] = {99, -61};      //тоже померил в каде

void setup() {
  Serial.begin(9600);
}

void loop() {
  float z;      // l от первого сустава до кончика ноги
  float alpha_; // угол треугольника против стороны c
  float alpha;  // угол треугольника против стороны c
  float beta_;  // угол между перпендикуляром к Ох и отр. z
  float beta;   //угол поворота servo2 (вторая серва на ноге)
  float omega;  // угол между z и b

  float x_diff = start[0] - pos[0];
  float y_diff = start[1] - pos[1];
  z = sqrt(sq(x_diff) + sq(y_diff));                                           //находим длинну отрезка между координатами
  alpha_ = degrees(acos((sq(b) + sq(c) - sq(z)) / (2 * b * c)));               // alpha это угол между 2 и третьей фалангой
  beta_ = degrees(acos((sq(z) + sq(y_diff) - sq(x_diff)) / (2 * z * y_diff))); // beta  это угол между z и y_diff
  omega = degrees(acos((sq(b) + sq(z) - sq(c)) / (2 * b * z)));
  beta = 180 - beta_ - omega - q; // вычисляются значения серво 2 и 3
  alpha = alpha_ - q;

  Serial.print(alpha);
  Serial.print("   ");
  Serial.print(beta);
  Serial.println("");
}
