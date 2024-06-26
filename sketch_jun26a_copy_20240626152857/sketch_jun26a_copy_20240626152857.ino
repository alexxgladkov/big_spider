#define a 36  // константа:  плеча А
#define b 46  // константа: l плеча В
#define c 85  // константа: l плеча C
// ИЗ КАДА НЕ ТРОГАЙ 

float z; // l от первого сустава до кончика ноги
float alpha;  // угол треугольника против стороны c
float beta;   // угол треугольника обр. перпендикуляром к основанию координат
              // стороны треугольника: z, x1 - x2, y1 - y2


float start[] = {0 + a, 0};    // тут система x; y
float pos[] = {50, -41.5};      //тоже померил в каде

void setup() {
  Serial.begin(9600);
}

void loop() {
  float x_diff = start[0] - pos[0];
  float y_diff = start[1] - pos[1];
  z = sqrt(sq(x_diff) + sq(y_diff));                                           //находим длинну отрезка между координатами
  alpha = degrees(acos((sq(b) + sq(c) - sq(z)) / (2 * b * c)));                // alpha это угол между 2 и третьей фалангой
  beta = degrees(acos((sq(z) + sq(y_diff) - sq(x_diff)) / (2 * z * y_diff)));  // beta  это угол между z и y_diff

  Serial.print(alpha);
  Serial.print("   ");
  Serial.print(beta);
  Serial.println("");
}
