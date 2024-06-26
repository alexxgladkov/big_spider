#define a 36  // константа: l плеча А
#define b 46  // константа: l плеча В
#define c 85  // константа: l плеча В
// ИЗ КАДА НЕ ТРОГАЙ 

float z; // l от первого сустава до кончика ноги
float alpha;  // угол треугольника против стороны c

float start[] = {36, 45.5}; // тут система x; y
float pos[] = {50, 4};      //тоже померил в каде

void setup() {
  Serial.begin(9600);
}

void loop() {
  /*
  c = sqrt(sq(a) + sq(b) - 2*a*b*cos(alpha));
  */
  z = sqrt(sq(start[0] - pos[0]) + sq(start[1] - pos[1])); //находим длинну отрезка между координатами
  alpha = degrees(acos((sq(b) + sq(c) - sq(z)) / (2 * b * c))); // alpha это угол между 2 и третьей фалангой
  Serial.println(alpha);
}
