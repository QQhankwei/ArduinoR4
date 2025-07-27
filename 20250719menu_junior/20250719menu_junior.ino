#include "MatrixMiniR4.h"
#define PI 3.1415926

float degree_old;
float error_new;
float error_old;
float error;
float Kp;
float kd;
float speed;
float error3_new;
float error3_old;
float error3;
float speed3;
float degree2_old;
float degree3_old;
float degree4_old;
float light3;
float light4;
float light5;
float light6;
float color1;
float color2;
float color3;
float color4;
float color5;
float color6;
float light_Y;
float light_X;
float sensordata1;
float light1;
float light2;
float sensordata2;
float sensordata3;
float sensordata4;
float light7;
float light8;
float tempColor;
float EV3_Color_read;
float S_value;
float menu;

void TwoServo(float degree, float delay2) {
  if((degree - degree_old) > 0)
  {
    while(!(degree_old == degree))
    {
      degree_old = degree_old + 1;
      MiniR4.RC2.setAngle(degree_old);
      MiniR4.RC3.setAngle(degree_old);
      delay(delay2);
    }
  }
  else
  {
    if((degree - degree_old) < 0)
    {
      while(!(degree_old == degree))
      {
        degree_old = degree_old - 1;
        MiniR4.RC2.setAngle(degree_old);
        MiniR4.RC3.setAngle(degree_old);
        delay(delay2);
      }
    }
    else
    {
    }
  }
  MiniR4.OLED.clearDisplay();
  MiniR4.OLED.setCursor(10, 10);
  MiniR4.OLED.print(degree_old);
  MiniR4.OLED.display();
}

void move_PD_on(float power) {
  error_new = abs(MiniR4.M2.getCounter()) - abs(MiniR4.M3.getCounter());
  error = error_new - error_old;
  speed = (Kp * error_new) + (kd * error);
  MiniR4.M2.setPower((power - speed));
  MiniR4.M3.setPower((power + speed));
  error_old = error_new;
}

void motor2_PD() {
  error_new = 0 - MiniR4.M2.getCounter();
  error = error_new - error_old;
  speed = (Kp * error_new) + (kd * error);
  MiniR4.M2.setPower(speed);
  error_old = error_new;
}

void motor3_PD() {
  error3_new = 0 - MiniR4.M3.getCounter();
  error3 = error3_new - error3_old;
  speed3 = (Kp * error3_new) + (kd * error3);
  MiniR4.M3.setPower(speed3);
  error3_old = error3_new;
}
unsigned long timer_1 = 0;
void move_initial(float direction) {
  if(direction == 1)
  {
    MiniR4.M2.setReverse(true);
    MiniR4.M3.setReverse(false);
  }
  else
  {
    MiniR4.M2.setReverse(false);
    MiniR4.M3.setReverse(true);
  }
  error_new = 0;
  error_old = 0;
  Kp = 1.5;
  kd = 0.3;
  MiniR4.M2.resetCounter();
  MiniR4.M3.resetCounter();
}

void brake_hold(float time) {
  error_new = 0;
  error_old = 0;
  error3_new = 0;
  error3_old = 0;
  Kp = 10;
  kd = 10;
  timer_1 = millis();
  MiniR4.M2.resetCounter();
  MiniR4.M3.resetCounter();
  while(!((millis() - timer_1) > time))
  {
    motor2_PD();
    motor3_PD();
  }
  MiniR4.M2.setBrake(true);
  MiniR4.M3.setBrake(true);
}

void move_tank_off(float brake) {
  if(brake == 1)
  {
    brake_hold(20);
  }
  else
  {
  }
}

void move_Second(float direction, float power, float time, float brake) {
  move_initial(direction);
  timer_1 = millis();
  while(!((millis() - timer_1) > (time * 1000)))
  {
    move_PD_on(power);
  }
  move_tank_off(brake);
}

void Gyroturn(float p, float d, float power, float deg) {
  move_initial(1);
  error_new = 0;
  error_old = 0;
  speed = 0;
  while(!(MiniR4.Motion.getEuler(MiniR4Motion::AxisType::Yaw) == deg))
  {
    error_new = deg - MiniR4.Motion.getEuler(MiniR4Motion::AxisType::Yaw);
    error = error_new - error_old;
    speed = (p * error_new) + (d * error);
    if(error_new > 0)
    {
      MiniR4.M3.setPower((20 + speed));
      MiniR4.M2.setPower((-20 - speed));
    }
    if(error_new < 0)
    {
      MiniR4.M3.setPower(((-20) + speed));
      MiniR4.M2.setPower((20 - speed));
    }
    if(error_new == 0)
    {
      MiniR4.M3.setBrake(true);
    }
    error_old = error_new;
    MiniR4.OLED.clearDisplay();
    MiniR4.OLED.setCursor(10, 10);
    MiniR4.OLED.print(MiniR4.Motion.getEuler(MiniR4Motion::AxisType::Yaw));
    MiniR4.OLED.display();
  }
  move_tank_off(1);
}

void move_degree(float direction, float power, float degree, float brake) {
  move_initial(direction);
  while(!(((abs(MiniR4.M2.getCounter()) + abs(MiniR4.M3.getCounter())) / 2) > degree))
  {
    move_PD_on(power);
  }
  move_tank_off(brake);
}

void servo_initial(float degree, float port) {
  if(port == 1)
  {
    degree_old = degree;
    MiniR4.RC1.setHWDir(false);
    MiniR4.RC1.setAngle(degree);
  }
  if(port == 2)
  {
    degree2_old = degree;
    MiniR4.RC2.setHWDir(false);
    MiniR4.RC2.setAngle(degree);
  }
  if(port == 3)
  {
    degree3_old = degree;
    MiniR4.RC3.setHWDir(false);
    MiniR4.RC3.setAngle(degree);
  }
  if(port == 4)
  {
    degree4_old = degree;
    MiniR4.RC4.setHWDir(false);
    MiniR4.RC4.setAngle(degree);
  }
}

void servo_second(float degree, float delay2, float port) {
  if(port == 1)
  {
    if((degree - degree_old) > 0)
    {
      while(!(degree_old == degree))
      {
        degree_old = degree_old + 1;
        MiniR4.RC1.setAngle(degree_old);
        delay(delay2);
      }
    }
    else
    {
      if((degree - degree_old) < 0)
      {
        while(!(degree_old == degree))
        {
          degree_old = degree_old - 1;
          MiniR4.RC1.setAngle(degree_old);
          delay(delay2);
        }
      }
      else
      {
      }
    }
    MiniR4.OLED.clearDisplay();
    MiniR4.OLED.setCursor(10, 10);
    MiniR4.OLED.print(degree_old);
    MiniR4.OLED.display();
  }
  if(port == 2)
  {
    if((degree - degree2_old) > 0)
    {
      while(!(degree2_old == degree))
      {
        degree2_old = degree2_old + 1;
        MiniR4.RC2.setAngle(degree2_old);
        delay(delay2);
      }
    }
    else
    {
      if((degree - degree2_old) < 0)
      {
        while(!(degree2_old == degree))
        {
          degree2_old = degree2_old - 1;
          MiniR4.RC2.setAngle(degree2_old);
          delay(delay2);
        }
      }
      else
      {
      }
    }
    MiniR4.OLED.clearDisplay();
    MiniR4.OLED.setCursor(10, 10);
    MiniR4.OLED.print(degree2_old);
    MiniR4.OLED.display();
  }
  if(port == 3)
  {
    if((degree - degree3_old) > 0)
    {
      while(!(degree3_old == degree))
      {
        degree3_old = degree3_old + 1;
        MiniR4.RC3.setAngle(degree3_old);
        delay(delay2);
      }
    }
    else
    {
      if((degree - degree3_old) < 0)
      {
        while(!(degree3_old == degree))
        {
          degree3_old = degree3_old - 1;
          MiniR4.RC3.setAngle(degree3_old);
          delay(delay2);
        }
      }
      else
      {
      }
    }
    MiniR4.OLED.clearDisplay();
    MiniR4.OLED.setCursor(10, 10);
    MiniR4.OLED.print(degree3_old);
    MiniR4.OLED.display();
  }
  if(port == 4)
  {
    if((degree - degree4_old) > 0)
    {
      while(!(degree4_old == degree))
      {
        degree4_old = degree4_old + 1;
        MiniR4.RC4.setAngle(degree4_old);
        delay(delay2);
      }
    }
    else
    {
      if((degree - degree4_old) < 0)
      {
        while(!(degree4_old == degree))
        {
          degree4_old = degree4_old - 1;
          MiniR4.RC4.setAngle(degree4_old);
          delay(delay2);
        }
      }
      else
      {
      }
    }
    MiniR4.OLED.clearDisplay();
    MiniR4.OLED.setCursor(10, 10);
    MiniR4.OLED.print(degree4_old);
    MiniR4.OLED.display();
  }
}

void move_tank(float mode, float L_power, float R_power, float unit, float brake) {
  move_initial(1);
  if(mode == 1)
  {
    if(L_power == R_power)
    {
      if((L_power > 0) && (R_power > 0))
      {
        move_degree(1, L_power, unit, brake);
      }
      else
      {
        move_degree(0, abs(L_power), unit, brake);
      }
    }
    else
    {
      if((L_power == 0)||(R_power == 0))
      {
        move_initial(1);
        while(!(((abs(MiniR4.M2.getCounter()) + abs(MiniR4.M3.getCounter())) / 1) > unit))
        {
          MiniR4.M2.setPower(L_power);
          MiniR4.M3.setPower(R_power);
        }
        move_tank_off(brake);
      }
      else
      {
        move_initial(1);
        while(!(((abs(MiniR4.M2.getCounter()) + abs(MiniR4.M3.getCounter())) / 2) > unit))
        {
          MiniR4.M2.setPower(L_power);
          MiniR4.M3.setPower(R_power);
        }
        move_tank_off(brake);
      }
    }
  }
  else
  {
    if(mode == 2)
    {
      if(L_power == R_power)
      {
        if((L_power > 0) && (R_power > 0))
        {
          move_Second(1, L_power, unit, brake);
        }
        else
        {
          move_Second(0, abs(L_power), unit, brake);
        }
      }
      else
      {
        move_initial(1);
        timer_1 = millis();
        while(!((millis() - timer_1) > (unit * 1000)))
        {
          MiniR4.M2.setPower(L_power);
          MiniR4.M3.setPower(R_power);
        }
        move_tank_off(brake);
      }
    }
    else
    {
    }
  }
}

void BWB(float L, float R, float brake) {
  move_initial(1);
  light_8();
  while(!(((light3 < 5) && (light4 < 5)) && ((light5 < 5) && (light6 < 5))))
  {
    light_8();
    MiniR4.M2.setPower(L);
    MiniR4.M3.setPower(R);
  }
  move_tank_off(brake);
}
void PD_for_all(float mode, float kp, float kd, float power, float degree_width, float brake) {
  move_initial(1);
  if(mode == 0)
  {
    while(!(((abs(MiniR4.M3.getCounter()) + abs(MiniR4.M2.getCounter())) / 2) > degree_width))
    {
      error_new = (MiniR4.I2C4.MXColor.getColor(R) - 8) - 0;
      error = error_new - error_old;
      speed = (kp * error_new) + (kd * error);
      MiniR4.M2.setPower((power - speed));
      MiniR4.M3.setPower((power + speed));
      error_old = error_new;
    }
    brake_hold(brake);
  }
  if(mode == 2)
  {
    while(!(MiniR4.I2C4.MXColor.getColor(G) > 6))
    {
      error_new = (MiniR4.I2C4.MXColor.getColor(R) - 8) - 0;
      error = error_new - error_old;
      speed = (kp * error_new) + (kd * error);
      MiniR4.M2.setPower((power - speed));
      MiniR4.M3.setPower((power + speed));
      error_old = error_new;
    }
    brake_hold(brake);
  }
  if(mode == 3)
  {
    timer_1 = millis();
    while(!((millis() - timer_1) > degree_width))
    {
      error_new = 0 - MiniR4.Motion.getEuler(MiniR4Motion::AxisType::Yaw);
      error = error_new - error_old;
      speed = (kp * error_new) + (kd * error);
      MiniR4.M2.setPower((power - speed));
      MiniR4.M3.setPower((power + speed));
      error_old = error_new;
    }
    brake_hold(brake);
  }
}

void R_take(float GREEN_RED) {
  float colors[6] = {color1, color2, color3, color4, color5, color6};
  
  for (int i = 0; i < 6; i++) {
    if (colors[i] == GREEN_RED) {
      GR_1_6(i + 1); // index 從 0 開始，所以要 +1
      return;
    }
  }

  // 如果都沒找到就往後退
  Acc_move(-30, -50, 200, 50, 50, 1);
}

void L_take(float W6_Y4) {
  float colors[6] = {color1, color2, color3, color4, color5, color6};

  for (int i = 0; i < 6; i++) {
    if (colors[i] == W6_Y4) {
      WY_1_6(i + 1); // index 從 0 開始所以要 +1
      return;
    }
  }
  // 全部都沒找到 → 執行退後動作
  Acc_move(-30, -50, 200, 50, 50, 1);
}

void light_8() {
  sensordata1 = MiniR4.I2C4.MXColor.getColor(C);
  light1 = floor((sensordata1 / 10));
  light2 = floor((fmod(sensordata1, 10)));
  sensordata2 = MiniR4.I2C4.MXColor.getColor(M);
  light3 = floor((sensordata2 / 10));
  light4 = floor((fmod(sensordata2, 10)));
  sensordata3 = MiniR4.I2C4.MXColor.getColor(Y);
  light5 = floor((sensordata3 / 10));
  light6 = floor((fmod(sensordata3, 10)));
  sensordata4 = MiniR4.I2C4.MXColor.getColor(K);
  light7 = floor((sensordata4 / 10));
  light8 = floor((fmod(sensordata4, 10)));
  light_X = round(((light1 + (light2 + (light3 + light4))) / 4));
  light_Y = round(((light5 + (light6 + (light7 + light8))) / 4));
}

void AreaUP() { 
  MBC_degree(3, 7.5, 30, 100, 0);
  MBC_black(3, 3, 30, 1, 1,50);
  delay(100);
  move_tank(1, -70, 0, 298, 1);
  Acc_move(-30, -50, 350, 100, 100, 0);
  move_tank(2, -30, -30, 0.5, 1);
  delay(100);
  move_tank(1, 0, 50, 302, 1);
  delay(100);
  Acc_move(30, 50, 350, 150, 200, 1);
  MBC_degree(3, 7.5, 25, 100, 0);
  MBC_black(3, 3, 25, 1, 1,50);
  delay(100);
  degree_old = 120;
  TwoServo(82, 20);
  delay(500);
  Acc_move(-25, -80, 500, 150, 250, 1);
  delay(200);
  move_tank(1, 50, -50, 148, 1);
  move_tank(2, -30, -30, 0.5, 1);
  MiniR4.RC2.setAngle(135);
  MiniR4.RC3.setAngle(135);
  Acc_move(30, 70, 515, 100, 250, 1);
  delay(200);
  R_drift(35, 80);
}

void Area_Down() {
  MBC_degree(3, 7.5, 30, 100, 0);
  MBC_black(3, 3, 30, 1, 1,50);
  delay(200);
  move_tank(1, 0, -50, 304, 1);
  Acc_move(-30, -50, 350, 100, 100, 0);
  move_tank(2, -40, -40, 0.5, 1);
  delay(150);
  move_tank(1, 50, 50, 15, 1);
  delay(100);
  move_tank(1, 50, 0, 306, 1);
  delay(100);
  Acc_move(30, 50, 350, 150, 200, 1);
  MBC_degree(3, 7.5, 25, 100, 0);
  MBC_black(3, 3, 25, 1, 1,50);
  delay(100);
  degree_old = 120;
  TwoServo(82, 20);
  delay(500);
  Acc_move(-25, -80, 550, 150, 250, 1);
  delay(200);
  move_tank(1, -50, 50, 148, 1);
  move_tank(2, -40, -40, 0.6, 1);
  MiniR4.RC2.setAngle(135);
  MiniR4.RC3.setAngle(135);
  Acc_move(30, 70, 530, 100, 250, 1);
  delay(200);
  L_drift(30, 80);
}

void L_drift(float power, float degree) {
  move_initial(1);
  light_8();
  move_tank(1, -70, 70, degree, 0);
  while(!((light1 > 8) && (light2 > 8)))
  {
    light_8();
    MiniR4.M2.setPower((power * (-1)));
    MiniR4.M3.setPower(power);
  }
  brake_hold(1);
  while(!(light1 < 6))
  {
    light_8();
    MiniR4.M2.setPower((power * (-1)));
    MiniR4.M3.setPower(power);
  }
  brake_hold(1);
}

void R_drift(float power, float deg) {
  move_initial(1);
  light_8();
  move_tank(1, 70, -70, deg, 0);
  while(!((light8 > 8) && (light7 > 8)))
  {
    light_8();
    MiniR4.M2.setPower(power);
    MiniR4.M3.setPower((power * (-1)));
  }
  brake_hold(1);
  while(!(light8 < 6))
  {
    light_8();
    MiniR4.M2.setPower(power);
    MiniR4.M3.setPower((power * (-1)));
  }
  brake_hold(1);
}

void WY_1_6(float Num) {
  if(Num == 6)
  {
    move_tank(1, 0, 45, 313, 1);
    delay(150);
    move_tank(1, 30, 30, 170, 1);
    delay(150);
    move_tank(1, 0, -45, 309, 1);
    delay(150);
    servo_second(84, 1, 2);
    delay(250);
    move_tank(1, 30, 30, 70, 1);
    servo_second(120, 10, 2);
    delay(150);
    move_tank(1, -50, -50, 70, 1);
    delay(150);
    move_tank(1, -50, 0, 275, 1);
    delay(150);
    move_tank(1, 0, -50, 280, 1);
  }
  if(Num == 5)
  {
    move_tank(1, 0, 45, 313, 1);
    delay(150);
    move_tank(1, 30, 30, 70, 1);
    delay(150);
    move_tank(1, 0, -45, 309, 1);
    delay(150);
    servo_second(84, 1, 2);
    delay(250);
    move_tank(1, 30, 30, 70, 1);
    servo_second(120, 10, 2);
    delay(150);
    move_tank(1, -50, -50, 70, 1);
    delay(150);
    move_tank(1, -50, 0, 170, 1);
    delay(150);
    move_tank(1, 0, -50, 175, 1);
  }
  if(Num == 4)
  {
    servo_second(84, 1, 2);
    delay(250);
    move_tank(1, 50, 0, 20, 1);
    move_tank(1, 30, 30, 30, 1);
    servo_second(120, 10, 2);
    move_tank(1, -30, -30, 30, 1);
    move_tank(1, -40, 0, 40, 1);
    delay(100);
    move_tank(1, -50, -50, 150, 1);
  }
  if(Num == 3)
  {
    servo_second(84, 1, 2);
    delay(250);
    move_tank(1, 0, -50, 85, 1);
    move_tank(1, 30, 30, 100, 1);
    servo_second(120, 10, 2);
    move_tank(1, -30, -30, 100, 1);
    move_tank(1, 0, 50, 85, 1);
    move_tank(1, -50, -50, 150, 1);
  }
  if(Num == 2)
  {
    move_tank(1, 45, 0, 298, 1);
    delay(150);
    move_tank(1, 30, 30, 260, 1);
    delay(150);
    move_tank(1, -45, 0, 303, 1);
    delay(150);
    servo_initial(84, 2);
    delay(250);
    move_tank(1, 30, 30, 45, 1);
    servo_second(120, 10, 2);
    move_tank(1, -30, -30, 45, 1);
    move_tank(1, 0, -50, 250, 1);
    move_tank(1, -30, -30, 130, 1);
    move_tank(1, -50, 0, 250, 1);
  }
  if(Num == 1)
  {
    move_tank(1, 45, 0, 298, 1);
    delay(150);
    move_tank(1, 30, 30, 365, 1);
    delay(150);
    move_tank(1, -45, 0, 298, 1);
    delay(150);
    servo_initial(84, 2);
    delay(250);
    move_tank(1, 30, 30, 65, 1);
    servo_second(120, 10, 2);
    delay(150);
    move_tank(1, -30, -30, 65, 1);
    move_tank(1, 0, -50, 250, 1);
    move_tank(1, -30, -30, 250, 1);
    move_tank(1, -50, 0, 250, 1);
  }
}

void GR_1_6(float Num) {
  if(Num == 6)
  {
    move_tank(1, 0, 45, 313, 1);
    delay(150);
    move_tank(1, 30, 30, 365, 1);
    delay(150);
    move_tank(1, 0, -45, 309, 1);
    delay(150);
    servo_initial(84, 3);
    delay(250);
    move_tank(1, 30, 30, 70, 1);
    servo_second(120, 10, 3);
    delay(150);
    move_tank(1, -50, -50, 70, 1);
    delay(150);
    move_tank(1, -50, 0, 240, 1);
    delay(150);
    move_tank(1, -40, -40, 210, 1);
    delay(150);
    move_tank(1, 0, -50, 245, 1);
  }
  if(Num == 5)
  {
    move_tank(1, 0, 45, 313, 1);
    delay(150);
    move_tank(1, 30, 30, 250, 1);
    delay(150);
    move_tank(1, 0, -45, 309, 1);
    delay(150);
    servo_initial(84, 3);
    delay(250);
    move_tank(1, 30, 30, 70, 1);
    servo_second(120, 10, 3);
    delay(150);
    move_tank(1, -50, -50, 70, 1);
    delay(150);
    move_tank(1, -50, 0, 230, 1);
    delay(150);
    move_tank(1, -40, -40, 90, 1);
    delay(150);
    move_tank(1, 0, -50, 235, 1);
  }
  if(Num == 4)
  {
    servo_initial(84, 3);
    move_tank(1, -50, 0, 85, 1);
    delay(150);
    move_tank(1, 40, 40, 100, 1);
    servo_second(120, 10, 3);
    delay(250);
    move_tank(1, -40, -40, 90, 1);
    move_tank(1, 50, 0, 100, 1);
    move_tank(1, -50, -50, 175, 1);
  }
  if(Num == 3)
  {
    servo_initial(86, 3);
    delay(250);
    move_tank(1, 0, 50, 30, 1);
    move_tank(1, 50, 50, 40, 1);
    servo_second(120, 10, 3);
    delay(150);
    move_tank(1, -50, -50, 40, 1);
    delay(150);
    move_tank(1, 0, -50, 40, 1);
    move_tank(1, -50, -50, 175, 1);
  }
  if(Num == 2)
  {
    move_tank(1, 45, 0, 298, 1);
    delay(150);
    move_tank(1, 30, 30, 55, 1);
    delay(150);
    move_tank(1, -45, 0, 300, 1);
    delay(150);
    servo_initial(84, 3);
    delay(250);
    move_tank(1, 30, 30, 70, 1);
    servo_second(120, 10, 3);
    move_tank(1, -30, -30, 100, 1);
    move_tank(1, 0, -50, 180, 1);
    move_tank(1, -50, 0, 180, 1);
  }
  if(Num == 1)
  {
    move_tank(1, 45, 0, 298, 1);
    delay(150);
    move_tank(1, 30, 30, 170, 1);
    delay(150);
    move_tank(1, -45, 0, 298, 1);
    delay(150);
    servo_initial(84, 3);
    delay(250);
    move_tank(1, 30, 30, 80, 1);
    servo_second(120, 10, 3);
    delay(150);
    move_tank(1, -30, -30, 80, 1);
    move_tank(1, 0, -50, 250, 1);
    move_tank(1, -30, -30, 50, 1);
    move_tank(1, -50, 0, 250, 1);
  }
}

void Open_Takeball() {
  Acc_move(30, 60, 260, 100, 100, 1);
  move_tank(1, -40, 40, 152, 1);
  MBC_degree(2.5, 10, 30, 100, 0);
  MBC_black(1.5, 5, 50, 0, 0,50);
  MBC_degree(2, 10, 30, 50, 1);
  R_drift(35, 80);
  MBC_degree(1.5, 10, 35, 190, 1);
  servo_initial(90, 2);
  delay(200);
  servo_initial(87, 1);
  move_tank(1, -35, -35, 40, 0);
  servo_initial(82, 2);
  Acc_move(-35, -50, 150, 50, 50, 0);
  move_tank(2, -50, -50, 0.3, 1);
  for(int i_1 = 0; i_1 < 2; i_1++)
  {
    servo_initial(89, 1);
    move_tank(1, 50, 50, 45, 1);
    delay(100);
    servo_initial(87, 1);
    move_tank(2, -50, -50, 0.4, 1);
    delay(100);
  }
  servo_initial(89, 1);
  move_tank(1, 50, 50, 30, 1);
  servo_initial(92, 1);
  delay(100);
  move_tank(1, -30, 30, 152, 1);
}

void TO_PUT() {
  servo_second(115, 15, 1);
  MBC_degree(2.5, 5, 35, 100, 0);
  MBC_black(2.5, 5, 50, 1, 0,50);
  move_tank(1, 30, 30, 50, 1);
  delay(125);
  L_drift(35, 80);
}
void TakeArea(float see) {
  move_tank(1, -50, 50, 151, 1);
  move_tank(1, 58, 60, 470, 1);
  BWB(30, 31, 0);
  move_tank(1, 30, 31, see, 1);
  delay(100);
  move_tank(1, 50, 0, 305, 1);
  delay(100);
  move_tank(2, -25, -25, 0.6, 0);
}

void scan(float one, float two, float three, float four, float five, float six) {
  Acc_move(20, 35, one, 100, 100, 1);
  delay(200);
  EV3_ReadColor();
  color1 = tempColor;
  move_tank(1, 25, 25, two, 1);
  delay(200);
  EV3_ReadColor();
  color2 = tempColor;
  move_tank(1, 25, 25, three, 1);
  delay(200);
  EV3_ReadColor();
  color3 = tempColor;
  move_tank(1, 25, 25, four, 1);
  delay(200);
  EV3_ReadColor();
  color4 = tempColor;
  move_tank(1, 25, 25, five, 1);
  delay(200);
  EV3_ReadColor();
  color5 = tempColor;
  move_tank(1, 25, 25, six, 1);
  delay(200);
  EV3_ReadColor();
  color6 = tempColor;
  MiniR4.OLED.clearDisplay();
  MiniR4.OLED.setTextSize(1);
  MiniR4.OLED.setCursor(10, 5);
  MiniR4.OLED.print(color1);
  MiniR4.OLED.display();
  MiniR4.OLED.setCursor(40, 5);
  MiniR4.OLED.print(color2);
  MiniR4.OLED.display();
  MiniR4.OLED.setCursor(70, 5);
  MiniR4.OLED.print(color3);
  MiniR4.OLED.display();
  MiniR4.OLED.setCursor(10, 20);
  MiniR4.OLED.print(color4);
  MiniR4.OLED.display();
  MiniR4.OLED.setCursor(40, 20);
  MiniR4.OLED.print(color5);
  MiniR4.OLED.display();
  MiniR4.OLED.setCursor(70, 20);
  MiniR4.OLED.print(color6);
  MiniR4.OLED.display();
}

void EV3_ReadColor() {
  for(int i_2 = 0; i_2 < 5; i_2++)
  {
    EV3_Color_read = MiniR4.I2C1.MXColor.getColor(C);
  }
  tempColor = 0;
  if((((EV3_Color_read == 1)||(EV3_Color_read == 2))||(EV3_Color_read == 4))||(EV3_Color_read == 7))
  {
    tempColor = 10;
    MiniR4.Buzzer.Tone(NOTE_E5, 100);
  }
  else
  {
    if(EV3_Color_read == 3)
    {
      tempColor = 3;
      MiniR4.LED.setColor(1, 0, 255, 0);
      MiniR4.LED.setColor(2, 0, 255, 0);
    }
    else
    {
      if(EV3_Color_read == 5)
      {
        tempColor = 5;
        MiniR4.LED.setColor(1, 255, 0, 0);
        MiniR4.LED.setColor(2, 255, 0, 0);
      }
      else
      {
        tempColor = 0;
        MiniR4.LED.setColor(1, 0, 0, 0);
        MiniR4.LED.setColor(2, 0, 0, 0);
      }
    }
  }
  if(tempColor == 10)
  {
    for(int i_3 = 0; i_3 < 5; i_3++)
    {
      S_value = MiniR4.I2C1.MXColor.getColor(Y);
    }
    if(S_value > 170)
    {
      tempColor = 4;
      MiniR4.LED.setColor(1, 255, 255, 0);
      MiniR4.LED.setColor(2, 255, 255, 0);
    }
    else
    {
      tempColor = 6;
      MiniR4.LED.setColor(1, 255, 255, 255);
      MiniR4.LED.setColor(2, 255, 255, 255);
    }
  }
}

void fastscan() {
  for(int i_4 = 0; i_4 < 1; i_4++)
  {
    EV3_Color_read = MiniR4.I2C1.MXColor.getColor(C);
  }
  move_initial(1);
  menu = 0;
  while(!(((abs(MiniR4.M2.getCounter()) + abs(MiniR4.M3.getCounter())) / 2) > 350))
  {
    EV3_Color_read = MiniR4.I2C1.MXColor.getColor(C);
    if(EV3_Color_read > 0)
    {
      if((((abs(MiniR4.M2.getCounter()) + abs(MiniR4.M3.getCounter())) / 2) > 0) && (((abs(MiniR4.M2.getCounter()) + abs(MiniR4.M3.getCounter())) / 2) < 100))
      {
        EV3_ReadColor();
        color1 = tempColor;
        while(!(EV3_Color_read == 0))
        {
          EV3_Color_read = MiniR4.I2C1.MXColor.getColor(C);
          move_PD_on(25);
        }
      }
      else
      {
        if((((abs(MiniR4.M2.getCounter()) + abs(MiniR4.M3.getCounter())) / 2) > 100) && (((abs(MiniR4.M2.getCounter()) + abs(MiniR4.M3.getCounter())) / 2) < 200))
        {
          EV3_ReadColor();
          color2 = tempColor;
        }
        else
        {
          if((((abs(MiniR4.M2.getCounter()) + abs(MiniR4.M3.getCounter())) / 2) > 200) && (((abs(MiniR4.M2.getCounter()) + abs(MiniR4.M3.getCounter())) / 2) < 350))
          {
            EV3_ReadColor();
            color3 = tempColor;
          }
          else
          {
            while(!(EV3_Color_read == 0))
            {
              EV3_Color_read = MiniR4.I2C1.MXColor.getColor(C);
              move_PD_on(25);
            }
          }
        }
      }
    }
    else
    {
      move_PD_on(25);
    }
  }
  brake_hold(1);
  MiniR4.OLED.clearDisplay();
  MiniR4.OLED.setTextSize(1);
  MiniR4.OLED.setCursor(10, 5);
  MiniR4.OLED.print(color1);
  MiniR4.OLED.display();
  MiniR4.OLED.setCursor(40, 5);
  MiniR4.OLED.print(color2);
  MiniR4.OLED.display();
  MiniR4.OLED.setCursor(70, 5);
  MiniR4.OLED.print(color3);
  MiniR4.OLED.display();
  MiniR4.OLED.setCursor(10, 20);
  MiniR4.OLED.print(color4);
  MiniR4.OLED.display();
  MiniR4.OLED.setCursor(40, 20);
  MiniR4.OLED.print(color5);
  MiniR4.OLED.display();
  MiniR4.OLED.setCursor(70, 20);
  MiniR4.OLED.print(color6);
  MiniR4.OLED.display();
}

void putball() {
  servo_initial(80, 2);
  MBC_degree(2.5, 10, 35, 70, 1);
  delay(1);
  servo_initial(95, 2);
  delay(100);
  MBC_degree(2.5, 10, 40, 35, 1);
  servo_initial(115, 2);
  move_tank(1, 50, 50, 45, 1);
  servo_initial(110, 2);
  servo_initial(165, 1);
  delay(900);
  move_tank(1, -70, 0, 306, 0);
  move_tank(2, -50, -50, 0.3, 1);
}

void END() {
  move_tank(1, -50, 0, 60, 1);
  move_tank(1, 0, -50, 70, 1);
  PD_for_all(3, 3, .5, -65, 3000, 1);
  move_tank(1, 50, 50, 120, 1);
  move_tank(1, 50, -50, 155, 1);
  move_initial(1);
  while(!(MiniR4.I2C4.MXColor.getColor(G) > 2))
  {
    MiniR4.M2.setPower(40);
    MiniR4.M3.setPower(40);
  }
  move_tank_off(1);
  move_tank(1, -50, -50, 40, 1);
  while(true)
  {
  }
}

void big_blue() {
  move_tank(1, 50, 50, 110, 0);
  move_tank(1, 50, 15, 280, 1);
  Acc_move(30, 90, 800, 150, 300, 1);
  delay(200);
  move_tank(1, 40, 0, 175, 1);
  delay(200);
  move_tank(1, -40, 0, 175, 1);
  delay(200);
  Acc_move(-30, -90, 1250, 200, 350, 1);
  delay(200);
  move_tank(1, 50, -50, 143, 1);
  delay(200);
  move_tank(2, -50, -50, 0.5, 1);
  MiniR4.RC2.setAngle(135);
  MiniR4.RC3.setAngle(135);
  Acc_move(30, 70, 525, 100, 250, 1);
  delay(200);
  L_drift(35, 80);
}

void Acc_move(float Vs, float Vf, float total, float plus_degree, float sub_degree, float brake) {
  if(Vs < 0)
  {
    move_initial(0);
  }
  else
  {
    move_initial(1);
  }
  if(plus_degree > 0)
  {
    while(!(((abs(MiniR4.M2.getCounter()) + abs(MiniR4.M3.getCounter())) / 2) > plus_degree))
    {
      move_PD_on(abs(Vs) + ((abs(Vf) - abs(Vs)) * (1 - cos(((3.1415926 * ((abs(MiniR4.M2.getCounter()) + abs(MiniR4.M3.getCounter())) / 2)) / (2 * plus_degree))))));
    }
  }
  while(!(((abs(MiniR4.M2.getCounter()) + abs(MiniR4.M3.getCounter())) / 2) > (total - sub_degree)))
  {
    move_PD_on(abs(Vf));
  }
  if(sub_degree > 0)
  {
    while(!(((abs(MiniR4.M2.getCounter()) + abs(MiniR4.M3.getCounter())) / 2) > total))
    {
      move_PD_on(abs(Vf) + ((abs(Vf) - abs(Vs)) * (cos(((3.1415926 * (((abs(MiniR4.M2.getCounter()) + abs(MiniR4.M3.getCounter())) / 2) - (total - sub_degree))) / (2 * sub_degree))) - 1)));
    }
  }
  brake_hold(brake);
}

void MBC_black(float kp, float kd, float max_power, float _012, float brake, float decel_before) {
  move_initial(1);
  MiniR4.M2.resetCounter();
  MiniR4.M3.resetCounter();

  float acc_deg = 150;       // S-curve 起步加速距離
  float min_power = 20;      // 起步最小功率
  float error, error_new, error_old = 0;
  float speed, base_power;
  bool is_done = false;

  while (!is_done) {
    light_8();

    // === 結束條件判斷 ===
    if (_012 == 0) {
      is_done = (light1 < 7) && (light8 < 7);
    } else if (_012 == 1) {
      is_done = (light1 < 5 && light2 < 5 && light3 < 5 && light4 < 5);
    } else if (_012 == 2) {
      is_done = (light5 < 5 && light6 < 5 && light7 < 5 && light8 < 5);
    } else {
      break;
    }

    if (is_done) break;

    // === 當前距離 ===
    float cur_deg = (abs(MiniR4.M2.getCounter()) + abs(MiniR4.M3.getCounter())) / 2.0;

    // === 功率區段決定（S-curve加速 + 減速） ===
    if (cur_deg < acc_deg) {
      // 加速區
      float ratio = 1 - cos((PI * cur_deg) / (2 * acc_deg));
      base_power = min_power + (max_power - min_power) * ratio;
    } else if (decel_before > 0 && cur_deg > (decel_before)) {
      // 減速區（靠近終點，開始降速）
      float d = cur_deg - decel_before;
      float ratio = cos((PI * d) / (2 * (200 - decel_before)));  // 200是假設總行程上限，可調整
      base_power = max_power * ratio;
      if (base_power < min_power) base_power = min_power;
    } else {
      // 等速區
      base_power = max_power;
    }

    // === PID 控制 ===
    error_new = light_Y - light_X;
    error = error_new - error_old;
    speed = (kp * error_new) + (kd * error);
    MiniR4.M2.setPower(base_power - speed);
    MiniR4.M3.setPower(base_power + speed);
    error_old = error_new;
  }

  brake_hold(brake);
}
void MBC_degree(float kp, float kd, float max_power, float deg, float brake) {
  move_initial(1);
  MiniR4.M2.resetCounter();
  MiniR4.M3.resetCounter();

  float acc_ratio = 0.2;
  float dec_ratio = 0.2;
  float acc_deg = deg * acc_ratio;
  float dec_deg = deg * dec_ratio;
  float mid_deg = deg - acc_deg - dec_deg;

  float min_power = 20;  // 起步最低功率

  float error, error_new, error_old = 0;
  float speed, base_power;

  while (true) {
    light_8();
    float cur_deg = (abs(MiniR4.M2.getDegrees()) + abs(MiniR4.M3.getDegrees())) / 2.0;

    // 結束條件
    if (cur_deg > deg) break;

    // === S-curve 區段判斷 ===
    if (cur_deg < acc_deg) {
      // 加速區
      float ratio = 1 - cos((PI * cur_deg) / (2 * acc_deg));
      base_power = min_power + (max_power - min_power) * ratio;
    } else if (cur_deg < acc_deg + mid_deg) {
      // 等速區
      base_power = max_power;
    } else {
      // 減速區
      float d = cur_deg - (acc_deg + mid_deg);
      float ratio = cos((PI * d) / (2 * dec_deg));
      base_power = max_power * ratio;
    }

    // PID 控制
    error_new = light_Y - light_X;
    error = error_new - error_old;
    speed = (kp * error_new) + (kd * error);
    MiniR4.M2.setPower(base_power - speed);
    MiniR4.M3.setPower(base_power + speed);
    error_old = error_new;
  }

  brake_hold(brake);
}

void setup()
{
  MiniR4.begin();
  MiniR4.PWR.setBattCell(2);
  Serial.begin(9600);
  servo_initial(105, 1);//165
  delay(300);
  servo_initial(0, 2);//150
  servo_initial(70, 3);//150
  MiniR4.OLED.setTextSize(3);
  // 記得隨時存檔
  // 記得隨時存檔
  // 記得隨時存檔
  // 記得隨時存檔
  // 記得隨時存檔
  // 記得隨時存檔
  MiniR4.OLED.setCursor(10, 10);
  MiniR4.OLED.print(MiniR4.PWR.getBattVoltage());
  MiniR4.OLED.display();
  MiniR4.Motion.resetIMUValues();
  MiniR4.I2C1.MXColor.begin();
  MiniR4.I2C4.MXColor.begin();
  for(int i_0 = 0; i_0 < 3; i_0++)
  {
    MiniR4.Buzzer.Tone(NOTE_C4, 100);
    delay(200);
  }
  MiniR4.M2.resetCounter();
}
void move_s_curve(float Vmax, float total) {
  float acc_ratio, dec_ratio;

  // 智慧選擇加減速比例
  if (total < 150) {
    acc_ratio = 0.3;
    dec_ratio = 0.3;
  } else if (total < 400) {
    acc_ratio = 0.3;
    dec_ratio = 0.3;
  } else {
    acc_ratio = 0.15;
    dec_ratio = 0.35;
  }

  float acc_dist = total * acc_ratio;
  float dec_dist = total * dec_ratio;
  float mid_dist = total - acc_dist - dec_dist;
  float min_power = 25; // 起步最小輸出（只用於加速段）

  // 判斷方向
  if (Vmax < 0) {
    move_initial(0);
  } else {
    move_initial(1);
  }

  // 加速區
  while ((((abs(MiniR4.M2.getCounter()) + abs(MiniR4.M3.getCounter())) / 2) < acc_dist)) {
    float d = (abs(MiniR4.M2.getCounter()) + abs(MiniR4.M3.getCounter())) / 2;
    float speed = abs(Vmax) * (1 - cos((PI * d) / (2 * acc_dist)));
    if (speed < min_power) speed = min_power;
    move_PD_on(speed);
  }

  // 等速區
  while ((((abs(MiniR4.M2.getCounter()) + abs(MiniR4.M3.getCounter())) / 2) < (acc_dist + mid_dist))) {
    move_PD_on(abs(Vmax));
  }

  // 減速區（不設最小限）
  while ((((abs(MiniR4.M2.getCounter()) + abs(MiniR4.M3.getCounter())) / 2) < total)) {
    float d = (abs(MiniR4.M2.getCounter()) + abs(MiniR4.M3.getCounter())) / 2 - (acc_dist + mid_dist);
    float speed = abs(Vmax) * (cos((PI * d) / (2 * dec_dist)));
    move_PD_on(speed);
  }

  brake_hold(1); // 剎車
}

//--------------------------------------------------------------------------------------------------------------------
//MBC_degree(3, 3, 50, 700, 1);  P值 D值  power 角度 煞車
//MBC_black(3, 3, 25, 1, 1,600); 固定會跑150度 ( P值 D值 power 煞車 要跑多少角度看黑線)
//void move_s_curve(50,1000);  s-curve 輸入最高速度,角度

void loop()
{
  while(!MiniR4.BTN_DOWN.getState());
  MiniR4.Buzzer.Tone(262, 100);
  Open_Takeball();
  TO_PUT();
  putball();
  TakeArea(125);
  move_tank(2, -30, -30, 0.3, 1);
  delay(100);
  scan(184, 104, 104, 100, 100, 100);
  big_blue();
  MBC_black(3, 3, 25, 1, 1,150);
  delay(100);
  L_take(4);
  MBC_black(3, 3, 25, 1, 1,150);
  delay(100);
  R_take(5);
  Area_Down();
  MBC_black(3, 3, 25, 1, 1,150);
  delay(100);
  L_take(6);
  MBC_black(3, 3, 25, 1, 1,150);
  delay(100);
  R_take(3);
  AreaUP();
  MBC_black(3, 3, 25, 1, 1,150);
  MiniR4.Motion.resetIMUValues();
  END();

}