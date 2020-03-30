#include <MPU6050_tockn.h>
#include <Wire.h>
#include "pinConfg.h"
#include "confg.h"

MPU6050 mpu6050(Wire);


int val = 900;
float smooted1;
void setup() {
  DDRD |= B1111000;
  DDRB |= B001110;
  
  Serial.begin(115200);
  Wire.begin();
  mpu6050.begin();
  mpu6050.calcGyroOffsets(true);

}

void loop() {
  
  if (Serial.available()) {
    val = Serial.parseInt();
    Serial.read();
  }

  read_gyro();
  pid_set();
  motor();
  esc_pwm();
  //Serial.print("m1 : ");Serial.println(m1);
  
}

void read_gyro() {
  mpu6050.update();
  imu_acc_x = mpu6050.getAccX();
  imu_acc_y = mpu6050.getAccY();

  //smoothed_angle += 0.2 * pow(abs((degrees - smoothed_angle)), 2.0 / 3.0) * (degrees - smoothed_angle) / abs(degrees - smoothed_angle)
  //imu_acc_z = mpu6050.getAccZ();
//  Serial.println(imu_acc_y);
}

void pid_set() {
  //Roll Pid Hesaplama
  pid_error_temp = imu_acc_x - setpoint_roll;
  pid_i_roll_toplamhata += ki_roll * pid_error_temp;
  if (pid_i_roll_toplamhata > ki_max_roll)pid_i_roll_toplamhata = ki_max_roll;
  else if (pid_i_roll_toplamhata < ki_max_roll * -1)pid_i_roll_toplamhata = ki_max_roll * -1;

  pid_output_roll = kp_roll * pid_error_temp + pid_i_roll_toplamhata + kd_roll * (pid_error_temp - pid_sonhata_roll);
  if (pid_output_roll > ki_max_roll)pid_output_roll = ki_max_roll;
  else if (pid_output_roll < ki_max_roll * -1)pid_output_roll = ki_max_roll * -1;

  pid_sonhata_roll = pid_error_temp;

  //Pitch Pid Hesaplama
  pid_error_temp = imu_acc_y - setpoint_pitch;
  pid_i_pitch_toplamhata += ki_pitch * pid_error_temp;
  if (pid_i_pitch_toplamhata > ki_max_pitch)pid_i_pitch_toplamhata = ki_max_pitch;
  else if (pid_i_pitch_toplamhata < ki_max_pitch * -1)pid_i_pitch_toplamhata = ki_max_pitch * -1;

  pid_output_pitch = kp_pitch * pid_error_temp + pid_i_pitch_toplamhata + kd_pitch * (pid_error_temp - pid_sonhata_pitch);
  if (pid_output_pitch > ki_max_pitch)pid_output_pitch = ki_max_pitch;
  else if (pid_output_pitch < ki_max_pitch * -1)pid_output_pitch = ki_max_pitch * -1;

  pid_sonhata_pitch = pid_error_temp;

}
void motor() {
  m1 = val - pid_output_pitch + pid_output_roll; //Calculate the pulse for esc 1 (front-right - CCW)
  m2 = val + pid_output_pitch + pid_output_roll; //Calculate the pulse for esc 2 (rear-right - CW)
  m3 = val + pid_output_pitch - pid_output_roll; //Calculate the pulse for esc 3 (rear-left - CCW)
  m4 = val - pid_output_pitch - pid_output_roll; //Calculate the pulse for esc 4 (front-left - CW)


  if (m1 < 1000) m1 = 1000;                                         //Keep the motors running.
  if (m2 < 1000) m2 = 1000;                                         //Keep the motors running.
  if (m3 < 1000) m3 = 1000;                                         //Keep the motors running.
  if (m4 < 1000) m4 = 1000;                                         //Keep the motors running.

  if (m1 > 2000)m1 = 2000;                                          //Limit the esc-1 pulse to 2000us.
  if (m2 > 2000)m2 = 2000;                                          //Limit the esc-2 pulse to 2000us.
  if (m3 > 2000)m3 = 2000;                                          //Limit the esc-3 pulse to 2000us.
  if (m4 > 2000) m4 = 2000;                                          //Limit the esc-4 pulse to 2000us.

//  Serial.print(m1);Serial.print("|");
//  Serial.print(m2);Serial.print("|");
//  Serial.print(m3);Serial.print("|");
//  Serial.print(m4);Serial.println("|");

}
void test_esc_pwm()
{
  while (micros() - loop_timer < 4000);
  loop_timer = micros();

  PORTD |= B01000000;
  PORTB |= B001110;
  timer_channel_1 = val + loop_timer;                                     //Calculate the time of the faling edge of the esc-1 pulse.
  timer_channel_2 = val + loop_timer;
  timer_channel_3 = val + loop_timer;
  timer_channel_4 = val + loop_timer;

  while (PORTD >= 16 || PORTB > 1) {                                                     //4 Pinde low seviyesine gelmesini bekliyoruz.
    esc_loop_timer = micros();
    if (timer_channel_1 <= esc_loop_timer)PORTD &= B10111111;               //Set digital output 4 to low if the time is expired.
    if (timer_channel_2 <= esc_loop_timer)PORTB &= B111101;               //Set digital output 5 to low if the time is expired.
    if (timer_channel_3 <= esc_loop_timer)PORTB &= B111011;               //Set digital output 6 to low if the time is expired.
    if (timer_channel_4 <= esc_loop_timer)PORTB &= B110111;

  }
}
void esc_pwm() {

  while (micros() - loop_timer < 4000);
  loop_timer = micros();

  PORTD |= B01000000;
  PORTB |= B001110;
  timer_channel_1 = m1 + loop_timer;
  timer_channel_2 = m2 + loop_timer;
  timer_channel_3 = m3 + loop_timer;
  timer_channel_4 = m4 + loop_timer;


  while (PORTD >= 16 || PORTB > 1) {
    esc_loop_timer = micros();
    if (timer_channel_1 <= esc_loop_timer)PORTD &= B10111111;
    if (timer_channel_2 <= esc_loop_timer)PORTB &= B111101;
    if (timer_channel_3 <= esc_loop_timer)PORTB &= B111011;
    if (timer_channel_4 <= esc_loop_timer)PORTB &= B110111;
  }
}
