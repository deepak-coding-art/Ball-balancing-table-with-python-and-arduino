#include <Servo.h>

int x_val;
int y_val;
int val_arr[2];

Servo servo_x;
Servo servo_y;

void setup() {
  Serial.begin(115200);
  servo_x.attach(8);
  servo_y.attach(9);
}

void loop() {
  receve_data();
  drive_servo('x', map(val_arr[0], 0, 127, 0, 180));
  drive_servo('y', map(val_arr[1], 0, 127, 0, 180));
}

void drive_servo(char sevo, int value){
  if(sevo == 'x'){servo_x.write(180 - value);}
  else if(sevo == 'y'){servo_y.write(180 - value);}
}

void receve_data(){
  int i = 0;
  delay(10);
  Serial.print('k');
  while(Serial.available() > 0){
    if(i < 2)
    {
      char data = Serial.read();
      val_arr[i] = int(data);
      i++;
    }
    else{
      break;
    }
  }
}
