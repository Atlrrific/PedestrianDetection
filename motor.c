#include <stdio.h>
#include <stdlib.h>
#include <wiringPi.h>

//#define servoPin 1  //Pin 1 Supports PWM
//#define position 1500

int main(void){
  if(wiringPiSetup ()== -1){
  printf("setup wiring PI failed!");
  return 1;
  }
int servoPin = 4;
int position = 1000;
//double position = 0;
int counter = 0;

pinMode(servoPin, OUTPUT);

//while(1){
/*printf("Please enter the pulse width (in terms of time)\n");
printf("The units entered will be in terms of miliseconds\n");
printf("Enter in the range of 500 to 2200.\n");
vxcvzxcvvprintf("500 is angle 0, 2100 is about 180.\n");
printf("Input: ");
//scanf("%lf",&position);
printf("\n");*/
//while(getchar() != '\n'){;}
    digitalWrite(servoPin,LOW);
  while(counter<200){
    printf("%u\n",counter);
    digitalWrite(servoPin, HIGH);
    delayMicroseconds(position); // NOTE: TIME IS IN MICROSECONDS
    // digitalWrite(servoPin,  LOW);
    //
    // delayMicroseconds(5000-position);
    // delay(15);
    //
    // counter++;
    //
    // if (counter == 100){break;}
    counter+=1;
  }

//  counter = 0;
//}
  digitalWrite(servoPin, LOW);
  return 1;

}
