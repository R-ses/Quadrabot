#include <SoftwareSerial.h>
#define MAX_INST 32

typedef struct InstructionBuffer{
  int8_t inst[MAX_INST];
  uint8_t IN;
  uint8_t OUT;
};

InstructionBuffer IB;

SoftwareSerial bt_serial(8,9);

const int trigPin = 6;
const int echoPin = 7;
bool f_pos = false;

long duration;
int distance;
uint16_t distance_trigger = 0;
uint8_t MOV_FLAG = 0; 
bool SENSOR_FLAG = false;


void setup() {
  
  pinMode(trigPin,OUTPUT);
  pinMode(echoPin,INPUT);
  Serial.begin(115200);

  DDRB |= (1<<PB2);
  PORTB &= ~(1<<PB2);
  bt_serial.begin(9600);
  bt_serial.print("BT Communication: Started");
  Stand_up_L();
      
}


void loop(){

   
   if(bt_serial.available())
   {
    Serial.print(bt_serial.read());
      if(IB.IN == IB.OUT-1)
      {
        bt_serial.print("Buffer lleno");
      }
      else
      {
        IB.inst[IB.IN] = bt_serial.read();
        IB.IN++;
        IB.IN &= ~MAX_INST; //IB.IN = IB.IN%MAX_INST;
      }
    
    }
    
    if(IB.IN != IB.OUT)
    {
      char Inst = IB.inst[IB.OUT];
      IB.OUT++;
      IB.OUT &= ~MAX_INST; // IB.OUT = IB.OUT%MAX_INST; // 
      switch (Inst)
      {
        
          case 'w':
          forward();
          break;

          case 'q':
          slide_left();
          break;

          case 'e':
          slide_right();
          break;

          case 's':
          backwards();
          break;

          case 'a':
          turn_left();
          break;

          case 'd':
          turn_right();
          break;

          case 'u':
          Stand_up_H();
          break;

          case 'c':
          Stand_up_L();
          break;

          case 'z':
          Toggle_pos();
          break;
          
          case 'f':
          SENSOR_FLAG = !SENSOR_FLAG;
          if(PINB & 1<<PB2)
          {
            PORTB &= ~(1<<PB2);            
          }
          else
          {
            PORTB |= (1<<PB2);  
          }
          break;
          
      }
    }
  
  
  if(SENSOR_FLAG)
  {
    PORTB |= (1<<PB2);
    if(distance_trigger >= 500)
    {
      
      distance_trigger -= 500;
      digitalWrite(trigPin, HIGH);
      delayMicroseconds(10);
      digitalWrite(trigPin, LOW);
      duration = pulseIn(echoPin,HIGH);
      distance = duration*0.034/2;
  
      if(distance < 15)
      {
        
        bt_serial.print("Muy cerca \n");  
        
      }
  
      if(distance < 10)
      { 
        backwards();  
        bt_serial.print("retrocediendo \n");
      }
          
      bt_serial.print("distance: ");
      bt_serial.print(distance);
      bt_serial.print("\n");
    
    } 
    distance_trigger++;
    delay(1);
  }
  
  
}

void slide_left()
{
  
}


void slide_right()
{
  
}



void turn_left()
{

   int serv[] = {0,15,16,31};
   int serv2[] = {1,14,17,30};


   multi_servo(880, serv,500);
   multi_servo(2400, serv2,500);
   multi_servo(2260, serv,500);
   multi_servo(2000, serv2,500);
   multi_servo(1500,serv,500);


}

//---------------------------------------------------------------------

void turn_right()
{
   int serv[] = {0,15,16,31};
   int serv2[] = {1,14,17,30};

   multi_servo(2260, serv,500);
   multi_servo(2400, serv2,500);
   multi_servo(880, serv,500);
   multi_servo(2000, serv2,500);
   multi_servo(1500,serv,500);
   
   //Serial.print("#0 P2260 #15 P2260 #16 P2260 #31 P2260 T1000");
 
}

void multi_servo(int pos, int serv[], int del )
{
  int sizearr = (sizeof(serv)/sizeof(serv[0]));
    for (int i=0;i<4;i++)
    {  
     Serial.print("#");
     Serial.print(serv[i]);
     Serial.print(" P");
     Serial.print(pos);
     Serial.print(" ");
 
    }
    
     Serial.print("T");
     Serial.println(del); 

  hold(1000);
}

void move_servo(int servo,int pos,int time)
{
  
  Serial.print("#");
  Serial.print(servo);
  Serial.print(" P");
  Serial.print(pos);
  Serial.print(" T");
  Serial.println(time);
  hold(time);
  
}


void hold(uint8_t time)
{
  
  uint8_t t_hold = 0;
  
  while(t_hold < time) //second_loop
  {
    delay(1);
    second_loop();
    t_hold++;
  }

}

void second_loop()
{
  
  if(bt_serial.available())
  {
    IB.inst[IB.IN] = bt_serial.read();
    IB.IN++;
    IB.IN = IB.IN%MAX_INST; 
  }
  
}

void Stand_up_H()
{


  /*int serv[] = {0,1,2,13,14,15,16,17,18,29,30,31};
  int pos[] = {1500,1450,500,500,1450,1500,1500,1450,500,500,1450,1500};
  int dly = 500;
  for(uint8_t i = 0;i<12;i++)
  {
    move_servo(serv[i],pos[i],dly);
  }
  */

  Serial.print("#0 P1500 #1 P1450 #2 P500 #13 P500 #14 P1450 #15 P1500 #16 P1500 #17 P1450 #18 P500 #29 P500 #30 P1450 #31 P1500");
  Serial.print(" T");
  Serial.println("1000");
  hold(1000);
  
}


void Stand_up_L()
{
  
 /* int serv[] = {0,1,2,13,14,15,16,17,18,29,30,31};
  int pos[] = {1500,2000,900,900,2000,1500,1500,2000,900,900,2000,1500};
  int dly = 500;
  for(uint8_t i = 0;i<12;i++)
  {
    move_servo(serv[i],pos[i],dly);
  }

  //Serial.println("#0 P1500 #1 P2000 #2 P900 #13 P900 #14 P2000 #15 P1500 ")
*/
  Serial.print("#0 P1500 #1 P2000 #2 P900 #13 P900 #14 P2000 #15 P1500 #16 P1500 #17 P2000 #18 P900 #29 P900 #30 P2000 #31 P1500 T");
  Serial.println("1000");
  hold(1000);

  
}

void forward()
{
  Stand_up_L();
  
  int serv[] = {30,31,30,17,16,17};
  int pos[] = {2500,1900,2000,2500,1900,2000};
  int dly = 250;
  for(uint8_t i=0;i<6;i++)
  {
     move_servo(serv[i],pos[i],dly);
  }  

  Serial.println("#0 P1900 #15 P1900 #16 P1500 #31 P1500 T500");
  hold(1000);

  int serv2[] = {14,15,14,1,0,1};
  int pos2[] = {2500,1100,2000,2500,1100,2000};
  
    for(uint8_t i=0;i<6;i++)
  {
     move_servo(serv2[i],pos2[i],dly);
  }  

  Serial.println("#0 P1500 #15 P1500 #16 P1100 #31 P1100 T500");
  hold(1000);

  Stand_up_L();

}

void backwards()
{

    Stand_up_L();
  
  int serv[] = {1,0,1,14,15,14};//{30,31,30,17,16,17};
  int pos[] = {2500,1900,2000,2500,1900,2000};
  int dly = 1000;
  for(uint8_t i=0;i<6;i++)
  {
     move_servo(serv[i],pos[i],dly);
  }  

  Serial.println("#0 P1500 #15 P1500 #16 P1900 #31 P1900 T500");
  hold(500);

  int serv2[] = {17,16,17,30,31,30};//{14,15,14,1,0,1};
  int pos2[] = {2500,1100,2000,2500,1100,2000};
  
    for(uint8_t i=0;i<6;i++)
  {
     move_servo(serv2[i],pos2[i],dly);
  }  

  Serial.println("#0 P1100 #15 P1100 #16 P1500 #31 P1500 T500");
  hold(500);

  Stand_up_L();

  
  
}

void Toggle_pos()
{
  f_pos = !f_pos;

  if(f_pos)
  {
      Stand_up_H();
  }
  else
  {
      Stand_up_L();
  } 
  
}
