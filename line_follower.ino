//Motor left
#define ENA 9
#define IN1 A0
#define IN2 A1

//Motor right
#define IN3 A3
#define IN4 A2
#define ENB 6

#define rightSensor 2
#define centerRightSensor 3
#define centerSensor 12
#define centerLeftSensor 4
#define leftSensor 5
#define centerSensorVcc 13

short Speed=70;
float shortFactor=0.8;


struct sensor
{
  bool S1,S2,S3,S4,S5;

  void sensorInput()
  {
    S1 = digitalRead(rightSensor);
    S2 = digitalRead(centerRightSensor);
    S3 = digitalRead(centerSensor);
    S4 = digitalRead(centerLeftSensor);
    S5 = digitalRead(leftSensor);

  }

};
struct motion
{
  bool forward=false;
  bool left=false;
  bool right=false;
  bool back;

  void setMotion(sensor s)
  {
    bool S1 = s.S1;
    bool S2 = s.S2;
    bool S3 = s.S3;
    bool S4 = s.S4;
    bool S5 = s.S5;
    
    if (S3 == 1 && S5 == 0 && S1 == 0 ) 
    {
      forward=true;
      left=false;
      right=false;
      back=false;
      moveForward();
    }

    else if (S1 == 1 && S2 == 1 && S3 == 1 && S4 == 1 && S5 == 1) 
    {
      {
      forward=false;
      left=false;
      right=false;
      back=true;
      stop();
    }
    }
    else if (S1 == 0 && S2 == 0 && S3 == 0 && S4 == 0 && S5 == 0) 
    {
      forward=false;
      left=false;
      right=false;
      back=false;
      stop();
    }

    else if (S2 == 0 && S4 == 1 && S1 == 0 && S5  == 0) 
    {
      forward=false;
      left=true;
      right=false;
      back=false;
      shortLeft();      
    }

    else if (S1 == 0 && S5 == 1) 
    {
      forward=false;
      left=true;
      right=false;
      back=false;
      turnleft();
    }
    else if (S2 == 1 && S4 == 0 && S1 == 0 && S5 == 0) 
    {
      forward=false;
      left=false;
      right=true;
      back=false;
      shortRight();
    }

    else if (S1 == 1 && S5 == 0) 
    {
      forward=false;
      left=false;
      right=true;
      back=false;
      turnright();
    }
  

  }
  void moveForward() {
  analogWrite(ENA, Speed);
  analogWrite(ENB, Speed);
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
}
void turnright() {
  analogWrite(ENA, Speed);
  analogWrite(ENB, Speed);
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
}
void turnleft() {
  analogWrite(ENA, Speed);
  analogWrite(ENB, Speed);
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
}
void shortLeft() {
  analogWrite(ENA, Speed*shortFactor);
  analogWrite(ENB, Speed*shortFactor);
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
}

void shortRight() {
  analogWrite(ENA, Speed*shortFactor);
  analogWrite(ENB, Speed*shortFactor);
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
}
void stop() {
  analogWrite(ENA, 0);
  analogWrite(ENB, 0);
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
}

void turn180()
{
  turnright();
}



  
};

  

void setup() {
  //Sensor pins
  digitalWrite(centerSensorVcc,HIGH);


  pinMode(centerSensorVcc,OUTPUT);  
  pinMode(rightSensor, INPUT);
  pinMode(centerRightSensor, INPUT);
  pinMode(centerLeftSensor, INPUT);
  pinMode(leftSensor, INPUT);
  //Motor one
  pinMode(ENA, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  //Motor two
  pinMode(ENB, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);

}

void loop() {
  motion move;
  sensor sense;
  sense.sensorInput();
  move.setMotion(sense);
  
}



// void rightWall(motion data)
// {
//   if(data.forward)
//     data.moveForward();
  
//   if(data.right)
//   {
//     while(data.right)
//     {
//       data.turnright();
//       data.setMotion();
//     }

//   }
//   if(data.back)
//   {
//     while(!data.)
//     turn180();

//   }

  

// }
// void directionTest()
// {

// }