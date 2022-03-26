#define div_theta 48
/*main関数
引数は目標角、現在位置の入った配列のアドレス
必要物
#include‥<math.h>,<Wire.h>,<DFRobot_QMC5883.h>,<Servo.h>
setup‥serial.begin(9600)
関数‥SD_read_map,max,qmc5883_2,ultrasonic(),exporting,possibility_theta
,setup_qmc5883(),mod_theta*/
#include <MadgwickAHRS.h>
#include <Wire.h>
#include <DFRobot_QMC5883.h>
#include <Servo.h>
Servo myservo;
const int motor11 = 2;
const int motor12 = 3;
const int motor21 = 6;
const int motor22 = 5;
const int pinA = 7;
const int pinB = 8;
const int SV_PIN = 9;
const int PWMa = A4;
const int PWMb = A5;
int minX = 0;
int maxX = 0;
int minY = 0;
int maxY = 0;
int offX = 0;
int offY = 0;
int i = 0;

void SD_read_coner(short int x[12],short int y[12],char map_r[11]){
}

DFRobot_QMC5883 compass;
void exporting(){
}

void setup_servo() {
  myservo.attach(SV_PIN, 500, 2400);  // サーボの割当(パルス幅500~2400msに指定)
}

void setup_qmc5883() {
  compass.begin();
  if (compass.isQMC()) {
    compass.setRange(QMC5883_RANGE_2GA);
    compass.setMeasurementMode(QMC5883_CONTINOUS);
    compass.setDataRate(QMC5883_DATARATE_50HZ);
    compass.setSamples(QMC5883_SAMPLES_8);
  }
}

int qmc5883() {
  double X = 0;
  double Y = 0;
  for (int i = 0; i < 5; i++) {
    Vector mag = compass.readRaw();
    X += mag.XAxis * 0.000118522 + 0.232580541;
    Y += mag.YAxis * 0.000115348 + 0.211307431;
  }
  int theta;
  theta =  atan2 ( Y, X ) * 57.2958;
  return theta;
}

void turn_left(){
  analogWrite( PWMb, 249 );
  analogWrite( PWMa, 254 );
  digitalWrite(motor11, LOW);
  digitalWrite(motor22, HIGH);
  digitalWrite(motor12, HIGH);
  digitalWrite(motor21, LOW);
}
void turn_right(){
  analogWrite( PWMb, 249 );
  analogWrite( PWMa, 254 );
  digitalWrite(motor11, HIGH);
  digitalWrite(motor22, LOW);
  digitalWrite(motor12, LOW);
  digitalWrite(motor21, HIGH);
}
void stop_turn(){
  analogWrite( PWMb, 249 );
  analogWrite( PWMa, 254 );
  digitalWrite(motor11, LOW);
  digitalWrite(motor22, LOW);
  digitalWrite(motor12, LOW);
  digitalWrite(motor21, LOW);
}

void Turn_with_MOD_LOC(short int theta,short int loc[2]){
    short int theta_now;
    short int delta_theta;
    char D1;
    char D2;
    char D;
    short int theta_C;
    short int theta_firt;
    theta_now=qmc5883();
    theta_firt=theta_now;
    delta_theta=mod_theta(theta-theta_now);
    short int delta_theta_first;
    D1=ultrasonic();
    theta_C=0;
    if(delta_theta>0){
        turn_left();
        while(delta_theta>=1){
            theta_now=qmc5883();
            delta_theta=mod_theta(theta-theta_now);
            if(delta_theta_first-delta_theta-7.5*theta_C>=8){
                D2=D1;
                D1=ultrasonic();
                D=max(D1,D2);
                if(D<50){
                    MOD_LOC_LIN(mod_theta(theta_now-94),loc,D);
                }
                theta_C=theta_C+1;
            }           
        }
    }
    if(delta_theta>0){
        turn_right();
        while(delta_theta<=-1){
            theta_now=qmc5883();
            delta_theta=mod_theta(theta-theta_now);
            if(delta_theta-delta_theta_first-7.5*theta_C>=8){
                D2=D1;
                D1=ultrasonic();
                D=max(D1,D2);
                if(D<50){
                    MOD_LOC_LIN(mod_theta(theta_now-86),loc,D);
                }
                theta_C=theta_C+1;
            }           
        }
    }
    stop_turn();
}
void MOD_LOC_LIN(short int theta,short int loc[2],char D){
    short int R=-129;
    short int r_C;
    exporting();
    short int x[11];
    short int y[11];
    for(r_C=-5;r_C<6;r_C++){
        x[r_C+5]=loc[0]+round((D+r_C*0.5)*cos(theta));
        y[r_C+5]=loc[1]+round((D+r_C*0.5)*sin(theta));
    }
    char map_r[11];
    SD_read_coner(x,y,map_r);
    for(r_C=0;r_C<6;r_C++){
        if(map_r[5+r_C]>R){
            loc[0]=loc[0]-round(r_C*0.5*cos(theta));
            loc[1]=loc[0]-round(r_C*0.5*sin(theta));
            R=map_r[5+r_C];
        }
        if(map_r[5-r_C]>R){
            loc[0]=loc[0]+round(r_C*0.5*cos(theta));
            loc[1]=loc[0]+round(r_C*0.5*sin(theta));
            R=map_r[5-r_C];
        }
    }
}

short int mod_theta(short int theta){
    short int result;
    result=theta%360-180;
    return result;
}
char max(char a,char b){
    char c;
    if (a>b){
        c=a;
    }
    else{
        c=b;
    }
    return c;
}

char ultrasonic(){
  return 80;
}
