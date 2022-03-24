/*main関数
引数は目標角、現在位置の入った配列のアドレス
必要物
#include‥<math.h>,<Wire.h>,<DFRobot_QMC5883.h>,<Servo.h>
setup‥serial.begin(9600)
関数‥SD_read_map,max,qmc5883_2,ultrasonic(),exporting,possibility_theta
,setup_qmc5883(),mod_theta*/
void Turn_with_MOD_LOC(short int theta,short int loc[2]){
    short int theta_now;
    short int delta_theta;
    char D1;
    char D2;
    char D;
    short int theta_C;
    short int theta_firt;
    theta_now=qmc5883_2();
    theta_firt=theta_now;
    delta_theta=mod_theta(theta-theta_now);
    short int delta_theta_first;
    D1=ultrasonic();
    theta_C=0;
    if(delta_theta>0){
        turn_left();
        while(delta_theta>=1){
            theta_now=qmc5883_2();
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
            theta_now=qmc5883_2();
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
    stop();
}
void MOD_LOC_LIN(short int theta,short int loc[2],char D){
    short int R=-129;
    short int r_C;
    exporting();
    char x[11];
    char y[11];
    for(r_C=-5;r_C<6;r_C++){
        x[r_C+5]=loc[0]+round((D+r_C*0.5)*cos(theta));
        y[r_C+5]=loc[1]+round((D+r_C*0.5)*sin(theta));
    }
    char map_r[11];
    SD_read_coner(x[12],y[12],map_r);
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

short int mod_theta(short int theta,short int loc[2]){
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
