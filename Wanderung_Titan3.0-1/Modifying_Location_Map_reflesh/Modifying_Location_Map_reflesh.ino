/*
main関数、引数は(ロリコンから判明した現在地X,ロリコンから判明した現在地Y,補正された現在地を入れる戻り値用の配列)。
必要物
#include‥<math.h>,<Wire.h>,<DFRobot_QMC5883.h>,<Servo.h>
setup‥serial.begin(9600)
関数‥map_reforming,MOD_LOC,SD_read_map,max,qmc5883_2,ultrasonic(),importing,exporting,possibility_theta,r_theta_table
,setup_qmc5883(),turn_to_theta,mod_theta*/

void MOD_LOC_Map_reflesh_main(short int loc[2]){
    char D[49];
    short int theta_C;
    int minX = 0;
    int maxX = 0;
    int minY = 0;
    int maxY = 0;
    int offX = 0;
    int offY = 0;
    short int loc[2];
    setup_qmc5883();
    short int theta_now;
    theta_now=qmc5883_2();
    short int theta_C_first;
    theta_C_first=((theta_now+180)/7.5-12)%48;
    for(theta_C=0;theta_C<48;theta_C++){
        turn_to_theta(round((theta_C_first+theta_C)*7.5-3.75));
        D[(theta_C+theta_C_first)%48]=ultrasonic();//東がx軸
    }
    D[48]=D[0];
    MOD_LOC(loc,D);
    map_reforming(loc[0],loc[1],D);
}

/*sub関数(自作)*/
void turn_to_theta(short int theta){
    short int theta_now;
    short int delta_theta;
    theta_now=qmc5883_2();
    delta_theta=mod_theta(theta-theta_now);
    while(abs(delta_theta)>2){
        if(delta_theta>0){
            turn_right(delta_theta*0.0113);
        }
        else{
            turn_right(-delta_theta*0.0113);
        }
        delta_theta=mod_theta(theta-theta_now);
    }
}

short int mod_theta(short int theta){
    short int result;
    result=theta%360-180;
    return result;
}

short int possibility_theta(short int d,char r){//各方向の存在確率を示す関数
  int possibility;
  if(d>50){
    possibility=r*2.4-128;
   }
  else{
    if(d>r){
      possibility=-128+r*2.4;
     }
    else if(d>r-5){
      possibility=120-24*(r-d);
    }
    else{
      possibility=0;
    }
  }
    return possibility;
}


void map_reforming(short int loc_x,short int loc_y,char D[49]){
    char map_memory[4][11][11];//11*11のマップ
    unsigned char table_memory[15][15][2];//15*15のテーブル、ただし、chunk_x_c=0の時はy二つx軸方向にずれている
    short int chank_x_C;//チャンクを数えるカウンタ
    short int chank_y_C;
    short int x_C;//チャンク内のマップのマスを数えるカウンタ
    short int y_C;
    short int theta_C;//角度情報を取り出すカウンタ。反時計回り
    short int theta_C_margin;
    short int possi;
    short int table;
    for(chank_x_C=0;chank_x_C<3;chank_x_C++){
        for (chank_y_C=0;chank_y_C<3&&chank_y_C<3;chank_y_C++)
        {
            r_theta_table(table_memory,11*chank_x_C,11*chank_y_C);
            exporting(map_memory,loc_x+11*chank_x_C,loc_y+11*chank_y_C);
            for(x_C = 0; x_C < 11; x_C++)
            {
                for (y_C = 0; y_C < 11; y_C++)
                {
                    if((x_C+11*chank_x_C)*(x_C+11*chank_x_C)+(y_C+11*chank_y_C)*(y_C+11*chank_y_C)<626){
                        theta_C=table_memory[x_C][y_C][1]/12;
                        table=table_memory[x_C][y_C][1];
                        theta_C_margin=(table*3)%12;
                        possi=possibility_theta(D[theta_C],table_memory[x_C][y_C][0])*(11-theta_C_margin)+
                        +possibility_theta(D[theta_C+1],table_memory[x_C][y_C][0])*theta_C_margin;
                        map_memory[0][x_C][y_C]=2*(map_memory[0][x_C][y_C]*possi-64*map_memory[0][x_C][y_C]-64*possi)/(map_memory[0][x_C][y_C]+possi-256);

                        possi=possibility_theta(D[24-theta_C],table_memory[x_C][y_C][0])*(11-theta_C_margin)+
                        +possibility_theta(23-D[theta_C],table_memory[x_C][y_C][0])*theta_C_margin;
                        map_memory[1][x_C][y_C]=2*(map_memory[1][x_C][y_C]*possi-64*map_memory[1][x_C][y_C]-64*possi)/(map_memory[1][x_C][y_C]+possi-256);

                        possi=possibility_theta(D[theta_C+12],table_memory[x_C][y_C][0])*(11-theta_C_margin)+
                        +possibility_theta(D[theta_C+13],table_memory[x_C][y_C][0])*theta_C_margin;
                        map_memory[2][x_C][y_C]=2*(map_memory[2][x_C][y_C]*possi-64*map_memory[2][x_C][y_C]-64*possi)/(map_memory[2][x_C][y_C]+possi-256);

                        possi=possibility_theta(D[48-theta_C],table_memory[x_C][y_C][0])*(11-theta_C_margin)+
                        +possibility_theta(47-D[theta_C],table_memory[x_C][y_C][0])*theta_C_margin;
                        map_memory[3][x_C][y_C]=2*(map_memory[3][x_C][y_C]*possi-64*map_memory[3][x_C][y_C]-64*possi)/(map_memory[3][x_C][y_C]+possi-256);
                    }
                }
            }
            importing(map_memory,loc_x+11*chank_x_C+1,loc_y+11*chank_y_C+1);
        }
    }
    
}


void MOD_LOC(short int loc_x,short int loc_y,int loc[2],char D[49]){
    short int theta_C;
    char map_memory[12][5][5];//11*11のマップ
    int R_xy[5][5];//相関係数を入れる箱
    short int X_C;//相関係数を求めるために、テーブルと捜査範囲をずらすカウンタ
    short int Y_C;

    char X;//最終結果を入れる
    char Y;
    short int object_x[12];
    short int object_y[12];
    for (theta_C = 0; theta_C < 12; theta_C++)//第1象限
    {
        object_x[theta_C]=loc_x+sin_table[12-theta_C]*max(D[theta_C],D[theta_C+1])-2;
        object_y[theta_C]=loc_y+sin_table[theta_C]*max(D[theta_C],D[theta_C+1])-2;
    }
    SD_read_map(map_memory,object_x, object_y);
    for(theta_C=0;theta_C<12;theta_C++){
        for (X_C; X_C < 5; X_C++)
        {
            for (Y_C = 0; Y_C < 5; Y_C++)
            {
                R_xy[X_C][Y_C]=R_xy[X_C][Y_C]+map_memory[theta_C][X_C][Y_C];
            }
        
        }
    }
    for (theta_C = 0; theta_C < 12; theta_C++)//第2象限
    {
        object_x[theta_C]=loc_x-sin_table[theta_C]*max(D[theta_C+12],D[theta_C+13])-2;
        object_y[theta_C]=loc_y+sin_table[12-theta_C]*max(D[theta_C+12],D[theta_C+13])-2;
    }
    SD_read_map(map_memory,object_x, object_y);
    for(theta_C=0;theta_C<12;theta_C++){
        for (X_C; X_C < 5; X_C++)
        {
            for (Y_C = 0; Y_C < 5; Y_C++)
            {
                R_xy[X_C][Y_C]=R_xy[X_C][Y_C]+map_memory[theta_C][X_C][Y_C];
            }
        
        }
    }
    for (theta_C = 0; theta_C < 12; theta_C++)//第3象限
    {
        object_x[theta_C]=loc_x-sin_table[12-theta_C]*max(D[theta_C+24],D[theta_C+25])-2;
        object_y[theta_C]=loc_y-sin_table[theta_C]*max(D[theta_C+24],D[theta_C+25])-2;
    }
    SD_read_map(map_memory,object_x, object_y);
    for(theta_C=0;theta_C<12;theta_C++){
        for (X_C; X_C < 5; X_C++)
        {
            for (Y_C = 0; Y_C < 5; Y_C++)
            {
                R_xy[X_C][Y_C]=R_xy[X_C][Y_C]+map_memory[theta_C][X_C][Y_C];
            }
        
        }
    }
    for (theta_C = 0; theta_C < 12; theta_C++)//第4象限
    {
        object_x[theta_C]=loc_x+sin_table[theta_C]*max(D[theta_C+36],D[theta_C+37])-2;
        object_y[theta_C]=loc_y-sin_table[12-theta_C]*max(D[theta_C+36],D[theta_C+37])-2;
    }
    SD_read_map(map_memory,object_x, object_y);
    for(theta_C=0;theta_C<12;theta_C++){
        for (X_C; X_C < 5; X_C++)
        {
            for (Y_C = 0; Y_C < 5; Y_C++)
            {
                R_xy[X_C][Y_C]=R_xy[X_C][Y_C]+map_memory[theta_C][X_C][Y_C];
            }
        
        }
    }
    X=0;
    Y=0;
    int R;
    R=R_xy[2][2];
    for(X_C=0;X_C<2;X_C++){
        for (Y_C= 0; Y_C< 2; Y_C++)
        {
            if(R_xy[2+X_C][2+Y_C]>R){
                R=R_xy[X_C][Y_C];
                X=X_C;
                Y=Y_C;
            }
            if(R_xy[2+X_C][2-Y_C]>R){
                R=R_xy[X_C][Y_C];
                X=X_C;
                Y=Y_C;
            }
            if(R_xy[2-X_C][2+Y_C]>R){
                R=R_xy[X_C][Y_C];
                X=X_C;
                Y=Y_C;
            }
            if(R_xy[2-X_C][2-Y_C]>R){
                R=R_xy[X_C][Y_C];
                X=X_C;
                Y=Y_C;
            }
        }
        
    }
    loc[0]=loc_x+X-3;
    loc[1]=loc_y+Y-3;
}

void setup_qmc5883() {
  if (compass.isQMC()) {
    compass.setRange(QMC5883_RANGE_2GA);
    compass.setMeasurementMode(QMC5883_CONTINOUS);
    compass.setDataRate(QMC5883_DATARATE_50HZ);
    compass.setSamples(QMC5883_SAMPLES_8);
    }
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
int qmc5883_2() {
  double X = 0;
  double Y = 0;
  for (int i = 0; i < 100; i++) {
    Vector mag = compass.readRaw();
    X += mag.XAxis * 0.000105862 + 0.000113317;
    Y += mag.YAxis * 0.319926492 - 0.21266429 + 0.0000000000000001;
  }
  short int theta;
  theta = atan2 (Y,X) * 57.2958;
  return theta;
}
