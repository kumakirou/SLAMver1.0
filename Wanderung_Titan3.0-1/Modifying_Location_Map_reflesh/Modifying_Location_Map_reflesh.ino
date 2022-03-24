/*
main関数、引数は(ロリコンから判明した現在地X,ロリコンから判明した現在地Y,補正された現在地を入れる戻り値用の配列)。
必要物
#include‥<math.h>,<Wire.h>,<DFRobot_QMC5883.h>,<Servo.h>
setup‥serial.begin(9600)
関数‥map_reforming,MOD_LOC,SD_read_map,max,qmc5883_2,ultrasonic(),importing,exporting,possibility_theta,r_theta_table
,setup_qmc5883(),turn_to_theta,mod_theta*/

void MOD_LOC_Map_reflesh_main(short int loc[2]){//x座標y座標の配列。引数にも戻り値にも使用
    char D[49];
    short int theta_C;
    int minX = 0;//磁気センサーのセットアップに使う変数？？？
    int maxX = 0;
    int minY = 0;
    int maxY = 0;
    int offX = 0;
    int offY = 0;
    setup_qmc5883();//磁気センサーセットアップ。変数が多いのでいちいちセットアップする
    short int theta_now;//現在のローバーから見た磁北線の角
    theta_now=qmc5883_2();
    short int theta_C_first;//
    theta_C_first=(-theta_now/7.5-12)%48;//最初に距離を測る角度のインデント
    for(theta_C=0;theta_C<48;theta_C++){//角度を刻みながら左回転し超音波センサーの距離をＤ[]に格納
        turn_to_theta(round((theta_C_first+theta_C)*7.5-3.75));
        D[(theta_C+theta_C_first)%48]=ultrasonic();//東がx軸
        }
    }
    D[48]=D[0];//計算のためサイクリックに
    MOD_LOC(loc,D);//自己位置推定
    map_reforming(loc[0],loc[1],D);//マップ作製
}

/*sub関数(自作)*/
void turn_to_theta(short int theta){//引数は目標角。目標角に向けて旋回する
    short int theta_now;//現在角(-180°～180°)
    short int delta_theta;//目標角との差(-180°～180°)
    theta_now=qmc5883_2();//測定
    delta_theta=mod_theta(theta+theta_now);//目標角との差。現在角はローバーの方位ではなくローバーから見た磁北線の方位でありローバーの方位はそのマイナスで与えられることに注意。
    while(abs(delta_theta)>2){
        if(delta_theta>0){
            turn_left(delta_theta*0.0113);//目標角の方が大きいので左旋回
        }
        else{
            turn_right(-delta_theta*0.0113);//目標角の方が小さいので右旋回
        }
        delta_theta=mod_theta(theta+theta_now);
    }
}

short int mod_theta(short int theta){//引数は一般角、-180°から108°に変換
    short int result;
    result=theta%360-180;
    return result;
}

short int possibility_theta(short int d,char r){//各方向の存在確率を示す関数
  int possibility;
  if(d>50){
    possibility=r*2.4-128;//障害物が検知されなかった時、近くほど平坦である確率が高い
   }
  else{
    if(d>r){
      possibility=-128+r*2.4;//障害物が検知されても障害物までは平坦
     }
    else if(d>r-5){
      possibility=120-24*(r-d);//障害物付近に強いピーク。その後は徐々に減少
    }
    else{
      possibility=0;//障害物から十分に離れた背後は完全不明
    }
  }
    return possibility;
}


void map_reforming(short int loc_x,short int loc_y,char D[49]){//地図更新,引数は(x座標　y座標　各方位ごとの障害物の距離
    char map_memory[4][11][11];//11*11のマップ
    unsigned char table_memory[15][15][2];//15*15のテーブル、ただし、chunk_x_c=0の時はy二つx軸方向にずれている
    short int chank_x_C;//チャンクを数えるカウンタ
    short int chank_y_C;
    short int x_C;//チャンク内のマップのマスを数えるカウンタ
    short int y_C;
    short int theta_C;//角度情報を取り出すカウンタ。反時計回り
    short int theta_C_margin;//上の測定角からの各マスの偏角
    short int possi;//存在確率を格納
    short int table;//char型(文字型)をint型に変換して剰余計算ができるようにする
    for(chank_x_C=0;chank_x_C<3;chank_x_C++){
        for (chank_y_C=0;chank_y_C<3&&chank_y_C<3;chank_y_C++)
        {
            r_theta_table(table_memory,11*chank_x_C,11*chank_y_C);//必要データの読み出し
            exporting(map_memory,loc_x+11*chank_x_C,loc_y+11*chank_y_C);
            for(x_C = 0; x_C < 11; x_C++)
            {
                for (y_C = 0; y_C < 11; y_C++)
                {
                    if((x_C+11*chank_x_C)*(x_C+11*chank_x_C)+(y_C+11*chank_y_C)*(y_C+11*chank_y_C)<626){
                        theta_C=table_memory[x_C][y_C][1]/12;//そのマスの計算に必要な測定方位
                        table=table_memory[x_C][y_C][1];
                        theta_C_margin=(table*3)%12;
                        possi=possibility_theta(D[theta_C],table_memory[x_C][y_C][0])*(11-theta_C_margin)+
                        +possibility_theta(D[theta_C+1],table_memory[x_C][y_C][0])*theta_C_margin;
                        map_memory[0][x_C][y_C]=2*(map_memory[0][x_C][y_C]*possi-64*map_memory[0][x_C][y_C]-64*possi)/(map_memory[0][x_C][y_C]+possi-256);//第一象限

                        possi=possibility_theta(D[24-theta_C],table_memory[x_C][y_C][0])*(11-theta_C_margin)+
                        +possibility_theta(23-D[theta_C],table_memory[x_C][y_C][0])*theta_C_margin;
                        map_memory[1][x_C][y_C]=2*(map_memory[1][x_C][y_C]*possi-64*map_memory[1][x_C][y_C]-64*possi)/(map_memory[1][x_C][y_C]+possi-256);//第二象限

                        possi=possibility_theta(D[theta_C+12],table_memory[x_C][y_C][0])*(11-theta_C_margin)+
                        +possibility_theta(D[theta_C+13],table_memory[x_C][y_C][0])*theta_C_margin;
                        map_memory[2][x_C][y_C]=2*(map_memory[2][x_C][y_C]*possi-64*map_memory[2][x_C][y_C]-64*possi)/(map_memory[2][x_C][y_C]+possi-256);

                        possi=possibility_theta(D[48-theta_C],table_memory[x_C][y_C][0])*(11-theta_C_margin)+
                        +possibility_theta(47-D[theta_C],table_memory[x_C][y_C][0])*theta_C_margin;
                        map_memory[3][x_C][y_C]=2*(map_memory[3][x_C][y_C]*possi-64*map_memory[3][x_C][y_C]-64*possi)/(map_memory[3][x_C][y_C]+possi-256);
                    }
                }
            }
            importing(map_memory,loc_x+11*chank_x_C+1,loc_y+11*chank_y_C+1);//書き込み
        }
    }
    
}


void MOD_LOC(short int loc[2],char D[49]){//自己位置推定関数。引数は現在地のポインタ(返り値を入れるためにも使用),超音波センサーの距離
    short int theta_C;
    char map_memory[12][5][5];//11*11のマップ
    int R_xy[5][5];//相関係数を入れる箱
    short int X_C;//相関係数を求めるために、テーブルと捜査範囲をずらすカウンタ
    short int Y_C;
    loc_x=loc[0];
    loc_y=loc[1];

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
        if(D[theta_C]<50)
        {
            for (X_C; X_C < 5; X_C++)
            {
                for (Y_C = 0; Y_C < 5; Y_C++)
                {
                    R_xy[X_C][Y_C]=R_xy[X_C][Y_C]+map_memory[theta_C][X_C][Y_C];
                }
        
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
        if(D[theta_C]<50){
            for (X_C; X_C < 5; X_C++)
            {
                for (Y_C = 0; Y_C < 5; Y_C++)
                {
                    R_xy[X_C][Y_C]=R_xy[X_C][Y_C]+map_memory[theta_C][X_C][Y_C];
                }
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
        if(D[theta_C]<50){
            for (X_C; X_C < 5; X_C++)
            {
                for (Y_C = 0; Y_C < 5; Y_C++)
                {
                    R_xy[X_C][Y_C]=R_xy[X_C][Y_C]+map_memory[theta_C][X_C][Y_C];
                }
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
        if(D[theta_C]<50)
        {
            for (X_C; X_C < 5; X_C++)
            {
                for (Y_C = 0; Y_C < 5; Y_C++)
                {
                    R_xy[X_C][Y_C]=R_xy[X_C][Y_C]+map_memory[theta_C][X_C][Y_C];
                }
        
            }
        }
    }
    X=0;
    Y=0;
    int R;
    R=R_xy[2][2];
    for(X_C=0;X_C<2;X_C++){
        for (Y_C= 0; Y_C< 2; Y_C++)//共分散の比較for文が変なことになっているのはもし異なるずらし方の値が等しくなった時、ロリコンを信じて中央付近を優先するため
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
    loc[0]=loc_x+X-3;//位置更新
    loc[1]=loc_y+Y-3;
}

void setup_qmc5883() {//磁気センサーセットアップ
  if (compass.isQMC()) {
    compass.setRange(QMC5883_RANGE_2GA);
    compass.setMeasurementMode(QMC5883_CONTINOUS);
    compass.setDataRate(QMC5883_DATARATE_50HZ);
    compass.setSamples(QMC5883_SAMPLES_8);
    }
}


char max(char a,char b){//最大値を求める関数 arduinoのライブラリにも同じ内容で同じ名前の関数があるので消した方がいいかも
    char c;
    if (a>b){
        c=a;
    }
    else{
        c=b;
    }
    return c;
}
int qmc5883_2() {//加計君の磁気センサー関数の改良型。atan2を使っている。ローバーの角度ではなくローバーから見た磁北線の角度なのに注意。
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
