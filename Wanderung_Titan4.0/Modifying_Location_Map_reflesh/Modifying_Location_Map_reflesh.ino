/*
main関数、引数は(ロリコンから判明した現在地X,ロリコンから判明した現在地Y,補正された現在地を入れる戻り値用の配列)。
必要物
#include‥<math.h>,<Wire.h>,<DFRobot_QMC5883.h>,<Servo.h>
setup‥serial.begin(9600)
*/

void MOD_LOC_Map_reflesh_main(short int loc[2]){//引数はx座標y座標の配列。戻り値格納にも使用。setup_qmc5883,qmc_5883,turn_to_theta,ultrasonic_behind,MOD_LOC,map_reforming,maxが必要
    char D_raw[48];
    short int theta_C;
    int minX = 0;//磁気センサーのセットアップに使う変数？？？
    int maxX = 0;
    int minY = 0;
    int maxY = 0;
    int offX = 0;
    int offY = 0;
    setup_qmc5883();//磁気センサーセットアップ。変数が多いのでいちいちセットアップする
    short int theta_now;//現在のローバーから見た磁北線の角
    theta_now=qmc5883();
    short int theta_C_first;//
    theta_C_first=(-theta_now/7.5)%48;//最初に距離を測る角度のインデント
    for(theta_C=0;theta_C<48;theta_C++){//角度を刻みながら左回転し超音波センサーの距離をＤ[]に格納
        turn_to_theta(round((theta_C_first+theta_C)*7.5-3.75));
        D[(theta_C+theta_C_first)%48]=ultrasonic_behind();//南がx軸
    }
    char D_fixed[49];
    for(theta_C=0;theta_C<47;theta_C++){
        D_fixed[theta_C]=max(D_raw[theta_C],D_raw[theta_C+1])
    }
    D_fixed[47]=max(D_raw[47],D_raw[48]);
    D_fixed[48]=D_fixed[0];//計算のためサイクリックに
    MOD_LOC(loc,D_fixed);//自己位置推定
    map_reforming(loc[0],loc[1],D_fixed);//マップ作製
}

/*sub関数(自作)*/
void turn_to_theta(short int theta){//引数は目標角。目標角に向けて旋回する。qmc5883,mod_theta,turn_left,turn_right,stop_movingが必要
    short int theta_now;//現在角(-180°～180°)
    short int delta_theta;//目標角との差(-180°～180°)
    theta_now=qmc5883();//測定
    delta_theta=mod_theta(theta+theta_now);//目標角との差。現在角はローバーの方位ではなくローバーから見た磁北線の方位でありローバーの方位はそのマイナスで与えられることに注意。
    if(delta_theta>2){//目標角の方が大きいので左旋回
        turn_left();//左旋回のピン出力
        while (1<delta_theta&&delta_theta<70)
        {
            theta_now=qmc5883();//測定
            delta_theta=mod_theta(theta+theta_now);
        }
    }
        
    if(delta_theta<2){
        turn_right();//右旋回のピン出力
        while (2>delta_theta&&delta_theta>-70)//目標角の方が小さいので右旋回
        {
            theta_now=qmc5883();//測定
            delta_theta=mod_theta(theta+theta_now);
        }
    }
    stop_moving()//旋回をやめるピン出力
}

short int mod_theta(short int theta){//引数は一般角、-180°から108°に変換
    short int result;
    result=theta%360-180;//剰余算
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


void map_reforming(short int loc_x,short int loc_y,char D[49]){//地図更新,引数は(x座標　y座標　各方位ごとの障害物の距離)。r_theta_table,exporting,importing,possibility_theta,synsesize_map_pointが必要
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
                        possi=possibility_theta(D[theta_C],table_memory[x_C][y_C][0])*(1-theta_C_margin/11)+
                        +possibility_theta(D[theta_C+1],table_memory[x_C][y_C][0])*theta_C_margin/11;//その点の存在確率を線形推定
                        map_memory[0][x_C][y_C]=synsesize_map_point(map_memory[0][x_C][y_C],possi);//第一象限

                        possi=possibility_theta(D[24-theta_C],table_memory[x_C][y_C][0])*(1-theta_C_margin/11)+
                        +possibility_theta(23-D[theta_C],table_memory[x_C][y_C][0])*theta_C_margin/11;
                        map_memory[1][x_C][y_C]=synsesize_map_point(map_memory[1][x_C][y_C],possi);//第二象限

                        possi=possibility_theta(D[theta_C+12],table_memory[x_C][y_C][0])*(1-theta_C_margin/11)+
                        +possibility_theta(D[theta_C+13],table_memory[x_C][y_C][0])*theta_C_margin/11;
                        map_memory[2][x_C][y_C]=synsesize_map_point(map_memory[2][x_C][y_C],possi);

                        possi=possibility_theta(D[48-theta_C],table_memory[x_C][y_C][0])*(1-theta_C_margin/11)+
                        +possibility_theta(47-D[theta_C],table_memory[x_C][y_C][0])*(theta_C_margin/11;
                        map_memory[3][x_C][y_C]=synsesize_map_point(map_memory[3][x_C][y_C],possi);
                    }
                }
            }
            importing(map_memory,loc_x+11*chank_x_C+1,loc_y+11*chank_y_C+1);//書き込み
        }
    }
    
}


void MOD_LOC(short int loc[2],char D[49]){//自己位置推定関数。引数は現在地のポインタ(返り値を入れるためにも使用),超音波センサーの距離。,SD_read_mapが必要
    const float sin_table[13]={0,0.131,0.259,0.383,0.5,0.609,0.707,0.793,0.866,0.924,0.966,0.991,1};
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
        object_x[theta_C]=loc[0]+sin_table[12-theta_C]*D[theta_C]-2;
        object_y[theta_C]=loc[1]+sin_table[theta_C]*D[theta_C]-2;
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
        object_x[theta_C]=loc[0]-sin_table[theta_C]*D[theta_C+12]-2;
        object_y[theta_C]=loc[1]+sin_table[12-theta_C]*D[theta_C+12]-2;
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
        object_x[theta_C]=loc[0]-sin_table[12-theta_C]*D[theta_C+24]-2;
        object_y[theta_C]=loc[1]-sin_table[theta_C]*D[theta_C+24]-2;
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
        object_x[theta_C]=loc[0]+sin_table[theta_C]*D[theta_C+36]-2;
        object_y[theta_C]=loc[1]-sin_table[12-theta_C]*D[theta_C+36]-2;
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
    loc[0]=loc[0]+X-2;//位置更新
    loc[1]=loc[1]+Y-2;
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

char synsesize_map_point(char old_map,short int ultra_result){//マップの各点における確率を合成するための関数。計算量を度外視しやや複雑に設定しているので遅ければ相加平均に変える
    char new_map;
    new_map=2*(old_map*ultra_result-64*old_map-64*ultra_result)/ultra_result+old_map-256);
    return new_map;
}