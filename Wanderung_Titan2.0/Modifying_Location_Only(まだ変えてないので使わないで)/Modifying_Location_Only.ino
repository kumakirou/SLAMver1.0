/*main関数、引数は(ロリコンから判明した現在地X,ロリコンから判明した現在地Y,補正された現在地を入れる戻り値用の配列。)
必要物
#include‥<math.h>,<Wire.h>,<DFRobot_QMC5883.h>,<Servo.h>
setup‥serial.begin(9600)
関数‥SD_read_map,max,qmc5883,ultrasonic()*/
void MOD_LOC_main(short int loc_x,short int loc_y,int loc[2]){
    const short int div_theta=48;
    short int theta_C;
    char D[div_theta+1];//角度ごとの距離
    short int theta;
    theta=qmc5883();
        /*@角度を-thetaに向ける*/
    for(theta_C=0; theta_C< div_theta; theta_C++)
    {
        
        D[theta_C]=ultrasonic();
        /*@7.5°回転*/
    }
    D[div_theta]=D[0];
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
    SD_read_map(map_memory,object_x[12], object_y[12]);
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
    SD_read_map(map_memory,object_x[12], object_y[12]);
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
    SD_read_map(map_memory,object_x[12], object_y[12]);
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
    SD_read_map(map_memory,object_x[12], object_y[12]);
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
    R=R_xy[0][0];
    for(X_C=0;X_C<5;X_C++){
        for (Y_C= 0; Y_C< 5; Y_C++)
        {
            if(R_xy[X_C][Y_C]>R){
                R=R_xy[X_C][Y_C];
                X=X_C;
                Y=Y_C;
            }
        }
        
    }
    loc[0]=loc_x+X-3;
    loc[1]=loc_y+Y-3;
}

/*sub関数(自作)*/
short int max(char a,char b){
    char c;
    if (a>b){
        c=a;
    }
    else{
        c=b;
    }
    return c;
}
