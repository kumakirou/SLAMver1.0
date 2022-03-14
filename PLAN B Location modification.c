#include<stdio.h>
#include<math.h>
const short int div_theta=48;//角度分解能
const float sin_table[]={0,0.1305,0.2588,0.3827,0.5,0.6088,0.7071,0.7934,0.866,0.9239,0.9659,0.9914,1};//7.5°ごとのsinを入れてあるテーブル
int ultra_sonic(){/*超音波センサーで距離を測定する関数*/
    return 20;//一応20を返すようにしてある
}
int max(char a,char b){//最大値を返す関数、7.5°離れた測定結果二つからその間に挟まれた部分の障害物までの距離を返すために作成
    char c;
    if (a>b){
        c=a;
    }
    else{
        c=b;
    }
    return c;
}
int SD_read_chunk(short int x[12],short int y[12],char map_memory[12][5][5]){//チャンクの情報をSDからメモリに読み込む関数。一象限あたり、90°/7.5で12個の観測データがあり、それぞれが5×5で動く
    short int x_C;
    short int y_C;
    short int chunk_C;
    for (chunk_C = 0; chunk_C < 12; chunk_C++)
    {
        for (x_C = 0; x_C < 5; x_C++)
        {
            for (y_C = 0; y_C < 5; y_C++)
            {
                map_memory[x_C][y_C][chunk_C]=0;//＠一応0を返すようにしてある
            }
        }
        
    }
}
int LOC(short int loc_x,short int loc_y,short int loc[2]){
    char D[div_theta+1];//角度ごとの距離
    short int theta_C;
    /*@角度を-3.75°に向ける。二つの観測で挟まれた間の距離を測っているため*/
    for(theta_C=0; theta_C< div_theta; theta_C++)
    {
        D[theta_C]=ultra_sonic();//距離を測って格納
        /*@7.5°回転*/
    }
    D[div_theta]=D[0];//サイクリックな処理のために配列を一つ長くする
    char map_memory[12][5][5];//11*11のマップ
    int R_xy[5][5];//相関係数を入れる箱
    short int X_C;//相関係数を求めるために、テーブルと捜査範囲をずらすカウンタ
    short int Y_C;

    char X;//最終結果を入れる
    char Y;
    int object_x[12];//障害物の存在する座標
    int object_y[12];
    for (theta_C = 0; theta_C < 12; theta_C++)//第1象限
    {
        object_x[theta_C]=loc_x+sin_table[12-theta_C]*max(D[theta_C],D[theta_C+1])-2;//障害物の存在する座標を計算
        object_y[theta_C]=loc_y+sin_table[theta_C]*max(D[theta_C],D[theta_C+1])-2;
    }
    SD_read_chunk(object_x,object_y,map_memory);
    for(theta_C=0;theta_C<12;theta_C++){
        for (X_C; X_C < 5; X_C++)
        {
            for (Y_C = 0; Y_C < 5; Y_C++)
            {
                R_xy[X_C][Y_C]=R_xy[X_C][Y_C]+map_memory[theta_C][X_C][Y_C];//マップとの相関係数を足す
            }
        
        }
    }
    for (theta_C = 0; theta_C < 12; theta_C++)//第2象限
    {
        object_x[theta_C]=loc_x-sin_table[theta_C]*max(D[theta_C+12],D[theta_C+13])-2;
        object_y[theta_C]=loc_y+sin_table[12-theta_C]*max(D[theta_C+12],D[theta_C+13])-2;
    }
    SD_read_chunk(object_x,object_y,map_memory);
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
    SD_read_chunk(object_x,object_y,map_memory);
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
    SD_read_chunk(object_x,object_y,map_memory);
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
    for(X_C=0;X_C<5;X_C++){//どのずらし方の相関係数が最も高いか計算
        for (Y_C= 0; Y_C< 5; Y_C++)
        {
            if(R_xy[X_C][Y_C]>R){
                R=R_xy[X_C][Y_C];
                X=X_C;
                Y=Y_C;
            }
        }
        
    }
    loc[0]=loc_x+X-3;//位置更新
    loc[1]=loc_y+Y-3;
}
