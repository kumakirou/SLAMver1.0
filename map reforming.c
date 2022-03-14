// Online C compiler to run C program online
#include<stdio.h>
#include<math.h>
const short int div_theta=48;
int ultra_sonic(){/*超音波センサーで距離を測定する関数*/
    return 20;//一応20を返すようにしてある
}
int bloodhound(char D[div_theta]){
    short int theta_C;
    /*@超音波測距センサーをx軸正方向に向けるコードを挿入*/
    for (theta_C; theta_C>div_theta+1;theta_C++)//サイクリック処理が可能なように０以上2π以下で格納
    {
        D[theta_C]=ultra_sonic();
        /*@超音波センサーをずらすコードを挿入*/
    }
    
}

int SD_read_chunk(short int x,short int y,char map_memory[4][11][11]){//チャンクの情報をSDからメモリに読み込む関数
    short int x_C;
    short int y_C;
    short int quad_C;
    for (quad_C = 0; quad_C < 4; quad_C++)
    {
        for (x_C = 0; x_C < 11; x_C++)
        {
            for (y_C = 0; y_C < 11; y_C++)
            {
                map_memory[quad_C][x_C][y_C]=0;//＠一応128を返すようにしてある
            }
        }
        
    }
}
int SD_make_table(unsigned char SD_table[60][60][2]){//テーブル作成
    short int x_C;
    short int y_C;
    for (x_C = 0; x_C < 60; x_C++)
    {
        for (y_C = 0; y_C < 60; y_C++)
        {
            SD_table[x_C][y_C][0]=sqrt(x_C^2+y_C^2);
            if (x_C>y_C)
            {
                SD_table[x_C][y_C][1]=atan(y_C/x_C)*40.28;
            }
            else
            {
                SD_table[x_C][y_C][1]=128-atan(x_C/y_C)*40.28;
            }
        }
        
    }
    
}
int SD_read_table(short int x,short int y,unsigned char table_memory[15][15][2]){//チャンクに合わせたテーブルの読み込み
    short int x_C;
    short int y_C;
    for (y_C = 0; y_C < 15; y_C++){
        for (x_C; x_C< 15;x_C++){
            table_memory[x_C][y_C][0]=sqrt((x_C+x)^2+(y_C+y)^2);
            if (x_C>y_C)
            {
                table_memory[x_C][y_C][1]=atan((y+y_C)/(x+x_C))*40.28;
            }
            else
            {
                table_memory[x_C][y_C][1]=128-atan((x_C+x)/(y_C+y))*40.28;
            }
        }
        
    }
}
int possibility_theta(short int d,char r){//各方向の存在確率を示す関数
int possibility;
    if(d>50){
        possibility=r*2.4-128;
    }
    else{
        if(d>r){
            possibility=-128+r*2.4;
        }
        else if(d>r-5)
        {
            possibility=120-24*(r-d);
        }
        else{
            possibility=0;
        }
    }
    return possibility;
}
int write_map_SD(short int x,short int y,char map_memory[4][11][11]){
}
int map_reforming(short int loc_x,short int loc_y,char D[div_theta+1]){
    bloodhound(D);
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
            SD_read_table(11*chank_x_C,11*chank_y_C,table_memory);
            SD_read_chunk(loc_x+11*chank_x_C,loc_y+11*chank_y_C,map_memory);
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
            write_map_SD(loc_x+11*chank_x_C+1,loc_y+11*chank_y_C+1,map_memory);
        }
    }
    
}
