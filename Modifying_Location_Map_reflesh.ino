#define div_theta 24
#define map_size_x 200
#define map_size_y 100
#define map_mesh_x 20
#define map_mesh_y 10
#define inv_div_theta 15

/*
main関数、引数は(ロリコンから判明した現在地X,ロリコンから判明した現在地Y,補正された現在地を入れる戻り値用の配列)。
必要物
#include‥<math.h>,<Wire.h>,<DFRobot_QMC5883.h>,<Servo.h>
setup‥serial.begin(9600)
*/



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

char synsesize_map_point(char old_map,char ultra_result){//マップの各点における確率を合成するための関数。計算量を度外視しやや複雑に設定しているので遅ければ相加平均に変える
    char new_map;
    new_map=2*(old_map*ultra_result-64*old_map-64*ultra_result)/(ultra_result+old_map-256);
    return new_map;
}

void map_reforming(short int loc_x,short int loc_y,char D[24],char mapdata[20][10]){//地図更新,引数は(x座標　y座標　各方位ごとの障害物の距離)。r_theta_table,exporting,importing,possibility_theta,synsesize_map_pointが必要
    short int x_C;//チャンク内のマップのマスを数えるカウンタ
    short int y_C;
    char possi;//存在確率を格納
    short int theta;
    short int theta_C;
    float theta_margin;
    for(x_C=0;x_C<5;x_C++){
      if(x_C+loc_x>=0&x_C+loc_x<=19){
        for(y_C=0;y_C<5;y_C++){
          if(y_C+loc_y>=0&y_C+loc_y<=9){
            theta=atan2(y_C,x_C)/6.2832*360+360;
            theta=theta%360;
            theta_C=theta/inv_div_theta;
            theta_margin=(theta%inv_div_theta)/inv_div_theta;
            possi=(1-theta_margin)*possibility_theta(D[theta_C],10*sqrt(x_C*x_C+y_C*y_C))+theta_margin*possibility_theta(D[theta_C+1],10*sqrt(x_C*x_C+y_C*y_C));
            mapdata[x_C+loc_x][y_C+loc_y]=synsesize_map_point(mapdata[x_C+loc_x][y_C+loc_y],possi);
          }
        }
      }
    }
}

void MOD_LOC(short int loc[2],char D[24],char mapdata[20][10]){//自己位置推定関数。引数は現在地のポインタ(返り値を入れるためにも使用),超音波センサーの距離。,SD_read_mapが必要
    int R_xy[3][3];//相関係数を入れる箱
    short int X_C;//相関係数を求めるために、テーブルと捜査範囲をずらすカウンタ
    short int Y_C;
    short int theta_C;
    char X;//最終結果を入れる
    char Y;
    short int object_x;
    short int object_y;
    for(theta_C=0;theta_C<div_theta;theta_C++){
      object_x=round(D[theta_C]*cos(theta_C*6.2832/div_theta)/10);
      object_y=round(D[theta_C]*sin(theta_C*6.2832/div_theta)/10);
      if(D[theta_C]<50){
        for(X_C=0;X_C<5;X_C++){
          if(object_x+loc[0]+X_C-1>=0&object_x+loc[0]+X_C-1<=19){
            for(Y_C=0;Y_C<5;Y_C++){
              if(object_y+loc[1]+Y_C-1>=0&object_y+loc[1]+Y_C-1<=9){
                R_xy[X_C][Y_C]=R_xy[X_C][Y_C]+mapdata[object_x+loc[0]+X_C-1][object_y+loc[1]+Y_C-1];
              }
            }
          }
        }
      }
    }
    X=0;
    Y=0;
    int R;
    R=R_xy[1][1];
    for(X_C=0;X_C<2;X_C++){
        for (Y_C= 0; Y_C< 2; Y_C++)//共分散の比較。for文が変なことになっているのはもし異なるずらし方の値が等しくなった時、ロリコンを信じて中央付近を優先するため
        {
            if(R_xy[1+X_C][1+Y_C]>R){
                R=R_xy[1+X_C][1+Y_C];
                X=-X_C;
                Y=-Y_C;
            }
            if(R_xy[1+X_C][1-Y_C]>R){
                R=R_xy[1+X_C][1-Y_C];
                X=-X_C;
                Y=Y_C;
            }
            if(R_xy[1-X_C][1+Y_C]>R){
                R=R_xy[1-X_C][1+Y_C];
                X=X_C;
                Y=-Y_C;
            }
            if(R_xy[1-X_C][1-Y_C]>R){
                R=R_xy[1-X_C][1-Y_C];
                X=X_C;
                Y=Y_C;
            }
        }
        
    }
    loc[0]=loc[0]+X-1;//位置更新
    loc[1]=loc[1]+Y-1;
}




void MOD_LOC_Map_reflesh_main(short int loc[2],char D_raw[div_theta],char mapdata[20][10]){//引数は現在地のx座標y座標の配列、x軸を始線とした左回りにdev_theta回サンプリングした超音波センサーの値、マップ。戻り値格納にも使用。ultrasonic_behind,MOD_LOC,map_reforming,maxが必要
    char D_fixed[div_theta+1];
    D_fixed[div_theta]=D_fixed[0];//計算のためサイクリックに
    MOD_LOC(loc,D_fixed,mapdata);//自己位置推定
    map_reforming(loc[0],loc[1],D_fixed,mapdata);//マップ作製
}
