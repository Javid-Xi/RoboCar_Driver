///*
// * ＠function 利用里程计数据实现位置积分
// * 
// */
//float s1=0,s2=0,s3=0,s4=0;
//float s1_last=0,s2_last=0,s3_last=0,s4_last=0;
//float position_x=0,position_y=0,position_w=0;
//void calculate_position_by_odometry(void)
//{
//  //方法１：　　计算每个轮子转动的位移，然后利用Ｆ矩阵合成Ｘ,Y,W三个方向的位移
//  float s1_delta=0,s2_delta=0,s3_delta=0,s4_delta=0;
//  float v1=0,v2=0,v3=0,v4=0;
//  float K4_1 = 1.0/(4.0*WHEEL_K);
//  float position_x_delta,position_y_delta,position_w_delta;
//  float linear_x,linear_y,linear_w;
// 
//  s1_last=s1;
//  s2_last=s2;
//  s3_last=s3;
//  s4_last=s4;
// 
//  //轮子转动的圈数乘以　N*２*pi*r
//  s1 =(moto_chassis[0].round_cnt+(moto_chassis[0].total_angle%8192)/8192.0)/WHEEL_RATIO*WHEEL_PI*WHEEL_D ; 
//  s2 =(moto_chassis[1].round_cnt+(moto_chassis[1].total_angle%8192)/8192.0)/WHEEL_RATIO*WHEEL_PI*WHEEL_D ; 
//  s3 =(moto_chassis[2].round_cnt+(moto_chassis[2].total_angle%8192)/8192.0)/WHEEL_RATIO*WHEEL_PI*WHEEL_D ; 
//  s4 =(moto_chassis[3].round_cnt+(moto_chassis[3].total_angle%8192)/8192.0)/WHEEL_RATIO*WHEEL_PI*WHEEL_D ; 
// 
//  s1_delta=s1-s1_last; //每个轮子转速的增量
//  s2_delta=s2-s2_last;
//  s3_delta=s3-s3_last;
//  s4_delta=s4-s4_last;
//  
//  //逆运动学模型转化为X、Y方向的位移以及航向角的变化
//  position_x_delta= 0.25*s1_delta+ 0.25*s2_delta+ 0.25*s3_delta+ 0.25*s4_delta;
//  position_y_delta = -0.25*s1_delta+ 0.25*s2_delta- 0.25*s3_delta+ 0.25*s4_delta;
//  position_w_delta = -K4_1*s1_delta-K4_1*s2_delta+K4_1*s3_delta+ K4_1*s4_delta; //w 单位为弧度
// 
//  //以上电时候的坐标作为里程计的全局坐标 
//  position_x=position_x+cos(position_w)*position_x_delta-sin(position_w)*position_y_delta;
//  position_y=position_y+sin(position_w)*position_x_delta+cos(position_w)*position_y_delta;
//  position_w=position_w+position_w_delta;
//  
//  if(position_w>2*WHEEL_PI)
//  {
//     position_w=position_w-2*WHEEL_PI;	
//  }
//  else if(position_w<-2*WHEEL_PI)
//  {
//     position_w=position_w+2*WHEEL_PI;
//  }
//  else;
// 
//  v1 =    (moto_chassis[0].speed_rpm)/WHEEL_RATIO/60.0*WHEEL_R *WHEEL_PI*2;
//  v2 =    (moto_chassis[1].speed_rpm)/WHEEL_RATIO/60.0*WHEEL_R *WHEEL_PI*2; 
//  v3 =    (moto_chassis[2].speed_rpm)/WHEEL_RATIO/60.0*WHEEL_R *WHEEL_PI*2; 
//  v4 =    (moto_chassis[3].speed_rpm)/WHEEL_RATIO/60.0*WHEEL_R *WHEEL_PI*2; 
//  
//  linear_x = 0.25*v1+ 0.25*v2+ 0.25*v3+ 0.25*v4;
//  linear_y = -0.25*v1+ 0.25*v2- 0.25*v3+ 0.25*v4;
//  linear_w = -K4_1*v1-K4_1*v2+K4_1*v3+ K4_1*v4;
//  
//  cout<<"position_x:  "<<position_x<<"   position_y: " <<position_y<<"   position_w: " <<position_w*57.3<<endl;
//  cout<<"linear_x:  "<<linear_x<<"   linear_y: " <<linear_y<<"   linear_w: " <<linear_w<<endl<<endl;
//    
//  publish_odomtery( position_x,position_y,position_w,linear_x,linear_y,linear_w);
//    //方法２;利用轮子的转速来推算
//}
