import math

import numpy as np
import walk1 
x1=0;x2=0;x3=0;x4=0;y1=0;y2=0;y3=0;y4=0
def swing_curve_generate(t,Tf,xt,zh,x0,z0,xv0):
  #输入参数：当前时间；支撑相占空比；x方向目标位置，z方向抬腿高度，摆动相抬腿前腿速度
  # X Generator
  if t>=0 and t<Tf/4:
    xf=(-4*xv0/Tf)*t*t+xv0*t+x0
    
  if t>=Tf/4 and t<(3*Tf)/4:
    xf=((-4*Tf*xv0-16*xt+16*x0)*t*t*t)/(Tf*Tf*Tf)+((7*Tf*xv0+24*xt-24*x0)*t*t)/(Tf*Tf)+((-15*Tf*xv0-36*xt+36*x0)*t)/(4*Tf)+(9*Tf*xv0+16*xt)/(16)
    
  if t>(3*Tf)/4:
    xf=xt

  # Z Generator
  if t>=0 and t<Tf/2:
    zf=(16*z0-16*zh)*t*t*t/(Tf*Tf*Tf)+(12*zh-12*z0)*t*t/(Tf*Tf)+z0
  
  if t>=Tf/2:
    zf=(4*z0-4*zh)*t*t/(Tf*Tf)-(4*z0-4*zh)*t/(Tf)+z0
      
  #Record touch down position
  x_past=xf
  t_past=t
  
  # # Avoid zf to go zero
  #x,z position,x_axis stop point,t_stop point;depend on when the leg stop
  
  return xf,zf,x_past,t_past

def support_curve_generate(t,Tf,x_past,t_past,zf):
  # 当前时间；支撑相占空比；摆动相 x 最终位;t最终时间，支撑相 zf
  # Only X Generator
  average=x_past/(1-Tf)
  xf=x_past-average*(t-t_past)
  return xf,zf

def walkt(t,x_target,z_target):
  global x1,x2,x3,x4,y1,y2,y3,y4
  Tf=0.5
  if t<Tf:
    
      
      phase_w_swing=swing_curve_generate(t,Tf,x_target,z_target,0,0,0)
      print (f"1 {phase_w_swing} t = {t}")
      x1=phase_w_swing[0];x2=0;x3=0;x4=0
      y1=phase_w_swing[1];y2=0;y3=0;y4=0

  if t>=Tf and t<2*Tf:

      
      phase_w_swing=swing_curve_generate(t-0.5,Tf,x_target,z_target,0,0,0)
      print (f"2 {phase_w_swing} t = {t}")
      x1=x_target;x2=phase_w_swing[0];x3=0;x4=0
      y1=0;y2=phase_w_swing[1];y3=0;y4=0
  
  if t>=2*Tf and t<3*Tf:
      
      phase_w_swing=swing_curve_generate(t-1,Tf,x_target,z_target,0,0,0)
      print (f"3 {phase_w_swing} t = {t}")
      x1=x_target;x2=x_target;x3=phase_w_swing[0];x4=0
      y1=0;y2=0;y3=phase_w_swing[1];y4=0

  if t>=3*Tf and t<4*Tf:
      
      phase_w_swing=swing_curve_generate(t-1.5,Tf,x_target,z_target,0,0,0)
      print (f"4 {phase_w_swing} t = {t}")
      x1=x_target;x2=x_target;x3=x_target;x4=phase_w_swing[0]
      y1=0;y2=0;y3=0;y4=phase_w_swing[1]

  if t>=4*Tf:
     
     phase_w_support=support_curve_generate(t-1.5,Tf,x_target,0.5,0.04)
     print (f"5 {phase_w_support} t = {t}")
     x1=phase_w_support[0];x2=phase_w_support[0];x3=phase_w_support[0];x4=phase_w_support[0]
     y1=phase_w_support[1];y2=phase_w_support[1];y3=phase_w_support[1];y4=phase_w_support[1]
    

  x = [x1, x2, x3, x4]
  y = [y1, y2, y3, y4]
  z = [z_target, z_target, z_target, z_target]  


  return x, y, z



def swing_phase(xf, xs, yf, ys, zs, h, t, Ts, fai):
    # 计算 σ
    sigma = (2 * math.pi * t) / ((1-fai) * Ts)
    
    # 计算 x_exp, y_exp, z_exp
    x_exp = (xf - xs) * (sigma - math.sin(sigma)) / (2 * math.pi) + xs
    y_exp = (yf - ys) * (sigma - math.sin(sigma)) / (2 * math.pi) + ys
    z_exp = h * (1 - math.cos(sigma)) / 2 + zs
    
    return x_exp, y_exp, z_exp

def support_phase(xf, xs, yf, ys, zs, h, t, Ts, fai):# 计算 σ
    sigma = (2 * math.pi * t) / (fai * Ts)
    
    # 计算 x_exp, y_exp, z_exp
    x_exp = (xs - xf) * (sigma - math.sin(sigma)) / (2 * math.pi) + xf
    y_exp = (ys - yf) * (sigma - math.sin(sigma)) / (2 * math.pi) + yf
    z_exp = zs
    
    return x_exp, y_exp, z_exp


def walk_rotate(xf, xs, yf, ys, zs, h, t, Ts, turn_factor):
    fai = 0.75

    # 计算转弯偏移，turn_factor可以是一个[-1, 1]的值，负值表示左转，正值表示右转
    delta_x = turn_factor * 0.02  # 控制转弯的幅度
    delta_y = turn_factor * 0.01  # 控制转弯时前后脚的差异

    t3 = t - Ts * 0.25

    if t < Ts * 0.25:
        t1 = t + Ts * 0.50
        t4 = t + Ts * 0.25
        swing = swing_phase(xf + delta_x, xs, yf + delta_y, ys, zs, h, t, Ts, fai)
        support1 = support_phase(xf + delta_x, xs, yf + delta_y, ys, zs, h, t1, Ts, fai)
        support2 = support_phase(xf - delta_x, xs, yf - delta_y, ys, zs, h, t, Ts, fai)
        support4 = support_phase(xf - delta_x, xs, yf - delta_y, ys, zs, h, t4, Ts, fai)
        x1 = support1[0]; x2 = support2[0]; x3 = swing[0]; x4 = support4[0]
        y1 = support1[1]; y2 = support2[1]; y3 = swing[1]; y4 = support4[1]
        z1 = support1[2]; z2 = support2[2]; z3 = swing[2]; z4 = support4[2]

    elif t >= Ts * 0.25 and t < Ts * 0.5:
        tswing = t - Ts * 0.25
        t4 = t + Ts * 0.25
        swing = swing_phase(xf + delta_x, xs, yf + delta_y, ys, zs, h, tswing, Ts, fai)
        support2 = support_phase(xf - delta_x, xs, yf - delta_y, ys, zs, h, t, Ts, fai)
        support3 = support_phase(xf + delta_x, xs, yf + delta_y, ys, zs, h, t3, Ts, fai)
        support4 = support_phase(xf - delta_x, xs, yf - delta_y, ys, zs, h, t4, Ts, fai)
        x1 = swing[0]; x2 = support2[0]; x3 = support3[0]; x4 = support4[0]
        y1 = swing[1]; y2 = support2[1]; y3 = support3[1]; y4 = support4[1]
        z1 = swing[2]; z2 = support2[2]; z3 = support3[2]; z4 = support4[2]

    elif t >= Ts * 0.5 and t < Ts * 0.75:
        tswing = t - Ts * 0.5
        t1 = t - Ts * 0.5
        swing = swing_phase(xf - delta_x, xs, yf - delta_y, ys, zs, h, tswing, Ts, fai)
        support1 = support_phase(xf + delta_x, xs, yf + delta_y, ys, zs, h, t1, Ts, fai)
        support2 = support_phase(xf - delta_x, xs, yf - delta_y, ys, zs, h, t, Ts, fai)
        support3 = support_phase(xf + delta_x, xs, yf + delta_y, ys, zs, h, t3, Ts, fai)
        x1 = support1[0]; x2 = support2[0]; x3 = support3[0]; x4 = swing[0]
        y1 = support1[1]; y2 = support2[1]; y3 = support3[1]; y4 = swing[1]
        z1 = support1[2]; z2 = support2[2]; z3 = support3[2]; z4 = swing[2]

    elif t >= Ts * 0.75 and t < Ts:
        tswing = t - Ts * 0.75
        t1 = t - Ts * 0.5
        t4 = t - Ts * 0.75
        swing = swing_phase(xf - delta_x, xs, yf - delta_y, ys, zs, h, tswing, Ts, fai)
        support1 = support_phase(xf + delta_x, xs, yf + delta_y, ys, zs, h, t1, Ts, fai)
        support3 = support_phase(xf + delta_x, xs, yf + delta_y, ys, zs, h, t3, Ts, fai)
        support4 = support_phase(xf - delta_x, xs, yf - delta_y, ys, zs, h, t4, Ts, fai)
        x1 = support1[0]; x2 = swing[0]; x3 = support3[0]; x4 = support4[0]
        y1 = support1[1]; y2 = swing[1]; y3 = support3[1]; y4 = support4[1]
        z1 = support1[2]; z2 = swing[2]; z3 = support3[2]; z4 = support4[2]

    x = [x1, x2, x3, x4]
    y = [y1, y2, y3, y4]
    z = [z1, z2, z3, z4]

    return x, y, z

def trot(xf, xs, yf, ys, zs, h, t, Ts,turn_factor):
  fai = 0.5

  delta_x = turn_factor * 0.02  # 控制转弯的幅度
  delta_y = turn_factor * 0.01  # 控制转弯时前后脚的差异

  if t < Ts * fai:
    swing1 = swing_phase(xf + delta_x, xs, yf + delta_y, ys, zs, h, t, Ts, fai)
    swing4 = swing_phase(xf - delta_x, xs, yf - delta_y, ys, zs, h, t, Ts, fai)
    support2 = support_phase(xf - delta_x, xs, yf - delta_y, ys, zs, h, t, Ts, fai)
    support3 = support_phase(xf + delta_x, xs, yf + delta_y, ys, zs, h, t, Ts, fai)
    x1 = swing1[0];x2 = support2[0];x3 = support3[0];x4 = swing4[0]
    y1 = swing1[1];y2 = support2[1];y3 = support3[1];y4 = swing4[1]
    z1 = swing1[2];z2 = support2[2];z3 = support3[2];z4 = swing4[2]

  if t >= Ts * fai and t<Ts:
    t2 = t-Ts*fai
    swing2 = swing_phase(xf - delta_x, xs, yf - delta_y, ys, zs, h,t2, Ts, fai)
    swing3 = swing_phase(xf + delta_x, xs, yf + delta_y, ys, zs, h,t2, Ts, fai)
    support1 = support_phase(xf + delta_x, xs, yf + delta_y, ys, zs, h, t2, Ts, fai)
    support4 = support_phase(xf - delta_x, xs, yf - delta_y, ys, zs, h, t2, Ts, fai)
    x1 = support1[0];x2 = swing2[0];x3 = swing3[0];x4 = support4[0]
    y1 = support1[1];y2 = swing2[1];y3 = swing3[1];y4 = support4[1]
    z1 = support1[2];z2 = swing2[2];z3 = swing3[2];z4 = support4[2]

  x = [x1, x2, x3, x4]
  y = [y1, y2, y3, y4]
  z = [z1, z2, z3, z4]  

  return x, y, z

def  walk(xf, xs, yf, ys, zs, h, t, Ts):
  fai = 0.75

  t3 = t - Ts * 0.25

  if t < Ts * 0.25 :
    t1 = t + Ts * 0.50
    t4 = t + Ts * 0.25
    swing = swing_phase(xf, xs, yf, ys, zs, h, t, Ts, fai)
    support1 = support_phase(xf, xs, yf, ys, zs, h, t1, Ts, fai)
    support2 = support_phase(xf, xs, yf, ys, zs, h, t, Ts, fai)
    
    support4 = support_phase(xf, xs, yf, ys, zs, h, t4, Ts, fai)
    x1 = support1[0];x2 = support2[0];x3 = swing[0];x4 = support4[0]
    y1 = support1[1];y2 = support2[1];y3 = swing[1];y4 = support4[1]
    z1 = support1[2];z2 = support2[2];z3 = swing[2];z4 = support4[2]
  
  if t >= Ts * 0.25 and t < Ts * 0.5:
    tswing = t - Ts * 0.25
    t4 = t + Ts * 0.25
    swing = swing_phase(xf, xs, yf, ys, zs, h,tswing, Ts, fai)
    
    support2 = support_phase(xf, xs, yf, ys, zs, h, t, Ts, fai)
    support3 = support_phase(xf, xs, yf, ys, zs, h, t3, Ts, fai)
    support4 = support_phase(xf, xs, yf, ys, zs, h, t4, Ts, fai)
    x1 = swing[0];x2 = support2[0];x3 = support3[0];x4 = support4[0]
    y1 = swing[1];y2 = support2[1];y3 = support3[1];y4 = support4[1]
    z1 = swing[2];z2 = support2[2];z3 = support3[2];z4 = support4[2]

  if t >= Ts * 0.5 and t < Ts * 0.75:
    tswing = t - Ts * 0.5
    t1 = t - Ts * 0.5
    swing = swing_phase(xf, xs, yf, ys, zs, h,tswing, Ts, fai)
    support1 = support_phase(xf, xs, yf, ys, zs, h, t1, Ts, fai)
    support2 = support_phase(xf, xs, yf, ys, zs, h, t, Ts, fai)
    support3 = support_phase(xf, xs, yf, ys, zs, h, t3, Ts, fai)
    
    x1 = support1[0];x2 = support2[0];x3 = support3[0];x4 = swing[0]
    y1 = support1[1];y2 = support2[1];y3 = support3[1];y4 = swing[1]
    z1 = support1[2];z2 = support2[2];z3 = support3[2];z4 = swing[2]

  if t >= Ts * 0.75 and t < Ts :
    tswing = t - Ts * 0.75
    t1 = t - Ts * 0.5
    t4 = t - Ts * 0.75
    swing = swing_phase(xf, xs, yf, ys, zs, h,tswing, Ts, fai)
    support1 = support_phase(xf, xs, yf, ys, zs, h, t1, Ts, fai)
    
    support3 = support_phase(xf, xs, yf, ys, zs, h, t3, Ts, fai)
    support4 = support_phase(xf, xs, yf, ys, zs, h, t4, Ts, fai)
    x1 = support1[0];x2 = swing[0];x3 = support3[0];x4 = support4[0]
    y1 = support1[1];y2 = swing[1];y3 = support3[1];y4 = support4[1]
    z1 = support1[2];z2 = swing[2];z3 = support3[2];z4 = support4[2]

    
  x = [x1, x2, x3, x4]
  y = [y1, y2, y3, y4]
  z = [z1, z2, z3, z4]  

  return x, y, z


def  walktest(xf, xs, yf, ys, zs, h, t, Ts):
  fai = 0.75

  if t < Ts * 0.25 :
    swing = swing_phase(xf, xs, yf, ys, zs, h, t, Ts, fai)
    support = support_phase(xf, xs, yf, ys, zs, h, t, Ts, fai)
    x1 = support[0];x2 = support[0];x3 = swing[0];x4 = support[0]
    y1 = support[1];y2 = support[1];y3 = swing[1];y4 = support[1]
    z1 = support[2];z2 = support[2];z3 = swing[2];z4 = support[2]
  
  if t >= Ts * 0.25 and t < Ts * 0.5:
    tswing = t - Ts * 0.25
    swing = swing_phase(xf, xs, yf, ys, zs, h,tswing, Ts, fai)
    support = support_phase(xf, xs, yf, ys, zs, h, t, Ts, fai)
    x1 = swing[0];x2 = support[0];x3 = support[0];x4 = support[0]
    y1 = swing[1];y2 = support[1];y3 = support[1];y4 = support[1]
    z1 = swing[2];z2 = support[2];z3 = support[2];z4 = support[2]

  if t >= Ts * 0.5 and t < Ts * 0.75:
    tswing = t - Ts * 0.5
    swing = swing_phase(xf, xs, yf, ys, zs, h,tswing, Ts, fai)
    support = support_phase(xf, xs, yf, ys, zs, h, t, Ts, fai)
    
    x1 = support[0];x2 = support[0];x3 = support[0];x4 = swing[0]
    y1 = support[1];y2 = support[1];y3 = support[1];y4 = swing[1]
    z1 = support[2];z2 = support[2];z3 = support[2];z4 = swing[2]

  if t >= Ts * 0.75 and t < Ts :
    tswing = t - Ts * 0.75
    tsupport = tswing
    swing = swing_phase(xf, xs, yf, ys, zs, h,tswing, Ts, fai)
    support = support_phase(xf, xs, yf, ys, zs, h, tsupport, Ts, fai)
    x1 = support[0];x2 = swing[0];x3 = support[0];x4 = support[0]
    y1 = support[1];y2 = swing[1];y3 = support[1];y4 = support[1]
    z1 = support[2];z2 = swing[2];z3 = support[2];z4 = support[2]

    
  x = [x1, x2, x3, x4]
  y = [y1, y2, y3, y4]
  z = [z1, z2, z3, z4]  

  return x, y, z