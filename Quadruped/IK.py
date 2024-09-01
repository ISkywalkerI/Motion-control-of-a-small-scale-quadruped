import math

def ik(x, y, z):
    angle = []
    h= 0.01065
    hu = 0.07306
    hl = 0.05955

    lf1, lf2, lf3 = 0, 0, 0
    rf1, rf2, rf3 = 0, 0, 0
    lb1, lb2, lb3 = 0, 0, 0
    rb1, rb2, rb3 = 0, 0, 0

    for i in range(len(x)):
        # 计算 d 和 l
        d = math.sqrt(y[i]**2 + z[i]**2)
        l = math.sqrt(d**2 - h**2)
        
        # # 计算 γ2 和 γ1
        # gamma_2 = -math.atan2(y[i], z[i])
        # gamma_1 = -math.atan2(h, l)

        # # 计算 γ
        # gamma = gamma_2 - gamma_1 + math.pi
        

        v1 = y[i] / z[i]
        v2 = h / l
        gamma_2 = -math.atan(v1)
        gamma_1 = -math.atan(v2)
        gamma = gamma_2 - gamma_1

        # 计算 s
        s = math.sqrt(l**2 + x[i]**2)
        
        # 计算 n 和 β
        n = (s**2 - hl**2 - hu**2) / (2 * hu)
        print (f"s = {s} n = {n} hl = {hl}")
        beta = -math.acos(n / hl) + 1.851096
        
        # 计算 α1, α2 和 α
        alpha_1 = -math.atan2(x[i], l)
        alpha_2 = math.acos((hu + n) / s)
        alpha = alpha_2 + alpha_1 - 0.7010987
        
        # Assign values based on index
        if i == 0:  # Assuming first input is for lf
            lf1, lf2, lf3 = gamma, alpha, -beta
        elif i == 1:  # Assuming second input is for rf
            rf1, rf2, rf3 = -gamma, alpha, -beta
        elif i == 2:  # Assuming third input is for lb
            lb1, lb2, lb3 = -gamma, alpha, -beta
        elif i == 3:  # Assuming fourth input is for rb
            rb1, rb2, rb3 = gamma, alpha, -beta
    
        angle = {
            "lf1": lf1, "lf2": lf2, "lf3": lf3,
            "rf1": rf1, "rf2": rf2, "rf3": rf3,
            "lb1": lb1, "lb2": lb2, "lb3": lb3,
            "rb1": rb1, "rb2": rb2, "rb3": rb3
        }       
        
    return angle


# 示例输入
x = [0.02, 0.02, -0.02, -0.02]
y = [0.0, 0.0, 0.0, 0.0]
z = [-0.05, -0.10, -0.1074, -0.1074]

# 调用逆运动学函数
angles = ik(x, y, z)
print(angles)
# 输出结果