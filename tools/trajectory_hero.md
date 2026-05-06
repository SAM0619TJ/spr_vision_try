# Trajectory_Hero 弹道解算推导

英雄（42 mm 弹丸）距离较远、初速较低，必须考虑空气阻力。本方案在 `Trajectory`（无阻力解析二次方程）基础上引入水平方向二次阻力，并以牛顿迭代求解满足落点条件的仰角 $\theta$。

## 1. 阻力模型

弹丸在空气中受到的阻力沿速度反方向，大小：

$$
F_{\text{drag}} = \tfrac12 \rho C_d A v^2
$$

定义阻力系数

$$
k_1 \;=\; \frac{\rho C_d A}{2 m}
\quad\Longrightarrow\quad
\dot{\vec v} \;=\; -\,k_1\,\lVert\vec v\rVert\,\vec v + \vec g
$$

其中 $\rho = 1.225\ \text{kg/m}^3$、$C_d \approx 0.47$（球形弹丸）、$A = \pi r^2$（$r = 0.021$ m）、$m = 0.042$ kg，得 $k_1 \approx 9.5\times10^{-3}\ \text{m}^{-1}$。

## 2. 水平方向：解析积分

为了得到闭式解，**对水平分量做近似解耦**：忽略 $v_y$ 对水平阻力大小的贡献，认为

$$
\dot v_x \;=\; -k_1\,v_x^2
$$

初值 $v_x(0) = v_0\cos\theta$。分离变量积分：

$$
v_x(t) \;=\; \frac{v_0\cos\theta}{1 + k_1\,v_0\cos\theta\cdot t},\qquad
x(t) \;=\; \frac{1}{k_1}\ln\!\bigl(1 + k_1\,v_0\cos\theta\cdot t\bigr)
$$

由 $x(T) = d$ 反解出飞行时间：

$$
\boxed{\,T(\theta) \;=\; \frac{e^{k_1 d} - 1}{k_1\,v_0\cos\theta}\,}
$$

记常数

$$
C \;\triangleq\; \frac{e^{k_1 d} - 1}{k_1\,v_0}
\quad\Longrightarrow\quad
T(\theta) \;=\; \frac{C}{\cos\theta}
$$

## 3. 垂直方向：忽略垂直阻力的近似

精确的 $v_y$ 方程

$$
\dot v_y \;=\; -g - k_1\,v\,v_y
$$

不存在闭式解。本方案做进一步近似：**垂直方向不计阻力**，仍按自由落体处理：

$$
z(T) \;=\; v_0\sin\theta\cdot T - \tfrac12 g T^2
$$

代入 $T = C/\cos\theta$：

$$
z(\theta) \;=\; v_0\,C\,\tan\theta - \tfrac12 g\,\frac{C^2}{\cos^2\theta}
$$

落点残差函数：

$$
\boxed{\,f(\theta) \;=\; h - v_0\,C\,\tan\theta + \tfrac12 g\,\frac{C^2}{\cos^2\theta}\,}
$$

求解 $f(\theta) = 0$ 即得满足落点高度 $h$ 的仰角 $\theta$。

## 4. 牛顿迭代

对 $f(\theta)$ 解析求导：

$$
f'(\theta) \;=\; -\frac{v_0\,C}{\cos^2\theta} + \frac{g\,C^2\,\sin\theta}{\cos^3\theta}
$$

迭代格式：

$$
\theta_{n+1} \;=\; \theta_n - \frac{f(\theta_n)}{f'(\theta_n)}
$$

实现细节：

- 初值：$\theta_0 = \operatorname{atan2}(h,\,d)$；
- 每步对 $\Delta\theta$ 做 clamp（上限 0.1 rad），防止远离根时发散；
- 对 $\theta$ 做 clamp（$|\theta| \le 1.4$ rad）保留在物理合理区间；
- 收敛判据：$|f| < 10^{-5}$ m；
- 异常处理：$|\cos\theta| < 10^{-3}$、$|f'| < 10^{-12}$、超过 30 次仍未收敛 → 置 `unsolvable = true`。

## 5. 适用范围与局限

适用：典型英雄交战距离（$d \lesssim 12$ m），仰角 $|\theta| \lesssim 30^\circ$，落点厘米级精度。

局限：

1. 垂直阻力被忽略，远距离 + 大仰角时高度估计偏低（实际落点比模型更靠下）。
2. 水平方向用 $\dot v_x = -k_1 v_x^2$ 而非完整的 $-k_1 v\,v_x$，对小仰角误差小，对大仰角偏乐观。
3. 牛顿迭代仅追求一个解（由初值决定），未显式区分直射 / 吊射。

如需更高精度，可将 $f(\theta)$ 改为对二维弹道做 RK4 数值积分后的 $z(T) - h$，外层仍用牛顿/二分迭代仰角。
