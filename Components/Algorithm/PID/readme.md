# 关于SmartPID的简单说明
## SmartPID的目标：
- 1. 实现通过外加部分优化算法使得PID能更加的优秀
- 2. 要求有足够良好的可拓展性，能够应对未来的添加和维护

## 食用指北
- 1. 通过下面的三个函数中的一个实现初始化
```c++
PID()//默认初构造函数，所有参数为0
PID(float p, float i, float d)//最普通的PID，鉴定为单细胞生物
PID(float p, float i, float d, float max_sum, float max_i, float delta, //重要参数，delta用于限制PID输出变化速度
    uint16_t ctr, //控制代码，控制启用和禁用功能
    float (*f)(float err), float (*flt)(float output)) //两个回调函数，通过ctr启用
```
- 2. 在开发过程中有两个需要调用的函数，调用返回输出值
```c++
PID_calculate(float current, float target)//最普通的PID，一点不智能
Smart_PID_calculate(float current, float target)//加入了智能要素的PID
```