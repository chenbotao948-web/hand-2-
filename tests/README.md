# 控制与陀螺仪软件回归

`run_tests.py` 用当前 Keil ARMCC 编译真实 `pid.c/usart.c/imu.c/icm42688.c`，
在 Unicorn Cortex-M3 仿真器执行；仅替换外设及输入，不将控制逻辑另写成 Python。
不访问或驱动实车。临时目标文件自动清理。

PowerShell 中运行：

```powershell
python -m pip install --target "$env:TEMP/2026myproject-test-deps" unicorn pyelftools
python tests/run_tests.py --deps "$env:TEMP/2026myproject-test-deps"
```

默认编译器位于 `D:/Keil_v5/ARM/ARMCC/bin`，可用 `--keil` 指定其他目录。

- control：门槛、丢线、左右容差边界、见线转 PID2、计时/角度回绕、停止/重启/手动模式、
  数据失效/过期/故障计数变化、串口非法参数、超长命令、TX 缓存回绕及拥堵停车；
  原地双向转向、±180°回绕、短时补偿、在线调容差、独立对准后直行及运行中接管。
- stage：主工程默认值、无等待/非零等待、CENTER→AUTO_ALIGN→ANGLE→EDGE、
  对准期间忽略白线、连续三拍到位、自动直行纠偏方向、在线容差清历史、
  超过20秒再切入的独立10秒对准超时、重复命令不重置超时、运行中保留起跑方向、
  独立目标与自动目标分离、无效IMU的入口停车、全部调参命令、IR回报和Stage=STOP拥堵停车。
- imu：静止校准、零偏扣除、90°/s 积分、真实 dt、断连/恢复/无新样本、计时回绕、运动中拒绝校准。
- driver：寄存器配置、0x1F 起始的12字节读取、SPI CS 释放、六轴换算及正常极值、
  六轴分别为 -32768、加速度全 -1、无效帧保持输出、数据未就绪、有限超时、身份检查失败。
  同时链接真实 imu/pid，验证异常帧不计入校准、不刷新样本时间/角度、短暂异常恢复、
  超过100ms持续异常触发停车，以及恢复校准后必须重新启动。

仿真不能验证实际 SPI 时序、传感器安装方向、车辆动力学或实机采样抖动。
