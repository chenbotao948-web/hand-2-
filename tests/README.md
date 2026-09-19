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
  数据失效/过期/故障计数变化、串口非法参数、超长命令、TX 缓存回绕及拥堵停车。
- imu：静止校准、零偏扣除、90°/s 积分、真实 dt、断连/恢复/无新样本、计时回绕、运动中拒绝校准。
- driver：寄存器配置、0x1F 起始的12字节读取、SPI CS 释放、六轴换算及正常极值、
  六轴分别为 -32768、加速度全 -1、无效帧保持输出、数据未就绪、有限超时、身份检查失败。
  同时链接真实 imu/pid，验证异常帧不计入校准、不刷新样本时间/角度、短暂异常恢复、
  超过100ms持续异常触发停车，以及恢复校准后必须重新启动。

仿真不能验证实际 SPI 时序、传感器安装方向、车辆动力学或实机采样抖动。
