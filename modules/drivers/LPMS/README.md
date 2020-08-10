### LPMS Component

本驱动是IMU惯性导航模块驱动，使用LPMS-IG1惯性导航模块

## 输出

方向角，角速度，线加速度，磁感应强度，包含自动定位service

## 启动模块

```bash
# in docker
mainboard -d modules/drivers/LPMS/dag/LPMS.dag
```


