# ![RTK logo](https://github.com/eluosir/ReconGNSSPosition/blob/main/logo.jpg) Real-time kinematic
基于最小二乘解算的短基线RTK定位，精度厘米级

感谢王甫红老师的指导！！

# 问题日志：
2022.12.27：SPV单点测速计算静态站点时速度不为0，Sigma较大，已修复

2023.03.28：计算BDS中GEO卫星时会导致SPP结果误差极大，导致RTK结果不可用，已修复

2023.04.14：无recv网络堵塞容错，已修复

2023.05.15：修复BDS卫星GEO位置计算问题
