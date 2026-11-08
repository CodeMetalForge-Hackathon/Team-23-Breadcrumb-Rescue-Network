# 项目构想 (2026-11-07 讨论稿)

## 名称: SIGNAL OF LIFE: "The network that finds you"
一句话: 救援队为了通信部署的 mesh 网络, 本身就是搜救传感器 (ISAC = Integrated Sensing and Communication, 通信感知一体化)

## 三层
1. 感知 (ISAC): mesh 节点被动监听被困者手机的 Wi-Fi probe request / BLE 广播 -> 根据多节点 RSSI 定位; 可选 Wi-Fi CSI 检测呼吸和运动
2. 确认 (边缘 AI): Pi 5 + Hailo-8 摄像头做人体检测
3. 传递 (分层 mesh): Persistent MPU5 / Wi-Fi mesh 作为高带宽层; Meshtastic LoRa 作为兜底层; 语义压缩 (每条消息 <100 B); 根据链路质量自动选层, 断网时存储转发; 输出为 CoT 消息, 显示在 ATAK 上

## 演示脚本
- 另一个房间藏一部"被困者"手机; 3 到 4 个节点放在角落
- ATAK 上出现估计位置和误差圈 -> 节点靠近, 摄像头确认 -> 标点升级为"已确认, 1 人"
- 拔掉高带宽链路 -> 消息改走 LoRa 仍能送达, 显示字节数和延迟对比

## 待确认
现场有没有 MPU5 / LoRa / ESP32 / Pi 摄像头; 场景是否假设被困者带手机; mentor 是否推荐用 ATAK
