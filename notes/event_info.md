# Code Metal Forge 2026: "Operation Lost and Found"

来源: https://www.codemetal.ai/forge-hackathon 和 MIT CAPD 活动页 (2026-11-07 抓取); photos/2026-11-07_comms_and_submission.jpg

- 时间地点: 2026-11-06 至 11-08, BASE-X, 60 State St Suite 1400, Boston。USSOCOM 赞助, Code Metal 主办
- 主题: 危险环境中的搜救 (Search & Rescue)。核心问题: 在 GPS、通信、基础设施都不可用的情况下, 如何定位、识别并救出被困人员
- 四个目标 (不是分赛道):
  - OBJ.01 定位被困人员 (没有 GPS、蜂窝网络或固定通信)
  - OBJ.02 探测生命迹象和危险 (热、运动、声音; 气体、结构风险)
  - OBJ.03 绘制不断变化的态势 (火、天气、水、地形), 形成共享作战图
  - OBJ.04 在退化或拥塞的信道上把可用信息送达救援人员
- 评分: 任务相关性 / 能跑的现场演示 / 可部署性 (细则未公布)
- 队伍: 最多 4 人; 单人报名者在周六早上组队
- 提供的设备: 嵌入式计算和 MCU、SDR 和无线设备、成像/热成像/环境传感器、电子调试设备、合作方的开发套件和边缘计算
- 日程: 周六 08:00 kickoff, 10:00 开始开发, 通宵开放; **周日 11-08 12:00 提交截止 (硬性)**; 16:30 公布结果
- 奖金: $10k / $5k / $3k; 获胜队的大学生可获得 Code Metal 实习面试
- 提交: 每队一个 repo, 放在 github.com/CodeMetalForge-Hackathon org 下, repo 即提交物, 要经常 push
- Discord: discord.com/invite/codemetalforge-hackathon

## 现场硬件 (拍到的)
- Raspberry Pi 5 + Raspberry Pi AI HAT+ (Hailo-8, 26 TOPS), 装在带 Pi-Camera 开口的外壳里, 见 photos/2026-11-07_hw_rpi5_ai_hat_plus_26tops.jpg

## 主办方提供的硬件 (来源: data/organizer_email/Hackathon_Hardware_Details.pdf, 附在 2026-11-06 主办方邮件里)
- 每队基础套件: Pi 5 (16GB) + 26 TOPS AI HAT; Arduino 兼容 MCU + Grove 扩展板; Grove 40 合 1 传感器套件
- 共享借用库: Pi Zero / ESP32 / Jetson; 各类摄像头 (含深度相机和 NoIR 夜视); 热成像; 2D/3D LiDAR, ToF;
  GPS/IMU; SDR, Wi-Fi, BT, LoRa, UWB 测距; mmWave 雷达, 多普勒雷达, PIR; 气体等环境传感器; 麦克风;
  地面机器人; 微型无人机和 FPV 无人机; 示波器等测试设备
- 周六 08:45 主办方做了 mission overview 和技术问答; 具体场景简报还没有拿到
