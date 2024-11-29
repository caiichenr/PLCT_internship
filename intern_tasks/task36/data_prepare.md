# 可抓取物体的数据采集与标注

## 确认可抓取物体类别

1. 苹果 (apple)
2. 立方体 (cube)
3. 圆形 (sphere)
4. 圆柱体 (cylinder)

## 采集情况

采集设备：Intel RealSense D415

| 数据编号 | 场景 | 日期 | 采集分辨率 | 备注 |
| --- | --- | --- | --- | --- |
| 01 | 地面1 | 20240924 |  | 有长方体作为干扰类 |
| 02 | 桌面1 + 桌面2 | 20241016 |  |  |

## 标注情况

| 文件名称 | 涵盖数据 | 手工标注/算法预标注 | 链接 |
| --- | --- | --- | --- |
| 训练集01 | 02 | 算法标注 | - |
| 测试集01 | 01 | 算法标注 | - |


## 数据集情况

| 数据集 | 训练集涵盖 | 测试集涵盖 | 链接 |
| --- | --- | --- | --- |
| datas1016 | 训练集01 | 测试集01 | [Google Drive](https://drive.google.com/file/d/1-MmeK4NscioGbMAXJuYQMneKJT9jJG5J/view?usp=drive_link)

## 标注工具

1. [Eiseg](https://github.com/PaddleCV-SIG/EISeg): 分割标注工具
2. [Cutie](https://github.com/hkchengrex/Cutie): 用于自动推理视频进行分割标注（有多种类似工具，根据个人习惯选择）
3. [RealSense 采集脚本](https://github.com/Incalos/Image-Capture-With-RealSense): 用于采集视频

## 标注 pipeline

1. 针对不同的场景进行数据采集，为便于标注可用视频形式采集；
2. 对于每一段视频，进行标注；
3. 由于视频重复度高，随机筛选适当量用作数据集。


