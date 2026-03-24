# grab_skill部署

## 依赖

1. SAM3分割模型:[https://github.com/facebookresearch/sam3](https://github.com/facebookresearch/sam3)
2. pyAgxArm机械臂驱动:[https://github.com/agilexrobotics/pyAgxArm](https://github.com/agilexrobotics/pyAgxArm)
3. realsense-ros驱动:[https://github.com/realsenseai/realsense-ros](https://github.com/realsenseai/realsense-ros)
4. 手眼标定:[https://github.com/agilexrobotics/Agilex-College/tree/master/piper/handeye](https://github.com/agilexrobotics/Agilex-College/tree/master/piper/handeye)

## 部署

```bash
conda create -n sam3 python=3.12
conda deactivate
conda activate sam3

pip install torch==2.7.0 torchvision torchaudio --index-url https://download.pytorch.org/whl/cu126  #改成适配你显卡和显卡驱动的版本

cd sam3
pip install -e .

# 修改sam3/realsense-sam.py L57-L65的手眼标定参数为自己的参数

# 测试程序
python realsense-sam.py

# A=主臂零力  D=普通模式+记录位姿  S=回零  X=复现位姿  Q=夹爪开  E=夹爪合  p=点云/抓取  t=改提示词  g=下发抓取  Esc=退出
```

## 如何使用

```bash
user>    请学习一个新技能：在目录下“your_openclaw_path/skills/grab_skill”

openclaw>   .......(“已经学会了”)

user>   进行测试，现在抓取桌面上的萝卜

openclaw>   .......(“成功抓取”)

```