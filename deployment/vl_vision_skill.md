# vl_vision_skill部署

## 依赖

1. [ollama本地部署视觉语言模型](https://ollama.com/search)
2. [pyrealsense2](https://pypi.org/project/pyrealsense2/)

## 部署

```bash
conda create -n my_skills python=3.12

conda activate my_skills

pip install pyrealsense2


```

## 如何使用

```bash
user>    请学习一个新技能：在目录下“your_openclaw_path/skills/vl_vision_skill”

openclaw>   .......(“已经学会了”)

user>   进行测试，现在说说看你看到了什么

openclaw>   .......(“成功调用qwen3.5进行图像推理”)

```