# voice_skill部署

## 依赖

1. [CosyVoice](https://github.com/FunAudioLLM/CosyVoice)

## 部署

```bash
    git clone --recursive https://github.com/FunAudioLLM/CosyVoice.git
    # If you failed to clone the submodule due to network failures, please run the following command until success
    cd CosyVoice
    git submodule update --init --recursive

    conda create -n cosyvoice -y python=3.10
    conda activate cosyvoice
    pip install -r requirements.txt -i https://mirrors.aliyun.com/pypi/simple/ --trusted-host=mirrors.aliyun.com

    # If you encounter sox compatibility issues
    # ubuntu
    sudo apt-get install sox libsox-dev

    # modelscope SDK model download
    from modelscope import snapshot_download
    snapshot_download('FunAudioLLM/Fun-CosyVoice3-0.5B-2512', local_dir='pretrained_models/Fun-CosyVoice3-0.5B')

```

## 如何使用

```bash
user>    请学习一个新技能：在目录下“your_openclaw_path/skills/voice_skill”

openclaw>   .......(“已经学会了”)

user>   进行测试，现在开始以后的每次会话都要求使用voice_skill语音播报

openclaw>   .......(“成功”)

```