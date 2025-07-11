
https://blog.csdn.net/lanlinjnc/article/details/136709225

# 建议将上面这一行写入 ~/.bashrc。若没有写入，则每次下载时都需要先输入该命令
export HF_ENDPOINT=https://hf-mirror.com  
# shujuji download
huggingface-cli download --repo-type dataset --resume-download fleaven/Retargeted_AMASS_for_robotics --local-dir /home/z/code/data/Retargeted_AMASS_for_robotics --local-dir-use-symlinks False


https://zhuanlan.zhihu.com/p/663712983

./script/hfd.sh  fleaven/Retargeted_AMASS_for_robotics   --dataset  --tool wget

huggingface-cli download --repo-type dataset --resume-download fleaven/Retargeted_AMASS_for_robotics --local-dir /home/z/code/data/Retargeted_AMASS_for_robotics