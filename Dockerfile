FROM ubuntu:22.04

ENV DEBIAN_FRONTEND=noninteractive
ENV container=docker

# 安装 systemd 和依赖
RUN apt-get update && apt-get install -y \
    systemd \
    systemd-sysv \
    git \
    python3 \
    python3-pip \
    python3-venv \
    sudo \
    curl \
    wget \
    && apt-get clean

# 清理不需要的 systemd 服务
RUN cd /lib/systemd/system/sysinit.target.wants/ && \
    rm -f $(ls | grep -v systemd-tmpfiles-setup) && \
    rm -f /lib/systemd/system/multi-user.target.wants/* && \
    rm -f /etc/systemd/system/*.wants/* && \
    rm -f /lib/systemd/system/local-fs.target.wants/* && \
    rm -f /lib/systemd/system/sockets.target.wants/*udev* && \
    rm -f /lib/systemd/system/sockets.target.wants/*initctl* && \
    rm -f /lib/systemd/system/basic.target.wants/* && \
    rm -f /lib/systemd/system/anaconda.target.wants/*

# 创建非 root 用户
RUN useradd -m -s /bin/bash kiauh && \
    echo "kiauh ALL=(ALL) NOPASSWD:ALL" >> /etc/sudoers

# 复制项目
COPY --chown=kiauh:kiauh . /home/kiauh/kiauh

VOLUME ["/sys/fs/cgroup"]
STOPSIGNAL SIGRTMIN+3

CMD ["/sbin/init"]
