#!/bin/bash

# 确保挂载点目录存在
if [ ! -d /mnt/hgfs ]; then
    echo "创建挂载点 /mnt/hgfs"
    sudo mkdir -p /mnt/hgfs
fi

# 如果尚未挂载，则执行挂载
if ! mountpoint -q /mnt/hgfs; then
    echo "正在挂载共享文件夹..."
    sudo mount -t fuse.vmhgfs-fuse .host:/ /mnt/hgfs -o allow_other
else
    echo "/mnt/hgfs 已经挂载，跳过挂载步骤"
fi

# 检查源文件是否存在
if [ ! -f build/MicoAir743v2/bin/arducopter.apj ]; then
    echo "错误：源文件 build/MicoAir743v2/bin/arducopter.apj 不存在"
    exit 1
fi

# 复制文件
echo "正在复制文件到共享文件夹..."
cp build/MicoAir743v2/bin/arducopter.apj /mnt/hgfs/SharedFolder/arducopter.apj

if [ $? -eq 0 ]; then
    echo "✅ 文件复制成功：/mnt/hgfs/SharedFolder/arducopter.apj"
else
    echo "❌ 文件复制失败"
    exit 1
fi
