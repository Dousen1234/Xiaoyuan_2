# 离线依赖包放置目录

板上不能访问外网时，请在一台**联网的、同为 aarch64 / 同版本系统**的机器上预先下载 `.deb`，
然后把文件全部拷贝到本目录（`deploy/offline-debs/`）。

## 需要下载的包

```bash
cd deploy/offline-debs
apt-get download \
    build-essential gcc g++ make cmake pkg-config \
    libc6-dev \
    can-utils iproute2 usbutils \
    python3-serial \
    python3-openpyxl
```

> 注意：`apt-get download` 只下载**当前机器架构**的包。
> 如果下载机是 x86-64，下到的包在 aarch64 板上无法安装。
> 正确做法二选一：
> 1. 在一台 aarch64 机器上执行上面的命令；或
> 2. 指定架构：`apt-get download -o APT::Architecture=arm64 <包名>`

## 更稳妥的做法：连同依赖一起下

```bash
mkdir -p deploy/offline-debs && cd deploy/offline-debs
apt-get install --download-only --reinstall -y \
    build-essential cmake pkg-config \
    can-utils iproute2 usbutils python3-openpyxl
cp /var/cache/apt/archives/*.deb .
```

这样会把**完整依赖树**都下载下来，板上 `dpkg -i *.deb` 一次装完，不会缺依赖。

## 板上安装

把整个项目目录拷到板上后：

```bash
sudo ./deploy/install_env.sh
```

脚本检测到本目录有 `.deb` 时会优先离线安装，缺失部分再尝试联网 apt。

## macOS / Windows 拷贝注意事项

从 U 盘或 scp 拷进来后，确认文件权限正常：

```bash
ls -l deploy/offline-debs/*.deb
sudo chmod 644 deploy/offline-debs/*.deb
```