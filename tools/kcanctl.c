/* kcanctl.c
 * 鲲弘 USB2CAN 通道控制工具
 *
 * 功能:
 *   - 列出所有 CAN 通道及状态 (up/down) 与波特率
 *   - 打开指定通道并配置波特率 (经典 CAN / CAN FD)
 *   - 关闭指定通道 / 关闭所有通道
 *
 * 用法:
 *   kcanctl list                        列出所有 CAN 通道及状态
 *   kcanctl down-all                    关闭所有通道 (插入 USB 后想全关时运行)
 *   kcanctl up <通道> <波特率>           打开通道并设置波特率
 *                                       例: kcanctl up can0 1000000
 *   kcanctl down <通道>                 关闭通道
 *                                       例: kcanctl down can0
 *   kcanctl fd <通道> <仲裁> <数据>      以 CAN FD 波特率打开
 *                                       例: kcanctl fd can0 1000000 2000000
 *
 * 说明:
 *   - 修改波特率前必须先 down, 本工具已自动处理 (先 down 再配再 up)
 *   - 需要 root 权限运行 (sudo kcanctl ...)
 *   - 通道名形如 can0 / can1 / can2 ..., 对应设备各路通道
 *
 * 编译:
 *   gcc -O2 -o kcanctl kcanctl.c
 *   sudo install -m 755 kcanctl /usr/local/bin/
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdarg.h>
#include <dirent.h>
#include <unistd.h>

#define MAX_CH 64

/* 执行 shell 命令, 返回退出码 */
static int run(const char *fmt, ...)
{
    char cmd[512];
    va_list ap;
    va_start(ap, fmt);
    vsnprintf(cmd, sizeof cmd, fmt, ap);
    va_end(ap);
    int rc = system(cmd);
    return WEXITSTATUS(rc);
}

/* 读取 /sys/class/net/<ch>/flags, 判断 IFF_UP (bit0), 得到 UP/DOWN
 * 注: CAN 接口的 operstate 常为 unknown, 不可靠, 故直接查 flags */
static int read_state(const char *ch, char *out, size_t n)
{
    char path[256];
    snprintf(path, sizeof path, "/sys/class/net/%s/flags", ch);
    FILE *f = fopen(path, "r");
    if (!f)
        return -1;
    unsigned int flags = 0;
    if (fscanf(f, "%x", &flags) != 1) {
        fclose(f);
        return -1;
    }
    fclose(f);
    snprintf(out, n, "%s", (flags & 0x1) ? "UP" : "DOWN");
    return 0;
}

/* 扫描 /sys/class/net/, 收集所有 can* 通道名 */
static int list_channels(char chs[MAX_CH][64], int *n)
{
    DIR *d = opendir("/sys/class/net");
    if (!d)
        return -1;
    struct dirent *e;
    *n = 0;
    while ((e = readdir(d)) && *n < MAX_CH) {
        if (strncmp(e->d_name, "can", 3) == 0) {
            strncpy(chs[*n], e->d_name, 63);
            chs[*n][63] = '\0';
            (*n)++;
        }
    }
    closedir(d);
    return 0;
}

/* 用 ip -details 读取通道当前波特率 (经典 CAN bitrate) */
static void read_bitrate(const char *ch, char *out, size_t n)
{
    char cmd[256];
    snprintf(cmd, sizeof cmd,
             "ip -details link show %s 2>/dev/null | "
             "grep -o 'bitrate [0-9]*' | head -1 | awk '{print $2}'",
             ch);
    FILE *f = popen(cmd, "r");
    if (!f) {
        snprintf(out, n, "-");
        return;
    }
    if (!fgets(out, (int)n, f))
        snprintf(out, n, "-");
    pclose(f);
    out[strcspn(out, "\r\n")] = 0;
    if (out[0] == 0)
        snprintf(out, n, "-");
}

static int cmd_list(void)
{
    char chs[MAX_CH][64];
    int n = 0;
    if (list_channels(chs, &n) != 0) {
        fprintf(stderr, "无法读取 /sys/class/net\n");
        return 1;
    }
    if (n == 0) {
        printf("未发现任何 CAN 通道 (设备未插入?)\n");
        return 0;
    }
    printf("%-8s %-8s %-12s\n", "通道", "状态", "波特率");
    for (int i = 0; i < n; i++) {
        char state[32], rate[32];
        read_state(chs[i], state, sizeof state);
        read_bitrate(chs[i], rate, sizeof rate);
        printf("%-8s %-8s %-12s\n", chs[i], state, rate);
    }
    return 0;
}

static int cmd_down_all(void)
{
    char chs[MAX_CH][64];
    int n = 0;
    if (list_channels(chs, &n) != 0) {
        fprintf(stderr, "无法读取 /sys/class/net\n");
        return 1;
    }
    if (n == 0) {
        printf("未发现任何 CAN 通道\n");
        return 0;
    }
    for (int i = 0; i < n; i++) {
        if (run("ip link set %s down", chs[i]) == 0)
            printf("已关闭 %s\n", chs[i]);
        else
            fprintf(stderr, "关闭 %s 失败\n", chs[i]);
    }
    return 0;
}

static int cmd_down(const char *ch)
{
    if (run("ip link set %s down", ch) != 0) {
        fprintf(stderr, "关闭 %s 失败 (通道不存在?)\n", ch);
        return 1;
    }
    printf("已关闭 %s\n", ch);
    return 0;
}

/* 打开通道并设置经典 CAN 波特率: 先 down, 再配 bitrate, 再 up */
static int cmd_up(const char *ch, unsigned long bitrate)
{
    if (bitrate == 0) {
        fprintf(stderr, "波特率无效\n");
        return 1;
    }
    if (run("ip link set %s down", ch) != 0) {
        fprintf(stderr, "无法 down %s (通道不存在?)\n", ch);
        return 1;
    }
    if (run("ip link set %s type can bitrate %lu", ch, bitrate) != 0) {
        fprintf(stderr, "配置 %s 波特率 %lu 失败\n", ch, bitrate);
        return 1;
    }
    if (run("ip link set %s up", ch) != 0) {
        fprintf(stderr, "无法 up %s\n", ch);
        return 1;
    }
    printf("已打开 %s, 波特率 %lu\n", ch, bitrate);
    return 0;
}

/* 打开通道并设置 CAN FD 波特率: 先 down, 再配 bitrate+dbitrate+fd on, 再 up */
static int cmd_fd(const char *ch, unsigned long arb, unsigned long data)
{
    if (arb == 0 || data == 0) {
        fprintf(stderr, "波特率无效\n");
        return 1;
    }
    if (run("ip link set %s down", ch) != 0) {
        fprintf(stderr, "无法 down %s (通道不存在?)\n", ch);
        return 1;
    }
    if (run("ip link set %s type can bitrate %lu dbitrate %lu fd on",
            ch, arb, data) != 0) {
        fprintf(stderr, "配置 %s CAN FD 波特率失败\n", ch);
        return 1;
    }
    if (run("ip link set %s up", ch) != 0) {
        fprintf(stderr, "无法 up %s\n", ch);
        return 1;
    }
    printf("已打开 %s, 仲裁段 %lu, 数据段 %lu (CAN FD)\n", ch, arb, data);
    return 0;
}

static void usage(void)
{
    printf(
        "鲲弘 USB2CAN 通道控制工具\n"
        "\n"
        "用法:\n"
        "  kcanctl list                        列出所有 CAN 通道及状态\n"
        "  kcanctl down-all                    关闭所有通道\n"
        "  kcanctl up <通道> <波特率>           打开通道并设置波特率\n"
        "                                      例: kcanctl up can0 1000000\n"
        "  kcanctl down <通道>                 关闭通道\n"
        "                                      例: kcanctl down can0\n"
        "  kcanctl fd <通道> <仲裁> <数据>      以 CAN FD 波特率打开\n"
        "                                      例: kcanctl fd can0 1000000 2000000\n"
        "\n"
        "说明: 需要 root 权限 (sudo kcanctl ...)\n");
}

int main(int argc, char **argv)
{
    if (argc < 2) {
        usage();
        return 1;
    }
    if (strcmp(argv[1], "list") == 0)
        return cmd_list();

    /* 其余子命令都会修改接口状态, 需要 root */
    if (geteuid() != 0) {
        fprintf(stderr, "需要 root 权限, 请用: sudo %s ...\n", argv[0]);
        return 1;
    }
    if (strcmp(argv[1], "down-all") == 0)
        return cmd_down_all();
    if (strcmp(argv[1], "down") == 0 && argc >= 3)
        return cmd_down(argv[2]);
    if (strcmp(argv[1], "up") == 0 && argc >= 4)
        return cmd_up(argv[2], strtoul(argv[3], NULL, 10));
    if (strcmp(argv[1], "fd") == 0 && argc >= 5)
        return cmd_fd(argv[2], strtoul(argv[3], NULL, 10),
                      strtoul(argv[4], NULL, 10));
    usage();
    return 1;
}
