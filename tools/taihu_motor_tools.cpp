/**
 * @file taihu_motor_tools.cpp
 * @brief 电机工具总入口（菜单式）：将所有独立工具打包为一个交互程序。
 *
 * 运行流程：
 *   1. 数字键选择电机所在的 CAN 总线（0~5，对应鲲弘 KH-UCANFDX6-Mini 的 can0~can5）；
 *   2. 数字键选择要运行的具体工具函数；
 *   3. 工具执行完毕后回到菜单，可继续选择其他工具、更换总线或退出。
 *
 * 包含的工具：
 *   1 —— 扫描总线上的 CAN ID
 *   2 —— 通信诊断
 *   3 —— 修改 CAN ID
 *   4 —— 恢复出厂设置
 *   5 —— 编码器标零
 *   6 —— 打开刹车并重新标零
 *   7 —— 设置低压阈值
 *
 * 用法: ./taihu_motor_tools
 */

#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <string>
#include <thread>
#include <chrono>

#include "taihu/joint_module.h"
#include "taihu/socket_can.h"
#include "taihu/taihu_tools.h"

namespace {

/// 读取一个整数，输入无效时返回默认值。
int inputInt(const char* prompt, int def, int lo, int hi) {
    std::printf("%s", prompt);
    std::fflush(stdout);
    int v = 0;
    if (std::scanf("%d", &v) != 1 || v < lo || v > hi) {
        std::printf("[提示] 输入无效，使用默认值 %d\n", def);
        return def;
    }
    return v;
}

/// 丢弃 stdin 中的残留换行（scanf 后调用 getchar 前使用）。
void flushStdin() {
    int c;
    while ((c = std::getchar()) != '\n' && c != EOF) {}
}

// =====================================================================
// 工具 1：扫描总线上的 CAN ID（同 taihu_motor_tools_scan）
// =====================================================================
void toolScan(taihu::CanInterface& can) {
    using namespace taihu;

    std::printf("[信息] 开始扫描 CAN ID 1~127 ...\n");
    int found = 0;
    for (uint32_t id = 1; id <= 127; ++id) {
        JointModule joint(can, id, id);
        int32_t cnt = 0;
        if (joint.readPositionCnt(cnt)) {
            std::printf("[发现] ID=%u，位置=%d cnt (%.3f rad)\n",
                        id, cnt, static_cast<double>(cnt) * 2.0 * 3.14159265358979323846 / 262144.0);
            ++found;
        }
    }
    std::printf("[信息] 扫描结束，共发现 %d 个设备\n", found);
}

// =====================================================================
// 工具 2：通信诊断（同 taihu_motor_tools_diag）
// =====================================================================
void toolDiag(taihu::CanInterface& can) {
    const uint32_t can_id = static_cast<uint32_t>(
        inputInt("请输入电机 CAN ID（1~127，默认 1）：", 1, 1, 127));
    taihu::diagnose(can, can_id);
}

// =====================================================================
// 工具 3：修改 CAN ID（同 taihu_motor_tools_change_id）
// =====================================================================
void toolChangeId(taihu::CanInterface& can) {
    const uint32_t cur_id = static_cast<uint32_t>(
        inputInt("请输入电机当前 CAN ID（1~127，默认 1）：", 1, 1, 127));
    const int new_id_in =
        inputInt("请输入新 CAN ID（1~127，且不能与当前相同）：", 0, 1, 127);
    if (new_id_in < 1 || new_id_in > 127) {
        std::printf("[错误] 新 ID 必须在 1~127 之间，未修改\n");
        return;
    }
    const uint32_t new_id = static_cast<uint32_t>(new_id_in);
    if (new_id == cur_id) {
        std::printf("[错误] 新 ID 与当前 ID 相同，未修改\n");
        return;
    }
    const bool ok = taihu::changeCanId(can, cur_id, new_id);
    std::printf("%s\n", ok ? "[信息] 修改 CAN ID 成功" : "[错误] 修改 CAN ID 失败");
}

// =====================================================================
// 工具 4：恢复出厂设置（同 taihu_motor_tools_reset_factory）
// =====================================================================
void toolResetFactory(taihu::CanInterface& can) {
    const uint32_t can_id = static_cast<uint32_t>(
        inputInt("请输入电机 CAN ID（1~127，默认 1）：", 1, 1, 127));
    std::printf("[警告] 即将对 ID=%u 执行恢复出厂设置，参数将被清除！\n", can_id);
    if (inputInt("确认执行？（1=确认，0=取消）：", 0, 0, 1) != 1) {
        std::printf("[信息] 已取消\n");
        return;
    }
    const bool ok = taihu::resetFactory(can, can_id);
    std::printf("%s\n", ok ? "[信息] 恢复出厂设置成功" : "[错误] 恢复出厂设置失败");
}

// =====================================================================
// 工具 5：编码器标零（同 taihu_motor_tools_set_zero）
// =====================================================================
void toolSetZero(taihu::CanInterface& can) {
    const uint32_t can_id = static_cast<uint32_t>(
        inputInt("请输入电机 CAN ID（1~127，默认 1）：", 1, 1, 127));
    const bool ok = taihu::setZeroPosition(can, can_id);
    std::printf("%s\n", ok ? "[信息] 标零成功" : "[错误] 标零失败");
}

// =====================================================================
// 工具 6：打开刹车并重新标零（同 taihu_motor_tools_openbreak）
// =====================================================================
void toolOpenBreak(taihu::CanInterface& can) {
    using namespace taihu;

    const uint32_t can_id = static_cast<uint32_t>(
        inputInt("请输入电机 CAN ID（1~127，默认 1）：", 1, 1, 127));

    JointModule joint(can, can_id, can_id);

    // 1. 清除锁存错误（如欠压），确保可正常使能
    joint.clearError();
    std::this_thread::sleep_for(std::chrono::milliseconds(100));

    // 2. 打开刹车：设置目标电流 0，进入电流模式（使能 + 零转矩），解除抱闸
    if (!joint.setTargetCurrent(0)) {
        std::printf("[错误] 打开刹车（使能）失败\n");
        return;
    }
    std::printf("[信息] 刹车已打开，电机可自由旋转，请手动旋转到目标位置\n");

    // 3. 等待用户确认
    std::printf("[提示] 旋转完成后按回车键确认保存零位...\n");
    flushStdin();   // 丢弃前面 scanf 的残留换行，避免误触
    std::getchar();

    // 4. 标零并保存 Flash（setZeroPosition 内部会先 stop 去使能，即关闭刹车抱闸）
    const bool ok = setZeroPosition(can, can_id);
    std::printf("[信息] 刹车已关闭（重新抱闸）\n");
    std::printf("%s\n", ok ? "[信息] 标零成功" : "[错误] 标零失败");
}

// =====================================================================
// 工具 7：设置低压阈值（同 taihu_motor_tools_set_low_voltage）
// =====================================================================
void toolSetLowVoltage(taihu::CanInterface& can) {
    const uint32_t can_id = static_cast<uint32_t>(
        inputInt("请输入电机 CAN ID（1~127，默认 1）：", 1, 1, 127));
    const int32_t voltage_v =
        inputInt("请输入新的低压阈值（V，应低于供电电压，如 28V 供电设 24）：", 24, 0, 60);
    const bool ok = taihu::setLowVoltageThreshold(can, can_id, voltage_v);
    if (ok) {
        std::printf("[提示] 请断电重新上电使设置生效\n");
    } else {
        std::printf("[错误] 设置低压阈值失败\n");
    }
}

// =====================================================================
// 菜单打印
// =====================================================================
void printMenu(const std::string& ifname) {
    std::printf("\n");
    std::printf("========================================\n");
    std::printf("  电机工具集  （当前总线：%s）\n", ifname.c_str());
    std::printf("========================================\n");
    std::printf("  1 —— 扫描总线上的 CAN ID\n");
    std::printf("  2 —— 通信诊断\n");
    std::printf("  3 —— 修改 CAN ID\n");
    std::printf("  4 —— 恢复出厂设置\n");
    std::printf("  5 —— 编码器标零\n");
    std::printf("  6 —— 打开刹车并重新标零\n");
    std::printf("  7 —— 设置低压阈值\n");
    std::printf("----------------------------------------\n");
    std::printf("  9 —— 更换 CAN 总线\n");
    std::printf("  0 —— 退出\n");
}

/// 选择 CAN 总线（0~5），返回接口名；返回空串表示用户要求退出。
std::string selectBus() {
    while (true) {
        std::printf("\n");
        std::printf("========================================\n");
        std::printf("  请选择电机所在的 CAN 总线\n");
        std::printf("  （鲲弘 KH-UCANFDX6-Mini：0~5 → can0~can5）\n");
        std::printf("========================================\n");
        for (int i = 0; i <= 5; i++)
            std::printf("  %d —— can%d\n", i, i);
        std::printf("  x —— 退出程序\n");

        char buf[16] = {0};
        std::printf("请输入总线编号（0~5）：");
        std::fflush(stdout);
        if (std::scanf("%15s", buf) != 1)
            continue;
        if (buf[0] == 'x' || buf[0] == 'X' || buf[0] == 'q' || buf[0] == 'Q')
            return "";

        char* end = nullptr;
        const long v = std::strtol(buf, &end, 10);
        if (end == buf || *end != '\0' || v < 0 || v > 5) {
            std::printf("[提示] 无效输入，请输入 0~5 或 x\n");
            continue;
        }
        char ifname[16];
        std::snprintf(ifname, sizeof ifname, "can%ld", v);
        return ifname;
    }
}

} // namespace

int main() {
    using namespace taihu;

    std::printf("========================================\n");
    std::printf("  钛虎电机工具集（taihu_motor_tools）\n");
    std::printf("========================================\n");

    std::string ifname;
    SocketCanInterface can;
    bool can_opened = false;

    while (true) {
        // 1. 选择总线（若尚未打开或用户要求更换）
        if (!can_opened) {
            ifname = selectBus();
            if (ifname.empty()) {
                std::printf("[信息] 已退出\n");
                return 0;
            }
            if (!can.open(ifname.c_str(), 0)) {
                std::printf("[错误] 打开 %s 失败，请确认接口已 up"
                            "（sudo ./build/kcanctl up %s 1000000）\n",
                            ifname.c_str(), ifname.c_str());
                continue;   // 重新选择总线
            }
            can_opened = true;
            std::printf("[信息] 已打开 %s\n", ifname.c_str());
        }

        // 2. 工具菜单循环
        printMenu(ifname);
        const int choice = inputInt("请选择要运行的工具：", -1, 0, 9);

        switch (choice) {
        case 1: toolScan(can);            break;
        case 2: toolDiag(can);            break;
        case 3: toolChangeId(can);        break;
        case 4: toolResetFactory(can);    break;
        case 5: toolSetZero(can);         break;
        case 6: toolOpenBreak(can);       break;
        case 7: toolSetLowVoltage(can);   break;
        case 9:
            can.close();
            can_opened = false;
            std::printf("[信息] 已关闭 %s，请重新选择总线\n", ifname.c_str());
            break;
        case 0:
            can.close();
            std::printf("[信息] 已退出\n");
            return 0;
        default:
            std::printf("[提示] 无效选择（1~7 选工具，9 换总线，0 退出）\n");
            break;
        }
    }
}
