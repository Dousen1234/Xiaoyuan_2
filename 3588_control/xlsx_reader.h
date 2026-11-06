/**
 * @file xlsx_reader.h
 * @brief 纯 C++ 的 xlsx 读取与轨迹加载（仅依赖 zlib，不需要 openpyxl/pip）
 *
 * 3588 上没有 pip / openpyxl，所以这里自己实现最小可用的 xlsx 解析：
 *   xlsx 本质是一个 zip 包，用 zlib 的 raw-inflate 解压，再对
 *   xl/sharedStrings.xml 与 xl/worksheets/sheetN.xml 做极简 XML 扫描。
 *
 * 支持的轨迹表格式（17 列）：
 *
 *   | Time | A1 | A2 | A3 | A4 | B1 | B2 | B3 | B4 | C1 | C2 | C3 | C4 | D1 | D2 | D3 | D4 |
 *
 * 其中 A/B/C/D 对应腿臂 A~D（等价仓库旧称 L1~L4），A1 = L1_J1。
 * 为方便直接复用历史文件，也接受旧表头 Time + L1_J1..L4_J4。
 */

#ifndef X3588_XLSX_READER_H
#define X3588_XLSX_READER_H

#include <array>
#include <cstddef>
#include <string>
#include <vector>

namespace x3588
{

    /// 一张工作表：全部单元格按文本保存，空单元格为空串。
    struct Sheet
    {
        std::string name;                           ///< 工作表名
        std::vector<std::vector<std::string>> rows; ///< rows[行][列]，行列均 0 起
    };

    /// 角度单位（表内数值的单位）。Auto = 自动判别：最大绝对值 > 4.0 视为度。
    enum class AngleUnit
    {
        Radian,
        Degree,
        Auto
    };

    /// 一条 16 关节轨迹
    struct Trajectory
    {
        std::string source; ///< 来源 xlsx 路径
        std::string sheet;  ///< 实际使用的工作表名

        std::vector<double> time_s;                  ///< 每个样点的时间戳（秒）
        std::vector<std::array<double, 16>> pos_rad; ///< [样点][关节 0~15] 关节端 rad
        std::vector<bool> have;                      ///< [样点][关节] 该关节本样点是否有值

        size_t sampleCount() const { return time_s.size(); }

        /// 轨迹总时长（秒）
        double durationS() const
        {
            return time_s.size() >= 2 ? time_s.back() - time_s.front() : 0.0;
        }

        /// 平均采样间隔（秒）
        double intervalS() const
        {
            return time_s.size() >= 2 ? durationS() / double(time_s.size() - 1) : 0.0;
        }

        /// 相邻样点间的最大单关节角度变化（rad），用于检查是否超出协议节奏建议
        double maxStepRad() const;
    };

    /// 列出 xlsx 中所有工作表名。失败返回 false 并填 err。
    bool listXlsxSheets(const std::string &path, std::vector<std::string> &names,
                        std::string &err);

    /// 读取 xlsx 中指定工作表（sheetName 为空 → 取第一张）。
    bool readXlsxSheet(const std::string &path, const std::string &sheetName,
                       Sheet &out, std::string &err);

    /// 加载轨迹文件并做格式校验。
    /// @param path       xlsx 路径
    /// @param sheetName  工作表名（空 = 第一张）
    /// @param unit       表内数值的角度单位
    /// @param out        解析结果
    /// @param err        失败原因（已含「格式不正确，读取失败」等中文提示）
    /// @param warn       非致命提示（表头别名、多余列、样点过密等）
    /// @return 成功返回 true
    bool loadTrajectory(const std::string &path, const std::string &sheetName,
                        AngleUnit unit, Trajectory &out, std::string &err,
                        std::string &warn);

} // namespace x3588

#endif // X3588_XLSX_READER_H