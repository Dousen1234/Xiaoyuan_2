/**
 * @file xlsx_reader.cpp
 * @brief 纯 C++ 的 xlsx 读取实现：zip 解包 (zlib raw inflate) + 极简 XML 扫描
 */

#include "xlsx_reader.h"

#include "proto.h"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <map>
#include <string>
#include <vector>

#include <zlib.h>

namespace x3588
{
    namespace
    {

        // =========================================================================
        // 一、ZIP 解包
        // =========================================================================

        struct ZipEntry
        {
            std::string name;
            uint32_t method = 0; ///< 0 = stored, 8 = deflate
            uint32_t comp_size = 0;
            uint32_t uncomp_size = 0;
            uint32_t local_offset = 0;
        };

        uint16_t rd16(const uint8_t *p)
        {
            return static_cast<uint16_t>(p[0] | (p[1] << 8));
        }

        uint32_t rd32(const uint8_t *p)
        {
            return static_cast<uint32_t>(p[0]) | (static_cast<uint32_t>(p[1]) << 8) |
                   (static_cast<uint32_t>(p[2]) << 16) |
                   (static_cast<uint32_t>(p[3]) << 24);
        }

        /// 整个 zip 文件读入内存（xlsx 一般几百 KB，直接全读最简单可靠）
        bool readWholeFile(const std::string &path, std::vector<uint8_t> &buf,
                           std::string &err)
        {
            std::FILE *fp = std::fopen(path.c_str(), "rb");
            if (fp == nullptr)
            {
                err = "无法打开文件：" + path;
                return false;
            }
            std::fseek(fp, 0, SEEK_END);
            const long sz = std::ftell(fp);
            if (sz <= 0)
            {
                std::fclose(fp);
                err = "文件为空或不可读：" + path;
                return false;
            }
            std::fseek(fp, 0, SEEK_SET);
            buf.resize(static_cast<size_t>(sz));
            const size_t got = std::fread(buf.data(), 1, buf.size(), fp);
            std::fclose(fp);
            if (got != buf.size())
            {
                err = "读取文件不完整：" + path;
                return false;
            }
            return true;
        }

        /// 解析中央目录，建立「文件名 → 条目」索引
        bool parseZipDirectory(const std::vector<uint8_t> &zip,
                               std::map<std::string, ZipEntry> &index, std::string &err)
        {
            if (zip.size() < 22)
            {
                err = "不是有效的 xlsx（文件太小）";
                return false;
            }

            // 从尾部向前找 EOCD 签名 PK\x05\x06（最多回扫 64KB + 22）
            const size_t scan_min = zip.size() > (65536 + 22) ? zip.size() - 65536 - 22 : 0;
            size_t eocd = std::string::npos;
            for (size_t i = zip.size() - 22 + 1; i-- > scan_min;)
            {
                if (zip[i] == 0x50 && zip[i + 1] == 0x4B && zip[i + 2] == 0x05 &&
                    zip[i + 3] == 0x06)
                {
                    eocd = i;
                    break;
                }
            }
            if (eocd == std::string::npos)
            {
                err = "不是有效的 xlsx（找不到 ZIP 中央目录）";
                return false;
            }

            const uint16_t total = rd16(&zip[eocd + 10]);
            uint32_t cd_off = rd32(&zip[eocd + 16]);
            if (total == 0xFFFFu || cd_off == 0xFFFFFFFFu)
            {
                err = "不支持 ZIP64 格式的 xlsx";
                return false;
            }

            size_t p = cd_off;
            for (uint16_t n = 0; n < total; ++n)
            {
                if (p + 46 > zip.size())
                    break;
                if (!(zip[p] == 0x50 && zip[p + 1] == 0x4B && zip[p + 2] == 0x01 &&
                      zip[p + 3] == 0x02))
                    break; // 中央目录头签名 PK\x01\x02

                ZipEntry e;
                e.method = rd16(&zip[p + 10]);
                e.comp_size = rd32(&zip[p + 20]);
                e.uncomp_size = rd32(&zip[p + 24]);
                const uint16_t name_len = rd16(&zip[p + 28]);
                const uint16_t extra_len = rd16(&zip[p + 30]);
                const uint16_t comment_len = rd16(&zip[p + 32]);
                e.local_offset = rd32(&zip[p + 42]);

                if (p + 46 + name_len > zip.size())
                    break;
                e.name.assign(reinterpret_cast<const char *>(&zip[p + 46]), name_len);
                index[e.name] = e;

                p += 46u + name_len + extra_len + comment_len;
            }

            if (index.empty())
            {
                err = "不是有效的 xlsx（中央目录为空）";
                return false;
            }
            return true;
        }

        /// 解出某个条目的原始字节（自动处理 stored / deflate）
        bool extractEntry(const std::vector<uint8_t> &zip,
                          const std::map<std::string, ZipEntry> &index,
                          const std::string &name, std::string &out, std::string &err)
        {
            auto it = index.find(name);
            if (it == index.end())
            {
                err = "xlsx 内缺少 " + name;
                return false;
            }
            const ZipEntry &e = it->second;

            const size_t lh = e.local_offset;
            if (lh + 30 > zip.size())
            {
                err = "xlsx 结构损坏（局部文件头越界）";
                return false;
            }
            if (!(zip[lh] == 0x50 && zip[lh + 1] == 0x4B && zip[lh + 2] == 0x03 &&
                  zip[lh + 3] == 0x04))
            {
                err = "xlsx 结构损坏（局部文件头签名错误）";
                return false;
            }
            const uint16_t name_len = rd16(&zip[lh + 26]);
            const uint16_t extra_len = rd16(&zip[lh + 28]);
            const size_t data_off = lh + 30u + name_len + extra_len;
            if (data_off + e.comp_size > zip.size())
            {
                err = "xlsx 结构损坏（数据段越界）";
                return false;
            }

            if (e.method == 0)
            {
                out.assign(reinterpret_cast<const char *>(&zip[data_off]), e.comp_size);
                return true;
            }
            if (e.method != 8)
            {
                err = "不支持的压缩方式（仅支持 stored/deflate）";
                return false;
            }

            // raw deflate（负 windowBits），一次性 inflate
            out.assign(e.uncomp_size ? e.uncomp_size : (e.comp_size * 4u + 1024u), '\0');
            z_stream zs;
            std::memset(&zs, 0, sizeof(zs));
            if (inflateInit2(&zs, -MAX_WBITS) != Z_OK)
            {
                err = "zlib 初始化失败";
                return false;
            }
            zs.next_in = const_cast<Bytef *>(&zip[data_off]);
            zs.avail_in = e.comp_size;
            zs.next_out = reinterpret_cast<Bytef *>(&out[0]);
            zs.avail_out = static_cast<uInt>(out.size());

            int rc = inflate(&zs, Z_FINISH);
            size_t produced = out.size() - zs.avail_out;

            // 输出缓冲不够时扩容重试
            while (rc == Z_BUF_ERROR || (rc == Z_OK && zs.avail_in > 0))
            {
                const size_t old = out.size();
                out.resize(old * 2 + 1024);
                zs.next_out = reinterpret_cast<Bytef *>(&out[old]);
                zs.avail_out = static_cast<uInt>(out.size() - old);
                rc = inflate(&zs, Z_FINISH);
                produced = out.size() - zs.avail_out;
            }
            inflateEnd(&zs);

            if (rc != Z_STREAM_END)
            {
                err = "解压失败（zlib 错误码 " + std::to_string(rc) + "）";
                return false;
            }
            out.resize(produced);
            return true;
        }

        // =========================================================================
        // 二、极简 XML 扫描工具
        // =========================================================================

        /// 把 XML 实体还原为 UTF-8 明文
        std::string decodeEntities(const std::string &s)
        {
            std::string r;
            r.reserve(s.size());
            for (size_t i = 0; i < s.size(); ++i)
            {
                if (s[i] != '&')
                {
                    r.push_back(s[i]);
                    continue;
                }
                const size_t semi = s.find(';', i);
                if (semi == std::string::npos || semi - i > 12)
                {
                    r.push_back(s[i]);
                    continue;
                }
                const std::string ent = s.substr(i + 1, semi - i - 1);
                if (ent == "amp")
                    r += '&';
                else if (ent == "lt")
                    r += '<';
                else if (ent == "gt")
                    r += '>';
                else if (ent == "quot")
                    r += '"';
                else if (ent == "apos")
                    r += '\'';
                else if (!ent.empty() && ent[0] == '#')
                {
                    const long cp = (ent.size() > 1 && (ent[1] == 'x' || ent[1] == 'X'))
                                        ? std::strtol(ent.c_str() + 2, nullptr, 16)
                                        : std::strtol(ent.c_str() + 1, nullptr, 10);
                    if (cp > 0 && cp < 0x80)
                    {
                        r.push_back(static_cast<char>(cp));
                    }
                    else if (cp >= 0x80 && cp < 0x800)
                    {
                        r.push_back(static_cast<char>(0xC0 | (cp >> 6)));
                        r.push_back(static_cast<char>(0x80 | (cp & 0x3F)));
                    }
                    else if (cp >= 0x800 && cp < 0x10000)
                    {
                        r.push_back(static_cast<char>(0xE0 | (cp >> 12)));
                        r.push_back(static_cast<char>(0x80 | ((cp >> 6) & 0x3F)));
                        r.push_back(static_cast<char>(0x80 | (cp & 0x3F)));
                    }
                    else if (cp >= 0x10000 && cp < 0x110000)
                    {
                        r.push_back(static_cast<char>(0xF0 | (cp >> 18)));
                        r.push_back(static_cast<char>(0x80 | ((cp >> 12) & 0x3F)));
                        r.push_back(static_cast<char>(0x80 | ((cp >> 6) & 0x3F)));
                        r.push_back(static_cast<char>(0x80 | (cp & 0x3F)));
                    }
                }
                else
                {
                    r.push_back('&');
                    r += ent;
                    r.push_back(';');
                }
                i = semi;
            }
            return r;
        }

        /// 取标签内某属性的值（如 <sheet name="A" sheetId="1"/> 取 name）
        std::string attrOf(const std::string &tag, const std::string &key)
        {
            const std::string pat = key + "=\"";
            size_t p = tag.find(pat);
            if (p == std::string::npos)
            {
                const std::string pat2 = key + "='";
                p = tag.find(pat2);
                if (p == std::string::npos)
                    return std::string();
                const size_t s = p + pat2.size();
                const size_t e = tag.find('\'', s);
                if (e == std::string::npos)
                    return std::string();
                return decodeEntities(tag.substr(s, e - s));
            }
            const size_t s = p + pat.size();
            const size_t e = tag.find('"', s);
            if (e == std::string::npos)
                return std::string();
            return decodeEntities(tag.substr(s, e - s));
        }

        /// 读 <t> 与 <t xml:space="preserve"> 的文本内容
        std::string textOfT(const std::string &s, size_t from, size_t to)
        {
            std::string r;
            size_t p = from;
            while (p < to)
            {
                const size_t a = s.find("<t", p);
                if (a == std::string::npos || a >= to)
                    break;
                const size_t gt = s.find('>', a);
                if (gt == std::string::npos || gt >= to)
                    break;
                const size_t b = s.find("</t>", gt);
                if (b == std::string::npos || b > to)
                    break;
                r += decodeEntities(s.substr(gt + 1, b - gt - 1));
                p = b + 4;
            }
            return r;
        }

        // =========================================================================
        // 三、sharedStrings / workbook / worksheet 解析
        // =========================================================================

        /// sharedStrings.xml → 字符串数组
        void parseSharedStrings(const std::string &xml, std::vector<std::string> &out)
        {
            size_t p = 0;
            while (true)
            {
                const size_t a = xml.find("<si", p);
                if (a == std::string::npos)
                    break;
                const size_t aend = xml.find("</si>", a);
                if (aend == std::string::npos)
                    break;
                const size_t gt = xml.find('>', a);
                if (gt == std::string::npos || gt > aend)
                {
                    p = aend + 5;
                    continue;
                }

                // 富文本 <r><t>..</t></r> 需全部拼接；纯文本只有 <t>
                out.push_back(textOfT(xml, gt + 1, aend));
                p = aend + 5;
            }
        }

        /// workbook.xml → [(sheetName, rId)] 顺序表
        void parseWorkbookSheets(const std::string &xml,
                                 std::vector<std::pair<std::string, std::string>> &out)
        {
            size_t p = 0;
            while (true)
            {
                const size_t a = xml.find("<sheet ", p);
                if (a == std::string::npos)
                    break;
                const size_t e = xml.find('>', a);
                if (e == std::string::npos)
                    break;
                const std::string tag = xml.substr(a, e - a + 1);
                const std::string name = attrOf(tag, "name");
                std::string rid = attrOf(tag, "r:id");
                if (rid.empty())
                    rid = attrOf(tag, "id"); // 无前缀命名空间的情况
                if (!name.empty())
                    out.emplace_back(name, rid);
                p = e + 1;
            }
        }

        /// xl/_rels/workbook.xml.rels → [rId → 目标路径（相对 xl/）]
        void parseRels(const std::string &xml, std::map<std::string, std::string> &out)
        {
            size_t p = 0;
            while (true)
            {
                const size_t a = xml.find("<Relationship", p);
                if (a == std::string::npos)
                    break;
                const size_t e = xml.find('>', a);
                if (e == std::string::npos)
                    break;
                const std::string tag = xml.substr(a, e - a + 1);
                const std::string id = attrOf(tag, "Id");
                std::string target = attrOf(tag, "Target");
                if (!id.empty() && !target.empty())
                {
                    if (target.rfind("/xl/", 0) == 0)
                    {
                        target = target.substr(4); // 绝对路径 /xl/worksheets/... → worksheets/...
                    }
                    else if (target.rfind("xl/", 0) == 0)
                    {
                        target = target.substr(3);
                    }
                    else if (target.rfind("../", 0) == 0)
                    {
                        target = target.substr(3);
                    }
                    out[id] = target;
                }
                p = e + 1;
            }
        }

        /// "AB12" → 列下标 0 起的数值（A=0，AA=26），失败返回 -1
        int colOfRef(const std::string &ref)
        {
            int c = 0;
            size_t i = 0;
            for (; i < ref.size(); ++i)
            {
                const unsigned char ch = static_cast<unsigned char>(ref[i]);
                if (ch >= 'A' && ch <= 'Z')
                    c = c * 26 + (ch - 'A' + 1);
                else if (ch >= 'a' && ch <= 'z')
                    c = c * 26 + (ch - 'a' + 1);
                else
                    break;
            }
            return i == 0 ? -1 : c - 1;
        }

        /// 工作表 XML → 二维字符串表
        void parseWorksheet(const std::string &xml,
                            const std::vector<std::string> &shared, Sheet &sheet)
        {
            size_t p = 0;
            int row_idx = -1;
            while (true)
            {
                const size_t ra = xml.find("<row", p);
                if (ra == std::string::npos)
                    break;
                const size_t rae = xml.find("</row>", ra);
                const size_t next = (rae == std::string::npos) ? xml.size() : rae + 6;
                const size_t rgt = xml.find('>', ra);
                if (rgt == std::string::npos)
                    break;

                // <row r="N"> 的行号（1 起）；自闭合 <row .../> 无内容
                if (xml[rgt - 1] != '/')
                {
                    ++row_idx;
                    std::vector<std::string> row;

                    size_t cp = rgt + 1;
                    while (cp < rae)
                    {
                        const size_t ca = xml.find("<c", cp);
                        if (ca == std::string::npos || ca >= rae)
                            break;
                        const size_t cgt = xml.find('>', ca);
                        if (cgt == std::string::npos || cgt > rae)
                            break;

                        const std::string ctag = xml.substr(ca, cgt - ca + 1);
                        const std::string ref = attrOf(ctag, "r");
                        const std::string type = attrOf(ctag, "t");

                        std::string value;
                        if (ctag[ctag.size() - 2] == '/')
                        {
                            // 自闭合空单元格
                            cp = cgt + 1;
                        }
                        else
                        {
                            const size_t cend = xml.find("</c>", cgt);
                            if (cend == std::string::npos || cend > rae)
                                break;
                            const std::string body = xml.substr(cgt + 1, cend - cgt - 1);

                            if (type == "s")
                            {
                                const size_t vs = body.find("<v>");
                                if (vs != std::string::npos)
                                {
                                    const size_t ve = body.find("</v>", vs);
                                    if (ve != std::string::npos)
                                    {
                                        const long idx = std::strtol(
                                            body.c_str() + int(vs) + 3, nullptr, 10);
                                        if (idx >= 0 &&
                                            size_t(idx) < shared.size())
                                            value = shared[size_t(idx)];
                                    }
                                }
                            }
                            else if (type == "inlineStr")
                            {
                                value = textOfT(body, 0, body.size());
                            }
                            else
                            {
                                const size_t vs = body.find("<v>");
                                if (vs != std::string::npos)
                                {
                                    const size_t ve = body.find("</v>", vs);
                                    if (ve != std::string::npos)
                                        value = decodeEntities(
                                            body.substr(vs + 3, ve - vs - 3));
                                }
                                else if (body.find("<t") != std::string::npos)
                                {
                                    value = textOfT(body, 0, body.size());
                                }
                            }
                            cp = cend + 4;
                        }

                        int col = colOfRef(ref);
                        if (col < 0)
                            col = static_cast<int>(row.size());
                        if (static_cast<size_t>(col) >= row.size())
                            row.resize(static_cast<size_t>(col) + 1);
                        row[static_cast<size_t>(col)] = value;
                    }
                    sheet.rows.push_back(std::move(row));
                }
                p = next;
            }
        }

        // =========================================================================
        // 四、表头解析
        // =========================================================================

        /// 归一化表头：小写、去掉所有非字母数字字符
        std::string normHeader(const std::string &s)
        {
            std::string r;
            for (char ch : s)
            {
                const unsigned char u = static_cast<unsigned char>(ch);
                if (std::isalnum(u))
                    r.push_back(static_cast<char>(std::tolower(u)));
            }
            return r;
        }

        /// 是否时间列
        bool isTimeHeader(const std::string &n)
        {
            return n == "time" || n == "times" || n == "t" || n == "timestamp" ||
                   n == "times" || n == "sec" || n == "seconds";
        }

        /// 表头 → 全局关节下标 0~15；不是关节列返回 -1
        /// 支持 a1..d4 / l1j1..l4j4 / j1..j16 / a_1 / legaj1 等常见写法
        int jointOfHeader(const std::string &n)
        {
            if (n.size() == 2 && n[0] >= 'a' && n[0] <= 'd' && n[1] >= '1' && n[1] <= '4')
                return (n[0] - 'a') * 4 + (n[1] - '1');

            // l<leg>j<joint>：l1j1 … l4j4
            if (n.size() == 4 && n[0] == 'l' && n[1] >= '1' && n[1] <= '4' && n[2] == 'j' &&
                n[3] >= '1' && n[3] <= '4')
                return (n[1] - '1') * 4 + (n[3] - '1');

            // j1 … j16
            if (n.size() >= 2 && n.size() <= 3 && n[0] == 'j')
            {
                const int v = std::atoi(n.c_str() + 1);
                if (v >= 1 && v <= kTotalJoints)
                    return v - 1;
                return -1;
            }

            // legaj1 / leg1j1 / al1 等带 leg 前缀的写法
            if (n.rfind("leg", 0) == 0)
            {
                for (size_t i = 3; i + 2 <= n.size(); ++i)
                {
                    if (n[i] >= 'a' && n[i] <= 'd' && i + 1 < n.size() &&
                        n[i + 1] >= '1' && n[i + 1] <= '4')
                        return (n[i] - 'a') * 4 + (n[i + 1] - '1');
                }
            }
            return -1;
        }

        /// 文本 → double（容忍前后空格、全角空格、正负号）
        bool toDouble(const std::string &s, double &v)
        {
            if (s.empty())
                return false;
            char *end = nullptr;
            const double d = std::strtod(s.c_str(), &end);
            if (end == s.c_str())
                return false;
            while (*end == ' ' || *end == '\t' || *end == '\r' || *end == '\n')
                ++end;
            if (*end != '\0')
                return false;
            v = d;
            return true;
        }

    } // namespace

    // =========================================================================
    // 公共接口
    // =========================================================================

    double Trajectory::maxStepRad() const
    {
        double m = 0.0;
        for (size_t i = 1; i < pos_rad.size(); ++i)
        {
            for (int j = 0; j < kTotalJoints; ++j)
            {
                const double d = std::fabs(pos_rad[i][j] - pos_rad[i - 1][j]);
                if (d > m)
                    m = d;
            }
        }
        return m;
    }

    bool listXlsxSheets(const std::string &path, std::vector<std::string> &names,
                        std::string &err)
    {
        names.clear();
        std::vector<uint8_t> zip;
        if (!readWholeFile(path, zip, err))
            return false;
        std::map<std::string, ZipEntry> index;
        if (!parseZipDirectory(zip, index, err))
            return false;

        std::string wb;
        if (!extractEntry(zip, index, "xl/workbook.xml", wb, err))
            return false;
        std::vector<std::pair<std::string, std::string>> sheets;
        parseWorkbookSheets(wb, sheets);
        for (auto &kv : sheets)
            names.push_back(kv.first);
        return !names.empty();
    }

    bool readXlsxSheet(const std::string &path, const std::string &sheetName,
                       Sheet &out, std::string &err)
    {
        out = Sheet();
        std::vector<uint8_t> zip;
        if (!readWholeFile(path, zip, err))
            return false;
        std::map<std::string, ZipEntry> index;
        if (!parseZipDirectory(zip, index, err))
            return false;

        // 共享字符串
        std::vector<std::string> shared;
        {
            std::string sst;
            std::string e2;
            if (extractEntry(zip, index, "xl/sharedStrings.xml", sst, e2))
                parseSharedStrings(sst, shared);
        }

        // 工作表名 → 工作表文件路径
        std::string wb;
        if (!extractEntry(zip, index, "xl/workbook.xml", wb, err))
            return false;
        std::vector<std::pair<std::string, std::string>> sheets;
        parseWorkbookSheets(wb, sheets);
        if (sheets.empty())
        {
            err = "xlsx 内没有任何工作表";
            return false;
        }

        std::map<std::string, std::string> rels;
        {
            std::string rel_xml, e2;
            if (extractEntry(zip, index, "xl/_rels/workbook.xml.rels", rel_xml, e2))
                parseRels(rel_xml, rels);
        }

        size_t pick = 0;
        if (!sheetName.empty())
        {
            bool found = false;
            for (size_t i = 0; i < sheets.size(); ++i)
            {
                if (sheets[i].first == sheetName)
                {
                    pick = i;
                    found = true;
                    break;
                }
            }
            if (!found)
            {
                std::string avail;
                for (auto &s : sheets)
                    avail += " [" + s.first + "]";
                err = "找不到工作表 \"" + sheetName + "\"，现有：" + avail;
                return false;
            }
        }

        std::string target;
        auto rt = rels.find(sheets[pick].second);
        if (rt != rels.end())
            target = rt->second;

        std::string ws;
        if (!target.empty() &&
            extractEntry(zip, index, "xl/" + target, ws, err))
        {
            // 成功
        }
        else
        {
            // 关系表缺失时退化为按序号猜 sheetN.xml
            const std::string guess = "xl/worksheets/sheet" +
                                      std::to_string(pick + 1) + ".xml";
            if (!extractEntry(zip, index, guess, ws, err))
                return false;
        }

        out.name = sheets[pick].first;
        parseWorksheet(ws, shared, out);
        if (out.rows.empty())
        {
            err = "工作表 \"" + out.name + "\" 为空";
            return false;
        }
        return true;
    }

    bool loadTrajectory(const std::string &path, const std::string &sheetName,
                        AngleUnit unit, Trajectory &out, std::string &err,
                        std::string &warn)
    {
        out = Trajectory();
        out.source = path;
        err.clear();
        warn.clear();

        Sheet sh;
        if (!readXlsxSheet(path, sheetName, sh, err))
            return false;
        out.sheet = sh.name;

        // —— 找表头行：第一行含「时间列 + 至少 1 个关节列」——
        size_t hdr = std::string::npos;
        int time_col = -1;
        std::map<int, int> col2joint; // 列下标 → 全局关节下标
        for (size_t r = 0; r < sh.rows.size() && r < 20; ++r)
        {
            int tc = -1;
            std::map<int, int> m;
            for (size_t c = 0; c < sh.rows[r].size(); ++c)
            {
                const std::string n = normHeader(sh.rows[r][c]);
                if (n.empty())
                    continue;
                if (tc < 0 && isTimeHeader(n))
                {
                    tc = static_cast<int>(c);
                    continue;
                }
                const int j = jointOfHeader(n);
                if (j < 0)
                    continue;
                // 同一个关节只保留首次出现（按关节号去重，不能按列号）
                bool dup = false;
                for (auto &kv : m)
                    if (kv.second == j)
                    {
                        dup = true;
                        break;
                    }
                if (!dup)
                    m[static_cast<int>(c)] = j;
            }
            if (tc >= 0 && m.size() >= 4)
            {
                hdr = r;
                time_col = tc;
                col2joint = m;
                break;
            }
        }
        if (hdr == std::string::npos)
        {
            err = "格式不正确，读取失败：前 20 行内找不到「Time + A1~D4」形式的表头";
            return false;
        }

        // 表头里出现过的关节集合
        std::vector<bool> seen(kTotalJoints, false);
        for (auto &kv : col2joint)
            seen[static_cast<size_t>(kv.second)] = true;
        int n_seen = 0;
        std::string missing;
        for (int j = 0; j < kTotalJoints; ++j)
        {
            if (seen[static_cast<size_t>(j)])
            {
                ++n_seen;
            }
            else
            {
                missing += std::string(1, legLetter(j / 4)) +
                           std::string(1, char('1' + j % 4)) + " ";
            }
        }
        if (n_seen < kTotalJoints)
        {
            err = "格式不正确，读取失败：表头缺少关节列 " + missing +
                  "（要求 17 列：Time + A1~A4, B1~B4, C1~C4, D1~D4）";
            return false;
        }

        // —— 逐行读数据 ——
        double max_abs = 0.0;
        bool any_val = false;
        size_t bad_rows = 0;

        for (size_t r = hdr + 1; r < sh.rows.size(); ++r)
        {
            const std::vector<std::string> &row = sh.rows[r];

            double t = 0.0;
            if (time_col >= static_cast<int>(row.size()) ||
                !toDouble(row[static_cast<size_t>(time_col)], t))
            {
                // 整行为空 → 直接跳过；有内容但时间非法 → 计入坏行
                bool has_any = false;
                for (const auto &cell : row)
                    if (!normHeader(cell).empty())
                    {
                        has_any = true;
                        break;
                    }
                if (has_any)
                    ++bad_rows;
                continue;
            }

            std::array<double, kTotalJoints> p{};
            std::array<bool, kTotalJoints> hv{};
            bool ok = false;
            for (auto &kv : col2joint)
            {
                const int c = kv.first;
                const int j = kv.second;
                if (c >= static_cast<int>(row.size()))
                    continue;
                double v = 0.0;
                if (!toDouble(row[static_cast<size_t>(c)], v))
                    continue;
                hv[static_cast<size_t>(j)] = true;
                p[static_cast<size_t>(j)] = v;
                const double a = std::fabs(v);
                if (a > max_abs)
                    max_abs = a;
                any_val = true;
                ok = true;
            }
            if (!ok)
                continue; // 整行没有可用数值 → 跳过

            // 某关节缺值：沿用上一个样点（首行缺则填 0）
            for (int j = 0; j < kTotalJoints; ++j)
            {
                if (!hv[static_cast<size_t>(j)])
                {
                    p[static_cast<size_t>(j)] =
                        out.pos_rad.empty() ? 0.0
                                            : out.pos_rad.back()[static_cast<size_t>(j)];
                    hv[static_cast<size_t>(j)] = true;
                }
            }

            out.time_s.push_back(t);
            out.pos_rad.push_back(p);
            out.have.push_back(true);
        }

        if (out.time_s.empty())
        {
            err = "格式不正确，读取失败：表头下方没有可用的数据行";
            return false;
        }
        if (out.time_s.size() < 2)
        {
            err = "格式不正确，读取失败：至少需要 2 行数据（当前只有 1 行）";
            return false;
        }
        if (!any_val)
        {
            err = "格式不正确，读取失败：数据列全为空";
            return false;
        }

        // —— 单位换算：统一成 rad ——
        bool to_deg = false;
        if (unit == AngleUnit::Degree)
        {
            to_deg = true;
        }
        else if (unit == AngleUnit::Auto)
        {
            std::string all;
            for (size_t c = 0; c < sh.rows[hdr].size(); ++c)
                all += normHeader(sh.rows[hdr][c]);
            if (all.find("deg") != std::string::npos)
            {
                to_deg = true;
            }
            else if (all.find("rad") != std::string::npos)
            {
                to_deg = false;
            }
            else
            {
                // 无单位提示：关节行程以度计通常 > 4，以弧度计通常 < 4
                to_deg = max_abs > 4.0;
            }
        }
        if (to_deg)
        {
            for (auto &p : out.pos_rad)
                for (int j = 0; j < kTotalJoints; ++j)
                    p[static_cast<size_t>(j)] *= kDegToRad;
            warn += "表内数值按【度】解读（已换算为弧度）。\n";
        }
        else
        {
            warn += "表内数值按【弧度】解读。\n";
        }

        // —— 时间轴检查 ——
        for (size_t i = 1; i < out.time_s.size(); ++i)
        {
            if (out.time_s[i] <= out.time_s[i - 1])
            {
                err = "格式不正确，读取失败：时间戳非递增（第 " +
                      std::to_string(i + 1) + " 个样点：" +
                      std::to_string(out.time_s[i - 1]) + " → " +
                      std::to_string(out.time_s[i]) + "）";
                return false;
            }
        }
        if (bad_rows > 0)
        {
            warn += "跳过 " + std::to_string(bad_rows) + " 行时间戳非法的数据。\n";
        }

        return true;
    }

} // namespace x3588