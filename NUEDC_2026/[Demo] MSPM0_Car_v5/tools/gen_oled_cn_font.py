# -*- coding: utf-8 -*-
"""Render 24px / 16px 1bpp Chinese glyph bitmaps for the SSD1306 calibration UI."""
import io
import os
from PIL import Image, ImageDraw, ImageFont

HERE = os.path.dirname(os.path.abspath(__file__))
OUT = os.path.join(HERE, "..", "bsp", "oled", "oled_font_cn.h")

FONT_BOLD = "C:/Windows/Fonts/msyhbd.ttc"

CN24 = "循线校准成功失败"
CN16 = "请放在白色上黑失败重试"


def render(ch, box, font_px, thr):
    ss = 8
    img = Image.new("L", (box * ss, box * ss), 0)
    d = ImageDraw.Draw(img)
    font = ImageFont.truetype(FONT_BOLD, font_px * ss)
    # center on the ink bbox so glyphs are optically centred, not baseline-centred
    bbox = d.textbbox((0, 0), ch, font=font)
    w = bbox[2] - bbox[0]
    h = bbox[3] - bbox[1]
    x = (box * ss - w) / 2 - bbox[0]
    y = (box * ss - h) / 2 - bbox[1]
    d.text((x, y), ch, font=font, fill=255)
    small = img.resize((box, box), Image.LANCZOS)
    px = small.load()
    return [[1 if px[c, r] >= thr else 0 for c in range(box)] for r in range(box)]


def dump(name, chars, box, font_px, thr, ctype):
    rows = []
    for ch in chars:
        bm = render(ch, box, font_px, thr)
        rows.append((ch, bm))
    out = []
    out.append("static const %s %s[%d][%d] = {" % (ctype, name, len(chars), box))
    for ch, bm in rows:
        lines = []
        for r in range(box):
            bits = 0
            for c in range(box):
                if bm[r][c]:
                    bits |= 1 << c
            lines.append("0x%0*X" % (box // 4, bits))
        out.append("    { /* %s */" % ch)
        for i in range(0, box, 8):
            out.append("        " + ", ".join(lines[i:i + 8]) + ",")
        out.append("    },")
    out.append("};")
    return "\n".join(out)


def preview(path, box24, box16, scale=6):
    W, H = 128, 64
    img = Image.new("L", (W, H), 0)
    px = img.load()

    def blit(bm, x0, y0):
        for r, row in enumerate(bm):
            for c, v in enumerate(row):
                if v and 0 <= x0 + c < W and 0 <= y0 + r < H:
                    px[x0 + c, y0 + r] = 255

    t = [render(ch, 24, 24, 110) for ch in "循线校准"]
    b = [render(ch, 16, 16, 110) for ch in "请放在白色上"]
    tw = len(t) * 26 - 2
    x = (W - tw) // 2
    for g in t:
        blit(g, x, 8)
        x += 26
    bw = len(b) * 17 - 1
    x = (W - bw) // 2
    for g in b:
        blit(g, x, 40)
        x += 17
    img.resize((W * scale, H * scale), Image.NEAREST).save(path)

    img2 = Image.new("L", (W, H), 0)
    px = img2.load()

    def blit2(bm, x0, y0):
        for r, row in enumerate(bm):
            for c, v in enumerate(row):
                if v and 0 <= x0 + c < W and 0 <= y0 + r < H:
                    px[x0 + c, y0 + r] = 255
    t2 = [render(ch, 24, 24, 110) for ch in "校准成功"]
    tw = len(t2) * 26 - 2
    x = (W - tw) // 2
    for g in t2:
        blit2(g, x, 20)
        x += 26
    img2.resize((W * scale, H * scale), Image.NEAREST).save(path.replace(".png", "_ok.png"))


header = """/**
 * @file      oled_font_cn.h
 * @brief     校准向导用的 24px / 16px 汉字点阵（1bpp，行优先，bit0 = 最左列）
 * @details   由 tools/gen_oled_cn_font.py 用微软雅黑 Bold 渲染后二值化生成，
 *            不要手工修改。改字请改脚本里的字符表并重新生成。
 */

#ifndef OLED_FONT_CN_H
#define OLED_FONT_CN_H

#include <stdint.h>

/* 索引顺序：循 线 校 准 成 功 失 败 */
#define CN24_IDX_XUN    0U
#define CN24_IDX_XIAN   1U
#define CN24_IDX_JIAO   2U
#define CN24_IDX_ZHUN   3U
#define CN24_IDX_CHENG  4U
#define CN24_IDX_GONG   5U
#define CN24_IDX_SHI    6U
#define CN24_IDX_BAI    7U

/* 索引顺序：请 放 在 白 色 上 黑 失 败 重 试 */
#define CN16_IDX_QING   0U
#define CN16_IDX_FANG   1U
#define CN16_IDX_ZAI    2U
#define CN16_IDX_BAI    3U
#define CN16_IDX_SE     4U
#define CN16_IDX_SHANG  5U
#define CN16_IDX_HEI    6U
#define CN16_IDX_SHI    7U   /* 失 */
#define CN16_IDX_BAI2   8U   /* 败 */
#define CN16_IDX_ZHONG  9U   /* 重 */
#define CN16_IDX_SHI3  10U   /* 试 */

"""

body = dump("g_oled_font_cn24", CN24, 24, 24, 110, "uint32_t") + "\n\n"
body += dump("g_oled_font_cn16", CN16, 16, 16, 110, "uint16_t") + "\n"
header += body + "\n#endif /* OLED_FONT_CN_H */\n"

io.open(OUT, "w",
        encoding="utf-8", newline="\n").write(header)
print("written header, bytes:", len(header))
