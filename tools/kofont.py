#!/usr/bin/env python3
"""
kofont.py — 펌웨어에 쓰인 한글만 골라 GFX 글꼴(ko_font.h)로 만든다.

    python3 tools/kofont.py

* iss_tracker.ino 안의 한글·기호(→)를 자동으로 모은다. 문구를 바꾸면 다시 돌릴 것.
* 글꼴: IBM Plex Sans KR SemiBold (SIL OFL). 없으면 아래 주소에서 받는다.
* 두 배 크기로 그려 두고 보드에서 0.5배로 그린다 → 2x2 샘플링이 그대로 안티에일리어싱이 된다.
* GFX 글꼴은 글자 번호가 이어져야 해서, 아스키(0x20~0x7E) 뒤 0x7F 부터 한글을 차례로 붙이고
  펌웨어의 ko() 가 UTF-8 을 이 번호로 바꾼다.
"""
import os, re, urllib.request
from PIL import Image, ImageDraw, ImageFont

HERE = os.path.dirname(os.path.abspath(__file__))
TTF = os.path.join(HERE, "IBMPlexSansKR-SemiBold.ttf")
URL = "https://github.com/google/fonts/raw/main/ofl/ibmplexsanskr/IBMPlexSansKR-SemiBold.ttf"
INO = os.path.join(HERE, "..", "firmware", "iss_tracker", "iss_tracker.ino")
OUT = os.path.join(HERE, "..", "firmware", "iss_tracker", "ko_font.h")
SPECIAL = {"·": "*", "°": "^"}  # 펌웨어 글꼴 그리기에서 따로 그리는 기호

if not os.path.exists(TTF):
    urllib.request.urlretrieve(URL, TTF)

src = open(INO, encoding="utf-8").read()
src = re.sub(r"//[^\n]*", "", src)                  # 주석은 빼고
src = re.sub(r"/\*.*?\*/", "", src, flags=re.S)
extra = sorted({c for c in src if ord(c) > 0x7E and c not in SPECIAL})
print("한글·기호", len(extra), "자:", "".join(extra))

def build(name, px):
    font = ImageFont.truetype(TTF, px)
    chars = [chr(c) for c in range(0x20, 0x7F)] + extra
    bitmap, glyphs = [], []
    for ch in chars:
        adv = round(font.getlength(ch))
        box = font.getbbox(ch, anchor="ls")
        w, h = max(0, box[2] - box[0]), max(0, box[3] - box[1])
        off = len(bitmap)
        if w and h and ch != " ":
            im = Image.new("L", (w, h), 0)
            ImageDraw.Draw(im).text((-box[0], -box[1]), ch, font=font, fill=255, anchor="ls")
            bits = [1 if im.getpixel((x, y)) >= 110 else 0 for y in range(h) for x in range(w)]
            for i in range(0, len(bits), 8):
                b = 0
                for k, v in enumerate(bits[i:i + 8]): b |= v << (7 - k)
                bitmap.append(b)
        else:
            w = h = 0
        glyphs.append((off, w, h, adv, box[0], box[1], ch))
    out = [f"const uint8_t {name}Bitmaps[] PROGMEM = {{"]
    for i in range(0, len(bitmap), 16):
        out.append("  " + ", ".join(f"0x{b:02X}" for b in bitmap[i:i + 16]) + ",")
    out.append("};")
    out.append(f"const GFXglyph {name}Glyphs[] PROGMEM = {{")
    for off, w, h, adv, xo, yo, ch in glyphs:
        out.append(f"  {{{off}, {w}, {h}, {adv}, {xo}, {yo}}},  // {'0x%02X' % ord(ch) if ord(ch) < 0x7F else ch}")
    out.append("};")
    out.append(f"const GFXfont {name} PROGMEM = {{(uint8_t *){name}Bitmaps, (GFXglyph *){name}Glyphs, 0x20, 0x{0x7E + len(extra):X}, {round(px * 1.4)}}};")
    print(name, px, "px:", len(bitmap), "바이트")
    return "\n".join(out)

h = ["// tools/kofont.py 가 만든 파일 — 손으로 고치지 말 것",
     "// IBM Plex Sans KR SemiBold, SIL Open Font License 1.1",
     "#pragma once",
     f"#define KO_N {len(extra)}",
     "const uint16_t KO_CP[KO_N] = {" + ", ".join(f"0x{ord(c):04X}" for c in extra) + "};  // 0x7F 부터 차례로",
     build("KoS", 24), build("KoL", 40)]
open(OUT, "w").write("\n".join(h) + "\n")
print("→", os.path.relpath(OUT))
