/*
 * iss_tracker — 국제우주정거장 추적기 (NU40 DK + GC9A01 원형 240x240)
 *
 * 맥(host/iss.py)이 ISS 위치·다음 통과·탑승자를 USB 시리얼로 보내면 보드가 그린다.
 * 위치 사이(5초)는 보드가 원궤도로 이어서 계산한다.
 *
 * 배선 (sky-radar 와 같음)
 *   VCC→3V3, GND→GND, SCL→47, SDA→45, CS→44, DC→43, RES→42, BLK→41
 *
 * 시리얼 한 줄 명령 (115200)
 *   T <UTC epoch>                       시간
 *   Z <초>                              시간대 (KST = 32400)
 *   L <위도> <경도>                      내 위치
 *   P <ts> <lat> <lon> <alt> <vel> <asc> ISS 위치 (asc=1 북쪽으로 가는 중)
 *   N <시작ts> <끝ts> <최대고도각>        다음 통과
 *   Q az,el az,el ...                   통과 경로 (N 다음에)
 *   C                                   탑승자 비우기
 *   M <우주선>|<이름>|<기관>             탑승자 한 명
 *   S <0|1|2|3|9>                       화면 고정, 9 = 자동
 */
#include <Arduino_GFX_Library.h>
#include "FreeMonoBold9pt7b.h"
#include "FreeMonoBold12pt7b.h"
#include "FreeMonoBold18pt7b.h"
#include "FreeSansBold24pt7b.h"
#include "Org_01.h"
#include "land.h"
#include "types.h"

#define LCD_CS  44
#define LCD_DC  43
#define LCD_RST 42
#define LCD_BLK 41

Arduino_DataBus *bus = new Arduino_HWSPI(LCD_DC, LCD_CS);
Arduino_GFX *panel = new Arduino_GC9A01(bus, LCD_RST, 0, true);
Arduino_Canvas *g = new Arduino_Canvas(240, 240, panel);
uint16_t *fb;

// ── 색 ───────────────────────────────────────────
constexpr uint16_t H(uint32_t h) { return ((h >> 8) & 0xF800) | ((h >> 5) & 0x07E0) | ((h & 0xFF) >> 3); }
const uint16_t C_CYAN = H(0x4fe3ff), C_ORANGE = H(0xff8a3d), C_MAG = H(0xff4fa3), C_VIOLET = H(0x8b6cff),
               C_FG = H(0xe6ecff), C_MUTED = H(0x8c97c2), C_LAND = H(0x41e0c0), C_LANDDIM = H(0x1f8f9a),
               C_NIGHT = H(0x16305e), C_CITY = H(0xffb547), C_GREEN = H(0x5fffa0), C_GOLD = H(0xffc24a),
               C_WHITE = 0xFFFF, C_RED = H(0xff6b6b), C_PANEL = H(0xffd27a);

static inline uint16_t mix(uint16_t a, uint16_t b, int t) {  // t 0..256 : a→b
  int ar = a >> 11, ag = (a >> 5) & 63, ab = a & 31;
  int br = b >> 11, bg = (b >> 5) & 63, bb = b & 31;
  return ((ar + (((br - ar) * t) >> 8)) << 11) | ((ag + (((bg - ag) * t) >> 8)) << 5) | (ab + (((bb - ab) * t) >> 8));
}
static inline uint16_t mixf(uint16_t a, uint16_t b, float t) { return mix(a, b, (int)(constrain(t, 0.f, 1.f) * 256)); }
static inline void blendPx(int x, int y, uint16_t c, int a) {
  if ((unsigned)x >= 240 || (unsigned)y >= 240 || a <= 0) return;
  uint16_t &p = fb[y * 240 + x]; p = a >= 256 ? c : mix(p, c, a);
}
static inline void setPx(int x, int y, uint16_t c) { if ((unsigned)x < 240 && (unsigned)y < 240) fb[y * 240 + x] = c; }

// ── 시간 ─────────────────────────────────────────
double epochBase = 0; uint32_t epochAtMs = 0; long tzSec = 32400;
double nowUtc() { return epochBase + (millis() - epochAtMs) / 1000.0; }

// ── 궤도 ─────────────────────────────────────────
const float DEG = PI / 180, RE = 6371, INC = 51.64f * DEG, PERIOD = 92.9f * 60, WE = 2 * PI / 86164.0f, PREC = -5.0f * DEG / 86400;
const double LAUNCH = 911544000;  // 1998-11-20 06:40 UTC 자랴 발사
struct Fix { double ts = 0; float lat, lon, alt = 416, vel = 27600, u0, om0; bool ok = false; } fix;
float meLat = 37.50f * DEG, meLon = 127.06f * DEG;

void setFix(double ts, float latd, float lond, float alt, float vel, bool asc) {
  fix.ts = ts; fix.lat = latd * DEG; fix.lon = lond * DEG; fix.alt = alt; fix.vel = vel;
  float s = constrain(sinf(fix.lat) / sinf(INC), -1.f, 1.f);
  fix.u0 = asc ? asinf(s) : PI - asinf(s);
  fix.om0 = fix.lon - atan2f(cosf(INC) * sinf(fix.u0), cosf(fix.u0));
  fix.ok = true;
}
float wrapA(float a) { return atan2f(sinf(a), cosf(a)); }
Pos iss(double t) {
  float dt = (float)(t - fix.ts), u = fix.u0 + 2 * PI * dt / PERIOD;
  Pos p; p.lat = asinf(sinf(INC) * sinf(u));
  p.lon = wrapA(fix.om0 - WE * dt + PREC * dt + atan2f(cosf(INC) * sinf(u), cosf(u)));
  p.alt = fix.alt; p.phase = fmodf(fmodf(u, 2 * PI) + 2 * PI, 2 * PI);
  return p;
}
V3 vec(float la, float lo) { return { cosf(la) * cosf(lo), cosf(la) * sinf(lo), sinf(la) }; }
float dot(V3 a, V3 b) { return a.x * b.x + a.y * b.y + a.z * b.z; }
V3 sunVec(double t) {
  long days = (long)(t / 86400); long z = days + 719468, era = z / 146097, doe = z - era * 146097;
  long yoe = (doe - doe / 1460 + doe / 36524 - doe / 146096) / 365;
  long doyM = doe - (365 * yoe + yoe / 4 - yoe / 100);           // 3월 1일 기준
  float doy = (doyM + 59) % 365;                                   // 1월 1일 기준(근사)
  float dec = -23.44f * cosf(2 * PI * (doy + 10) / 365) * DEG;
  float h = fmod(t, 86400.0) / 3600;
  return vec(dec, wrapA((12 - h) * 15 * DEG));
}
bool sunlit(Pos p, V3 s) {
  V3 v = vec(p.lat, p.lon); float c = dot(v, s);
  return c > 0 || (RE + p.alt) * sqrtf(1 - c * c) > RE;
}
float groundKm(float la1, float lo1, float la2, float lo2) {
  return RE * acosf(constrain(dot(vec(la1, lo1), vec(la2, lo2)), -1.f, 1.f));
}
void lookAngles(Pos p, float &el, float &az) {
  float r = RE + p.alt; V3 v = vec(p.lat, p.lon), o = vec(meLat, meLon);
  V3 d = { r * v.x - RE * o.x, r * v.y - RE * o.y, r * v.z - RE * o.z };
  V3 e = { -sinf(meLon), cosf(meLon), 0 };
  V3 n = { -sinf(meLat) * cosf(meLon), -sinf(meLat) * sinf(meLon), cosf(meLat) };
  float E = dot(d, e), N = dot(d, n), U = dot(d, o);
  el = atan2f(U, sqrtf(E * E + N * N)) / DEG; az = fmodf(atan2f(E, N) / DEG + 360, 360);
}

// ── 통과 · 탑승자 ─────────────────────────────────
#define MAXQ 90
struct Pass { double start = 0, end = 0; float maxEl = 0; int n = 0; float az[MAXQ], el[MAXQ]; bool ok = false; } pass;
struct Crew { char craft[12], name[24], ag[6]; };
#define MAXC 16
Crew crew[MAXC]; int nCrew = 0;
int screenMode = 9;

// ── 글자 (GFX 글꼴을 늘리고 돌려서, 2x2 샘플 안티에일리어싱) ─────
Paint P1(uint16_t c) { return { c, c, 0, 0, 1 }; }
float capH(const GFXfont *f) { return f->glyph['A' - f->first].height; }
float advOf(const GFXfont *f, char ch, float sc) {
  if (ch == '*') return 5 * sc; if (ch == '^') return 4 * sc;
  if (ch < f->first || ch > f->last) return 0;
  return f->glyph[ch - f->first].xAdvance * sc;
}
float textW(const char *s, const GFXfont *f, float sc, float sp) {
  float w = 0; for (; *s; s++) w += advOf(f, *s, sc) + sp; return w - sp;
}
void drawGlyph(const GFXfont *f, char ch, float ox, float oy, float Tx, float Ty, float sc, Paint &p, float uBase) {
  float Dx = -Ty, Dy = Tx;
  if (ch == '*' || ch == '^') {
    float ch2 = capH(f) * sc;
    float cx = ox + (ch == '*' ? 2.5f * sc : 2 * sc) * Tx + (ch == '*' ? -ch2 / 2 : -ch2 + 1.4f * sc) * Dx;
    float cy = oy + (ch == '*' ? 2.5f * sc : 2 * sc) * Ty + (ch == '*' ? -ch2 / 2 : -ch2 + 1.4f * sc) * Dy;
    float rr = ch == '*' ? 0.9f * sc : 1.4f * sc;
    for (int y = (int)(cy - rr - 1); y <= cy + rr + 1; y++)
      for (int x = (int)(cx - rr - 1); x <= cx + rr + 1; x++) {
        float d = sqrtf((x - cx) * (x - cx) + (y - cy) * (y - cy));
        float a = ch == '*' ? (rr + .5f - d) : (0.6f - fabsf(d - rr) + .4f);
        if (a > 0) blendPx(x, y, p.c1, (int)(min(a, 1.f) * 256));
      }
    return;
  }
  if (ch < f->first || ch > f->last) return;
  GFXglyph *gl = &f->glyph[ch - f->first];
  int w = gl->width, h = gl->height; if (!w || !h) return;
  const uint8_t *bm = f->bitmap + gl->bitmapOffset;
  float u0 = gl->xOffset * sc, u1 = (gl->xOffset + w) * sc, v0 = gl->yOffset * sc, v1 = (gl->yOffset + h) * sc;
  float xs[4] = { ox + u0 * Tx + v0 * Dx, ox + u1 * Tx + v0 * Dx, ox + u0 * Tx + v1 * Dx, ox + u1 * Tx + v1 * Dx };
  float ys[4] = { oy + u0 * Ty + v0 * Dy, oy + u1 * Ty + v0 * Dy, oy + u0 * Ty + v1 * Dy, oy + u1 * Ty + v1 * Dy };
  int x0 = floorf(min(min(xs[0], xs[1]), min(xs[2], xs[3]))) - 1, x1 = ceilf(max(max(xs[0], xs[1]), max(xs[2], xs[3]))) + 1;
  int y0 = floorf(min(min(ys[0], ys[1]), min(ys[2], ys[3]))) - 1, y1 = ceilf(max(max(ys[0], ys[1]), max(ys[2], ys[3]))) + 1;
  x0 = max(x0, 0); y0 = max(y0, 0); x1 = min(x1, 239); y1 = min(y1, 239);
  const float SO[4][2] = { { .25f, .25f }, { .75f, .25f }, { .25f, .75f }, { .75f, .75f } };
  for (int y = y0; y <= y1; y++)
    for (int x = x0; x <= x1; x++) {
      int cov = 0; float uu = 0;
      for (int k = 0; k < 4; k++) {
        float dx = x + SO[k][0] - ox, dy = y + SO[k][1] - oy;
        float u = (dx * Tx + dy * Ty) / sc - gl->xOffset, v = (dx * Dx + dy * Dy) / sc - gl->yOffset;
        int bx = (int)floorf(u), by = (int)floorf(v);
        if (bx < 0 || by < 0 || bx >= w || by >= h) continue;
        int idx = by * w + bx;
        if (bm[idx >> 3] & (0x80 >> (idx & 7))) { cov++; uu = u; }
      }
      if (!cov) continue;
      uint16_t c = p.c1;
      if (p.mode == 1) c = mixf(p.c1, p.c2, (uBase + (gl->xOffset + uu) * sc) / p.len);
      else if (p.mode == 2) c = mixf(p.c1, p.c2, (y - p.a) / p.len);
      blendPx(x, y, c, cov * 64);
    }
}
// y = 글자 바닥선, align -1 왼쪽 0 가운데 1 오른쪽
float drawText(const char *s, float x, float y, const GFXfont *f, float sc, Paint p, int align = 0, float sp = 0) {
  float w = textW(s, f, sc, sp), cur = align == 0 ? x - w / 2 : align < 0 ? x : x - w;
  if (p.mode == 1) { p.len = w; }
  float start = cur;
  for (; *s; s++) { drawGlyph(f, *s, cur, y, 1, 0, sc, p, cur - start); cur += advOf(f, *s, sc) + sp; }
  return w;
}
// 원 둘레 글자. bottom=false 면 위쪽(바깥으로 서서 시계방향), true 면 아래쪽(왼→오)
void arcText(const char *s, float r, float center, bool bottom, const GFXfont *f, float sc, Paint p, float sp) {
  float total = textW(s, f, sc, sp) / r, a = bottom ? center + total / 2 : center - total / 2;
  for (; *s; s++) {
    float w = (advOf(f, *s, sc) + sp) / r;
    float am = bottom ? a - w / 2 : a + w / 2;
    float Tx = bottom ? sinf(am) : -sinf(am), Ty = bottom ? -cosf(am) : cosf(am);
    float hw = advOf(f, *s, sc) / 2;
    float ox = 120 + r * cosf(am) - hw * Tx, oy = 120 + r * sinf(am) - hw * Ty;
    drawGlyph(f, *s, ox, oy, Tx, Ty, sc, p, 0);
    a = bottom ? a - w : a + w;
  }
}

// ── 공통 그림 ────────────────────────────────────
uint16_t bgLut[4][272];
void makeBg(int i, uint32_t c1, uint32_t c2) {
  for (int k = 0; k < 272; k++) { float d = sqrtf(k * 128 + 64); bgLut[i][k] = mixf(H(c1), H(c2), (d - 10) / 160); }
}
void fillBg(int i) {
  for (int y = 0; y < 240; y++) { int dy = y - 100; for (int x = 0; x < 240; x++) { int dx = x - 120; fb[y * 240 + x] = bgLut[i][min((dx * dx + dy * dy) >> 7, 271)]; } }
}
struct Star { uint8_t x, y; float k; uint8_t big; };
Star stars[90];
void drawStars(float t) {
  for (auto &s : stars) {
    int b = (int)((.35f + .5f * fabsf(sinf(t * .7f + s.k * 20))) * 256);
    uint16_t c = s.k > .85f ? H(0xbcd0ff) : C_WHITE;
    blendPx(s.x, s.y, c, b); if (s.big) blendPx(s.x + 1, s.y, c, b / 2);
  }
}
void glow(float x, float y, float r, uint16_t c, float amax) {
  for (int yy = (int)(y - r); yy <= y + r; yy++)
    for (int xx = (int)(x - r); xx <= x + r; xx++) {
      float d = sqrtf((xx - x) * (xx - x) + (yy - y) * (yy - y)); if (d >= r) continue;
      float a = 1 - d / r; blendPx(xx, yy, c, (int)(a * a * amax * 256));
    }
}
void discAA(float x, float y, float r, uint16_t c) {
  for (int yy = (int)(y - r - 1); yy <= y + r + 1; yy++)
    for (int xx = (int)(x - r - 1); xx <= x + r + 1; xx++) {
      float a = r + .5f - sqrtf((xx - x) * (xx - x) + (yy - y) * (yy - y));
      if (a > 0) blendPx(xx, yy, c, (int)(min(a, 1.f) * 256));
    }
}
void quad(float x, float y, float ca, float sa, float lx, float ly, float w, float h, uint16_t c) {
  float px[4] = { lx, lx + w, lx + w, lx }, py[4] = { ly, ly, ly + h, ly + h }; int X[4], Y[4];
  for (int i = 0; i < 4; i++) { X[i] = lroundf(x + px[i] * ca - py[i] * sa); Y[i] = lroundf(y + px[i] * sa + py[i] * ca); }
  g->fillTriangle(X[0], Y[0], X[1], Y[1], X[2], Y[2], c); g->fillTriangle(X[0], Y[0], X[2], Y[2], X[3], Y[3], c);
}
void issIcon(float x, float y, float s, float ang) {
  glow(x, y, 15 * s, C_CYAN, .55f);
  float ca = cosf(ang) * s, sa = sinf(ang) * s;
  quad(x, y, ca, sa, -10, -.8f, 20, 1.6f, C_WHITE);
  const float px[4] = { -10, -6.6f, 4.6f, 8 };
  for (float p : px) { quad(x, y, ca, sa, p, -6, 2.2f, 4.4f, C_PANEL); quad(x, y, ca, sa, p, 1.6f, 2.2f, 4.4f, C_PANEL); }
  quad(x, y, ca, sa, -1.4f, -3.4f, 2.8f, 6.8f, C_WHITE);
}
void thickLine(float x0, float y0, float x1, float y1, uint16_t c, float r) {
  float L = sqrtf((x1 - x0) * (x1 - x0) + (y1 - y0) * (y1 - y0)); int n = max(1, (int)(L / .7f));
  for (int i = 0; i <= n; i++) discAA(x0 + (x1 - x0) * i / n, y0 + (y1 - y0) * i / n, r, c);
}
const char *dirName(float az) { static const char *d[8] = { "N", "NE", "E", "SE", "S", "SW", "W", "NW" }; return d[((int)lroundf(az / 45)) % 8]; }

// ── 01 궤도 ──────────────────────────────────────
float sinR[90], cosR[90], sinC[180], cosC[180];
uint16_t oceanLut[64];
void drawOrbit(double t) {
  Pos p = iss(t); V3 s = sunVec(t);
  const int cx = 120, cy = 120; const float R = 84;
  fillBg(0); drawStars((float)fmod(t, 10000.0));
  float sl = sinf(p.lat), cl = cosf(p.lat), so = sinf(p.lon), co = cosf(p.lon);
  V3 E = { -so, co, 0 }, N = { -sl * co, -sl * so, cl }, U = { cl * co, cl * so, sl };
  V3 sv = { dot(s, E), dot(s, N), dot(s, U) };
  const float hx = cx - R * .35f, hy = cy - R * .4f;
  const uint16_t atm1 = C_CYAN, atm2 = C_VIOLET;
  for (int y = cy - R - 17; y <= cy + R + 17; y++)
    for (int x = cx - R - 17; x <= cx + R + 17; x++) {
      float dx = x - cx, dy = y - cy, d = sqrtf(dx * dx + dy * dy);
      if (d < R) {
        float nx = dx / R, ny = -dy / R, nz = sqrtf(max(0.f, 1 - nx * nx - ny * ny));
        float tt = sqrtf((x - hx) * (x - hx) + (y - hy) * (y - hy)) / (R * 1.45f);
        uint16_t c = oceanLut[min(63, (int)(tt * 63))];
        float L = nx * sv.x + ny * sv.y + nz * sv.z;
        float f = L > .08f ? 1 : L < -.05f ? .36f : .36f + .64f * (L + .05f) / .13f;
        c = mix(0, c, (int)(f * 256));
        if (d > R - 1) c = mix(c, H(0x96dcff), (int)((d - (R - 1)) * 140));
        fb[y * 240 + x] = c;
      } else if (d < R + 16) {
        float tt = (d - R + 2) / 18; int a; uint16_t c;
        if (tt < .35f) { c = mixf(atm1, atm2, tt / .35f); a = (int)((.55f - (.33f * tt / .35f)) * 256); }
        else { c = atm2; a = (int)(.22f * (1 - (tt - .35f) / .65f) * 256); }
        blendPx(x, y, c, a);
      }
    }
  // 육지 점
  for (int row = 0; row < 90; row++)
    for (int col = 0; col < 180; col++) {
      int i = row * 180 + col;
      if (!(LAND[i >> 3] >> (i & 7) & 1)) continue;
      V3 v = { cosR[row] * cosC[col], cosR[row] * sinC[col], sinR[row] };
      float vz = dot(v, U); if (vz <= .02f) continue;
      float X = cx + R * dot(v, E), Y = cy - R * dot(v, N);
      bool day = dot(v, s) > -.05f;
      int ix = lroundf(X), iy = lroundf(Y);
      if (day) {
        uint16_t c = vz > .55f ? C_LAND : C_LANDDIM;
        if (vz > .45f) { setPx(ix, iy, c); setPx(ix + 1, iy, c); setPx(ix, iy + 1, c); setPx(ix + 1, iy + 1, c); }
        else setPx(ix, iy, c);
      } else if (((uint32_t)i * 2654435761u) % 100 < 16) {
        setPx(ix, iy, C_CITY); if (vz > .5f) blendPx(ix + 1, iy, C_CITY, 110);
      } else {
        setPx(ix, iy, C_NIGHT); if (vz > .45f) { setPx(ix + 1, iy, C_NIGHT); setPx(ix, iy + 1, C_NIGHT); setPx(ix + 1, iy + 1, C_NIGHT); }
      }
    }
  // 궤적: 지나온 길 파랑, 갈 길 주황 점선
  float px = 0, py = 0, pz = -1; const uint16_t sea = H(0x0c2766);
  for (int k = -55; k <= 55; k++) {
    Pos q = iss(t + k * 60.0); V3 v = vec(q.lat, q.lon);
    float X = cx + R * dot(v, E), Y = cy - R * dot(v, N), Z = dot(v, U);
    if (pz > 0 && Z > 0) {
      if (k <= 0) { uint16_t c = mixf(sea, C_CYAN, .15f + .8f * (1 + k / 55.f)); g->drawLine(px, py, X, Y, c); g->drawLine(px + 1, py, X + 1, Y, c); }
      else if (k % 2 == 0) { uint16_t c = mixf(C_ORANGE, sea, k / 70.f); g->drawLine(px, py, X, Y, c); g->drawLine(px + 1, py, X + 1, Y, c); }
    }
    px = X; py = Y; pz = Z;
  }
  // 내 위치
  V3 m = vec(meLat, meLon);
  if (dot(m, U) > 0) {
    float X = cx + R * dot(m, E), Y = cy - R * dot(m, N), rr = 3.6f + sinf(t * 3) * .7f;
    g->drawCircle(lroundf(X), lroundf(Y), lroundf(rr), C_MAG);
    drawText("SEOUL", X + 6, Y - 4, &Org_01, 1.2f, P1(C_MAG), -1, .4f);
  }
  Pos f = iss(t + 30); V3 fv = vec(f.lat, f.lon);
  issIcon(cx, cy, 1, atan2f(-(R * dot(fv, N)), R * dot(fv, E)));
  arcText("ISS * ZARYA * NORAD 25544", 106, -PI / 2, false, &Org_01, 1.5f, P1(C_CYAN), 1.6f);
  char b[48]; snprintf(b, sizeof b, "%.1f KM * %d,%03d KM/H", p.alt, (int)fix.vel / 1000, (int)fix.vel % 1000);
  arcText(b, 115, PI / 2, true, &Org_01, 1.5f, P1(C_FG), 1.2f);
}

// ── 02 원격측정 ──────────────────────────────────
void drawTele(double t) {
  Pos p = iss(t); V3 s = sunVec(t); bool lit = sunlit(p, s);
  fillBg(1);
  uint16_t tk = mixf(H(0x05071a), C_MUTED, .55f);
  for (int i = 0; i < 60; i++) {
    float a = i / 60.f * 2 * PI, l = i % 5 == 0 ? 109 : 112;
    g->drawLine(120 + l * cosf(a), 120 + l * sinf(a), 120 + 117 * cosf(a), 120 + 117 * sinf(a), tk);
  }
  float prog = p.phase / (2 * PI); const uint16_t track = H(0x1c2752);
  for (int y = 14; y < 226; y++)
    for (int x = 14; x < 226; x++) {
      float dx = x - 120, dy = y - 120, d = sqrtf(dx * dx + dy * dy), e = fabsf(d - 103);
      if (e > 3.6f) continue;
      float a = min(1.f, 3.6f - e);
      float fr = fmodf(atan2f(dy, dx) + PI / 2 + 2 * PI, 2 * PI) / (2 * PI);
      uint16_t c = fr <= prog ? (fr < .5f ? mixf(C_CYAN, C_VIOLET, fr * 2) : mixf(C_VIOLET, C_MAG, fr * 2 - 1)) : track;
      blendPx(x, y, c, (int)(a * 256));
    }
  float a1 = -PI / 2 + p.phase, ex = 120 + 103 * cosf(a1), ey = 120 + 103 * sinf(a1);
  discAA(120, 17, 3, C_CYAN);
  glow(ex, ey, 11, C_MAG, .8f); discAA(ex, ey, 3.4f, C_WHITE);

  char b[48]; double met = t - LAUNCH; long dd = (long)(met / 86400); long ss = (long)fmod(met, 86400.0);
  drawText("MISSION ELAPSED", 120, 41, &Org_01, 1.3f, P1(C_ORANGE), 0, 1.2f);
  snprintf(b, sizeof b, "D+%ld,%03ld  %02ld:%02ld:%02ld", dd / 1000, dd % 1000, ss / 3600, ss / 60 % 60, ss % 60);
  drawText(b, 120, 54, &Org_01, 1.6f, P1(C_FG), 0, .6f);
  drawText("VELOCITY", 120, 75, &Org_01, 1.3f, P1(C_MUTED), 0, 1.4f);
  snprintf(b, sizeof b, "%d,%03d", (int)fix.vel / 1000, (int)fix.vel % 1000);
  drawText(b, 120, 106, &FreeMonoBold18pt7b, 1, { C_CYAN, C_WHITE, 1, 0, 1 }, 0, -1);
  drawText("KM/H", 120, 118, &Org_01, 1.3f, P1(C_MUTED), 0, 1);
  uint16_t ln = mixf(H(0x070a1c), C_MUTED, .3f);
  g->drawFastHLine(50, 128, 140, ln); g->drawFastVLine(120, 134, 26, ln);
  drawText("ALTITUDE", 84, 141, &Org_01, 1.2f, P1(C_MUTED), 0, 1); drawText("PERIOD", 156, 141, &Org_01, 1.2f, P1(C_MUTED), 0, 1);
  snprintf(b, sizeof b, "%.1f", p.alt); drawText(b, 80, 157, &FreeMonoBold9pt7b, 1, P1(C_FG), 0, -1);
  drawText("92.9", 152, 157, &FreeMonoBold9pt7b, 1, P1(C_FG), 0, -1);
  drawText("KM", 108, 157, &Org_01, 1.1f, P1(C_MUTED), 0, .5f); drawText("MIN", 178, 157, &Org_01, 1.1f, P1(C_MUTED), 0, .5f);
  snprintf(b, sizeof b, "%05.2f^%c  %06.2f^%c", fabsf(p.lat / DEG), p.lat >= 0 ? 'N' : 'S', fabsf(p.lon / DEG), p.lon >= 0 ? 'E' : 'W');
  drawText(b, 120, 175, &Org_01, 1.5f, P1(C_CYAN), 0, .5f);
  int km = (int)groundKm(p.lat, p.lon, meLat, meLon);
  if (km >= 1000) snprintf(b, sizeof b, "%d,%03d KM FROM SEOUL", km / 1000, km % 1000); else snprintf(b, sizeof b, "%d KM FROM SEOUL", km);
  drawText(b, 120, 188, &Org_01, 1.2f, P1(C_MUTED), 0, .6f);
  const char *lab = lit ? "DAYLIGHT" : "ECLIPSE"; uint16_t col = lit ? C_GOLD : C_VIOLET;
  int w = (int)textW(lab, &Org_01, 1.3f, 1.2f) + 24;
  g->fillRoundRect(120 - w / 2, 195, w, 15, 7, mixf(H(0x070a1c), col, lit ? .16f : .22f));
  g->drawRoundRect(120 - w / 2, 195, w, 15, 7, col);
  discAA(120 - w / 2 + 9, 202.5f, 2.8f, col);
  if (!lit) discAA(120 - w / 2 + 10.4f, 201.3f, 2.3f, mixf(H(0x070a1c), col, .22f));
  drawText(lab, 120 - w / 2 + 15, 205.5f, &Org_01, 1.3f, P1(col), -1, 1.2f);
}

// ── 03 다음 통과 ─────────────────────────────────
void domeXY(float az, float el, float cx, float cy, float R, float &x, float &y) {
  float r = R * (90 - max(el, 0.f)) / 90, a = az * DEG; x = cx + r * sinf(a); y = cy - r * cosf(a);
}
void fmtHMS(char *b, size_t n, long s) { snprintf(b, n, "T-%02ld:%02ld:%02ld", s / 3600, s / 60 % 60, s % 60); }
void drawPass(double t) {
  fillBg(2); drawStars((float)fmod(t, 10000.0) * .5f);
  const float cx = 120, cy = 84, R = 60;
  for (int y = cy - R; y <= cy + R; y++)
    for (int x = cx - R; x <= cx + R; x++) {
      float d = sqrtf((x - cx) * (x - cx) + (y - cy) * (y - cy)); if (d > R) continue;
      float f = d / R; blendPx(x, y, mixf(C_CYAN, C_VIOLET, f), (int)((.10f + .12f * f) * 256));
    }
  uint16_t rc = mixf(H(0x0b1640), C_MUTED, .45f);
  g->drawCircle(cx, cy, R, rc); g->drawCircle(cx, cy, R * 2 / 3, rc); g->drawCircle(cx, cy, R / 3, rc);
  g->drawFastHLine(cx - R, cy, 2 * R, rc); g->drawFastVLine(cx, cy - R, 2 * R, rc);
  drawText("N", cx, cy - R - 4, &Org_01, 1.4f, P1(C_MUTED)); drawText("S", cx, cy + R + 10, &Org_01, 1.4f, P1(C_MUTED));
  drawText("E", cx + R + 7, cy + 3, &Org_01, 1.4f, P1(C_MUTED)); drawText("W", cx - R - 7, cy + 3, &Org_01, 1.4f, P1(C_MUTED));
  drawText("30^", cx + 9, cy - R * 2 / 3 - 2, &Org_01, 1.05f, P1(C_MUTED)); drawText("60^", cx + 9, cy - R / 3 - 2, &Org_01, 1.05f, P1(C_MUTED));
  char b[48];
  if (!pass.ok) { drawText("CALCULATING PASS", 120, 186, &Org_01, 1.4f, P1(C_MUTED), 0, 1.2f); return; }
  float lx = 0, ly = 0;
  for (int i = 0; i < pass.n; i++) {
    float x, y; domeXY(pass.az[i], pass.el[i], cx, cy, R, x, y);
    if (i) thickLine(lx, ly, x, y, mixf(C_CYAN, C_MAG, (float)i / pass.n), 1.1f);
    lx = x; ly = y;
  }
  if (pass.n >= 3) {
    float bx, by; domeXY(pass.az[pass.n - 3], pass.el[pass.n - 3], cx, cy, R, bx, by);
    float an = atan2f(ly - by, lx - bx);
    g->fillTriangle(lx + 6 * cosf(an), ly + 6 * sinf(an), lx + 5 * cosf(an + 2.4f), ly + 5 * sinf(an + 2.4f), lx + 5 * cosf(an - 2.4f), ly + 5 * sinf(an - 2.4f), C_MAG);
    float sx, sy; domeXY(pass.az[0], pass.el[0], cx, cy, R, sx, sy); discAA(sx, sy, 2.6f, C_CYAN);
  }
  bool live = t >= pass.start && t <= pass.end;
  if (live) {
    float el, az; lookAngles(iss(t), el, az);
    float x, y; domeXY(az, el, cx, cy, R, x, y); issIcon(x, y, .8f, 0);
    drawText("OVERHEAD NOW", 120, 170, &Org_01, 1.5f, P1(C_GREEN), 0, 1.4f);
    snprintf(b, sizeof b, "EL %d^", (int)lroundf(max(el, 0.f)));
    drawText(b, 120, 195, &FreeMonoBold12pt7b, 1, { C_CYAN, C_MAG, 1, 0, 1 }, 0, 0);
    snprintf(b, sizeof b, "AZ %d^ %s", (int)lroundf(az), dirName(az)); drawText(b, 120, 210, &Org_01, 1.4f, P1(C_FG), 0, .8f);
  } else {
    double st = pass.start + tzSec; long sd = (long)fmod(st, 86400.0);
    snprintf(b, sizeof b, "NEXT PASS  %02ld:%02ld", sd / 3600, sd / 60 % 60);
    drawText(b, 120, 170, &Org_01, 1.5f, P1(C_ORANGE), 0, 1.2f);
    long rem = (long)(pass.start - t); if (rem < 0) rem = 0; fmtHMS(b, sizeof b, rem);
    drawText(b, 120, 194, &FreeMonoBold12pt7b, 1, { C_CYAN, C_MAG, 1, 0, 1 }, 0, -1);
    snprintf(b, sizeof b, "MAX %d^ * %s > %s", (int)lroundf(pass.maxEl), dirName(pass.az[0]), dirName(pass.az[pass.n - 1]));
    drawText(b, 120, 210, &Org_01, 1.4f, P1(C_FG), 0, .8f);
  }
}

// ── 04 탑승자 ────────────────────────────────────
void drawCrew(double t) {
  fillBg(3); float tf = (float)fmod(t, 10000.0); drawStars(tf * .3f);
  const float ra = -.35f; uint16_t oc = mixf(H(0x120e3a), C_VIOLET, .4f);
  for (int i = 0; i < 360; i++) {
    float a = i * DEG; int x = lroundf(120 + 112 * cosf(a) * cosf(ra) - 34 * sinf(a) * sinf(ra)), y = lroundf(120 + 112 * cosf(a) * sinf(ra) + 34 * sinf(a) * cosf(ra));
    setPx(x, y, oc);
  }
  float oa = tf * .6f; float ox = 120 + 112 * cosf(oa) * cosf(ra) - 34 * sinf(oa) * sinf(ra), oy = 120 + 112 * cosf(oa) * sinf(ra) + 34 * sinf(oa) * cosf(ra);
  glow(ox, oy, 6, C_CYAN, .6f); discAA(ox, oy, 1.8f, C_CYAN);
  drawText("PEOPLE IN SPACE", 120, 34, &Org_01, 1.3f, P1(C_MUTED), 0, 1.4f);
  char b[8]; snprintf(b, sizeof b, "%d", nCrew);
  drawText(b, 120, 76, &FreeSansBold24pt7b, 1, { C_WHITE, C_VIOLET, 2, 42, 34 }, 0, 0);
  if (!nCrew) { drawText("LOADING", 120, 110, &Org_01, 1.4f, P1(C_MUTED), 0, 1.2f); return; }
  // 우주선별로 묶기 (도착 순서 유지)
  float y = 94; char done[MAXC] = { 0 };
  for (int i = 0; i < nCrew; i++) {
    if (done[i]) continue;
    int cnt = 0; for (int j = i; j < nCrew; j++) if (!strcmp(crew[j].craft, crew[i].craft)) cnt++;
    char hd[24]; snprintf(hd, sizeof hd, "%s * %d", crew[i].craft, cnt);
    drawText(hd, 120, y, &Org_01, 1.2f, P1(C_ORANGE), 0, 1.2f); y += 10.5f;
    for (int j = i; j < nCrew; j++) {
      if (strcmp(crew[j].craft, crew[i].craft)) continue; done[j] = 1;
      drawText(crew[j].name, 146, y, &Org_01, 1.3f, P1(C_FG), 1, .3f);
      const char *ag = crew[j].ag;
      uint16_t c = !strcmp(ag, "NASA") ? C_CYAN : !strcmp(ag, "CSA") ? C_MAG : !strcmp(ag, "RFSA") ? C_GOLD : !strcmp(ag, "CNSA") ? C_RED : !strcmp(ag, "ESA") ? H(0x6fa8ff) : !strcmp(ag, "JAXA") ? C_GREEN : C_MUTED;
      drawText(ag, 151, y, &Org_01, 1.3f, P1(c), -1, .3f); y += 9.2f;
    }
    y += 3.5f;
  }
}

// ── 기다림 ───────────────────────────────────────
void drawWait(double t) {
  fillBg(0); float tf = (float)fmod(t, 10000.0); drawStars(tf);
  for (int i = 0; i < 40; i++) {
    float a = tf * 3 + i * 0.06f; discAA(120 + 40 * cosf(a), 110 + 40 * sinf(a), 2.2f, mixf(H(0x0a1238), C_CYAN, i / 40.f));
  }
  drawText("ACQUIRING SIGNAL", 120, 176, &Org_01, 1.5f, P1(C_CYAN), 0, 1.4f);
  drawText("ISS * NORAD 25544", 120, 192, &Org_01, 1.2f, P1(C_MUTED), 0, 1.2f);
}

// ── 시리얼 ───────────────────────────────────────
char line[1400]; int lineLen = 0;
// 이 보드의 sscanf 는 소수(%f)를 못 읽는다 → strtod 로 숫자만 차례로 뽑는다 (공백·쉼표 구분)
int nums(const char *s, double *out, int maxn) {
  int n = 0; char *e;
  while (*s && n < maxn) {
    while (*s == ' ' || *s == ',') s++;
    if (!*s) break;
    double v = strtod(s, &e); if (e == s) break;
    out[n++] = v; s = e;
  }
  return n;
}
void handle(char *s) {
  char c = s[0]; char *a = s + 1; while (*a == ' ') a++;
  if (c == 'T') { epochBase = atof(a); epochAtMs = millis(); }
  else if (c == 'Z') tzSec = atol(a);
  else if (c == 'L') { double v[2]; if (nums(a, v, 2) == 2) { meLat = v[0] * DEG; meLon = v[1] * DEG; } }
  else if (c == 'P') {
    double v[6];
    if (nums(a, v, 6) == 6) setFix(v[0], v[1], v[2], v[3], v[4], v[5] > .5);
  }
  else if (c == 'N') { double v[3]; if (nums(a, v, 3) == 3) { pass.start = v[0]; pass.end = v[1]; pass.maxEl = v[2]; pass.n = 0; } }
  else if (c == 'Q') {  // v 는 static: loop 스택이 4KB 라 1.4KB 배열을 두면 넘쳐서 보드가 죽는다
    pass.n = 0; static double v[2 * MAXQ]; int k = nums(a, v, 2 * MAXQ);
    for (int i = 0; i + 1 < k; i += 2) { pass.az[pass.n] = v[i]; pass.el[pass.n] = v[i + 1]; pass.n++; }
    pass.ok = pass.n > 1;
  }
  else if (c == 'C') nCrew = 0;
  else if (c == 'M' && nCrew < MAXC) {
    char *p1 = strchr(a, '|'); if (!p1) return; char *p2 = strchr(p1 + 1, '|'); if (!p2) return;
    *p1 = 0; *p2 = 0; Crew &m = crew[nCrew++];
    strncpy(m.craft, a, 11); m.craft[11] = 0; strncpy(m.name, p1 + 1, 23); m.name[23] = 0; strncpy(m.ag, p2 + 1, 5); m.ag[5] = 0;
  }
  else if (c == 'S') screenMode = atoi(a);
}
void pollSerial() {
  while (Serial.available()) {
    char ch = Serial.read();
    if (ch == '\n' || ch == '\r') { if (lineLen) { line[lineLen] = 0; handle(line); lineLen = 0; } }
    else if (lineLen < (int)sizeof(line) - 1) line[lineLen++] = ch;
  }
}

// ── 시작 ────────────────────────────────────────
void setup() {
  Serial.begin(115200);
  pinMode(LCD_BLK, OUTPUT); digitalWrite(LCD_BLK, HIGH);
  g->begin(); fb = g->getFramebuffer();
  makeBg(0, 0x0a1238, 0x02030a); makeBg(1, 0x101a4a, 0x03040d); makeBg(2, 0x0b1640, 0x02030a); makeBg(3, 0x1a1250, 0x03040d);
  for (int i = 0; i < 64; i++) { float t = i / 63.f; oceanLut[i] = t < .5f ? mixf(H(0x2160c4), H(0x0c2766), t * 2) : mixf(H(0x0c2766), H(0x050f33), t * 2 - 1); }
  for (int r = 0; r < 90; r++) { float la = (90 - (r + .5f) * 2) * DEG; sinR[r] = sinf(la); cosR[r] = cosf(la); }
  for (int c = 0; c < 180; c++) { float lo = (-180 + (c + .5f) * 2) * DEG; sinC[c] = sinf(lo); cosC[c] = cosf(lo); }
  uint32_t seed = 7;
  for (auto &s : stars) {
    seed = seed * 1103515245 + 12345; s.x = (seed >> 8) % 240; seed = seed * 1103515245 + 12345; s.y = (seed >> 8) % 240;
    seed = seed * 1103515245 + 12345; s.k = ((seed >> 8) % 1000) / 1000.f; s.big = s.k > .9f;
  }
}

const uint16_t DUR[4] = { 15000, 8000, 8000, 6000 };
int autoScreen = 0; uint32_t screenAt = 0, fpsAt = 0; int frames = 0;
void loop() {
  pollSerial();
  double t = epochBase > 0 ? nowUtc() : millis() / 1000.0;
  int scr;
  if (!fix.ok) scr = -1;
  else if (screenMode != 9) scr = screenMode;
  else {
    bool live = pass.ok && t >= pass.start - 60 && t <= pass.end;
    if (live) scr = 2;
    else { if (millis() - screenAt > DUR[autoScreen]) { autoScreen = (autoScreen + 1) % 4; screenAt = millis(); } scr = autoScreen; }
  }
  switch (scr) { case 0: drawOrbit(t); break; case 1: drawTele(t); break; case 2: drawPass(t); break; case 3: drawCrew(t); break; default: drawWait(t); }
  pollSerial();
  g->flush();
  frames++;
  if (millis() - fpsAt > 10000) { Serial.printf("fps %.1f scr %d crew %d pass %d\n", frames * 1000.0 / (millis() - fpsAt), scr, nCrew, pass.n); fpsAt = millis(); frames = 0; }
}
