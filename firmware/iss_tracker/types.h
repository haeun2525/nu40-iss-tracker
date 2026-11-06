#pragma once
// .ino 자동 프로토타입이 구조체보다 먼저 생겨서 여기 따로 둔다
struct Pos { float lat, lon, alt, phase; };
struct V3 { float x, y, z; };
struct Paint { uint16_t c1, c2; uint8_t mode; float a, len; };  // mode 0 단색, 1 가로, 2 세로
// 정거장 위치 (원궤도 근사)
struct Fix { double ts; float inc, P, prec, alt, vel, u0, om0, si, ci; bool ok; };
// 지구 그림 기준 (화면 가운데의 동·북·위 방향)
struct View { V3 E, N, U; float R; int cx, cy; };
