#!/bin/bash
# 빌드·업로드·시리얼 — Arduino IDE 를 안 열고 터미널에서 끝낸다.
#
#   ./build.sh            컴파일만 (보드 없어도 된다)
#   ./build.sh upload     컴파일 + 업로드
#   ./build.sh monitor    시리얼만 열기
#
# arduino-cli 는 따로 설치할 필요 없다. Arduino IDE.app 안에 들어 있다.
#
# ★ 이 보드는 nrfutil 시리얼 DFU 가 안 된다. UF2 부트로더다.
#   `arduino-cli upload` 는 "Timed out waiting for acknowledgement" 로 계속 죽는다.
#   .hex → .uf2 로 바꿔 NRF52BOOT 드라이브에 복사하는 게 맞는 길이다.
#   1200bps 터치로 DFU 진입도 안 먹으므로 **리셋 버튼 더블탭**이 필요하다.
set -e
cd "$(dirname "$0")"

CLI="/Applications/Arduino IDE.app/Contents/Resources/app/lib/backend/resources/arduino-cli"
FQBN="nucode:nrf52:nu40dk"
SKETCH="$PWD/firmware/iss_tracker"
NAME="iss_tracker"
ACTION="${1:-compile}"

[ -x "$CLI" ] || { echo "arduino-cli 를 못 찾았다: $CLI"; exit 1; }

# .hex → .uf2 변환. compile 과 upload 가 같이 쓴다.
# ★ compile 에서도 만드는 이유: upload 에서만 만들면, compile 만 돌리고 /tmp/*.uf2 를
#   손으로 복사했을 때 **직전 빌드가 구워진다.** 보드엔 옛 코드가 도는데 새 코드가
#   올라간 줄 알고 엉뚱한 데서 원인을 찾게 된다.
make_uf2() {
  HEX=$(ls -t "$HOME/Library/Caches/arduino/sketches/"*/"$NAME.ino.hex" 2>/dev/null | head -1)
  [ -f "$HEX" ] || { echo "빌드 결과(.hex)를 못 찾았다"; exit 1; }
  UF2="/tmp/$NAME.uf2"
  # 0xADA52840 = boards.txt 의 nu40dk.build.uf2_family
  python3 "$HOME/Library/Arduino15/packages/nucode/hardware/nrf52/1.0.2/tools/uf2conv/uf2conv.py" \
    "$HEX" -c -f 0xADA52840 -o "$UF2" >/dev/null
  echo "$UF2  ($(date -r "$UF2" '+%H:%M:%S'))"
}

case "$ACTION" in
  compile)
    "$CLI" compile --fqbn "$FQBN" "$SKETCH"
    make_uf2
    ;;

  upload)
    # ⚠️ 굽기 전에 반드시 사람에게 묻고 진행할 것 (CLAUDE.md 작업자 규칙)
    "$CLI" compile --fqbn "$FQBN" "$SKETCH"
    make_uf2

    if [ ! -d /Volumes/NRF52BOOT ] && [ ! -d /Volumes/BARAM-NU40 ]; then
      echo ""
      echo "▶ 보드의 리셋 버튼(S5)을 **빠르게 두 번** 누르세요. 10분 기다립니다."
      # 더블탭 타이밍이 잘 안 맞아서 넉넉히 기다린다. 23초 만에 뜬 적도 있다.
      for i in $(seq 1 600); do
        { [ -d /Volumes/NRF52BOOT ] || [ -d /Volumes/BARAM-NU40 ]; } && { echo "  드라이브 감지 (${i}초째)"; break; }
        sleep 1
      done
    fi
    { [ -d /Volumes/NRF52BOOT ] || [ -d /Volumes/BARAM-NU40 ]; } || { echo "DFU 드라이브(NRF52BOOT)가 안 잡혔다"; exit 1; }

    # ⚠️ 막 마운트된 직후에는 쓰기가 거부된다.
    #   "Device not configured" 와 "Permission denied" 두 가지로 나온다.
    #   2초 한 번으로는 모자랄 때가 있어서 될 때까지 재시도한다.
    OK=0
    for i in $(seq 1 15); do
      sleep 1
      if cp "$UF2" /Volumes/NRF52BOOT/ 2>/dev/null || cp "$UF2" /Volumes/BARAM-NU40/ 2>/dev/null; then OK=1; break; fi
    done
    [ "$OK" = 1 ] || { echo "복사 실패 — /Volumes/NRF52BOOT 에 직접 cp 해볼 것"; exit 1; }
    echo "★ 굽기 완료 — 보드가 재부팅된다"
    echo "  LED4 가 1→2→3→4 회 깜빡이면 정상. 멈춘 횟수가 멈춘 자리다 (README 참고)"
    ;;

  monitor)
    # ★ cat /dev/cu.usbmodem* 으로는 아무것도 안 나온다.
    #   TinyUSB CDC 는 호스트가 DTR 을 올려야 출력을 버리지 않는다.
    PORT=$(ls /dev/cu.usbmodem* 2>/dev/null | head -1)
    [ -n "$PORT" ] || { echo "보드를 못 찾았다 (/dev/cu.usbmodem*)"; exit 1; }
    echo "포트: $PORT"
    "$CLI" monitor -p "$PORT" -c baudrate=115200
    ;;

  *)
    echo "모르는 동작: $ACTION (compile | upload | monitor)"
    exit 1
    ;;
esac
