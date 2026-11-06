#!/usr/bin/env python3
"""
iss.py — ISS 위치·다음 통과·탑승자를 받아 NU40(iss_tracker)로 USB 시리얼 전송

    python3 host/iss.py              # 자동 화면
    python3 host/iss.py --screen 0   # 0 궤도 1 원격측정 2 다음 통과 3 탑승자

데이터 (키 필요 없음)
  위치   wheretheiss.at        5초마다 (지금 + 10초 뒤 두 점 → 북/남 방향)
  통과   CelesTrak 궤도값(TLE)  하루 1번 받고, skyfield 로 내 위치 기준 계산
  탑승자 Launch Library 2       1시간마다 (무료 한도 시간당 15회라 파일에 저장해 둠)

내 위치는 host/.env 의 LAT=, LON= (깃에 안 올라감). 없으면 서울 강남구 일대.
"""
import argparse, glob, json, os, sys, time, urllib.request

try:
    import serial
except ImportError:
    sys.exit("pyserial 이 필요합니다: pip3 install pyserial")
try:
    from skyfield.api import EarthSatellite, load, wgs84
except ImportError:
    sys.exit("skyfield 가 필요합니다: pip3 install skyfield")

HERE = os.path.dirname(os.path.abspath(__file__))
CACHE = os.path.join(HERE, ".cache"); os.makedirs(CACHE, exist_ok=True)
UA = {"User-Agent": "nu40-iss-tracker"}

env = {}
if os.path.exists(os.path.join(HERE, ".env")):
    for l in open(os.path.join(HERE, ".env")):
        if "=" in l and not l.startswith("#"):
            k, v = l.strip().split("=", 1); env[k] = v
ap = argparse.ArgumentParser()
ap.add_argument("--lat", type=float, default=float(env.get("LAT", 37.50)))
ap.add_argument("--lon", type=float, default=float(env.get("LON", 127.06)))
ap.add_argument("--screen", type=int, default=9)
ap.add_argument("--every", type=float, default=5)
ap.add_argument("--port", default=None)
a = ap.parse_args()

def get(url, timeout=10):
    with urllib.request.urlopen(urllib.request.Request(url, headers=UA), timeout=timeout) as r:
        return r.read().decode()

def cached(name, max_age, fetch):
    """파일 캐시. 새로 못 받으면 오래된 것이라도 쓴다."""
    p = os.path.join(CACHE, name)
    if os.path.exists(p) and time.time() - os.path.getmtime(p) < max_age:
        return open(p).read()
    try:
        s = fetch(); open(p, "w").write(s); return s
    except Exception as e:
        print("받기 실패:", name, e)
        return open(p).read() if os.path.exists(p) else None

# ── 위치 ──────────────────────────────────────────
def fix():
    now = int(time.time())
    d = json.loads(get(f"https://api.wheretheiss.at/v1/satellites/25544/positions?timestamps={now},{now + 10}"))
    p, q = d[0], d[1]
    asc = 1 if q["latitude"] > p["latitude"] else 0
    return f'P {p["timestamp"]} {p["latitude"]:.4f} {p["longitude"]:.4f} {p["altitude"]:.1f} {p["velocity"]:.0f} {asc}'

# ── 다음 통과 ─────────────────────────────────────
ts = load.timescale()
me = wgs84.latlon(a.lat, a.lon)
def tle():
    s = cached("stations.tle", 86400, lambda: get("https://celestrak.org/NORAD/elements/gp.php?CATNR=25544&FORMAT=tle"))
    l = [x.strip() for x in s.strip().splitlines()]
    return EarthSatellite(l[1], l[2], l[0], ts)

def next_pass():
    sat = tle(); now = ts.now()
    t, ev = sat.find_events(me, now - 0.01, now + 2, altitude_degrees=0.0)
    rise = None
    for ti, e in zip(t, ev):
        if e == 0: rise = ti
        elif e == 2 and rise is not None:
            if ti.tt < now.tt: rise = None; continue
            n = max(2, int((ti.tt - rise.tt) * 86400 / 15))
            times = ts.tt_jd([rise.tt + (ti.tt - rise.tt) * k / (n - 1) for k in range(n)])
            alt, az, _ = (sat - me).at(times).altaz()
            mx = float(max(alt.degrees))
            if mx < 10: rise = None; continue   # 지평선에 스치는 건 건너뜀
            pts = " ".join(f"{z:.1f},{max(h, 0):.1f}" for z, h in zip(az.degrees, alt.degrees))
            return [f"N {int(rise.utc_datetime().timestamp())} {int(ti.utc_datetime().timestamp())} {mx:.1f}", f"Q {pts}"]
    return None

# ── 탑승자 ────────────────────────────────────────
def short(name, ag):
    name = name.upper()
    if ag == "CNSA": return name
    parts = name.split()
    return f"{parts[0][0]}. {parts[-1]}"[:20]

def crew_lines():
    s = cached("crew.json", 3600, lambda: get("https://ll.thespacedevs.com/2.3.0/astronauts/?in_space=true&limit=30"))
    if not s: return None
    out = ["C"]
    people = [p for p in json.loads(s)["results"] if (p.get("type") or {}).get("name") != "Non-Human"]
    people.sort(key=lambda p: p["agency"]["abbrev"] == "CNSA")
    for p in people:
        ag = (p.get("agency") or {}).get("abbrev") or "?"
        craft = "TIANGONG" if ag == "CNSA" else "ISS"
        out.append(f"M {craft}|{short(p['name'], ag)}|{ag}")
    return out

# ── 시리얼 ────────────────────────────────────────
def find_port():
    if a.port: return a.port
    ports = glob.glob("/dev/cu.usbmodem*")
    if not ports: sys.exit("보드를 못 찾았습니다 (/dev/cu.usbmodem*). USB 연결을 확인하세요.")
    return ports[0]

def tz():
    return -time.altzone if time.localtime().tm_isdst else -time.timezone

def main():
    port = find_port(); print("보드:", port, "| 위치:", a.lat, a.lon)
    s = None; t_fix = t_pass = t_crew = 0; pass_lines = crew = None
    send = lambda *ls: s.write("".join(l + "\n" for l in ls).encode())
    while True:
        try:
            if s is None:
                s = serial.Serial(port, 115200, timeout=0); time.sleep(1.5)
                send(f"T {time.time():.1f}", f"Z {tz()}", f"L {a.lat} {a.lon}", f"S {a.screen}")
                t_pass = t_crew = 0
            now = time.time()
            if now >= t_fix:
                t_fix = now + a.every
                try:
                    p = fix(); send(f"T {time.time():.1f}", p)
                    print(time.strftime("%H:%M:%S"), p)
                except Exception as e:
                    print("위치 실패:", e)
            if now >= t_pass:
                t_pass = now + 60
                try:
                    pl = next_pass()
                    if pl: send(*pl)
                    if pl and pl != pass_lines:
                        pass_lines = pl; st = int(pl[0].split()[1])
                        print("다음 통과:", time.strftime("%m/%d %H:%M", time.localtime(st)), "최대", pl[0].split()[3], "도")
                except Exception as e:
                    print("통과 계산 실패:", e)
            if now >= t_crew:
                t_crew = now + 600
                cl = crew_lines()
                if cl: send(*cl); crew = cl
                if crew: print("탑승자", len(crew) - 1, "명")
            while s.in_waiting:
                l = s.readline().decode(errors="ignore").strip()
                if l: print("  보드:", l)
            time.sleep(0.2)
        except (serial.SerialException, OSError) as e:
            print("시리얼 끊김, 다시 연결:", e); s = None; time.sleep(2)
            try: port = find_port()
            except SystemExit: pass

if __name__ == "__main__":
    try: main()
    except KeyboardInterrupt: print("\n종료")
