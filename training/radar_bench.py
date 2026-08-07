#!/usr/bin/env python3
"""
HappyMac — 雷达基准测试（纯雷达，无摄像头）
三配置 × 四阶段，测试 X/Y/Em/Es 的信号质量 + 双雷达干扰
用法：C3 插 USB 后直接跑
"""
import serial, time, statistics, json, sys
from pathlib import Path
from datetime import datetime

PORT = "/dev/cu.usbmodem2101"
BAUD = 115200
DISTANCE = "0.7m"  # 雷达到你的距离
OUT_DIR = Path(__file__).resolve().parent / "bench_results"
PHASE_SEC = 8

PHASES = [
    ("still",      "坐定不动，正常呼吸"),
    ("left_right", "大幅左移(3s)→回中→右移(3s)→回中"),
    ("fwd_back",   "前倾凑近(3s)→正常→后靠(3s)→回正常"),
    ("head_turn",  "正前方(2s)→头右转(2s)→回正→头左转(2s)→回正。只转头，身不动！"),
]

def read_phase(ser, label, duration):
    """读一段时间，返回样本列表 [{x,y,v,em,es}]"""
    print(f"    {label}...", end=" ", flush=True)
    samples = []
    t0 = time.time()
    while time.time() - t0 < duration:
        d = ser.read(4096)
        if not d: continue
        for line in d.decode(errors='replace').strip().split('\n'):
            if line.startswith('RADAR,'):
                p = line.split(',')
                if len(p) >= 9:
                    samples.append({
                        'x': int(p[2]), 'y': int(p[3]), 'v': int(p[4]),
                        'em': int(p[5]), 'es': int(p[6])
                    })
    if not samples:
        print("❌ 无数据")
        return None
    xs = [s['x'] for s in samples]
    ys = [s['y'] for s in samples]
    print(f"{len(samples)}帧 Xσ={statistics.stdev(xs):.0f} Yσ={statistics.stdev(ys):.0f}")
    return samples

def run_config(ser, config_name, phases_to_run):
    """跑一个配置的全部阶段"""
    print(f"\n{'─'*50}")
    print(f"  配置: {config_name}")
    print(f"{'─'*50}")
    results = {}
    for phase_key, phase_desc in phases_to_run:
        print(f"\n  [{phase_key}] {phase_desc}")
        for i in range(3, 0, -1):
            print(f"    {i}...", end="\r", flush=True)
            time.sleep(1)
        data = read_phase(ser, phase_key, PHASE_SEC)
        results[phase_key] = {
            'desc': phase_desc,
            'samples': data,
            'n': len(data) if data else 0,
            'stats': {
                'x_mean': statistics.mean([d['x'] for d in data]) if data else 0,
                'x_std': statistics.stdev([d['x'] for d in data]) if data else 0,
                'x_span': max(d['x'] for d in data) - min(d['x'] for d in data) if data else 0,
                'y_mean': statistics.mean([d['y'] for d in data]) if data else 0,
                'y_std': statistics.stdev([d['y'] for d in data]) if data else 0,
                'y_span': max(d['y'] for d in data) - min(d['y'] for d in data) if data else 0,
                'em_mean': statistics.mean([d['em'] for d in data]) if data else 0,
                'es_mean': statistics.mean([d['es'] for d in data]) if data else 0,
                'es_span': max(d['es'] for d in data) - min(d['es'] for d in data) if data else 0,
            } if data else {}
        }
        time.sleep(1)
    return results

def analyze(all_results):
    """对比三个配置"""
    print(f"\n\n{'='*60}")
    print(f"  对比分析")
    print(f"{'='*60}")
    
    for phase_key, _ in PHASES:
        print(f"\n  [{phase_key}]")
        for cfg in all_results:
            r = all_results[cfg].get(phase_key, {})
            s = r.get('stats', {})
            if not s: continue
            print(f"    {cfg:12s}  Xσ={s.get('x_std',0):5.0f}  Yσ={s.get('y_std',0):5.0f}  "
                  f"Xspan={s.get('x_span',0):+5.0f}  Yspan={s.get('y_span',0):4.0f}  "
                  f"Em={s.get('em_mean',0):3.0f}  Es={s.get('es_mean',0):3.0f}")
        
        # Interference impact
        if 'both' in all_results and '2450_only' in all_results:
            bs = all_results['both'].get(phase_key, {}).get('stats', {})
            xs = all_results['2450_only'].get(phase_key, {}).get('stats', {})
            if bs and xs:
                dx = bs.get('x_std', 0) - xs.get('x_std', 0)
                pct = dx / max(xs.get('x_std', 1), 1) * 100
                print(f"    干扰影响: X精度劣化 {dx:+.0f}mm ({pct:+.0f}%)")

# ═══════════════════════════════════════════════════
if __name__ == '__main__':
    print("╔══════════════════════════════════════════════╗")
    print("║  HappyMac 雷达基准测试                        ║")
    print(f"║  距离: {DISTANCE}  每阶段: {PHASE_SEC}s  共计: ~5min ║")
    print("╚══════════════════════════════════════════════╝")
    print()
    print("硬件准备：")
    print("  雷达在正前方 0.7m，竖放（PCB 垂直），天线面朝你")
    print("  无遮挡，空桌子")
    print()

    OUT_DIR.mkdir(parents=True, exist_ok=True)
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")

    # ── 连接串口 ──
    try:
        ser = serial.Serial(PORT, BAUD, timeout=0.3)
    except Exception as e:
        print(f"❌ 无法连接 {PORT}: {e}")
        sys.exit(1)
    ser.reset_input_buffer()
    time.sleep(1)
    # 清缓冲
    while ser.read(4096): pass

    all_results = {}

    # ── 配置 1: 双雷达 ──
    input("\n[配置 1/3] 两个雷达都插着 → 按 Enter")
    all_results['both'] = run_config(ser, "双雷达 (both)", PHASES)

    # ── 配置 2: 仅 2450 ──
    input("\n[配置 2/3] 拔掉 LD2410C 的电源线 → 按 Enter")
    ser.reset_input_buffer()
    time.sleep(0.5)
    while ser.read(4096): pass
    all_results['2450_only'] = run_config(ser, "仅 LD2450", PHASES)

    # ── 配置 3: 仅 2410C ──
    input("\n[配置 3/3] 插回 2410C，拔掉 LD2450 的电源线 → 按 Enter")
    ser.reset_input_buffer()
    time.sleep(0.5)
    while ser.read(4096): pass
    all_results['2410C_only'] = run_config(ser, "仅 LD2410C", PHASES)

    ser.close()

    # ── 去掉原始样本（太大），只存统计 ──
    for cfg in all_results:
        for pk in all_results[cfg]:
            all_results[cfg][pk].pop('samples', None)

    all_results['_meta'] = {'distance': DISTANCE, 'phase_sec': PHASE_SEC, 'timestamp': timestamp}

    out_path = OUT_DIR / f"bench_{timestamp}.json"
    with open(out_path, 'w') as f:
        json.dump(all_results, f, indent=2, default=str)
    
    analyze(all_results)
    print(f"\n结果已保存: {out_path}")
