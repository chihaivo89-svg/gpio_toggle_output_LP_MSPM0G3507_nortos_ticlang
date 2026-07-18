import re, statistics

with open(r'C:\Users\CbkCbk\workspace_ccstheia\gpio_toggle_output_LP_MSPM0G3507_nortos_ticlang\test_results\road_batch_target12_15_20260718_111722.txt', 'r') as f:
    raw = f.read()

runs = []
current = None
for line in raw.splitlines():
    m = re.match(r'msg:run begin index=(\d+) target=([-\d]+) limit=(\d+) m4trim=([-\d]+) m2trim=([-\d]+) samples=(\d+)', line)
    if m:
        current = {'idx':int(m[1]),'target':int(m[2]),'limit':int(m[3]),'m4trim':int(m[4]),'m2trim':int(m[5]),'samples':int(m[6]),'data':[],'cs_rpt':None}
        runs.append(current); continue
    m = re.match(r'road:(\d+),(\d+),([-\d]+),([-\d]+),([-\d]+),([-\d]+)', line)
    if m and current: current['data'].append({'la':int(m[3]),'lo':int(m[4]),'ra':int(m[5]),'ro':int(m[6])})
    m = re.match(r'msg:run end index=(\d+) checksum=(\d+)', line)
    if m and current: current['cs_rpt'] = int(m[2])

print('='*70)
for r in runs:
    d = r['data']
    # FNV-1a style checksum
    h = 0x811C9DC5
    for s in d:
        h = ((h ^ (s['la'] & 0xFFFF)) * 0x01000193) & 0xFFFFFFFF
        h = ((h ^ (s['lo'] & 0xFFFF)) * 0x01000193) & 0xFFFFFFFF
        h = ((h ^ (s['ra'] & 0xFFFF)) * 0x01000193) & 0xFFFFFFFF
        h = ((h ^ (s['ro'] & 0xFFFF)) * 0x01000193) & 0xFFFFFFFF
    ck = 'PASS' if h == r['cs_rpt'] else 'FAIL'

    sd = d[10:]  # skip 200ms startup
    la = [s['la'] for s in sd]; ra = [s['ra'] for s in sd]
    lo = [s['lo'] for s in sd]; ro = [s['ro'] for s in sd]
    lo_all = [s['lo'] for s in d]; ro_all = [s['ro'] for s in d]
    lavg = statistics.mean(la); ravg = statistics.mean(ra)
    lerr = abs(lavg - r['target']); rerr = abs(ravg - r['target'])
    lsd = statistics.stdev(la) if len(la)>1 else 0
    rsd = statistics.stdev(ra) if len(ra)>1 else 0
    l_sat = sum(1 for v in lo_all if abs(v) >= r['limit'])
    r_sat = sum(1 for v in ro_all if abs(v) >= r['limit'])

    print(f"Run {r['idx']}: T={r['target']} lim={r['limit']} n={len(d)} ck={ck}")
    print(f"  L_act: avg={lavg:5.2f} [{min(la):3d},{max(la):3d}] sd={lsd:.2f} err={lerr:.2f}")
    print(f"  R_act: avg={ravg:5.2f} [{min(ra):3d},{max(ra):3d}] sd={rsd:.2f} err={rerr:.2f}  side_diff={abs(lavg-ravg):.2f}")
    print(f"  L_pwm: avg={statistics.mean(lo):6.1f} [{min(lo):4d},{max(lo):4d}] sat={l_sat}/{len(d)}")
    print(f"  R_pwm: avg={statistics.mean(ro):6.1f} [{min(ro):4d},{max(ro):4d}] sat={r_sat}/{len(d)}")
    print()

print('='*70)
print(f'{"Run":<5}{"Tgt":<5}{"Lim":<5}{"Lavg":<7}{"Ravg":<7}{"Lerr":<7}{"Rerr":<7}{"SideD":<7}{"Lsd":<7}{"Rsd":<7}{"Lsat":<6}{"Rsat":<6}{"CK":<5}')
for r in runs:
    d = r['data'][10:]
    la=[s['la'] for s in d]; ra=[s['ra'] for s in d]
    lo_all=[s['lo'] for s in r['data']]; ro_all=[s['ro'] for s in r['data']]
    h=0x811C9DC5
    for s in r['data']:
        h=((h^(s['la']&0xFFFF))*0x01000193)&0xFFFFFFFF
        h=((h^(s['lo']&0xFFFF))*0x01000193)&0xFFFFFFFF
        h=((h^(s['ra']&0xFFFF))*0x01000193)&0xFFFFFFFF
        h=((h^(s['ro']&0xFFFF))*0x01000193)&0xFFFFFFFF
    ck='PASS' if h==r['cs_rpt'] else 'FAIL'
    print(f"{r['idx']:<5}{r['target']:<5}{r['limit']:<5}{statistics.mean(la):<7.2f}{statistics.mean(ra):<7.2f}{abs(statistics.mean(la)-r['target']):<7.2f}{abs(statistics.mean(ra)-r['target']):<7.2f}{abs(statistics.mean(la)-statistics.mean(ra)):<7.2f}{statistics.stdev(la) if len(la)>1 else 0:<7.2f}{statistics.stdev(ra) if len(ra)>1 else 0:<7.2f}{sum(1 for v in lo_all if abs(v)>=r['limit']):<6}{sum(1 for v in ro_all if abs(v)>=r['limit']):<6}{ck:<5}")

t12=[r for r in runs if r['target']==12]; t15=[r for r in runs if r['target']==15]
for grp,name in [(t12,'Target 12'),(t15,'Target 15')]:
    al=[];ar=[];al_pwm=[];ar_pwm=[]
    for r in grp:
        al+=[s['la'] for s in r['data'][10:]]; ar+=[s['ra'] for s in r['data'][10:]]
        al_pwm+=[s['lo'] for s in r['data']]; ar_pwm+=[s['ro'] for s in r['data']]
    t=grp[0]['target']
    print(f"\n{name} Combined (3x250 samples):")
    print(f"  Left:  avg={statistics.mean(al):.2f} sd={statistics.stdev(al):.2f} err={abs(statistics.mean(al)-t):.2f} pwm_avg={statistics.mean(al_pwm):.1f} pwm_max={max(al_pwm)}")
    print(f"  Right: avg={statistics.mean(ar):.2f} sd={statistics.stdev(ar):.2f} err={abs(statistics.mean(ar)-t):.2f} pwm_avg={statistics.mean(ar_pwm):.1f} pwm_max={max(ar_pwm)}")
