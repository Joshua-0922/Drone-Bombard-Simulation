"""Gain of the adopted method over L0 by wind-speed bin (paired sim eval), and the power a real
outdoor campaign would have if each arm's landing errors in a bin looked like the sim's.
stdlib only:  python3 isaac_lab/_wind_bin_gain.py ROOT   (ROOT holds sl_GI/ and gru/ eval JSON dirs)"""
import json, math, random, statistics as st, sys
R = sys.argv[1]; SEEDS = (3000, 4000, 5000); rng = random.Random(0)
def load(fmt):
    w, e, l = [], [], []
    for s in SEEDS:
        r = json.load(open(fmt.format(S=s)))["episodes"]
        w += r["wind_speed"]; e += r["release_impact_err"]; l += r["landed"]
    return w, e, l
wL, eL, lL = load(R + "/sl_GI/L0_dr1.5_s{S}.json")
wG, eG, lG = load(R + "/gru/GRUS_E_tau0_s{S}.json")
# episode ORDER can differ between arms for some seeds (same scenarios, different completion order),
# so each arm is binned by its own recorded wind -- the unpaired view a real campaign also has.
def mw_p(a, b):   # one-sided Mann-Whitney, H1: a stochastically smaller than b (normal approx.)
    n1, n2 = len(a), len(b); allv = sorted((v, i) for i, v in enumerate(a + b))
    ranks = [0.0] * (n1 + n2); i = 0
    while i < len(allv):
        j = i
        while j + 1 < len(allv) and allv[j + 1][0] == allv[i][0]: j += 1
        for k in range(i, j + 1): ranks[allv[k][1]] = (i + j) / 2 + 1
        i = j + 1
    u = sum(ranks[:n1]) - n1 * (n1 + 1) / 2; mu = n1 * n2 / 2; sd = math.sqrt(n1 * n2 * (n1 + n2 + 1) / 12)
    return 0.5 * (1 + math.erf((u - mu) / sd / math.sqrt(2)))
def power(a, b, n, reps=1000):
    return sum(mw_p([rng.choice(a) for _ in range(n)], [rng.choice(b) for _ in range(n)]) < 0.05 for _ in range(reps)) / reps
BINS = [(0, 2), (2, 4), (4, 6), (6, 99)]
print("| 풍속 [m/s] | 에피소드 | L0 CEP50 | 본 방법 CEP50 | 이득 | 검출확률 n=20 | n=40 | n=60 |")
print("|---|---|---|---|---|---|---|---|")
for lo, hi in BINS:
    b = [e for w, e, l in zip(wL, eL, lL) if lo <= w < hi and l]
    a = [e for w, e, l in zip(wG, eG, lG) if lo <= w < hi and l]
    idx = a
    if min(len(a), len(b)) < 20: print(f"| {lo}–{hi} | {len(a)} | 표본 부족 |"); continue
    cl, cg = st.median(b), st.median(a)
    print(f"| {lo}–{hi if hi < 99 else ''} | {len(idx)} | {cl:.3f} | {cg:.3f} | {100*(cg-cl)/cl:+.0f}% | {power(a,b,20):.2f} | {power(a,b,40):.2f} | {power(a,b,60):.2f} |")
