"""Regenerate notes/00_toc.md: every note with its title, date and status, grouped by folder.
Run from the repo root:  python3 notes/_make_toc.py   (stdlib only)"""
import re, pathlib
N = pathlib.Path(__file__).resolve().parent
SECTIONS = [("루트", "", ["00_index"]),
            ("research — 현행 연구 노트", "research", None), ("research/legacy — 구식 연구 노트", "research/legacy", None),
            ("experiments — 논문 단계 실험 (exp_025~)", "experiments", None), ("experiments/legacy — exp_001~024", "experiments/legacy", None),
            ("errors — 에러 해결 기록", "errors", None), ("daily — 연구 일지", "daily", None),
            ("sessions — 세션 기록·명령", "sessions", None), ("Environment — VM·접속", "Environment", None)]


def meta(f):
    s = f.read_text(encoding="utf-8")
    m = re.search(r"^# (.+)$", s, re.M)
    t = re.sub(r"\*\*|`", "", m.group(1).strip() if m else f.stem)
    t = t[:95] + "…" if len(t) > 96 else t
    d = re.search(r"^date:\s*(\S+)", s, re.M)
    st = re.search(r"^status:\s*(.+)$", s, re.M)
    return t, d.group(1) if d else "", st.group(1).strip() if st else ""


out = ["---", "date: 2026-09-25", "tags: [index, toc]", "status: active", "type: index", "---", "",
       "# notes 목차 — 어떤 파일에 어떤 내용이 있나", "",
       "> 모든 노트의 전체 목록이다(자동 생성, 제목·날짜·상태). 무엇부터 읽을지는 대시보드 [[00_index]]의 \"빠른 참조\"를 본다.",
       "> 현행 문서만 추리면: [[research/paper_outline_v6]](논문 개요) · [[research/l1_sl_pipeline]](방법) · [[research/final_tables_v6]](표·그림) · [[research/code_map]](코드 위치) · [[sessions/commands]](명령).",
       "> 재생성: `python3 notes/_make_toc.py`.", ""]
total = 0
for name, folder, only in SECTIONS:
    d = N / folder if folder else N
    fs = sorted((f for f in d.glob("*.md") if not f.name.startswith("_") and f.stem != "00_toc"
                 and (only is None or f.stem in only)),
                key=lambda f: f.stem, reverse=folder in ("daily", "sessions", "errors"))
    if not fs:
        continue
    out += [f"## {name} ({len(fs)})", "", "| 파일 | 내용 | 날짜 | 상태 |", "|---|---|---|---|"]
    for f in fs:
        t, dt, st = meta(f)
        out.append(f"| [[{folder + '/' if folder else ''}{f.stem}]] | {t.replace('|', '/')} | {dt} | {st} |")
    out.append("")
    total += len(fs)
out += ["---", "", f"총 {total}편."]
(N / "00_toc.md").write_text("\n".join(out) + "\n", encoding="utf-8")
print("00_toc.md:", total, "notes")
