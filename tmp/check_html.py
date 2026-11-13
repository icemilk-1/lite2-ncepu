from pathlib import Path

h = Path(r"F:\airace\plan\四足机器人导航避障文献逐篇流程图讲解.html").read_text(encoding="utf-8")
checks = {
    "<table>": h.count("<table>"),
    "figure.diagram": h.count('<figure class="diagram"'),
    "<h2": h.count("<h2"),
    "<h3": h.count("<h3"),
    "<blockquote>": h.count("<blockquote>"),
    "DEFS array": 1 if "const DEFS =" in h else 0,
}
for k, v in checks.items():
    print(f"{k}: {v}")
print("size KB:", len(h) // 1024)