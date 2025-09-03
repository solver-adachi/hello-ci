import sys, re, xml.etree.ElementTree as ET

inp  = sys.argv[1] if len(sys.argv) > 1 else "build.log"
outp = sys.argv[2] if len(sys.argv) > 2 else "build_warnings.xml"

# ファイル:行:列: kind: メッセージ
pattern = re.compile(r"^(?P<file>[^:]+):(?P<line>\d+):(?P<col>\d+):\s*(?P<kind>warning|error):\s*(?P<msg>.*)", re.IGNORECASE)

suite = ET.Element("testsuite", name="colcon_build")
tests = fails = errs = 0

with open(inp, "r", errors="ignore") as f:
    for idx, raw in enumerate(f, 1):
        m = pattern.match(raw)
        if not m:
            continue

        kind = m.group("kind").lower()
        file = m.group("file")
        line = m.group("line")
        col  = m.group("col")
        msg  = m.group("msg").strip()

        tc_name = f"{file}:{line}:{col}:{kind}:{idx}"
        tc = ET.SubElement(suite, "testcase", classname="colcon-build", name=tc_name)

        if kind == "error":
            el = ET.SubElement(tc, "error", message=f"{file}:{line}:{col}")
            el.text = msg
            errs += 1
        elif kind == "warning":
            el = ET.SubElement(tc, "failure", message=f"{file}:{line}:{col}")
            el.text = msg
            fails += 1

        tests += 1

suite.set("tests", str(tests))
suite.set("failures", str(fails))
suite.set("errors", str(errs))

ET.ElementTree(suite).write(outp, encoding="utf-8", xml_declaration=True)
print(f"Wrote {outp}: tests={tests}, warnings={fails}, errors={errs}")