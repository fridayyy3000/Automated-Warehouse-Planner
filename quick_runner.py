
#!/usr/bin/env python3
\"\"\"quick_runner.py – doubles horizon until Clingo finds a plan\"\"\"
import subprocess, sys, shutil, pathlib

CLINGO = shutil.which("clingo") or "clingo"
MODEL  = pathlib.Path(__file__).with_name("smart_store.lp")

def attempt(inst, h):
    res = subprocess.run([CLINGO, str(MODEL), inst, "-c", f"H={h}", "--quiet=1"],
                         text=True, capture_output=True).stdout
    print(res)
    return "UNSATISFIABLE" not in res

if __name__ == "__main__":
    if len(sys.argv)<2:
        sys.exit("usage: python quick_runner.py instance.asp")
    inst, horizon = sys.argv[1], 6
    while True:
        print(f"--> trying horizon {horizon}")
        if attempt(inst, horizon):
            print(f\"✓ plan found at horizon {horizon}\")
            break
        horizon *= 2
