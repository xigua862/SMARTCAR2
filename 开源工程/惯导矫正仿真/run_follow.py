# -*- coding: utf-8 -*-
"""ASCII launcher so the .bat is not broken by Chinese filenames."""
import sys
import traceback
from importlib.util import module_from_spec, spec_from_file_location
from pathlib import Path

ROOT = Path(__file__).resolve().parent
sys.path.insert(0, str(ROOT))


def main():
    target = ROOT / "巡线演示.py"
    spec = spec_from_file_location("line_follow_demo", target)
    if spec is None or spec.loader is None:
        raise FileNotFoundError(str(target))
    mod = module_from_spec(spec)
    spec.loader.exec_module(mod)
    mod.main()


if __name__ == "__main__":
    try:
        main()
    except Exception:
        traceback.print_exc()
        input("出错了，按回车退出...")
        sys.exit(1)
