#!/usr/bin/env python3
"""Build script for mini-pid-tuning — generates all source files."""

import os

BASE = r"F:\nano-everything\mini-electronic-info\9. mini-control-automation\mini-pid-tuning"

def write_file(relpath, content):
    full = os.path.join(BASE, relpath)
    os.makedirs(os.path.dirname(full), exist_ok=True)
    with open(full, 'w', encoding='utf-8') as f:
        f.write(content)
    lines = content.count('\n') + 1
    print(f"  Wrote {relpath} ({lines} lines)")

print("Starting build...")
