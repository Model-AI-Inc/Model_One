#!/usr/bin/env python3

"""
Cleans humanoid.xml by making near-zero numbers 0, rounding to 6 decimals, removing scientific notation, and sorting meshes by filename.
"""

import re, argparse, math, os

# Regexes
FLOAT_RE = re.compile(r"[+-]?(?:\d+(?:\.\d*)?|\.\d+)(?:[eE][+-]?\d+)?")
ATTR_NAME_CLASS = r"[A-Za-z_][A-Za-z0-9_\-]*"
ATTR_RE = re.compile(rf'(?P<name>{ATTR_NAME_CLASS})\s*=\s*"(?P<vals>[^"]*)"')
ASSET_BLOCK_RE = re.compile(r"(<asset\b[^>]*>)(.*?)(</asset>)", re.DOTALL | re.IGNORECASE)
MESH_TAG_RE = re.compile(r"(<\s*mesh\b[^>]*?/>)", re.IGNORECASE | re.DOTALL)
FILE_ATTR_RE = re.compile(r'\bfile\s*=\s*"([^"]+)"', re.IGNORECASE)
NAME_ATTR_RE = re.compile(r'\bname\s*=\s*"([^"]+)"', re.IGNORECASE)

def clean_number(tok, eps):
    if not FLOAT_RE.fullmatch(tok): return tok
    try: v = float(tok)
    except ValueError: return tok
    if abs(v) < eps: return "0"
    s = f"{v:.6f}".rstrip("0").rstrip(".")
    return "0" if s in ("-0", "") else s

def parse_vec(s):
    parts = re.split(r"(\s+)", s); vals = []
    for i in range(0, len(parts), 2):
        tok = parts[i]
        if tok and FLOAT_RE.fullmatch(tok):
            try: vals.append(float(tok))
            except: vals.append(None)
        else: vals.append(None)
    return parts, vals

def rebuild(parts, eps):
    for i in range(0, len(parts), 2):
        tok = parts[i]
        if tok and FLOAT_RE.fullmatch(tok): parts[i] = clean_number(tok, eps)
    return "".join(parts)

def normalize_quat(parts, vals, eps):
    idxs, nums = [], []
    for i in range(0, len(parts), 2):
        tok = parts[i]
        if tok and FLOAT_RE.fullmatch(tok):
            try: nums.append(float(tok)); idxs.append(i)
            except: pass
    if len(nums) != 4: return parts
    n2 = sum(x*x for x in nums)
    nums = [0, 0, 0, 1] if n2 <= eps*eps else [x/math.sqrt(n2) for x in nums]
    for i, x in zip(idxs, nums): parts[i] = clean_number(f"{x:.6f}", eps)
    return parts

def process_attr(name, vals, eps, normalize):
    parts, raw = parse_vec(vals)
    if normalize and name.lower() in ("quat", "quaternion"): parts = normalize_quat(parts, raw, eps); return "".join(parts)
    return rebuild(parts, eps)

def mesh_key(tag):
    m = FILE_ATTR_RE.search(tag)
    if m: base = os.path.basename(m.group(1)); return (base.lower(), m.group(1).lower())
    n = NAME_ATTR_RE.search(tag)
    if n: return (n.group(1).lower(), n.group(1).lower())
    return (tag.lower(), tag.lower())

def reorder_meshes(block):
    meshes = [(m.span(), m.group(1)) for m in MESH_TAG_RE.finditer(block)]
    if len(meshes) <= 1: return block
    sorted_tags = sorted((t for _, t in meshes), key=mesh_key)
    out, cur = [], 0
    for (span, _), new_tag in zip(meshes, sorted_tags): s, e = span; out.append(block[cur:s]); out.append(new_tag); cur = e
    out.append(block[cur:]); return "".join(out)

def reorder_assets(text):
    return ASSET_BLOCK_RE.sub(lambda m: m.group(1) + reorder_meshes(m.group(2)) + m.group(3), text)

def main():
    ap = argparse.ArgumentParser(description="Clean near-zero floats, round to 6 decimals, normalize quats, and reorder meshes by filename.")
    ap.add_argument("input", nargs="?", default="humanoid.xml", help="Input XML (default humanoid.xml)")
    ap.add_argument("output", nargs="?", default=None, help="Output XML (default same as input)")
    ap.add_argument("--eps", type=float, default=0.001, help="Zero threshold")
    ap.add_argument("--attrs", default="pos,quat,range", help="Comma-separated attributes to clean")
    ap.add_argument("--no-normalize-quat", action="store_true", help="Disable quaternion normalization")
    ap.add_argument("--no-sort-meshes", action="store_true", help="Disable mesh reordering")
    a = ap.parse_args()
    infile, outfile = a.input, a.output or a.input
    allowed = {x.strip() for x in a.attrs.split(",") if x.strip()}
    with open(infile, "r", encoding="utf-8") as f: text = f.read()
    text = re.sub(ATTR_RE, lambda m: m.group(0) if m.group("name") not in allowed else
                  m.group(0)[:m.start("vals")-m.start()] + process_attr(m.group("name"), m.group("vals"), a.eps, not a.no_normalize_quat) +
                  m.group(0)[m.end("vals")-m.start():], text)
    if not a.no_sort_meshes: text = reorder_assets(text)
    with open(outfile, "w", encoding="utf-8") as f: f.write(text)
    print(f"Cleaned {infile} -> {outfile} (attrs={','.join(sorted(allowed))}, eps={a.eps}, normalize_quat={not a.no_normalize_quat}, sort_meshes={not a.no_sort_meshes})")

if __name__ == "__main__": main()
