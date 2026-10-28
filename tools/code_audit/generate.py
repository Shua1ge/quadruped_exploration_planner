#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Offline code-audit page generator for the SCAN-Planner (dyn) workspace.

Regenerate after code changes:
    python3 tools/code_audit/generate.py

Writes:
    docs/CONTRACT_STACK_INTERACTIVE_ZH.html   (single-file, offline, no CDN)
    tools/code_audit/audit_data.json          (index w/o embedded source)

Extraction is text/ast based (no build system). Call edges are graded:
  exact  qualified name or same-class hit, unique target
  name   bare name with exactly one workspace-wide hit
  ambig  several same-name symbols (listed, needs human check)
  unresolved  no workspace target (external/macro/dynamic) — kept, never dropped
"""

import ast
import datetime
import json
import os
import re
import shutil
import subprocess
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
HERE = os.path.dirname(os.path.abspath(__file__))
SKIP_DIRS = {'.git', 'build', 'install', 'log', '__pycache__', '.pytest_cache',
             'Img', 'assets', '.claude'}
SRC_SKIP_PREFIX = ('src/simulator/local_sensing/', 'tools/', 'docs/')
MAX_EMBED_SRC_BYTES = 700_000
LANG = {'.cpp': 'C++', '.h': 'C++', '.hpp': 'C++', '.cc': 'C++', '.hh': 'C++',
        '.py': 'Python', '.msg': 'msg', '.yaml': 'YAML', '.xml': 'xml',
        '.md': 'md', '.txt': 'txt'}
BIN_EXT = {'.pcd', '.dae', '.gif', '.jpg', '.jpeg', '.png', '.bin', '.so',
           '.stl', '.obj', '.webm', '.bag', '.db3', '.mp4', '.onnx', '.pt'}

SITE_LIST_CAP = 60          # sites listed per (field, kind); counts stay exact
FIELD_TOTAL_SITE_CAP = 240  # sites recorded per field overall (reads cut first)

# ---------------------------------------------------------------- utils

def git(*args):
    try:
        return subprocess.run(['git', *args], cwd=ROOT, capture_output=True,
                              text=True, timeout=10).stdout.strip()
    except Exception:
        return ''


def strip_noise(line, keep_strings=False):
    out = []
    i, n = 0, len(line)
    in_str = None
    while i < n:
        c = line[i]
        if in_str:
            if c == '\\' and i + 1 < n:
                if keep_strings:
                    out.append(c)
                    out.append(line[i + 1])
                i += 2
                continue
            if c == in_str:
                in_str = None
            if keep_strings:
                out.append(c)
            i += 1
            continue
        if c == '"' or c == "'":
            in_str = c
            if keep_strings:
                out.append(c)
            else:
                out.append(' ')
            i += 1
            continue
        if c == '/' and i + 1 < n and line[i + 1] == '/':
            break
        if c == '/' and i + 1 < n and line[i + 1] == '*':
            if not keep_strings:
                out.append(' ')
            j = line.find('*/', i + 2)
            i = n if j < 0 else j + 2
            continue
        out.append(c)
        i += 1
    return ''.join(out)


def brace_match_end(lines, start_idx):
    depth = 0
    opened = False
    for i in range(start_idx, len(lines)):
        s = strip_noise(lines[i])
        for ch in s:
            if ch == '{':
                depth += 1
                opened = True
            elif ch == '}':
                depth -= 1
                if opened and depth == 0:
                    return i
    return None


def offset_to_line(text, pos):
    return text.count('\n', 0, pos) + 1


def base_class_of(type_text):
    """Best-effort class name from a C++ member type declaration."""
    t = (type_text or '').strip()
    for pat in (r'(?:shared|unique)_ptr<\s*([\w:]+)',
                r'([\w:]+)::(?:Ptr|ConstPtr|SharedPtr)',
                r'([\w:]+)\s*\*',
                r'([\w:]+)'):
        m = re.search(pat, t)
        if m:
            return m.group(1).split('::')[-1]
    return None


def split_top_level(s):
    """Split call arguments on commas that are not inside (), [], {} or strings."""
    args, depth, cur, i = [], 0, [], 0
    while i < len(s):
        c = s[i]
        if c in '"\'':
            q = c
            cur.append(c)
            i += 1
            while i < len(s):
                cur.append(s[i])
                if s[i] == '\\':
                    i += 1
                    if i < len(s):
                        cur.append(s[i])
                elif s[i] == q:
                    break
                i += 1
        elif c in '([{':
            depth += 1
            cur.append(c)
        elif c in ')]}':
            depth -= 1
            cur.append(c)
        elif c == ',' and depth == 0:
            args.append(''.join(cur))
            cur = []
        else:
            cur.append(c)
        i += 1
    if cur:
        args.append(''.join(cur))
    return [a.strip() for a in args]


# ---------------------------------------------------------------- file walk

def walk_tree(path, rel, node):
    for name in sorted(os.listdir(path)):
        if name in SKIP_DIRS:
            continue
        full = os.path.join(path, name)
        r = os.path.join(rel, name) if rel else name
        if os.path.isdir(full):
            sub = {'t': 'd', 'n': name, 'p': r, 'c': {}}
            node['c'][name] = sub
            walk_tree(full, r, sub)
        else:
            ext = os.path.splitext(name)[1].lower()
            binary = ext in BIN_EXT or os.path.getsize(full) > 2_000_000
            loc = 0
            if not binary:
                try:
                    loc = sum(1 for _ in open(full, 'rb'))
                except Exception:
                    loc = 0
            node['c'][name] = {'t': 'f', 'n': name, 'p': r,
                               'loc': loc,
                               'lang': 'bin' if binary else LANG.get(ext, ext.lstrip('.') or '?')}
    return node


def collect_files(node, acc):
    if node['t'] == 'f':
        acc.append(node)
    else:
        for c in node.get('c', {}).values():
            collect_files(c, acc)
    return acc


# ---------------------------------------------------------------- C++ symbols

FUNC_RE = re.compile(
    r'^\s*(?:template\s*<[^>]*>\s*)?'
    r'(?:(?P<ret>[\w:<>,\*&\s]+?)\s+)?'
    r'(?P<qual>[A-Za-z_]\w*(?:::[A-Za-z_]\w*)*)\s*'
    r'\((?P<args>[^;]*)')
KEYWORDS = {'if', 'for', 'while', 'switch', 'catch', 'return', 'sizeof', 'else',
            'do', 'new', 'delete', 'alignof', 'decltype', 'static_assert'}
NS_SKIP = {'std', 'rclcpp', 'Eigen', 'geometry_msgs', 'nav_msgs', 'sensor_msgs',
           'std_msgs', 'visualization_msgs', 'scan_planner_msgs', 'tf2', 'tf2_ros',
           'gz', 'rcpputils', 'rcl', 'builtin_interfaces', 'yaml', 'cv',
           'onnxruntime', 'ort', 'shape_msgs', 'trajectory_msgs', 'nonstd'}

FIELD_RE = re.compile(
    r'^\s+(?:const(?:expr)?\s+)?(?:static\s+)?(?:mutable\s+)?'
    r'(?P<type>[\w:<>,\s\*&]+?)\s+'
    r'(?P<name>[a-z]\w*_)\s*'
    r'(?P<init>\{[^{};]*\}|=[^;]*)?;')
CLASS_RE = re.compile(r'^\s*(?:class|struct)\s+(\w+)\b')


def parse_cpp(path, rel, text):
    lines = text.splitlines()
    funcs, classes, fields = [], [], []
    scopes = []  # (name, start_line, end_line) 1-based
    n = len(lines)
    i = 0

    def enclosing_cls(idx):
        best = None
        for nm, st, en in scopes:
            if st <= idx + 1 <= en and (best is None or st > best[1]):
                best = (nm, st, en)
        return best[0] if best else None

    while i < n:
        s = strip_noise(lines[i])
        if not s.strip():
            i += 1
            continue
        m = CLASS_RE.match(s) if not s.startswith('//') else None
        if m:
            j = i
            while j < min(i + 4, n) and '{' not in strip_noise(lines[j]):
                j += 1
            if j < min(i + 4, n) and (j == i or re.match(r'^\s*\{', strip_noise(lines[j]))):
                endline = brace_match_end(lines, j)
                if endline is not None:
                    classes.append({'n': m.group(1), 'line': i + 1, 'end': endline + 1})
                    scopes.append((m.group(1), i + 1, endline + 1))
                i += 1
                continue
        fm = FUNC_RE.match(s) if not s.startswith('#') else None
        if fm and fm.group('qual') not in KEYWORDS:
            joined, j = s, i
            guard = 0
            while joined.count('(') > joined.count(')') and j + 1 < n and guard < 40:
                j += 1
                guard += 1
                joined += ' ' + strip_noise(lines[j])
            if joined.count('(') == joined.count(')'):
                tail = joined[joined.rfind(')'):]
                has_brace = '{' in tail
                if not has_brace:
                    k = j + 1
                    while k < min(j + 4, n) and not strip_noise(lines[k]).strip():
                        k += 1
                    has_brace = k < n and strip_noise(lines[k]).lstrip().startswith('{')
                qual = fm.group('qual')
                ret = (fm.group('ret') or '').strip()
                enc = enclosing_cls(i)
                ok_ret = bool(ret) or qual == enc or '::' in qual
                if has_brace and ok_ret and not joined.rstrip().endswith(';'):
                    endline = brace_match_end(lines, i)
                    if endline is not None:
                        parts = qual.split('::')
                        if len(parts) >= 3:
                            cls, name = parts[-2], parts[-1]
                        elif len(parts) == 2 and parts[0] in NS_SKIP:
                            cls, name = None, parts[1]
                        elif len(parts) == 2:
                            cls, name = parts[0], parts[1]
                        else:
                            cls, name = enc, parts[0]
                        funcs.append({'name': name, 'cls': cls, 'qual': qual,
                                      'file': rel, 'line': i + 1, 'end': endline + 1,
                                      'lang': 'cpp', 'sig': joined.strip()[:300]})
                        i = endline + 1
                        continue
        enc = enclosing_cls(i)
        if enc is not None and not s.lstrip().startswith('*'):
            js, jj = s, i
            guard = 0
            while ';' not in js and jj + 1 < n and guard < 4:
                jj += 1
                guard += 1
                js += ' ' + strip_noise(lines[jj])
            if ';' in js:
                fdm = FIELD_RE.match(js)
                if fdm:
                    fields.append({'name': fdm.group('name'),
                                   'type': fdm.group('type').strip(),
                                   'cls': enc, 'line': i + 1,
                                   'init': (fdm.group('init') or '').strip()})
        i += 1
    return funcs, classes, fields


# ---------------------------------------------------------------- Python

class _CallCollector(ast.NodeVisitor):
    def __init__(self):
        self.calls = []

    def visit_FunctionDef(self, node):
        return  # do not attribute nested defs' calls to the outer function

    visit_AsyncFunctionDef = visit_FunctionDef
    visit_Lambda = visit_FunctionDef

    def visit_Call(self, node):
        f = node.func
        if isinstance(f, ast.Name):
            self.calls.append((f.id, node.lineno, False, None))
        elif isinstance(f, ast.Attribute):
            recv = f.value.id if isinstance(f.value, ast.Name) else None
            self.calls.append((f.attr, node.lineno, True, recv))
        self.generic_visit(node)


class PyVisitor(ast.NodeVisitor):
    def __init__(self, rel, lines):
        self.rel = rel
        self.lines = lines
        self.funcs, self.classes, self.fields = [], [], []
        self.stack = []

    def visit_ClassDef(self, node):
        self.classes.append({'n': node.name, 'line': node.lineno})
        self.stack.append(node.name)
        self.generic_visit(node)
        self.stack.pop()

    def _self_attr(self, node):
        if isinstance(node, ast.Attribute) and isinstance(node.value, ast.Name) \
                and node.value.id == 'self':
            return node.attr
        return None

    def visit_Assign(self, node):
        if self.stack:
            t = node.targets[0]
            name = self._self_attr(t)
            if name:
                self.fields.append({'name': name, 'cls': self.stack[-1],
                                    'line': node.lineno, 'type': 'py'})
        self.generic_visit(node)

    def visit_AnnAssign(self, node):
        if self.stack and node.value:
            name = self._self_attr(node.target)
            if name:
                self.fields.append({'name': name, 'cls': self.stack[-1],
                                    'line': node.lineno, 'type': 'py'})
        self.generic_visit(node)

    def visit_FunctionDef(self, node):
        end = max(getattr(node, 'end_lineno', node.lineno), node.lineno)
        coll = _CallCollector()
        for stmt in node.body:
            coll.visit(stmt)
        self.funcs.append({'name': node.name,
                           'cls': self.stack[-1] if self.stack else None,
                           'qual': '.'.join(self.stack + [node.name]),
                           'file': self.rel, 'line': node.lineno, 'end': end,
                           'lang': 'py',
                           'sig': self.lines[node.lineno - 1].strip()[:300],
                           'calls': coll.calls})
        self.stack.append(node.name)
        self.generic_visit(node)
        self.stack.pop()


def parse_py(path, rel, text):
    lines = text.splitlines()
    v = PyVisitor(rel, lines)
    try:
        v.visit(ast.parse(text))
    except SyntaxError as e:
        print(f'  [warn] python parse failed {rel}: {e}', file=sys.stderr)
    return v.funcs, v.classes, v.fields


# ---------------------------------------------------------------- main

def main():
    tree = walk_tree(ROOT, '', {'t': 'd', 'n': 'SCAN-Planner', 'p': '', 'c': {}})
    all_files = collect_files(tree, [])
    text_files = [f for f in all_files
                  if f['lang'] in ('C++', 'Python', 'msg')
                  and not f['p'].startswith(SRC_SKIP_PREFIX)]

    funcs, classes_by_file, fields_by_file = [], {}, {}
    texts = {}
    src = {}
    for f in text_files:
        rel, path = f['p'], os.path.join(ROOT, f['p'])
        try:
            text = open(path, encoding='utf-8', errors='replace').read()
        except Exception as e:
            print(f'  [warn] read failed {rel}: {e}', file=sys.stderr)
            continue
        texts[rel] = text
        if f['lang'] == 'msg' or os.path.getsize(path) <= MAX_EMBED_SRC_BYTES:
            src[rel] = text.splitlines()
        if f['lang'] == 'C++':
            fs, cs, fd = parse_cpp(path, rel, text)
        elif f['lang'] == 'Python':
            fs, cs, fd = parse_py(path, rel, text)
        else:
            fs, cs, fd = [], [], []
        classes_by_file[rel] = cs
        fields_by_file[rel] = fd
        funcs.extend(fs)

    for k, fn in enumerate(funcs):
        fn['id'] = f'f{k}'
    by_name = {}
    by_qual = {}
    by_cls_name = {}
    for fn in funcs:
        by_name.setdefault((fn['lang'], fn['name']), []).append(fn['id'])
        if fn['name'] != fn['qual']:
            by_name.setdefault((fn['lang'], fn['qual']), []).append(fn['id'])
        by_qual[fn['qual']] = fn['id']
        if fn['cls']:
            by_cls_name.setdefault((fn['cls'], fn['name']), []).append(fn['id'])

    # ---- call edges ----
    CALL_TOKEN_RE = re.compile(r'([A-Za-z_]\w*(?:::[A-Za-z_]\w*)*)\s*\(')
    SKIP_CALLS = set(KEYWORDS) | {'main', 'sizeof', 'assert', 'CRITICAL_SECTION'}
    PY_BUILTINS = {'len', 'print', 'range', 'str', 'int', 'float', 'bool',
                   'list', 'dict', 'set', 'tuple', 'min', 'max', 'sum',
                   'sorted', 'abs', 'any', 'all', 'isinstance', 'getattr',
                   'setattr', 'super', 'enumerate', 'zip', 'round', 'open',
                   'type', 'repr', 'hasattr', 'iter', 'next', 'map', 'filter',
                   'format', 'vars', 'id', 'ord', 'chr', 'hex'}
    member_types = {}
    for _rel, _fd in fields_by_file.items():
        for _f0 in _fd:
            _base = base_class_of(_f0.get('type', ''))
            if _base and _base != 'py' and _f0.get('cls'):
                member_types.setdefault((_f0['cls'], _f0['name']), _base)
    LANG_OF = {f['id']: f['lang'] for f in funcs}
    FUNC_BY_ID = {f['id']: f for f in funcs}
    calls, unresolved = [], []
    unresolved_total = 0
    for fn in funcs:
        lang = fn['lang']
        if lang == 'cpp':
            lines = texts[fn['file']].splitlines()
            body_start = fn['line']
            for li0 in range(fn['line'], min(fn['end'], len(lines)) + 1):
                if '{' in strip_noise(lines[li0 - 1]):
                    body_start = li0
                    break
            seen = set()
            for li in range(body_start, min(fn['end'], len(lines)) + 1):
                rawline = lines[li - 1] if li - 1 < len(lines) else ''
                line = strip_noise(rawline)
                if li == body_start:
                    ob = line.find('{')
                    if ob >= 0:
                        line = line[ob + 1:]
                for m in CALL_TOKEN_RE.finditer(line):
                    tok = m.group(1)
                    if tok in SKIP_CALLS or (tok, li) in seen:
                        continue
                    seen.add((tok, li))
                    head = tok.split('::')[0]
                    if '::' in tok and head in NS_SKIP:
                        continue
                    member_call = m.start(1) > 0 and line[m.start(1) - 1] in '.>'
                    target, conf = None, None
                    if '::' in tok:
                        parts = tok.split('::')
                        cname, meth = parts[-2], parts[-1]
                        cands = [c for c in by_cls_name.get((cname, meth), [])
                                 if LANG_OF[c] == 'cpp']
                        if len(cands) == 1:
                            target, conf = cands[0], 'exact'
                        elif cands:
                            target, conf = cands[0], 'ambig'
                        else:
                            nm = by_name.get(('cpp', meth), [])
                            if nm:
                                target = nm[0]
                                conf = 'name' if len(nm) == 1 else 'ambig'
                    elif member_call:
                        meth = tok.split('::')[-1]
                        rm2 = re.search(r'(\w+)\s*(?:->|\.)\s*$', line[:m.start(1)])
                        recv = rm2.group(1) if rm2 else None
                        recv_cls = None
                        if recv == 'this':
                            recv_cls = fn['cls']
                        elif recv and fn['cls']:
                            recv_cls = member_types.get((fn['cls'], recv))
                        if recv_cls:
                            cands = [c for c in by_cls_name.get((recv_cls, meth), [])
                                     if LANG_OF[c] == 'cpp']
                            if len(cands) == 1:
                                target, conf = cands[0], 'exact'
                        if target is None:
                            continue  # receiver type unknown: never guess
                    else:
                        if fn['cls']:
                            same = [c for c in by_cls_name.get((fn['cls'], tok), [])
                                    if LANG_OF[c] == 'cpp']
                            if len(same) == 1:
                                target, conf = same[0], 'exact'
                        if target is None:
                            nm = by_name.get(('cpp', tok), [])
                            if nm:
                                target = nm[0]
                                conf = 'name' if len(nm) == 1 else 'ambig'
                    if target is None:
                        unresolved_total += 1
                        if len(unresolved) < 4000:
                            unresolved.append({'a': fn['id'], 'line': li, 'name': tok})
                        continue
                    tf = FUNC_BY_ID.get(target)
                    if tf and tf['cls'] and tf['name'] == tf['cls'] \
                            and tok.split('::')[-1] == tf['name']:
                        continue  # object construction, not a function call
                    calls.append({'a': fn['id'], 'b': target, 'conf': conf, 'line': li})
        else:
            for name, line, is_member, recv in fn.get('calls', []):
                target, conf = None, None
                if is_member and recv != 'self':
                    continue  # receiver type unknown: never guess
                if is_member and recv == 'self' and fn['cls']:
                    same = [c for c in by_cls_name.get((fn['cls'], name), [])
                            if LANG_OF[c] == 'py']
                    if len(same) == 1:
                        target, conf = same[0], 'exact'
                elif not is_member:
                    nm = [c for c in by_name.get(('py', name), []) if c != fn['id']]
                    top = [c for c in nm if not FUNC_BY_ID[c]['cls']]
                    same_file = [c for c in top if FUNC_BY_ID[c]['file'] == fn['file']]
                    if len(same_file) == 1:
                        target, conf = same_file[0], 'exact'
                    elif len(top) == 1:
                        target, conf = top[0], 'name'
                    if target is None and fn['cls']:
                        same = [c for c in by_cls_name.get((fn['cls'], name), [])
                                if LANG_OF[c] == 'py']
                        if len(same) == 1:
                            target, conf = same[0], 'exact'
                if target is None:
                    if not is_member and name not in PY_BUILTINS:
                        unresolved_total += 1
                        if len(unresolved) < 4000:
                            unresolved.append({'a': fn['id'], 'line': line, 'name': name})
                    continue
                calls.append({'a': fn['id'], 'b': target, 'conf': conf, 'line': line})

    # ---- fields + sites ----
    raw_fields = []
    for rel, fd in fields_by_file.items():
        for f0 in fd:
            raw_fields.append({'name': f0['name'], 'cls': f0['cls'], 'file': rel,
                               'line': f0['line'], 'type': f0.get('type', '')})
    merged = {}
    order = []
    for v in raw_fields:
        key = (v['file'], v['cls'], v['name'])
        if key not in merged:
            merged[key] = v
            order.append(key)
    CLEAR_LITERALS = {'0', '0.0', 'false', 'nullptr', '""', '{}', 'NULL', '0u',
                      "''", 'None', 'False', '[]', '""', '0.0f'}
    fields = []
    for key in order:
        v = merged[key]
        name = v['name']
        is_py_file = v['file'].endswith('.py')
        sites = []
        counts = {'set': 0, 'clear': 0, 'mod': 0, 'read': 0}
        esc_name = r'(?<![A-Za-z0-9_])' + re.escape(name)
        for rel, text in texts.items():
            rel_is_py = rel.endswith('.py')
            if rel_is_py != is_py_file:
                continue
            if rel_is_py and name.endswith('_') and not name.endswith('__') \
                    and name not in getattr(__builtins__, '__dict__', {}):
                # cpp-style name in py file: skip unless py file also uses it
                if not re.search(r'\bself\.' + esc_name + r'\b', text):
                    continue
            for li, line in enumerate(text.splitlines(), 1):
                if name not in line or (rel == v['file'] and li == v['line']):
                    continue
                s = strip_noise(line) if not rel_is_py else re.sub(r'#.*', '', line)
                if name not in s:
                    continue
                if not re.search(esc_name + r'(?![A-Za-z0-9_])', s):
                    continue
                kind, rhs = None, ''
                if rel_is_py:
                    am = re.search(esc_name + r'\s*=(?!=)', s)
                    if am:
                        rhs = s[am.end():].split('#')[0].strip()
                        kind = 'clear' if rhs in CLEAR_LITERALS else 'set'
                    elif re.search(esc_name + r'\.(clear|pop|remove|discard)\(', s):
                        kind = 'clear'
                    elif re.search(esc_name + r'\s*(\+=|-=|\*=|/=)|' + esc_name +
                                   r'\.(append|extend|add|update|insert|popitem)\(', s):
                        kind = 'mod'
                    else:
                        kind = 'read'
                else:
                    am = re.search(esc_name + r'\s*=(?!=)', s)
                    sm = re.search(esc_name + r'\.(store|exchange)\s*\(([^)]*)\)', s)
                    cm = re.search(esc_name + r'\.(clear|reset|release)\(', s)
                    wm = re.search(esc_name + r'\s*(\+=|-=|\+\+|--|\*=|/=)', s)
                    fm2 = re.search(esc_name + r'\.fetch_(add|sub)\s*\(', s)
                    mm = re.search(esc_name + r'\.(\w+)\s*\(', s)
                    MOD_METHODS = {'begin', 'observe', 'mark', 'update', 'consume',
                                   'advance', 'record', 'accumulate', 'push_back',
                                   'emplace_back', 'insert', 'erase', 'assign',
                                   'swap', 'tick', 'extend', 'append', 'add',
                                   'start', 'finish', 'commit', 'set'}
                    if am:
                        rhs = s[am.end():].strip().strip(';').split(';')[0].strip()
                        kind = 'clear' if rhs in CLEAR_LITERALS else 'set'
                    elif sm:
                        arg = sm.group(2).strip()
                        kind = 'clear' if arg in CLEAR_LITERALS else 'set'
                    elif cm:
                        kind = 'clear'
                    elif wm or fm2:
                        kind = 'mod'
                    elif mm and mm.group(1) in MOD_METHODS:
                        kind = 'mod'
                    else:
                        kind = 'read'
                counts[kind] += 1
                if kind == 'read':
                    if counts['read'] <= 120:
                        sites.append({'file': rel, 'line': li, 'kind': kind})
                elif counts[kind] <= 60:
                    sites.append({'file': rel, 'line': li, 'kind': kind,
                                  'ctx': line.strip()[:160]})
        if counts['set'] + counts['mod'] == 0 and counts['read'] > 400:
            continue  # noise field
        v['sites'] = sites
        v['counts'] = counts
        v['id'] = 'v%d' % len(fields)
        fields.append(v)

    # ---- messages ----
    msgs = []
    for f in all_files:
        if f['lang'] != 'msg':
            continue
        rel = f['p']
        mfields, consts = [], []
        pending_cmt = []
        for li, line in enumerate(texts[rel].splitlines(), 1):
            line = line.strip()
            if not line:
                continue
            if line.startswith('#'):
                pending_cmt.append(line.lstrip('# ').strip())
                continue
            parts = line.split(None, 1)
            if len(parts) == 2 and '=' in parts[1] and not parts[1].startswith('='):
                t, n_v = parts[0], parts[1].split('=', 1)
                consts.append({'type': t, 'name': n_v[0].strip(),
                               'val': n_v[1].strip(), 'line': li,
                               'comment': ' '.join(pending_cmt)})
            elif len(parts) == 2:
                mfields.append({'type': parts[0],
                                'name': parts[1].split('=')[0].strip(),
                                'line': li, 'comment': ' '.join(pending_cmt)})
            pending_cmt = []
        msgs.append({'name': os.path.splitext(f['n'])[0], 'file': rel,
                     'fields': mfields, 'consts': consts, 'topics': {}})
    msg_by_type = {}
    for m in msgs:
        msg_by_type['scan_planner_msgs::msg::' + m['name']] = m
        msg_by_type['scan_planner_msgs/msg/' + m['name']] = m
        msg_by_type[m['name']] = m

    # ---- topics (full-text regexes so multi-line decls are caught) ----
    topics = {}
    members = {}  # member -> {topic: set(files declaring it)} (publisher shared across files)
    PUB_RE = re.compile(
        r'(\w+)\s*=\s*[^;=]*?create_publisher<\s*([\w:]+)\s*>\s*\(\s*"([^"]+)"')
    SUBSCRIBE_RE = re.compile(
        r'create_subscription<\s*([\w:]+)\s*>\s*\(\s*"([^"]+)"')
    PY_PUB_RE = re.compile(
        r'self\.(\w+)\s*=\s*self\.create_publisher\(\s*(\w+)\s*,\s*[\'"]([^\'"]+)[\'"]')
    PY_SUB_RE = re.compile(
        r'self\.create_subscription\(\s*(\w+)\s*,\s*[\'"]([^\'"]+)[\'"]\s*,\s*'
        r'(?:self\.)?([A-Za-z_]\w*)')
    PUBCALL_RE = re.compile(r'\b(\w+)->publish\(')
    PY_PUBCALL_RE = re.compile(r'\bself\.(\w+)\.publish\(')

    py_alias = {}
    for rel, text in texts.items():
        if not rel.endswith('.py'):
            continue
        for m in re.finditer(r'from\s+scan_planner_msgs\.msg\s+import\s+([^\n#]+)', text):
            for part in m.group(1).split(','):
                mm = re.match(r'\s*(\w+)(?:\s+as\s+(\w+))?', part)
                if mm and mm.group(2):
                    py_alias[mm.group(2)] = mm.group(1)

    def enclosing(rel, line_no):
        best = None
        for fn in funcs:
            if fn['file'] == rel and fn['line'] <= line_no <= (fn['end'] or fn['line']):
                if best is None or fn['line'] > best['line']:
                    best = fn
        return best['id'] if best else None

    for rel, text in texts.items():
        is_py = rel.endswith('.py')
        for m in (PY_PUB_RE if is_py else PUB_RE).finditer(text):
            line_no = offset_to_line(text, m.start())
            member, mtype, topic = m.group(1), m.group(2), m.group(3)
            if is_py:
                mtype = py_alias.get(mtype, mtype)
            members.setdefault(member, {}).setdefault(topic, set()).add(rel)
            t = topics.setdefault(topic, {'type': mtype, 'pubs': [], 'subs': [],
                                          'publish_sites': []})
            t['pubs'].append({'file': rel, 'line': line_no, 'member': member})
        if is_py:
            for m in PY_SUB_RE.finditer(text):
                line_no = offset_to_line(text, m.start())
                mtype, topic, cb = m.group(1), m.group(2), m.group(3)
                mtype = py_alias.get(mtype, mtype)
                t = topics.setdefault(topic, {'type': mtype, 'pubs': [], 'subs': [],
                                              'publish_sites': []})
                t['subs'].append({'file': rel, 'line': line_no, 'cb': cb})
        else:
            for m in SUBSCRIBE_RE.finditer(text):
                line_no = offset_to_line(text, m.start())
                mtype, topic = m.group(1), m.group(2)
                k, depth = m.end(), 1
                while k < len(text) and depth:
                    ch = text[k]
                    if ch == '"':
                        k += 1
                        while k < len(text) and text[k] != '"':
                            k += 2 if text[k] == '\\' else 1
                    elif ch == '(':
                        depth += 1
                    elif ch == ')':
                        depth -= 1
                    k += 1
                cb = '<lambda>'
                for a in split_top_level(text[m.end():k-1]):
                    if not a or re.fullmatch(r'[0-9]+', a) or a.startswith('rclcpp::'):
                        continue
                    if a.startswith('['):
                        cb = '<lambda>'
                        break
                    if 'std::bind' in a or '&' in a:
                        cbm = re.search(r'&[A-Za-z_]\w*(?:::[A-Za-z_]\w*)*::(\w+)', a)
                        if not cbm:
                            cbm = re.search(r'self\.(\w+)', a)
                        cb = cbm.group(1) if cbm else '<lambda>'
                        break
                    if re.fullmatch(r'[A-Za-z_]\w*', a):
                        cb = a
                        break
                t = topics.setdefault(topic, {'type': mtype, 'pubs': [], 'subs': [],
                                              'publish_sites': []})
                t['subs'].append({'file': rel, 'line': line_no, 'cb': cb})
    # second pass: publish() sites (all publisher declarations are known now)
    for rel, text in texts.items():
        is_py = rel.endswith('.py')
        for m in (PY_PUBCALL_RE if is_py else PUBCALL_RE).finditer(text):
            member = m.group(1)
            cand = members.get(member, {})
            topic = None
            if len(cand) == 1:
                topic = next(iter(cand))
            elif cand:
                same = [t for t, files in cand.items() if rel in files]
                if len(same) == 1:
                    topic = same[0]
            if not topic:
                continue
            line_no = offset_to_line(text, m.start())
            if topics[topic]['type'].startswith('visualization') and 'disp' in member:
                continue
            fu = enclosing(rel, line_no)
            topics[topic]['publish_sites'].append(
                {'file': rel, 'line': line_no, 'member': member, 'func': fu})
    for topic, t in topics.items():
        m = msg_by_type.get(t['type'])
        if m:
            m['topics'][topic] = t

    # ---- logs ----
    logs = []
    LOG_CPP_RE = re.compile(
        r'RCLCPP_(DEBUG|INFO|WARN|ERROR|FATAL)(?:_EVERY|_THROTTLE|_SKIPFIRST)?\s*\(\s*[^,]+,')
    STR_RE = re.compile(r'"((?:[^"\\]|\\.)*)"')
    LOG_PY_RE = re.compile(
        r'get_logger\(\)\.(debug|info|warn|warning|error|fatal)\(\s*"((?:[^"\\]|\\.)*)"',
        re.S)
    for rel, text in texts.items():
        is_py = rel.endswith('.py')
        if is_py:
            for m in LOG_PY_RE.finditer(text):
                sev = {'warning': 'WARN'}.get(m.group(1).upper(), m.group(1).upper())
                line_no = offset_to_line(text, m.start())
                logs.append({'sev': sev, 'file': rel, 'line': line_no,
                             'func': enclosing(rel, line_no),
                             'msg': m.group(2)[:400]})
            continue
        for m in LOG_CPP_RE.finditer(text):
            line_no = offset_to_line(text, m.start())
            tail = text[m.end():m.end() + 2000]
            fm = STR_RE.search(tail)
            if not fm:
                continue
            logs.append({'sev': m.group(1), 'file': rel, 'line': line_no,
                         'func': enclosing(rel, line_no), 'msg': fm.group(1)[:400]})

    # ---- reason tokens ----
    reasons = {}
    TOK_RE = re.compile(r'"([A-Z][A-Z0-9_]{5,})"')
    for rel, text in texts.items():
        is_py = rel.endswith('.py')
        for li, line in enumerate(text.splitlines(), 1):
            if '"' not in line:
                continue
            s = strip_noise(line, keep_strings=True) if not is_py else re.sub(r'#.*', '', line)
            for m in TOK_RE.finditer(s):
                tok = m.group(1)
                if tok in ('DEFAULT', 'SHARED', 'SHUTDOWN', 'CREATE', 'SYSTEM',
                           'ROBOT_STATE', 'PHYSICAL', 'FAMILY'):
                    continue
                reasons.setdefault(tok, []).append({'file': rel, 'line': li})

    in_edges = {}
    for e in calls:
        in_edges.setdefault(e['b'], []).append(e['a'])
    meta = {
        'workspace': ROOT,
        'branch': git('branch', '--show-current') or '?',
        'head': git('rev-parse', '--short', 'HEAD') or '?',
        'dirty': [l[3:] for l in git('status', '--short').splitlines() if l][:40],
        'dirty_count': len([l for l in git('status', '--short').splitlines() if l]),
        'generated_at': datetime.datetime.now().strftime('%Y-%m-%d %H:%M:%S'),
        'total_files': len(all_files),
        'total_loc': sum(f['loc'] for f in all_files),
        'n_funcs': len(funcs), 'n_calls': len(calls), 'n_fields': len(fields),
        'n_logs': len(logs), 'n_msgs': len(msgs), 'n_topics': len(topics),
        'n_unresolved': unresolved_total, 'n_unresolved_listed': len(unresolved),
        'limits': [
            '调用边分级：exact=限定名/同类唯一命中；name=全工作区唯一同名；ambig=存在多个同名符号；'
            'unresolved=目标不在工作区索引内。页面不把 name/ambig/unresolved 显示为确定链路。',
            '函数识别基于定义行+花括号匹配：宏生成函数、模板特化、未闭合花括号的宏体可能漏提。',
            '字段的 设置/清除/修改/读取 分类是 RHS 字面量与方法名的模式匹配，非语义分析；'
            '读取点计数为完整计数，列表在超上限时截断（面板注明）。',
            '「保留(未见清除)」表示静态索引中未发现清除/复位点，是启发式结论，不是运行时证明。',
            '回调接线依据 create_subscription 处的 std::bind / self.cb 文本；lambda 回调记为 <lambda>；'
            '定时器与服务回调未索引。',
            '对象成员调用（obj->method()）仅在能从成员声明类型推断接收者类时入图；'
            '接收者类型无法静态推断时不建边，避免猜测。',
            'simulator/local_sensing 第三方代码、tools/ 与 docs/ 仅出现在文件树，未提取符号'
            '（工具不索引自身）；二进制/超 2MB 文件不计 LOC。',
        ],
    }
    DATA = {'meta': meta, 'tree': tree, 'files': {}, 'funcs': funcs, 'calls': calls,
            'unresolved': unresolved, 'fields': fields, 'msgs': msgs,
            'topics': topics, 'logs': logs, 'reasons': reasons, 'src': src}
    for f in all_files:
        rel = f['p']
        DATA['files'][rel] = {
            'lang': f['lang'], 'loc': f['loc'],
            'cls': classes_by_file.get(rel, []),
            'fields': [v['id'] for v in fields if v['file'] == rel],
            'funcs': [fn['id'] for fn in funcs if fn['file'] == rel],
        }

    # ---- emit ----
    out_html = os.path.join(ROOT, 'docs', 'CONTRACT_STACK_INTERACTIVE_ZH.html')
    backup = out_html.replace('.html', '.pre-audit.html')
    if not os.path.exists(backup):
        shutil.copyfile(out_html, backup)
        print('backup written:', backup)
    import template
    html = template.render(DATA)
    with open(out_html, 'w', encoding='utf-8') as fh:
        fh.write(html)
    with open(os.path.join(HERE, 'audit_data.json'), 'w', encoding='utf-8') as fh:
        slim = {k: v for k, v in DATA.items() if k != 'src'}
        json.dump(slim, fh, ensure_ascii=False)
    print(f'html: {out_html} ({len(html) // 1024} KB)')
    print('funcs=%d calls=%d fields=%d logs=%d msgs=%d topics=%d unresolved=%d'
          % (len(funcs), len(calls), len(fields), len(logs), len(msgs),
             len(topics), len(unresolved)))
    print('branch=%s head=%s dirty=%d' % (meta['branch'], meta['head'],
                                          meta['dirty_count']))


if __name__ == '__main__':
    main()
