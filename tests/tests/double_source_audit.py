"""Python-only lexical guard against native floating point in Teensy sources.

Run from the repository root: python -m tests.double_source_audit
No compiler, firmware build, object files, database, or third-party module needed.
This guards application source; it does not inspect macros after expansion,
generated code, vendor libraries, or machine instructions.
"""

from __future__ import annotations

import argparse
import re
import sys
from pathlib import Path
from typing import List, Sequence, Tuple


ROOT = Path(__file__).resolve().parents[1] / "pnc" / "firmware" / "teensy"
LEX = re.compile(
    r'//[^\n]*|/\*[\s\S]*?\*/|R"(?P<delimiter>[^ ()\\\t\r\n]{0,16})'
    r'\([\s\S]*?\)(?P=delimiter)"|"(?:\\.|[^"\\])*"|\'(?:\\.|[^\'\\])*\''
)
DELETED_OVERLOAD = re.compile(
    r'^\s*void add\(const char\* key, (?:float|double) value'
    r'(?:, int precision)?\) = delete;', re.MULTILINE
)
DECIMAL_LITERAL = re.compile(r'\b\d+(?:\.\d*)?(?:[eE][+-]?\d+)?_D\b')
NATIVE = re.compile(
    r'\b(?:float|double|__fp16|_Float\d+|__float\d+|strtod|strtof|strtold|atof|tempmonGetTemp)\b'
    r'|\b0[xX][0-9a-fA-F]*(?:\.[0-9a-fA-F]*)?[pP][+-]?\d+[fFlL]?'
    r'|\b\d+\.\d*(?:[eE][+-]?\d+)?[fFlL]?'
    r'|(?<![\w.])\.\d+(?:[eE][+-]?\d+)?[fFlL]?'
    r'|\b\d+[eE][+-]?\d+[fFlL]?'
)


def blank(match: re.Match) -> str:
    return re.sub(r'[^\n]', ' ', match.group())


def findings(source: str, name: str) -> List[Tuple[int, str]]:
    code = LEX.sub(blank, source)
    if name == "payload.h":
        # Existing deleted overloads prevent native floating-point payloads.
        code = DELETED_OVERLOAD.sub(blank, code)
    code = DECIMAL_LITERAL.sub(blank, code)
    return [(code.count('\n', 0, match.start()) + 1, match.group())
            for match in NATIVE.finditer(code)]


def self_test() -> None:
    cases = [
        ('Double x = 1.25_D; Double y = 1e9_D;', 'x.cpp', False),
        ('auto n = 0xdeadbeef; int a = 123;', 'x.cpp', False),
        ('// double x = 1.0;\nconst char* s = "float 1e3";', 'x.cpp', False),
        ('const char* s = R"tag(float x = 1.0;)tag";', 'x.cpp', False),
        ('/* float */\nDouble x;', 'x.cpp', False),
        ('void add(const char* key, double value) = delete;', 'payload.h', False),
        ('void add(const char* key, float value) = delete;', 'x.h', True),
        ('double x;', 'x.cpp', True),
        ('float x;', 'x.cpp', True),
        ('long double x;', 'x.cpp', True),
        ('auto x = 1.0;', 'x.cpp', True),
        ('auto x = .5f;', 'x.cpp', True),
        ('auto x = 1e-3;', 'x.cpp', True),
        ('auto x = 0x1.2p3;', 'x.cpp', True),
        ('auto x = strtod(s, nullptr);', 'x.cpp', True),
    ]
    for source, name, expected in cases:
        if bool(findings(source, name)) != expected:
            raise AssertionError((source, name, expected))
    if findings('// ignored\n\nfloat x;', 'x.cpp') != [(3, 'float')]:
        raise AssertionError('source line numbers were not preserved')
    print(f'PASS: {len(cases) + 1} source-audit self-tests')


def main(argv: Sequence[str]) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--firmware', type=Path, default=ROOT)
    parser.add_argument('--self-test', action='store_true')
    args = parser.parse_args(argv[1:])
    if args.self_test:
        self_test()
        return 0
    files = sorted(path for path in args.firmware.rglob('*')
                   if path.suffix in ('.cpp', '.h', '.hpp', '.ino', '.c')
                   and path.is_file())
    if not files:
        print(f'ERROR: no firmware sources under {args.firmware}', file=sys.stderr)
        return 2
    violations = 0
    for path in files:
        for line, token in findings(path.read_text(), path.name):
            violations += 1
            print(f'{path}:{line}: native floating-point token {token!r}')
    print(f"{'FAIL' if violations else 'PASS'}: {len(files)} source files, "
          f'{violations} native floating-point tokens')
    return 1 if violations else 0


if __name__ == '__main__':
    raise SystemExit(main(sys.argv))
