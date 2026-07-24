"""Generate settings artifacts from settings.yml (single source of truth).

Outputs:
    src/settings_gen.h          firmware table
    scripts/protocol_limits.py  shared limits for protocol.py
    docs/settings.md            documentation table

Run from anywhere:  python scripts/generate_settings.py
"""

from __future__ import annotations

import sys
from pathlib import Path

import yaml

ROOT = Path(__file__).resolve().parent.parent
HDR = "AUTO-GENERATED from settings.yml — do not edit; run scripts/generate_settings.py"


def main() -> int:
    cfg = yaml.safe_load((ROOT / "settings.yml").read_text(encoding="utf-8"))
    settings = cfg["settings"]

    for s in settings:
        for k in ("id", "name", "default", "min", "max", "desc"):
            if k not in s:
                sys.exit(f"settings.yml: entry missing '{k}': {s}")
        if not (0 <= s["min"] <= s["default"] <= s["max"] <= 0xFFFF):
            sys.exit(f"settings.yml: bad range for {s['name']}")
    ids = [s["id"] for s in settings]
    if len(set(ids)) != len(ids):
        sys.exit("settings.yml: duplicate ids")

    # ---- firmware header -------------------------------------------------
    lines = [
        f"/* {HDR} */",
        "#ifndef SETTINGS_GEN_H",
        "#define SETTINGS_GEN_H",
        "",
        "#include <stdint.h>",
        "",
    ]
    for s in settings:
        lines.append(f"#define SETTING_{s['name'].upper():<20} {s['id']}")
    lines += [
        "",
        f"#define SETTINGS_COUNT {len(settings)}",
        f"#define SETTINGS_MAX_ID {max(ids)}",
        "",
        "struct setting_meta {",
        "    uint8_t  id;",
        "    const char *name;",
        "    uint16_t def;",
        "    uint16_t min;",
        "    uint16_t max;",
        "};",
        "",
        "#define SETTINGS_TABLE { \\",
    ]
    for s in settings:
        lines.append(f"    {{ {s['id']}, \"{s['name']}\", "
                     f"{s['default']}, {s['min']}, {s['max']} }}, \\")
    lines += ["}", "", "#endif /* SETTINGS_GEN_H */", ""]
    (ROOT / "src" / "settings_gen.h").write_text("\n".join(lines),
                                                 encoding="utf-8")

    # ---- python limits ---------------------------------------------------
    py = [f'"""{HDR}"""', ""]
    for s in settings:
        up = s["name"].upper()
        py.append(f"{up}_MIN = {s['min']}")
        py.append(f"{up}_MAX = {s['max']}")
        py.append(f"{up}_DEFAULT = {s['default']}")
    py += [
        "",
        "LIMITS = {",
    ]
    for s in settings:
        py.append(f"    \"{s['name']}\": ({s['min']}, {s['max']}, "
                  f"{s['default']}),")
    py += ["}", ""]
    (ROOT / "scripts" / "protocol_limits.py").write_text("\n".join(py),
                                                         encoding="utf-8")

    # ---- docs table ------------------------------------------------------
    md = [
        f"<!-- {HDR} -->",
        "# Device Settings",
        "",
        "Persisted across reboots (spare 4 KB flash partition at",
        "0x164000); applied at boot.  Motor rail/driver enables are not",
        "settings — the motor always boots off.  Source: `settings.yml`.",
        "",
        "| ID | Setting | Default | Range | Description |",
        "|---|---|---|---|---|",
    ]
    for s in settings:
        md.append(f"| {s['id']} | `{s['name']}` | {s['default']} | "
                  f"{s['min']}–{s['max']} | {s['desc']} |")
    md.append("")
    (ROOT / "docs" / "settings.md").write_text("\n".join(md),
                                               encoding="utf-8")

    print(f"generated {len(settings)} settings -> settings_gen.h, "
          f"protocol_limits.py, docs/settings.md")
    return 0


if __name__ == "__main__":
    sys.exit(main())
