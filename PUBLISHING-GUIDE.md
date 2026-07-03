# Publishing an Obsidian subfolder as a GitHub Pages site (Quartz 5)

A repeatable workflow for sharing one folder of a private Obsidian vault as a
read-only website, **without ever copying or committing the notes**. The
Caterpillar dev-notes site (this branch) is the reference implementation.

## The idea

```
<project repo> (public, on GitHub)
├─ main       → your actual project (untouched)
├─ quartz     → orphan branch: Quartz 5 generator + publish.sh (this branch)
└─ gh-pages   → orphan branch: rendered HTML, served by GitHub Pages

~/Documents/Obsidian_repo/<subdir>   → notes stay here, read in place at build time
```

- The vault stays private; nothing under it is committed to the public repo.
- Quartz builds straight from the vault folder via `npx quartz build -d <path>`.
- Publishing runs **locally** (GitHub Actions can't see the private vault).
- Each publish force-pushes a fresh single-commit `gh-pages`, so deletes and
  renames never leave stale pages behind.

## Prerequisites

- Node.js ≥ 20 and git on the publishing machine (the one with the vault).
- A **public** GitHub repo (Pages is public-only on the free plan — and the
  site itself is world-readable, so only publish notes you're okay sharing).

## One-time setup

Commands below assume the project repo is cloned at `~/repo` and the notes live
in `~/Documents/Obsidian_repo/200_Projects/<Project>`.

### 1. Create the orphan branch as a worktree

```sh
cd ~/repo
git worktree add --orphan -b quartz ../<project>_quartz
```

This gives you a blank branch in its own directory; `main` stays checked out.

### 2. Drop in Quartz 5

```sh
git clone --depth 1 https://github.com/jackyzha0/quartz.git /tmp/quartz-template
rsync -a --exclude='.git' --exclude='docs' --exclude='.github' /tmp/quartz-template/ ../<project>_quartz/
rm -rf /tmp/quartz-template
cd ../<project>_quartz
npm install
npx quartz plugin install      # fetches plugins pinned in quartz.lock.json
```

The template's `.gitignore` lists `.gitignore` itself — delete that line so the
file actually gets committed.

### 3. Configure

`cp quartz.config.default.yaml quartz.config.yaml` and edit (the user config
**replaces** the default wholesale, so keep the full plugins list). What to change:

| Setting | Value |
|---|---|
| `pageTitle` / `pageTitleSuffix` | your site name |
| `baseUrl` | `<user>.github.io/<repo>` (no protocol, no trailing slash) |
| `analytics` | `null` |
| `theme.colors` | your palette (set `lightMode` = `darkMode` for a dark-only site) |
| plugin `cname` | **`enabled: false`** — otherwise it emits a bogus `CNAME` file that breaks project pages |
| plugin `darkmode` | `enabled: false` if the site is single-mode |
| plugin `footer` → `options.links` | link back to your repo |

Custom styling (fonts aside, this is where the theme personality lives):
`quartz/styles/custom.scss`. To mirror an Obsidian theme, lift the CSS
variables from `<vault>/.obsidian/themes/<Theme>/theme.css`.

### 4. Give the notes folder a homepage

Quartz requires `index.md` at the content root or the site root 404s. Create it
**in the vault subfolder** (it's a normal note you can edit in Obsidian):

```markdown
---
title: My Project — Dev Notes
---
Short intro, plus [[wikilinks]] to the key notes.
```

### 5. Add `publish.sh`

```bash
#!/usr/bin/env bash
set -euo pipefail

VAULT_CONTENT="$HOME/Documents/Obsidian_repo/200_Projects/<Project>"

cd "$(cd "$(dirname "$0")" && pwd)"
REMOTE=$(git remote get-url origin)

npx quartz build -d "$VAULT_CONTENT"

# GitHub Pages: don't run the output through Jekyll
touch public/.nojekyll

# Publish public/ as a fresh single-commit gh-pages branch
cd public
rm -rf .git
git init -q -b gh-pages
git add -A
git commit -qm "publish: $(date '+%Y-%m-%d %H:%M')"
git push -f "$REMOTE" gh-pages:gh-pages
rm -rf .git

echo "Published to https://<user>.github.io/<repo>/"
```

`chmod +x publish.sh`, then commit and push the branch:

```sh
git add -A && git commit -m "quartz: site generator for <project> notes"
git push -u origin quartz
```

### 6. First publish

```sh
./publish.sh
```

GitHub auto-enables Pages when a `gh-pages` branch appears (verify under repo
Settings → Pages: "Deploy from a branch", `gh-pages`, `/ (root)`). The site is
live at `https://<user>.github.io/<repo>/` within a minute or two.

## Daily workflow

1. Edit or add notes in Obsidian (no vault git commit needed — the build reads disk).
2. Run `./publish.sh`.
3. Hard-reload the page after ~a minute.

Preview without publishing:

```sh
npx quartz build -d "<vault subdir>" --serve    # live-reloads at localhost:8080
```

## Gotchas

- **Wikilinks pointing outside the published subfolder** render as links to the
  site's 404 page. Expected — those notes aren't published. Keep the shared
  folder reasonably self-contained.
- **Renaming a note changes its URL**; links others saved to the old URL break.
- **Moved the worktree or the main repo?** Git worktree pointers go stale; fix with
  `git -C <main-repo> worktree repair <worktree-path>` — don't recreate anything.
- **New machine:** `git clone -b quartz <repo> && npm install && npx quartz plugin install`,
  then make sure the vault exists at the path in `publish.sh`.
- **Attachments/images** embedded in notes only publish if they live inside the
  published subfolder. A vault-level `attachments/` folder won't be found.
