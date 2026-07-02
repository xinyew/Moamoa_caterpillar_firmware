#!/usr/bin/env bash
# Build the Caterpillar dev-notes site from the Obsidian vault and push it
# to the gh-pages branch. Run from anywhere: ./publish.sh
# Notes are read in place from the vault — never copied into this repo.
set -euo pipefail

VAULT_CONTENT="$HOME/Documents/Obsidian_repo/200_Projects/KaMoamoa/Caterpillar"

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

echo "Published to https://xinyew.github.io/Moamoa_caterpillar_firmware/"
