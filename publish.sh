#!/usr/bin/env bash
# Build the Caterpillar dev-notes site from the Obsidian vault and push it
# to the gh-pages branch. Run from anywhere: ./publish.sh
# Notes are read in place from the vault — never copied into this repo.
#
# Dry run (build only, no push):  PUBLISH_DRY_RUN=1 ./publish.sh
set -euo pipefail

VAULT_ROOT="$HOME/Documents/Obsidian_repo"
CONTENT_SUBDIR="200_Projects/KaMoamoa/Caterpillar"
VAULT_CONTENT="$VAULT_ROOT/$CONTENT_SUBDIR"

cd "$(cd "$(dirname "$0")" && pwd)"
REMOTE=$(git remote get-url origin)

# --- Stage notes + referenced images in a throwaway dir ---------------------
# Quartz only publishes files inside its build root, so images that live in a
# vault-level assets folder (e.g. 999_Assets/) are invisible when we build the
# Caterpillar subfolder directly. We copy the notes plus *only the images they
# actually embed* into a temp dir (preserving each image's vault-relative path)
# and build from there. Nothing here is committed; unrelated notes and unrelated
# images from the shared assets folder are never published.
STAGE="$(mktemp -d)"
trap 'rm -rf "$STAGE"' EXIT

cp -R "$VAULT_CONTENT/." "$STAGE/"

# Pull out every wikilink embed target: ![[path/img.png]] or ![[img.png|alt]].
# `|| true` so a note set with no embeds (grep exits 1) doesn't trip set -e.
embeds="$(grep -rhoE '!\[\[[^]|#]+' "$VAULT_CONTENT" 2>/dev/null | sed -E 's/^!\[\[//' | sort -u || true)"
if [ -n "$embeds" ]; then
  while IFS= read -r ref; do
    case "${ref##*.}" in
      png|jpg|jpeg|gif|webp|svg|bmp|avif|PNG|JPG|JPEG|GIF|WEBP|SVG|BMP|AVIF) ;;
      *) continue ;;
    esac
    src="$VAULT_ROOT/$ref"
    # Fall back to a vault-wide basename search for bare/moved filenames.
    [ -f "$src" ] || src="$(find "$VAULT_ROOT" -type f -name "$(basename "$ref")" -print -quit 2>/dev/null)"
    if [ -n "$src" ] && [ -f "$src" ]; then
      dest="$STAGE/$ref"
      mkdir -p "$(dirname "$dest")"
      cp -f "$src" "$dest"
      echo "  + asset: $ref"
    else
      echo "  ! missing asset (skipped): $ref" >&2
    fi
  done <<< "$embeds"
fi

npx quartz build -d "$STAGE"

# GitHub Pages: don't run the output through Jekyll
touch public/.nojekyll

if [ "${PUBLISH_DRY_RUN:-0}" = "1" ]; then
  echo "Dry run: built into public/ (not pushed)."
  exit 0
fi

# Publish public/ as a fresh single-commit gh-pages branch
cd public
rm -rf .git
git init -q -b gh-pages
git add -A
git commit -qm "publish: $(date '+%Y-%m-%d %H:%M')"
git push -f "$REMOTE" gh-pages:gh-pages
rm -rf .git

echo "Published to https://xinyew.github.io/Moamoa_caterpillar_firmware/"
