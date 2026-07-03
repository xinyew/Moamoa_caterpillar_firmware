# Caterpillar Dev Notes — site generator

This orphan `quartz` branch holds a [Quartz 5](https://quartz.jzhao.xyz) setup that
publishes the Caterpillar Obsidian dev notes to GitHub Pages:

**https://xinyew.github.io/Moamoa_caterpillar_firmware/**

It shares no history with `main` (the firmware). The rendered site lives on the
orphan `gh-pages` branch, which GitHub Pages serves.

## How it works

- Notes are **never copied or committed**. The build reads them in place from the
  Obsidian vault on the iMac:
  `~/Documents/Obsidian_repo/200_Projects/KaMoamoa/Caterpillar`
  (path set at the top of `publish.sh`).
- `index.md` in that vault folder is the site homepage.
- The look matches the vault's **Sweet Neon** Obsidian theme: palette in
  `quartz.config.yaml`, heading colors and glows in `quartz/styles/custom.scss`.
  The site is dark-only, like the theme.

## Publishing (iMac only)

After writing/editing notes in Obsidian:

```sh
./publish.sh
```

That builds the site from the vault and force-pushes the output to `gh-pages`.
Wiki-links pointing outside the Caterpillar folder (e.g. the KaMoamoa MOC) render
as links to a 404 page — expected, those notes aren't published.

## One-time setup (new machine)

```sh
git clone -b quartz git@github.com:xinyew/Moamoa_caterpillar_firmware.git
npm install
npx quartz plugin install   # fetches plugins pinned in quartz.lock.json
```

GitHub Pages must be set to deploy from the `gh-pages` branch, `/ (root)`
(repo Settings → Pages → Deploy from a branch).

To replicate this setup for another Obsidian subfolder/project, see
[PUBLISHING-GUIDE.md](PUBLISHING-GUIDE.md).
