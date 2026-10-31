# Media: every image, with the command that made it

| File | What it shows | Command | Captured | Shows |
|---|---|---|---|---|
| `docs/circle_swap.gif` | the README's hero: twenty agents swap sides through a vortex; 8 s, 96 frames | not recorded when it was made; `cargo run --release --example circle_swap` shows the same scene live | 2026-08-09 (`656c9e7`) | 0.1.0 |
| `docs/media/circle_swap.png` | the README's still: frame 30 of the hero, the vortex at the centre | `ffmpeg -i docs/circle_swap.gif -vf "select=eq(n\,30)" -frames:v 1 docs/media/circle_swap.png` | 2026-10-31 | 0.1.0 (from the GIF) |

`docs/circle_swap.gif` stays at that path: the READMEs of published releases name it, and a published README never changes.
