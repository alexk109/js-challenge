"""Independent second ISS of the MVP chip.

`pio_iss_b` is written only from `docs/mvp-isa.md` and `docs/mvp-regmap.md` (M0,
2026-10-01). It never imports `iss/`, `src/` or `asm/`. Where the spec left a
behaviour open, the reading taken is written down in `verif/iss_b/SPEC-QUESTIONS.md`.
"""