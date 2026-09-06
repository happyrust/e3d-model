# Native E3D STRT regression fixtures

- `cpv-hvac6.rvm`: E3D 3.1 DESIGN TTY export on 2026-09-05, BRAN `24381/76729`, `/-CPV-S-2-L-1301`. Three STRT members `76730..76732`, each with one body and four flange cylinders. Explicit `representation level hvac 6`, insulation/obstruction off, tubes on.
- `cam-hvac6.rvm`: existing 2026-09-02 E3D TTY export from `evidence/20260902-repre-level-tty-experiments/data/hvac-5201-levels/hvac-5201-hvac6.rvm`. STRT `24381/179334,179342,179348` covers vertical, horizontal and oblique orientations.

`tests/strt_rvm.rs` parses native Cylinder radius/height and PRIM transforms directly. It does not use RVM bounding-box fields or helper-generated tessellation as the oracle. Each generated body/flange is matched exactly once, with 0.51 mm tolerance for the 0.5 mm mesh chord bound and f32 RVM placement rounding.

The local database test pins AMS 7997 session 106 and opens its referenced catalogue files. See the gen-model report `docs/2026-09-05-strt-cpv-rvm-fix.md` for fresh capture commands and red/green evidence.
