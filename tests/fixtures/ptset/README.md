# P 点真值（E3D 3.1 TTY 只读取样）

`ams-ptset-truth-20261027.psv` 是 `tests/ptset_real.rs` 的真值：AMS 样例工程（`/ALL`）里 8 个根的每个直属成员、
每个目录 P 点一行，由 E3D 自己的设计级伪属性算出：

- 位置 `PPOS[n]`，方向 `PDIR[n]`，通径 `PPBORE[n]`（PML2 DBREF 属性式访问；`Q PBOR n` / `NPPOI` 在 3.1 里不存在）。
- `lx ly lz` / `ldx ldy ldz`：`POSITION.wrt(成员)` / `DIRECTION.wrt(成员)`，即成员自己的坐标系（mm，D6 / D9），
  与 `GenerationOutcome::point_sets` 交出的局部系同口径；`w*` 是 `WRT WORLD`。
- P 点编号来自目录 PTSE 的成员 `NUMB`（`SPRE → [SPCO → CATR →] SCOM → PTRE`；NOZZ 从 `CATR` 起跳），文件序。
- 没有目录链的成员也有一行（`numb` 空，`err` 说明：`SPRE/CATR unset or dangling` / `no SPRE/CATR attribute`）。
- `bore` 是 E3D 的字面（`500mm` / `Unset`）。

根（种子 → 根）：

| 库 | 根 | 名 | 成员 |
|---|---|---|---|
| ams8000 | `24384/2860` | BRAN `/M-OR-1RX-3081D` | 14（FTUB / ATTA / BEND / ELBO） |
| ams8000 | `24384/2986` | BRAN `/C-OR-1RX-3040A` | 14 |
| ams8000 | `24384/26229` | BRAN `/Copy-of-1RCV0101-60.3-VMB-R90-R332]R333` | 12（含 VALV `/Copy-of-1RCV002VP`） |
| ams7997 | `24381/30390` | HVAC BRAN `/-VES-E-4-L-2501` | 136（STRT / BRCO / BEND / THRE / OFST / DAMP） |
| ams7997 | `24381/550` | HANG `/1RCV013MN-SUPP-04/H1` | 2（PCLA / HELE） |
| ams7997 | `24381/57707` | SUBE `/-RX-CCV-001RF` | 21（8 NOZZ + 设计基本体） |
| ams7999 | `24383/66534` | BRAN `/1WCC0067B-114.3-NADB-R52-R710/B2` | 15（含 VALV `/1WCC172VN`） |
| ams7999 | `24383/68782` | HANG `/R220.156/HANG-01` | 1（PCLA） |

重取：gen-model `verification/civil-tty-20261021/ptset_truth.mac` + `ptset_truth_seeds.txt`，用
`scripts/e3d/run_ams_c_entrymacro.bat` 跑，`L3_E3D_INSTALL_DIR = E:\reverse\e3d\shadow_e3d31_gen_model_test`
（缺省的 `e3d31-runtime` 起不来，0xC0020001）。宏只读：无 SAVEWORK，不改任何元素。取样日 2026-10-27。
