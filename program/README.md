# MATLAB untuk skripsi

Folder ini sekarang hanya memuat kode dan hasil yang berkaitan langsung dengan MATLAB pada skripsi: model 6-DOF L1+MO+ETC, optimizer multiobjektif, tabel metrik, kandidat Pareto, dan figure.

## Skrip yang ada

- [`l1_pid_law_6dof_coupled.m`](l1_pid_law_6dof_coupled.m): model 6-DOF dengan kopling XY, ditandai sebagai v6/thesis-grade di header.
- [`l1_mo_pareto_optimization.m`](l1_mo_pareto_optimization.m): optimizer 3 objektif, multi-seed, memakai `gamultiobj` jika tersedia dan menyediakan fallback PSO. Keluaran yang disebut di header ialah `pareto_candidates_theta.csv`.

## File XY yang disebut lampiran skripsi

Lampiran C menyebut `l1_mo_pareto_optimization_xy.m` untuk optimasi NSGA-II sebelas parameter sadar-kopling XY. Salinan lokal file tersebut berukuran 0 byte, sehingga tidak dapat dijalankan. Salinan kosongnya dipertahankan hanya di arsip lokal; arsip dikecualikan dari repo induk. Sumber XY yang dapat dijalankan belum ditemukan.

`pareto_xy_candidates.csv` tetap disimpan karena kandidatnya cocok dengan nilai konfigurasi final di Lampiran A. Namun file hasil itu tidak membuktikan skrip mana yang menghasilkannya. Untuk reproduksi persis, pulihkan skrip XY asli dari backup atau repo pribadi.

## Hasil yang dipertahankan

- `pareto_xy_candidates.csv` dan `pareto_xy_baseline.csv`
- `pareto_candidates_theta.csv` dan `pareto_baseline_locked.csv`
- `metrics_single_drop_matlab.csv` dan `metrics_single_drop_6dof.csv`
- figure di `figures_paper/`

Program MPC/LESO lama, panduan HIL yang tidak terkait langsung dengan skripsi, log pengembangan, cache, dan file salinan dipindahkan ke `archive/legacy-programs/`. Arsip tersebut dikecualikan dari repo induk.

