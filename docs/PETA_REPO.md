# Peta Repo dan Data

Peta ini menjelaskan komponen setelah folder dikelompokkan. Repo bertingkat tetap independen; repo induk mengelola dokumentasi, MATLAB pendukung, dan metrik SITL final.

## Repo dan workspace

| Lokasi | Jenis | Fungsi / catatan |
|---|---|---|
| `firmware/PX4-main/` | Repo Git | Clone utama yang memuat `src/modules/mc_l1_mo/`; HEAD lokal `c503d366dd` (6 Juli 2026). Remote `origin` mengarah ke `PX4/PX4-Autopilot`, bukan repo pribadi. |
| `firmware/references/PX4-Autopilot/` | Repo Git | Clone PX4 lain di checkout detached; berisi perubahan berkaitan dengan MPC/LESO. Bukan clone yang memuat modul skripsi `mc_l1_mo`. |
| `firmware/references/PX4-v14/` | Repo Git | Clone PX4 versi lebih lama di checkout detached, disimpan sebagai referensi versi. |
| `archive/partial-remainders/PX4-v14/` | Arsip | Sisa folder yang ditemukan saat pengelompokan; dipertahankan terpisah agar tidak menimpa clone PX4-v14 lengkap. Tinjau manual bila diperlukan. |
| `ros2/drone_ws/` | Repo Git | Clone pribadi `karimn14/drone_ws` untuk proyek MPC/LESO; bukan source ROS 2 final penelitian L1+MO. Final SITL metrics dari ROS 2/Gazebo tersedia di `data/final-sitl/`. |
| `ros2/ros2_ws/` | Workspace ROS 2 | Berisi paket contoh (`my_cpp_pkg`, `my_first_pkg`, `my_python_pkg`), bukan node offboard final penelitian. Build/install/log adalah keluaran lokal. |
| `middleware/Micro-XRCE-DDS-Agent/` | Repo Git | Clone upstream eProsima Micro-XRCE-DDS-Agent v2.4.2. |
| `simulation/PX4-gazebo-models/` | Repo Git | Clone model PX4 Gazebo. |
| `simulation/PX4-SITL_gazebo-classic/` | Repo Git | Clone simulator PX4 Gazebo Classic. |
| `experiments/px4_mpc_leso_project/` | Folder proyek | Skrip, model, world, log, dan panduan MPC/LESO; bukan repo Git tersendiri. |
| `experiments/px4_hil_mpc/` | Kumpulan panduan | Dokumen firmware/HIL MPC-LESO; bukan repo Git tersendiri. |
| `data/sitl_campaigns/` | Data eksperimen | Folder hasil run; CSV telemetry dipertahankan. Log konsol PX4/Gazebo yang besar sudah dihapus karena bisa dibuat ulang saat simulasi. |
| `data/final-sitl/` | Hasil final untuk repo | CSV metrik agregat/per-run dan plot terpilih dari kampanye PX4 SITL dengan ROS 2 dan Gazebo Harmonic. Raw time-series lengkap tidak disalin ke repo induk. |
| `program/` | MATLAB dan hasil | Berisi model MATLAB L1+MO, optimizer, metrik, kandidat Pareto, dan figure skripsi. Catatan skrip ada di [program/README.md](../program/README.md). Eksperimen MPC/LESO lama berada di `archive/legacy-programs/` dan dikecualikan dari repo induk. |
| `docs/reference/2026 TA TF NABIL KARIM ABDURRAHMAN FULL TEXT.docx` | Naskah sumber | Dokumen skripsi yang menjadi acuan ringkasan proyek. Tidak dimasukkan ke riwayat repo induk. |
| `archive/snapshots/` | Arsip | File `.bak` yang ditemukan di root; dipindahkan utuh dan belum dibandingkan dengan sumber aktif. |

## Batas antar repo

- Clone-clone di bawah workspace tidak berbagi riwayat Git. Gunakan root repo yang tepat, misalnya `git -C firmware/PX4-main status --short --branch`.
- `origin` repo induk diarahkan ke `karimn14/px4-l1-payload-control`, tetapi GitHub menjawab Repository not found. Buat repo pribadi kosong atau koreksi URL sebelum push. Repo komponen tetap memiliki remote masing-masing.
- Beberapa remote masih menunjuk ke upstream. Untuk backup kode di repo pribadi, gunakan fork pribadi sebagai `origin` dan repo resmi sebagai `upstream`. Pastikan URL fork sebelum mengubah remote.
- Jangan memasukkan `build/`, `install/`, `log/`, firmware binary, atau hasil kampanye besar ke repo kode kecuali dipilih sebagai artefak tertentu.

## Pemeriksaan sebelum operasi Git

```powershell
git -C firmware/PX4-main status --short --branch
git -C ros2/drone_ws status --short --branch
git -C firmware/PX4-main remote -v
```

Jika status menunjukkan perubahan lokal, tinjau sebelum `checkout`, `reset`, `clean`, atau menghapus folder. Pada `ros2/drone_ws`, perubahan build tercampur dengan perubahan sumber sehingga perlu ditangani selektif.








