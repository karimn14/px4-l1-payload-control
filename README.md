# Sistem Kendali Drone Pengangkut Muatan

Workspace penelitian tugas akhir tentang kendali quadrotor untuk misi pengambilan dan pelepasan muatan. Fokus penelitian adalah modul firmware PX4 `mc_l1_mo`, yang menggabungkan L1 Adaptive Control, Momentum Observer untuk estimasi massa, dan Event-Triggered Control.

## Mulai dari sini

1. Baca [ringkasan skripsi](docs/RINGKASAN_SKRIPSI.md) untuk memahami tujuan, arsitektur, dan hasil penelitian.
2. Baca [peta repo dan data](docs/PETA_REPO.md) sebelum mengubah clone atau folder eksperimen.
3. Buka [firmware PX4 utama](firmware/PX4-main/), yang memuat modul `src/modules/mc_l1_mo/`.
4. Hasil simulasi berada di [data/sitl_campaigns](data/sitl_campaigns/).
5. Kode dan hasil MATLAB ada di [program/](program/); skrip optimasi serta status file XY dijelaskan di [program/README.md](program/README.md).
   Lampiran menyebut file optimizer XY, tetapi salinan lokalnya kosong dan sudah dikeluarkan dari folder aktif.

## Buka di VS Code

Buka [drone-payload-control.code-workspace](drone-payload-control.code-workspace) untuk melihat komponen penting sebagai satu workspace tanpa menggabungkan riwayat Git masing-masing.

## Struktur workspace

```text
.
├── firmware/
│   ├── PX4-main/                 # Firmware penelitian mc_l1_mo
│   └── references/               # Clone PX4 pembanding/versi lain
├── ros2/
│   ├── drone_ws/                 # Repo ROS 2 MPC/LESO dengan perubahan lokal
│   └── ros2_ws/                  # Workspace ROS 2 dan hasil build
├── middleware/Micro-XRCE-DDS-Agent/
├── simulation/                   # Model dan simulator Gazebo PX4
├── experiments/                  # Proyek dan panduan eksperimen MPC/LESO
├── data/sitl_campaigns/          # Hasil run simulasi
├── docs/                         # Ringkasan dan dokumentasi workspace
└── archive/                      # Snapshot lama dan sisa folder yang dipertahankan
```

## Cara kerja Git

Folder ini adalah repo induk lokal untuk dokumentasi dan kode MATLAB skripsi. `origin` sudah diarahkan ke repo pribadi `karimn14/px4-l1-payload-control`, tetapi GitHub saat ini menjawab Repository not found. Buat repo pribadi kosong dengan nama tersebut atau perbaiki URL sebelum push. Clone kode seperti `firmware/PX4-main/` tetap menjadi repo terpisah dengan remote masing-masing.

Jalankan perintah Git dari repo komponennya. Contoh:

```powershell
git -C firmware/PX4-main status --short --branch
git -C ros2/drone_ws status --short --branch
git -C firmware/PX4-main remote -v
```

Beberapa clone masih menunjuk ke repo upstream. Jangan push dari clone itu bila tidak ingin mempublikasikan perubahan. Jika ingin backup kode di GitHub, gunakan fork pribadi sebagai `origin` dan simpan repo resmi sebagai `upstream`; pastikan URL fork milik Anda sebelum mengganti remote.

## Catatan keselamatan workspace

`ros2/drone_ws` memiliki banyak perubahan lokal, termasuk perubahan di `build/`, `install/`, dan `log/`. Semua itu dipertahankan. Jangan menjalankan `git clean`, menghapus hasil build, atau mereset repo ini sebelum meninjau `git status` dan memilih perubahan yang ingin disimpan.








