# Ringkasan Skripsi

## Identitas dan tujuan

Tugas akhir ini mengembangkan sistem kendali dan estimasi massa untuk quadrotor yang menjalankan misi pengambilan serta pelepasan muatan. Implementasi tertanam ditujukan untuk PX4 v1.17 pada Pixhawk 2.4.8 (FMUv3, STM32F427), tanpa komputer pendamping.

## Arsitektur utama

Modul `mc_l1_mo` memadukan tiga mekanisme:

- **L1 Adaptive Control** untuk mengompensasi ketidakpastian dinamika dan gangguan.
- **Momentum Observer** untuk memperkirakan perubahan massa akibat pengambilan atau pelepasan muatan, lalu menghubungkan estimasi itu ke prediksi keadaan dan kompensasi gaya.
- **Event-Triggered Control** untuk menahan pembaruan setpoint aktuator saat perubahan kecil dan menerbitkannya lagi ketika ambang perubahan terlampaui.

Loop kendali berjalan sekitar 100 Hz. ETC menekan laju pembaruan setpoint; perhitungan loop kendali tetap berjalan setiap siklus.

## Jalur validasi

1. Prototipe algoritma dan penalaan parameter di MATLAB, termasuk model 6-DOF dan optimasi multiobjektif NSGA-II.
2. Validasi Software-in-the-Loop final melalui PX4, ROS 2, dan Gazebo Harmonic; metrik kampanye terpilih ada di [data/final-sitl](../data/final-sitl/).
3. Validasi embeddability melalui Simulation-in-Hardware pada Pixhawk target.

## Hasil yang dilaporkan

Skripsi melaporkan penurunan puncak galat ketinggian sekitar 38–41% dan pemulihan 2,2–2,6 kali lebih cepat daripada baseline PID pada transien perubahan massa. Estimasi perubahan massa dilaporkan memiliki galat sekitar 2–3%. Pada target FMUv3, pengukuran minimum komputasi kontrol sekitar 23 μs dan seluruh fungsi `Run()` sekitar 45 μs per siklus.

Angka performa tersebut berasal dari kondisi eksperimen yang dijelaskan di skripsi; bedakan hasil SIL dari hasil pengukuran on-target SIH ketika mengutip atau membandingkannya.

## Kode dan hasil MATLAB

Kode MATLAB disimpan di [program/](../program/). Lampiran C skripsi menyebut `l1_mo_pareto_optimization_xy.m` untuk optimasi NSGA-II dan `l1_pid_law_6dof_coupled.m` untuk model 6-DOF. Lihat [catatan MATLAB](../program/README.md) untuk kondisi file optimizer XY yang ada di workspace sekarang.

