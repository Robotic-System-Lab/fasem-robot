# Autonomous Exploration Visualization Guide

## Overview
Paket autonomous exploration ini telah ditingkatkan dengan fitur visualisasi RViz2 untuk membantu debug dan evaluasi robot behavior. Visualisasi menampilkan:

1. **Target Goal** (Sphere Merah) - Posisi target tujuan robot
2. **Path Planning** (Line Strip Biru) - Jalur yang direncanakan robot
3. **Frontier Points** (Points Hijau) - Area unexplored yang dideteksi
4. **Frontier Centroids** (Cylinder Kuning) - Pusat dari grup frontier

## Features yang Ditambahkan

### 1. Visualization Markers
- **Goal Marker**: Sphere merah yang menunjukkan target tujuan saat ini
- **Path Marker**: Garis biru yang menampilkan jalur yang direncanakan
- **Frontier Markers**: Points hijau yang menunjukkan frontier cells
- **Centroid Markers**: Cylinder kuning yang menunjukkan centroid dari frontier groups

### 2. Topics untuk Visualisasi
- `/goal_marker` - Target goal visualization
- `/path_marker` - Path planning visualization  
- `/frontier_markers` - Frontier points visualization
- `/exploration_markers` - Frontier centroids visualization

### 3. Launch File dengan RViz2
Launch file yang automatically menjalankan exploration node dan RViz2 dengan konfigurasi yang sudah dioptimasi.

## Cara Menggunakan

### 1. Build Package
```bash
cd /mnt/subsystem/Code/ResearchDev/BRIN_MBKM_Smt_6/semantic_mapping/fasem-robot
colcon build --packages-select autonomous_exploration
source install/setup.bash
```

### 2. Launch dengan Visualisasi
```bash
ros2 launch autonomous_exploration exploration_with_visualization.launch.py
```

### 3. Launch Parameters yang Tersedia
- `rviz_config`: Path ke konfigurasi RViz (default: package config)
- `use_sim_time`: Gunakan simulation time (default: true)

### 4. Monitoring Robot Behavior
Dalam RViz2, Anda dapat melihat:

1. **Robot Position**: Ditampilkan dengan odometry arrow (merah)
2. **Current Goal**: Sphere merah menunjukkan kemana robot akan pergi
3. **Planned Path**: Garis biru dari posisi robot ke goal
4. **Available Frontiers**: Points hijau menunjukkan area yang bisa dijelajahi
5. **Frontier Groups**: Cylinder kuning menunjukkan pusat grup frontier

### 5. Debug Collision Issues
Untuk menganalisis kenapa robot nabrak:

1. **Perhatikan Goal Position**: Apakah target goal terlalu dekat dengan obstacle?
2. **Check Path Planning**: Apakah jalur biru melewati obstacle?
3. **Monitor Frontier Selection**: Apakah robot memilih frontier yang appropriate?
4. **Laser Scan vs Map**: Bandingkan laser scan (putih) dengan map untuk melihat discrepancy

## Konfigurasi RViz2

File `exploration_visualization.rviz` sudah dikonfigurasi dengan:
- Grid dan map display
- Robot odometry dengan arrow
- Laser scan visualization
- Semua exploration markers
- Top-down orthographic view yang optimal untuk monitoring

## Troubleshooting

### Jika Markers Tidak Muncul:
1. Check topic list: `ros2 topic list | grep marker`
2. Check topic data: `ros2 topic echo /goal_marker`
3. Pastikan autonomous exploration node running
4. Restart RViz2 jika perlu

### Jika RViz2 Error:
1. Check konfigurasi file path
2. Update frame IDs jika berbeda (default: "map")
3. Adjust topic names jika remapping diperlukan

### Fixes yang Telah Diterapkan:
1. **RViz Config Fixed**: Menggunakan `rviz_default_plugins/` class names yang benar untuk ROS2 Humble
2. **Marker Update Consistency**: Menambahkan periodic visualization updates setiap 1 detik selama path following
3. **Marker Persistence**: Goal dan path markers persistent (lifetime = 0), frontier markers 60 detik
4. **Error Handling**: Improved error handling untuk visualisasi yang robust

## Customization

### Mengubah Marker Properties:
Edit fungsi di `control.py`:
- `create_goal_marker()` - Goal appearance
- `create_path_marker()` - Path visualization  
- `create_frontier_markers()` - Frontier points
- `create_centroid_markers()` - Centroid visualization

### Mengubah Colors:
Colors didefinisikan dalam fungsi marker creation:
- Goal: Merah (1.0, 0.0, 0.0)
- Path: Biru (0.0, 0.0, 1.0)  
- Frontiers: Hijau dengan variasi
- Centroids: Kuning (1.0, 1.0, 0.0)

### Performance Notes:
- Markers automatically cleared saat exploration selesai
- Frontier markers memiliki lifetime 30 detik
- Visualization hanya published saat ada update

## Emergency Controls
Robot masih support emergency stop via joystick:
- Button[2]: Emergency stop
- Button[1]: Resume exploration

Dengan visualisasi ini, Anda dapat lebih mudah mengidentifikasi masalah navigasi dan memperbaiki algorithm exploration.