# Quick Reference: Dynamic cmd_vel Topic Configuration

## Modifikasi Utama

File `nav_frontier.launch.py` telah dimodifikasi untuk mendukung konfigurasi dinamis topik `cmd_vel`.

### Perubahan Kode

1. **Import tambahan**:
```python
from launch.actions import GroupAction, SetRemap
```

2. **Parameter baru**:
```python
cmd_vel_topic_arg = DeclareLaunchArgument(
    'cmd_vel_topic',
    default_value='cmd_vel',
    description='Topic name for velocity commands output'
)
```

3. **Remapping dengan GroupAction**:
```python
GroupAction([
    SetRemap(src='cmd_vel', dst=LaunchConfiguration('cmd_vel_topic')),
    IncludeLaunchDescription(...)
])
```

## Cara Penggunaan

### Command Line

```bash
# Default (cmd_vel)
ros2 launch frontier_exp_cpp nav_frontier.launch.py

# Custom topic
ros2 launch frontier_exp_cpp nav_frontier.launch.py cmd_vel_topic:=/fasem/cmd_vel

# Dengan simulasi
ros2 launch frontier_exp_cpp nav_frontier.launch.py use_sim_time:=true cmd_vel_topic:=/robot/cmd_vel
```

### Dalam Launch File Lain

```python
IncludeLaunchDescription(
    PythonLaunchDescriptionSource(
        os.path.join(get_package_share_directory('frontier_exp_cpp'), 
                     'launch', 'nav_frontier.launch.py')
    ),
    launch_arguments={
        'use_sim_time': 'true',
        'cmd_vel_topic': '/fasem/cmd_vel'
    }.items()
)
```

## Topik yang Terpengaruh

Remapping `cmd_vel` akan mempengaruhi output dari:

1. **Controller Server** (`/controller_server`)
   - Default: `/cmd_vel`
   - Setelah remap: `/{cmd_vel_topic}`

2. **Behavior Server** (`/behavior_server`)
   - Default: `/cmd_vel`
   - Setelah remap: `/{cmd_vel_topic}`

3. **Velocity Smoother** (`/velocity_smoother`)
   - Default: `/cmd_vel`
   - Setelah remap: `/{cmd_vel_topic}`

## Verifikasi

```bash
# Lihat semua topik aktif
ros2 topic list | grep cmd_vel

# Monitor topik custom
ros2 topic echo /fasem/cmd_vel

# Check info topik
ros2 topic info /fasem/cmd_vel

# Lihat siapa yang publish
ros2 topic info /fasem/cmd_vel --verbose
```

## Troubleshooting

### Robot tidak bergerak?
1. Cek topik yang di-subscribe oleh robot controller:
   ```bash
   ros2 node info /your_robot_controller
   ```

2. Pastikan topik sama dengan yang di-set:
   ```bash
   ros2 topic list | grep cmd_vel
   ```

### Nav2 masih publish ke /cmd_vel?
- Pastikan parameter dipass dengan benar:
  ```bash
  ros2 param list /controller_server
  ros2 param get /controller_server use_sim_time
  ```

### Ingin kembali ke default?
```bash
ros2 launch frontier_exp_cpp nav_frontier.launch.py cmd_vel_topic:=cmd_vel
```

## Contoh Use Case

### FASEM Robot
```bash
ros2 launch frontier_exp_cpp nav_frontier.launch.py \
    use_sim_time:=true \
    cmd_vel_topic:=/fasem/cmd_vel
```

### Jackal Robot
```bash
ros2 launch frontier_exp_cpp nav_frontier.launch.py \
    use_sim_time:=false \
    cmd_vel_topic:=/jackal/cmd_vel
```

### TurtleBot3
```bash
ros2 launch frontier_exp_cpp nav_frontier.launch.py \
    use_sim_time:=true \
    cmd_vel_topic:=/cmd_vel
```

### Custom Namespace
```bash
ros2 launch frontier_exp_cpp nav_frontier.launch.py \
    cmd_vel_topic:=/my_namespace/my_robot/cmd_vel
```

## File yang Dimodifikasi

- ✅ `frontier_exp_cpp/launch/nav_frontier.launch.py` - Main launch file dengan dynamic remapping
- ✅ `frontier_exp_cpp/launch/LAUNCH_USAGE.md` - Dokumentasi lengkap
- ✅ `frontier_exp_cpp/launch/fasem_frontier_example.launch.py` - Contoh integrasi
- ✅ `use.md` - Updated dengan informasi baru

## Notes Penting

- Remapping tidak mempengaruhi topik input seperti `/map`, `/scan`, `/odom`
- Semua Nav2 node dalam navigation stack akan menggunakan topik yang sama
- Parameter ini backward compatible - default tetap `/cmd_vel`
- Dapat dikombinasikan dengan parameter lain seperti `use_sim_time`
