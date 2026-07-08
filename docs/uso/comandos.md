# Comandos rapidos

## Build y limpieza

```bash
./shs/build.sh
./shs/build.sh --clean
./shs/kill_all.sh
```

## Simulacion

```bash
./shs/casa.sh
./shs/casa.sh obs
./shs/slam_casa.sh
./shs/nav.sh
./shs/teleop.sh
```

## TurtleBot4 real

```bash
export ROS_DOMAIN_ID=<dominio_del_TB4>
./shs/mapear_tb4.sh --ns tb4_0
./shs/navegar_tb4.sh --ns tb4_0 --map maps/laberinto_lab_20260702.yaml
./shs/parte_c_tb4.sh --ns tb4_0 --map maps/laberinto_lab_20260702.yaml
```

## Evidencia

```bash
./scripts/lab_record_all.sh tb4_0
./scripts/lab_replay_rviz.sh results/labo_demo/<timestamp> tb4_0
./scripts/lab_record_rviz.sh results/labo_demo/<timestamp>
```

## Topics utiles

```bash
ros2 topic echo /nav_state
ros2 topic echo /mission_state
ros2 topic echo /amcl_pose
ros2 topic list | grep /tb4_0
```
