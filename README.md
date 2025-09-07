# ORB_SLAM3 | ROS 2 wrapper

Esse pacote ainda está sendo testado.

### Dependências

- [ORB_SLAM3 modificado](https://github.com/harpia-drones/ORB_SLAM3.git)

### Features implementadas até o momento:
- Slam monocular
    - Recebe imagens do tópico `/color`
    - Publica PoseStamped (posição + orientação) no tópico `/slam/pose`
    - Publica Path (trajetória acumulada) no tópico `/slam/trajectory`

### Uso

1. Clonar
```bash
cd ~/harpia_ws/src
git clone git@github.com:harpia-drones/orbslam3_ros2.git
```

2. Importar variável de ambiente para Launch File
```bash
cd orbslam3_ros2/config
echo "export SIMPLE_CAM_CONFIG_PATH='$(pwd)/simple_mono_cam.yaml'" >> ~/.bashrc
source ~/.bashrc
```

3. Compilar
```bash
cb orbslam3_ros2
```

4. Teste

- Inicar o node do slam monocular
```bash
ros2 launch orbslam3_ros2 monocular.launch.py
```

- Adicionar uma TF para a camera (caso necessário)
```
ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 map odom
```

- Visualização
```bashrc
rviz2
```
No rviz2: add > by topic > /pose

OBS.: 

1. Para funcionar deve existir um tópico chamado /color. Caso o tópico da camera tenha um namespace definido, modifique-o no Launch File. Caso o nome do tópico seja outro, modifique o remapping no Launch File

2. Por padrão, o node mono_cam considera que a camera publica dados a 30 FPS. Caso estiver usando um valor diferente, modifique o parametro `camera_fps` no Launch File