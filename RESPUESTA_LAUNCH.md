# Respuesta: Archivos Launch a Ejecutar

Para correr el sistema completo con RTAB-Map y segmentación, ejecuta estos comandos en orden:

### 1. Driver del Robot (QCar)
Asegúrate de que el robot esté publicando las imágenes y el IMU.
- Tópicos necesarios: `/camera/color_image`, `/camera/depth_image`, `/qcar2_imu`

### 2. Segmentación y Máscara
Inicia el procesamiento de la segmentación y el enmascarado de profundidad.
```bash
ros2 launch lane_mapping_acc road_segmentation.launch.py
```

### 3. Mapeo con RTAB-Map
Inicia la odometría visual y el mapeo 3D de la carretera.
```bash
ros2 launch lane_mapping_acc rtabmap_mapping.launch.py
```
