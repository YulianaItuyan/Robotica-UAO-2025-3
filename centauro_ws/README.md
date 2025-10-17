
## Estructura
```
centauro_ws/
  src/        # paquetes ROS2
  	robot/
  		robot/
  			fk_node
  			ik_node
  			joint_commander_deg
  			serial_node
  			pinterfaz
  build/      # (autogenerado por colcon)
  install/    # (autogenerado por colcon)
  log/        # (autogenerado por colcon)
```

## Cómo compilar
```bash
cd ~/"Ruta hasta donde tengan el repositorio"/centauro_ws
rm -rf build/ install/ log/#(Esto se hace para borrar lo relacionado al anterior workspace y evitar problemas)
colcon build --packages-select robot --symlink-install

## Despues de compilar
Recuerden en la terminal:
sudo nano ~/.bashrc
Dentro van al final y escriben

source ~/"Ruta hasta donde tengan el repositorio"/centauro_ws/install/setup.bash



