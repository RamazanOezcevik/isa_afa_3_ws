# ISA-AFA-3 Workspace

## Thema: Entwicklung einer Simulationsumgebung für autonomes Fahren mit ROS2
In dem Intelligente Sensoren und Autonomes Fahren Projekt wurde eine Simulationsumgebung für Autonomes Fahren entwickelt. 
Es kommt ein eigenes Fahrzeug sowie eigen erstellte World zum Einsatz. Das Fahrzeug ist mit wichtiger Sensorik wie IMU, Kamera, Sonar, Lidar ausgestattet. 
Somit können in Zukunft viele Funktionen entwickelt und getestet werden. 

Es kann die Karte mit der slam_toolbox aufgezeichnet werden. Das Fahrzeug fährt durch Punktsetzung autonom zum festgelegten Ziel.

### Installationsanleitung:
**Erforderlich:**
+ WSL2 installieren
+ Docker engine in WSL2 installieren
+ (Optional) Falls eine NVIDIA Grafikkarte vorhanden: Nvidia Treiber in WSL2 installieren und Nvidia Container-Toolbox in WSL2 installieren

**Aufsetzen des Dockers und automatische/vollständige Installation der Simulationsumgebung:**
+ Quellcode von GitHub aufrufen oder Abgegbenen Quellcode in Ubuntu (von der WSL2) kopieren

im workspace Verzeichnis den Installationsbefehl aufrufen: 
+ keine NVIDIA-GPU: docker compose -f docker/docker-compose.yaml up -d
+ mit NVIDIA-GPU: docker compose -f docker/docker-compose.yaml -f docker/docker-compose.gpu.yaml up -d

Im Anschluss wird die Simulationsumgebung installiert.

### Starten der Simulationsumgebung
Für das Starten der Simulationsumgebung muss erst die Verbindung mit dem Docker-Container hergestellt werden. 
Anschließend im Terminal: 
+ colcon build (Auch durchführen, wenn Änderungen vorgenommen wurden wie z.B. neue Dateien)
+ Starten der Simulationsumgebung: ros2 launch isa_afa_3 ackermann_sim.launch.py
+ Befehl, um das Fahrzeug zu steuern: ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -p stamped:=true -r cmd_vel:=/ackermann_steering_controller/reference
