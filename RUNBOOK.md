# RUNBOOK — Sim host + oráculo de validación

Manual operacional del lado **PC del simulador** (este host). La Jetson con
el brain se asume operativa; aquí solo cubrimos lo que corre del lado del
sim para que la PC arranque, el oráculo quede vivo y el agente LLM remoto
pueda consumir telemetría sin que nada frene el flujo.

> Para el contrato técnico de topics y la regla "the sim is the body, not
> the brain", ver [CLAUDE.md](CLAUDE.md). Para el detalle de generación de
> USDs y dependencias, ver [README.md](README.md). Para checklists granulares
> de cada launch, ver [VALIDATION.md](VALIDATION.md). Este RUNBOOK es la
> síntesis operacional — qué teclear, en qué terminal, en qué orden.

---

## 1. Inventario de red y credenciales

| Equipo | Rol | WiFi | USB ethernet | Acceso | Notas |
|---|---|---|---|---|---|
| **PC sim** (este host) | Isaac Sim + overlay + drive shaping | `192.168.15.79` (`wlp0s20f3`) | `192.168.55.100` (`enxcee573205a54`) | local | host del oráculo y FastAPI :8090 |
| **Jetson Orin** | Brain (EKF, Nav2, cuVSLAM, AprilTag) | `192.168.15.241` | `192.168.55.1` | SSH `orza / 1001`, VNC `:5900 / 1001` | corre `agv_hil_full.launch.py` |

Atajos SSH a la Jetson:

```bash
# Por USB (más estable, no depende del WiFi)
sshpass -p '1001' ssh -o StrictHostKeyChecking=no \
  -o PreferredAuthentications=password -o PubkeyAuthentication=no \
  orza@192.168.55.1

# Por WiFi
ssh orza@192.168.15.241          # contraseña: 1001

# Si hay alias 'jetson-orin' en ~/.ssh/config:
ssh jetson-orin
```

Puertos abiertos en la PC sim (ambos consumibles desde la Jetson o desde
otra máquina LAN):

| Puerto | Servicio | Para qué |
|---|---|---|
| `8090/tcp` | FastAPI `sim_api` | API REST del oráculo (LLM, scripts) |
| `8765/tcp` | `foxglove_bridge` | WebSocket para Foxglove Studio |

DDS:

- **Cyclone** (`rmw_cyclonedds_cpp`)
- **Domain ID 42** (forzado por `isaac_hil.launch.py`)
- Multicast OFF, peers unicast en [cyclonedds.xml](cyclonedds.xml):
  `localhost` + `192.168.15.241` (Jetson)

> Si la IP del Jetson cambia, **editar `<Peer address="...">` en
> `cyclonedds.xml`** y reiniciar terminales — DDS lee el archivo al boot.

---

## 2. Setup de una sola vez (saltar si ya está)

```bash
# 1) Dependencias del sistema
sudo apt install -y \
  ros-humble-robot-state-publisher ros-humble-xacro \
  ros-humble-teleop-twist-keyboard ros-humble-tf2-ros \
  ros-humble-pointcloud-to-laserscan ros-humble-topic-tools \
  ros-humble-foxglove-bridge ros-humble-rmw-cyclonedds-cpp

# 2) Build del workspace
cd ~/agv-sim
colcon build --symlink-install

# 3) Generar USDs (~5 min cada uno, requiere Isaac Sim 4.x ya instalado)
isaacsim --exec src/agv_isaac_sim/scripts/build_greenhouse_usd.py
isaacsim --exec src/agv_isaac_sim/scripts/import_robot_usd.py

# 4) Pip deps del overlay (no están en rosdep)
pip install fastapi uvicorn opencv-python pyyaml
```

**Re-generar USDs** únicamente cuando cambien
`world_config.yaml`, la URDF, o `import_robot_usd.py`. La nota en
[CLAUDE.md](CLAUDE.md) indica que tras el refactor 2026-04-15 el USD está
stale — vale la pena regenerarlo antes del primer arranque del día si no
se hizo desde el último cambio.

### Variables de entorno

Confirmar que `~/.bashrc` contiene (ya está según el contexto actual):

```bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI=file://$HOME/agv-sim/cyclonedds.xml
```

> Importante: `~/.bashrc` aquí define `ROS_DOMAIN_ID=0`. **El launch HIL
> fuerza Domain 42 internamente**, así que las terminales 1 y 2 funcionan.
> Pero **cualquier terminal de verificación que use `ros2 topic`** debe
> exportar `ROS_DOMAIN_ID=42` a mano antes, o no verá el tráfico del sim.

---

## 3. Encendido diario — secuencia exacta

### Pre-flight (en cada terminal nueva)

```bash
cd ~/agv-sim
source /opt/ros/humble/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID=42      # crítico — el bashrc tiene 0
```

(`RMW_IMPLEMENTATION` y `CYCLONEDDS_URI` ya vienen del bashrc.)

### Terminal 1 — Isaac Sim + handler in-sim (modo supervisado)

```bash
./run_isaac_supervised.sh
# Isaac arranca, carga el USD y presiona Play automáticamente.
# Si Isaac muere o recibe POST /sim/restart, el supervisor lo relanza.
# Para modo manual sin auto-reinicio: ./run_isaac_sim.sh
```

> Auto-play se activa por default. Para desactivar y requerir Play manual:
> `AGV_AUTO_PLAY=0 ./run_isaac_supervised.sh`

Esto:
- Lanza `isaacsim` bajo un supervisor que lo relanza si muere
- Carga `worlds/greenhouse_with_robot.usd`
- **Auto-play**: presiona Play automáticamente ~1 s después de cargar
- Boota dentro de Isaac el `sim_isaac_handler` (rclpy en hilo daemon)
  que hace:
  - subscribe a `/agv/sim/reset_request` → teleport físico
  - subscribe a `/agv/sim/control` → play/stop/pause timeline remoto
  - subscribe a contact reports de PhysX → publica `collision` events
  - publica `/agv/sim/reset_done` (latched)

Cosas a buscar en la consola de Isaac:

```
[AGV] Opened /home/andres/agv-sim/src/agv_isaac_sim/worlds/greenhouse_with_robot.usd
[AGV][handler] boot scheduled
[AGV][auto-play] scheduled (disable with AGV_AUTO_PLAY=0)
[AGV][handler] PhysxContactReportAPI applied to N prims    # N debería ser >0
[AGV][handler] subscribed to PhysX contact reports
[AGV][auto-play] simulation started automatically
[INFO] [sim_isaac_handler]: sim_isaac_handler ready (...)
```

### Terminal 2 — HIL launch + overlay + FastAPI

```bash
ros2 launch agv_isaac_sim isaac_hil.launch.py \
    validation:=true enable_api:=true
```

Esto levanta:

- **Drive chain:** `sim_motor_gate` + `sim_drive_shaping_node` +
  `sim_global_odom` + `pointcloud_to_laserscan`
- **Overlay de validación:** `ground_truth_publisher`,
  `localization_monitor`, `events_detector`, `episode_tracker`,
  `visible_markers`, `obstacles_publisher`
- **FastAPI** en `:8090`
- **foxglove_bridge** en `:8765`

### Terminal 3 — verificación rápida (opcional)

```bash
cd ~/agv-sim && source install/setup.bash && export ROS_DOMAIN_ID=42

ros2 topic hz /clock                                      # >0 Hz
ros2 topic hz /agv/wheel_odom                             # ~50 Hz
ros2 topic hz /agv/imu/data                               # ~200 Hz target
ros2 topic echo /agv/sim/ground_truth/obstacles --once    # 11 obstáculos
curl -s http://localhost:8090/state | jq                  # snapshot
curl -s http://localhost:8090/metrics | jq
```

Si los 5 comandos responden con datos, el sim está sano.

---

## 4. Topics y endpoints del oráculo (lo que el LLM consume)

### Topics ROS bajo `/agv/sim/*`

| Topic | Tipo | Rate | Para qué |
|---|---|---|---|
| `/agv/sim/ground_truth/pose` | `geometry_msgs/PoseStamped` | 10 Hz | pose verdadera de PhysX |
| `/agv/sim/ground_truth/twist` | `geometry_msgs/TwistStamped` | 10 Hz | velocidad numérica derivada |
| `/agv/sim/ground_truth/visible_markers` | `std_msgs/String` JSON | 5 Hz | AprilTags dentro del frustum del ZED ahora |
| `/agv/sim/ground_truth/obstacles` | `std_msgs/String` JSON | latched once | catálogo de paredes + cajas + props |
| `/agv/sim/localization_error` | `std_msgs/String` JSON | 1 Hz | `pos_err_m`, `yaw_err_rad` vs brain |
| `/agv/sim/events` | `std_msgs/String` JSON | event-driven | `collision`, `stuck`, `drift`, `marker_seen`, `goal_*` |
| `/agv/sim/episode_summary` | `std_msgs/String` JSON | latched, una por misión | `path_length`, `efficiency`, `collisions`, `bag_path` |
| `/agv/sim/reset_done` | `std_msgs/Bool` | latched | confirma teleport aplicado |

### Endpoints HTTP `:8090` (`sim_api`)

| Método | Ruta | Para qué |
|---|---|---|
| `GET` | `/` | índice (autodescriptivo) |
| `GET` | `/state` | pose + GT + último error + último evento |
| `GET` | `/metrics` | totales de sesión: distancia, colisiones, episodios |
| `GET` | `/events?since=<t_sim>` | timeline desde un t_sim |
| `GET` | `/episodes` | lista de resúmenes por episodio |
| `GET` | `/episodes/{id}/bag` | ruta server-local del rosbag (rsync hint) |
| `GET` | `/snapshot.jpg` | última frame del ZED izquierdo, JPEG |
| `GET` | `/viz_url` | hint de WebSocket de Foxglove |
| `POST` | `/goal {x,y,yaw}` | Nav2 NavigateToPose |
| `POST` | `/reset {x,y,yaw,z}` | teleport físico real (handler in-sim) |
| `POST` | `/motor/enable {on}` | armar/desarmar ODrive emulado |
| `POST` | `/e_stop {on}` | e-stop |
| `POST` | `/sim/play` | arrancar timeline de Isaac (auto-play hace esto al boot) |
| `POST` | `/sim/stop` | detener timeline (pausa física + reloj) |
| `POST` | `/sim/restart` | matar Isaac + el supervisor lo relanza con auto-play |

### Foxglove Studio remoto

Desde cualquier máquina del LAN: abrir https://studio.foxglove.dev y
conectar a `ws://192.168.15.79:8765`. Se ven todos los topics, incluyendo
los `/agv/sim/*` del oráculo.

---

## 5. Validación end-to-end (5 tests copiables)

> Sim corriendo (Terminales 1+2 listas), Terminal 3 con `ROS_DOMAIN_ID=42`.

```bash
# Test 1 — obstáculos GT (latched, sale al subscribir)
ros2 topic echo /agv/sim/ground_truth/obstacles --once
# Esperado: JSON con 11 obstáculos (4 paredes + 3 crates + 4 props)

# Test 2 — visible markers cambia con la pose
ros2 topic echo /agv/sim/ground_truth/visible_markers
# Conducir el robot con teleop o /goal; los IDs en "visible" cambian

# Test 3 — teleport físico real
curl -X POST -H 'Content-Type: application/json' \
     -d '{"x": 2.0, "y": -3.0, "yaw": 1.57}' \
     http://localhost:8090/reset
# Esperado: viewport de Isaac muestra el robot en (2, -3) mirando +Y
ros2 topic echo /agv/sim/reset_done --once
# Esperado: "data: true"

# Test 4 — collision event PhysX
ros2 topic pub --once /agv/motor_enable std_msgs/Bool 'data: true'
ros2 topic pub /agv/cmd_vel geometry_msgs/Twist '{linear: {x: 0.4}}' -r 10 &
# Esperar a que el robot choque con un crate (~5 s)
sleep 6; kill %1
ros2 topic echo /agv/sim/events --once
# Esperado: {"event":"collision","with":"Crate1",...}

# Test 5 — counter de colisiones en episode summary
# Tras una misión Nav2 que termine (succeeded/aborted)
curl -s http://localhost:8090/episodes | jq '.episodes[-1].collisions'
# Esperado: > 0 si chocó durante la misión
```

---

## 6. Control remoto desde la Jetson (LLM agent)

El agente LLM en la Jetson controla toda la simulación vía HTTP a `:8090`.
No necesita SSH, rclpy ni DDS — sólo `curl` (o `requests` en Python).

**Flujo de iteración típico del LLM:**

```bash
SIM=http://192.168.55.100:8090   # o 192.168.15.79 por WiFi

# 1. Verificar que el sim está vivo
curl -s $SIM/state | jq '.gt_pose'

# 2. Teleportar al punto de inicio
curl -s -X POST -H 'Content-Type: application/json' \
  -d '{"x":5.5, "y":0, "yaw":0}' $SIM/reset

# 3. Armar motores
curl -s -X POST -H 'Content-Type: application/json' \
  -d '{"on":true}' $SIM/motor/enable

# 4. Enviar goal Nav2
curl -s -X POST -H 'Content-Type: application/json' \
  -d '{"x":20, "y":-4.4, "yaw":0}' $SIM/goal

# 5. Polling hasta que termine (cada 2s)
while true; do
  STATE=$(curl -s $SIM/state)
  echo "$STATE" | jq '{gt: .gt_pose, err: .localization_error, ev: .last_event}'
  sleep 2
done

# 6. Leer métricas del episodio
curl -s $SIM/episodes | jq '.episodes[-1]'

# 7. Si la física se rompió — restart completo (~40s de downtime)
curl -s -X POST $SIM/sim/restart
# Esperar hasta que /state vuelva a dar gt_pose != null
```

**Controles de timeline (no necesitan restart):**

```bash
curl -X POST $SIM/sim/stop     # pausa la física y el reloj
curl -X POST $SIM/sim/play     # reanuda
```

**Restart completo** (`POST /sim/restart`): mata Isaac, el supervisor lo
relanza con auto-play. Downtime ~30-60 s. El endpoint retorna inmediatamente;
el LLM debe pollear `GET /state` hasta que `gt_pose` sea non-null.

---

## 7. Troubleshooting

| Síntoma | Causa probable | Fix |
|---|---|---|
| `ros2 topic list` no muestra nada del sim | Olvidaste `export ROS_DOMAIN_ID=42` o RMW mismatch | exportar el dominio; verificar `echo $RMW_IMPLEMENTATION` da `rmw_cyclonedds_cpp` |
| `/clock` no fluye | Isaac no en Play, o algún nodo sin `use_sim_time:=true` | presionar **Play** en el viewport; en HIL todos los nodos del launch lo activan |
| Brain no descubre topics del sim | IP del Jetson cambió y no está en `cyclonedds.xml`, o WiFi en otra subred | actualizar `<Peer address="...">`, salvar, reiniciar Terminal 2 |
| `POST /reset` responde OK pero el robot no se mueve | Handler in-sim no booteó (rclpy no disponible o ext no cargada) | revisar consola Isaac por `[AGV][handler]`; si falta el log, re-lanzar `./run_isaac_sim.sh` |
| Counter `collisions` queda en 0 siempre | `PhysxContactReportAPI` no se aplicó a los prims correctos | log de Isaac: `[AGV][handler] PhysxContactReportAPI applied to N prims`. Si N=0, el USD no contiene `/agv` o `/World/Crate*` con RigidBodyAPI — regenerar USD |
| `ros2 topic hz /agv/imu/data` da <100 Hz | RTF del sim < 1.0 (GPU saturada) | bajar resolución de cámara o cerrar otras ventanas; el target son 200 Hz |
| Cámara satura el WiFi al brain | Doble subscriber pesado al mismo topic | suscribirse al ZED **sólo** desde foxglove (no replicar al brain con `topic_tools`) |
| Foxglove Studio no conecta | puerto 8765 cerrado o `enable_foxglove:=false` | revisar firewall: `sudo ufw allow 8765/tcp`; relanzar con `enable_foxglove:=true` |

---

## 8. Apagado limpio

1. **Ctrl+C en Terminal 2** (HIL launch) → drena overlay, motor gate,
   relays, foxglove. `episode_tracker` cerrará el rosbag2 en curso si
   había uno.
2. **Cerrar Isaac Sim** (X de la ventana o Ctrl+C en Terminal 1) → mata
   el handler in-sim y la sesión Kit.
3. Si se grabaron episodios, los rosbags quedan en `/tmp/agv_runs/`.
   Mover lo que quieras conservar antes de reboot:
   ```bash
   rsync -av /tmp/agv_runs/ ~/agv_runs_archive/$(date +%F)/
   ```

---

## 9. Referencias rápidas (paths absolutos)

- Inicio: [run_isaac_sim.sh](run_isaac_sim.sh)
- USD del mundo: [src/agv_isaac_sim/worlds/greenhouse_with_robot.usd](src/agv_isaac_sim/worlds/greenhouse_with_robot.usd)
- Handler in-sim: [scripts/open_greenhouse.py](scripts/open_greenhouse.py)
- Launch HIL: [src/agv_isaac_sim/launch/isaac_hil.launch.py](src/agv_isaac_sim/launch/isaac_hil.launch.py)
- Launch overlay: [src/agv_sim_validation/launch/validation_overlay.launch.py](src/agv_sim_validation/launch/validation_overlay.launch.py)
- API FastAPI: [src/agv_sim_validation/agv_sim_validation/sim_api.py](src/agv_sim_validation/agv_sim_validation/sim_api.py)
- Config DDS: [cyclonedds.xml](cyclonedds.xml)
- Contrato de topics: [TOPIC_CONTRACT.md](TOPIC_CONTRACT.md)
