# TFG — Resolución Autónoma de un Cubo de Rubik con un Robot Colaborativo ABB GoFa

Trabajo de Fin de Grado que implementa un sistema completo de percepción, planificación y control para que un brazo colaborativo **ABB GoFa CRB15000** escanee, resuelva y manipule físicamente un **Cubo de Rubik 3x3**, de forma totalmente autónoma y sin intervención humana durante la resolución.

El sistema integra **ROS 2**, **visión por computador**, control de movimiento en tiempo real vía **EGM (Externally Guided Motion)** y programación **RAPID / ABB RobotStudio**, y puede ejecutarse tanto en un **gemelo digital simulado** (RobotStudio) como sobre el **robot físico**, reutilizando exactamente el mismo pipeline de software.

> Autor: **Antonio Balboa** ([abalpa19@gmail.com](mailto:abalpa19@gmail.com)) — UC3M, Grado en Ingeniería (RoboticsLab)
> Licencia: [MIT](LICENSE)

---

## Tabla de contenidos

- [Visión general](#visión-general)
- [Demo del flujo completo](#demo-del-flujo-completo)
- [Arquitectura del sistema](#arquitectura-del-sistema)
- [Estructura del repositorio](#estructura-del-repositorio)
- [Requisitos previos](#requisitos-previos)
- [Instalación](#instalación)
- [Puesta en marcha](#puesta-en-marcha)
- [Modo Virtual vs. Modo Real](#modo-virtual-vs-modo-real)
- [Detalle de los nodos ROS 2](#detalle-de-los-nodos-ros-2)
- [Comunicación EGM y protocolo de comandos](#comunicación-egm-y-protocolo-de-comandos)
- [Programación RAPID](#programación-rapid)
- [Add-in de Cámara Virtual (RobotStudio)](#add-in-de-cámara-virtual-robotstudio)
- [Visión por computador y calibración de color](#visión-por-computador-y-calibración-de-color)
- [Métricas y reportes técnicos](#métricas-y-reportes-técnicos)
- [Herramientas auxiliares](#herramientas-auxiliares)
- [Problemas conocidos y decisiones de diseño](#problemas-conocidos-y-decisiones-de-diseño)
- [Hoja de ruta](#hoja-de-ruta)
- [Licencia](#licencia)

---

## Visión general

El objetivo del proyecto es demostrar un pipeline **percepción → algoritmo → ejecución física** completo sobre un robot colaborativo real, usando el Cubo de Rubik como caso de estudio por su necesidad de:

- **Visión robusta** (detección de color/orientación bajo distintas condiciones de iluminación y punto de vista).
- **Planificación algorítmica** (resolución óptima con el algoritmo de Kociemba).
- **Control de movimiento preciso y de bajo nivel** en tiempo real (EGM sobre UDP, sincronización de agarres, evitar reconfiguraciones de la muñeca).
- **Arquitectura software desacoplada** que funcione igual en simulación que en campo.

El flujo de una ejecución completa es:

1. **Mezclado** — en modo virtual se genera un scramble aleatorio y se ejecuta sobre el gemelo digital; en modo real, el cubo se mezcla a mano.
2. **Escaneo** — el robot presenta cada cara del cubo a una cámara cenital y el sistema de visión reconstruye el estado de las 54 pegatinas.
3. **Resolución algorítmica** — el estado se pasa al solver de Kociemba, que calcula la secuencia óptima de movimientos (≤ 20 movimientos).
4. **Ejecución física** — la solución se traduce a comandos EGM y el robot gira las caras del cubo hasta resolverlo.
5. **Informe técnico** — se genera automáticamente un reporte con tiempos, cinemática y uso de recursos.

---

## Demo del flujo completo

```
┌────────────┐   scramble/estado   ┌──────────────┐   estado 54 chars   ┌─────────────┐
│  main.py   │ ───────────────────▶│  vision.py   │────────────────────▶│  solver.py  │
│ Orquestador│                     │  (cámara)    │                     │  (Kociemba) │
└─────┬──────┘                     └──────────────┘                     └──────┬──────┘
      │  comando_movimiento / comando_scramble (ROS 2 topics)                  │
      ▼                                                                        │
┌────────────┐        Float64MultiArray[40] (EGM)        ┌──────────────┐      │
│ robot.py   │ ───────────────────────────────────────── ▶│ abb_egm_driver│◀────┘
│(GofaControl│◀──────────────────────────────────────────  │  (driver del │  solución
│  lerNode)  │        state/data, state/joint (UDP EGM)    │  profesor)   │
└────────────┘                                             └──────┬───────┘
                                                                   │ UDP EGM
                                                          ┌────────▼────────┐
                                                          │  RobotStudio /   │
                                                          │  ABB GoFa físico │
                                                          └──────────────────┘
```

---

## Arquitectura del sistema

El proyecto se organiza como un **paquete ROS 2 (ament_python)** llamado `rubik_control`, compuesto por varios nodos independientes que se comunican por topics, más los proyectos externos de RobotStudio (RAPID, Smart Components, add-in de cámara virtual) necesarios para el gemelo digital.

| Componente | Rol |
|---|---|
| **`main.py`** (`orquestador`) | Nodo director. Genera el scramble, coordina el ciclo mezclado → visión → solver → ejecución, y guarda el historial/reporte de cada ejecución. |
| **`robot.py`** (`robot_node`) | Traduce comandos de alto nivel (letras de movimiento del cubo) al protocolo EGM de 40 elementos, gestiona el handshake con el driver y vuelca la telemetría cinemática. |
| **`vision.py`** | Escaneo y reconstrucción del estado del cubo a partir de una cámara (real o virtual), detección de color en HSV y generación del string de 54 caracteres para el solver. |
| **`solver.py`** | Resolución óptima del cubo con el algoritmo de **Kociemba** (biblioteca `kociemba`). |
| **`metricas.py`** (`metricas`) | Nodo observador **pasivo**: mide latencia del handshake EGM, cinemática por eje (velocidad/aceleración/jerk), distancia recorrida por el TCP y salud (frecuencia) de los topics, sin interferir en el pipeline principal. |
| **`calibrador_colores.py`** | Herramienta interactiva con trackbars de OpenCV para calibrar en vivo los rangos HSV de cada color del cubo. |
| **`generar_dashboard.py`** | Genera gráficas (matplotlib) a partir del último historial de ejecución. |
| **Driver EGM (`abb_egm_driver`)** | Paquete ROS 2 **externo**, desarrollado por el tutor del TFG, que hace de puente UDP↔ROS 2 entre `robot.py` y el controlador ABB (virtual u OmniCore físico). No forma parte de este repositorio, pero es una dependencia de ejecución obligatoria. |
| **RAPID (`rapid/`)** | Programa del controlador del robot: rutinas de movimiento, giro de cada cara del cubo, escaneo y máquina de estados EGM. |
| **Add-in de Cámara Virtual (`VirtualCameraAddin/`)** | Extensión de RobotStudio en C# que simula la webcam física dentro de la simulación, generando snapshots que `vision.py` puede leer igual que si viniesen de una cámara real. |
| **Modelos 3D (`3d/`, `rslib/`)** | Piezas de utillaje (base, columna, soporte del cubo) y librerías de RobotStudio (`.rslib`) del cubo inteligente, la mesa y el soporte. |

---

## Estructura del repositorio

```
TFG/
├── rubik_control/              # Paquete ROS 2 (ament_python) — lógica principal
│   ├── main.py                 # Nodo orquestador (entry point: orquestador)
│   ├── robot.py                # Nodo controlador EGM del GoFa (entry point: robot_node)
│   ├── vision.py                # Escaneo y detección de color del cubo
│   ├── solver.py                # Resolución con el algoritmo de Kociemba
│   ├── metricas.py              # Nodo observador de métricas (entry point: metricas)
│   ├── calibrador_colores.py    # Utilidad de calibración HSV interactiva
│   └── generar_dashboard.py     # Generación de gráficas a partir del historial
│
├── launch/
│   └── rubik.launch.py          # Lanzador único: driver EGM + robot_node + orquestador (+ métricas)
│
├── rapid/                       # Módulos RAPID del controlador ABB
│   ├── Module1.modx
│   ├── Module_Movimientos.modx  # Movimientos cartesianos / articulares del brazo
│   └── Module_Rubik.modx        # Rutinas de giro de cada cara del cubo (U/D/F/B/L/R)
│
├── VirtualCameraAddin/          # Add-in de RobotStudio (C#) — cámara virtual del gemelo digital
│   └── VirtualCameraAddin/
│       ├── Main.cs
│       ├── VirtualCameraButton.cs
│       ├── VirtualCameraControl.cs
│       └── VirtualCameraAddin.rsaddin
│
├── rslib/                       # Librerías/objetos de RobotStudio (.rslib)
├── 3d/                          # Piezas 3D del utillaje (.step)
├── test/                        # Tests de estilo/formato del paquete ROS 2 (flake8, pep257, copyright)
├── resource/                    # Marcador de paquete ament (rubik_control)
├── package.xml / setup.py / setup.cfg   # Definición del paquete ROS 2
└── LICENSE                      # MIT
```

---

## Requisitos previos

### Software

- **Ubuntu 24.04** (nativo o vía **WSL2** sobre Windows — este proyecto se ha desarrollado y probado sobre WSL2).
- **ROS 2 Jazzy Jalisco**.
- **Python 3.12+** con:
  - `opencv-python`
  - `numpy`
  - `kociemba`
  - `psutil`
  - `GPUtil`
  - `pandas`, `matplotlib` (para `generar_dashboard.py`)
- **xterm** (para que `rubik.launch.py` abra una ventana por nodo):
  ```bash
  sudo apt install -y xterm
  ```
- **Driver EGM del tutor** (`abb_egm_driver`): paquete ROS 2 que implementa el puente UDP con el controlador ABB. Debe estar instalado en el mismo workspace, ya que `rubik.launch.py` lo referencia directamente (`get_package_share_directory('abb_egm_driver')`, fichero de parámetros `crb-15000-5.yaml`).

### Software ABB (para el gemelo digital / robot real)

- **ABB RobotStudio** (con el controlador virtual del **GoFa CRB15000**) para el modo virtual.
- Controlador **ABB OmniCore** físico con licencia **EGM** habilitada para el modo real.
- **RobotWare** compatible con la familia GoFa CRB15000.

### Hardware (modo real)

- Robot colaborativo **ABB GoFa CRB15000**.
- Webcam USB montada en cenital sobre el área de manipulación.
- Cubo de Rubik estándar 3x3.
- PC/host con WSL2 configurado en `networkingMode=mirrored` para que el UDP del controlador físico llegue a la máquina virtual (ver [Hoja de ruta](#hoja-de-ruta)).

---

## Instalación

```bash
# 1. Crear/activar el workspace de ROS 2
mkdir -p ~/rubik_ros2_ws/src
cd ~/rubik_ros2_ws/src

# 2. Clonar este repositorio (y el driver EGM del tutor, si no está ya en el workspace)
git clone https://github.com/abp190904/TFG.git rubik_control
# git clone <repo-del-driver> abb_egm_driver

# 3. Instalar dependencias de Python
pip install opencv-python numpy kociemba psutil GPUtil pandas matplotlib

# 4. Compilar el workspace
cd ~/rubik_ros2_ws
colcon build --symlink-install
source install/setup.bash
```

> El paquete se instala como `rubik_control` (ver `package.xml` / `setup.py`) con tres *entry points*: `orquestador`, `robot_node` y `metricas`.

---

## Puesta en marcha

### Arranque con un solo comando (recomendado)

```bash
ros2 launch rubik_control rubik.launch.py
```

Esto abre **una ventana `xterm` por nodo** (driver EGM, robot y orquestador, más métricas si está activada), respetando un arranque **escalonado** para evitar condiciones de carrera:

| Instante | Nodo | Motivo |
|---|---|---|
| `t = 0s` | Driver EGM (`abb_egm_driver`) | Debe estar escuchando en el puerto UDP antes que ningún otro nodo. |
| `t = 2s` | `robot_node` + `metricas` | Se suscriben al driver y a los topics de comando. |
| `t = 6s` | `orquestador` | Publica el scramble ~1 s después de arrancar; el robot ya lleva 4 s suscrito, por lo que no se pierde ningún mensaje (reforzado además por QoS `TRANSIENT_LOCAL`, ver más abajo). |

Cada ventana muestra su propio menú interactivo **`[V] Virtual / [R] Real`**; gracias al QoS con memoria, **el orden en que respondas a los menús no importa**.

### Argumentos disponibles

```bash
# Fijar el modo sin menús interactivos (útil para automatizar/depurar)
ros2 launch rubik_control rubik.launch.py modo:=v      # o modo:=r

# Lanzar sin el nodo de métricas
ros2 launch rubik_control rubik.launch.py metricas:=false

# Todo en una sola terminal (sin xterm) — requiere modo:=v o modo:=r
ros2 launch rubik_control rubik.launch.py terminales:=false modo:=v
```

### Arranque manual (nodo a nodo, para depuración)

```bash
# Terminal 1 — Driver EGM (del tutor)
ros2 run abb_egm_driver egm_driver --ros-args --params-file <ruta-al-yaml>

# Terminal 2 — Controlador del robot
ros2 run rubik_control robot_node

# Terminal 3 — Nodo de métricas (opcional)
ros2 run rubik_control metricas

# Terminal 4 — Orquestador (arrancar el último)
ros2 run rubik_control orquestador
```

También existe un script `lanzar_rubik.sh` (tmux, panel 2×2, pensado para Windows Terminal) para levantar el mismo conjunto de nodos sin depender de `xterm`.

---

## Modo Virtual vs. Modo Real

El mismo código (`main.py`, `robot.py`, `vision.py`) se ejecuta en ambos modos; lo único que cambia es el origen de los datos:

| | **Virtual** | **Real** |
|---|---|---|
| Mezclado | Scramble aleatorio (15 movimientos) ejecutado sobre el gemelo digital | Se mezcla el cubo físico a mano; el robot pasa directo al escaneo |
| Cámara | Snapshot generado por el **Add-in de Cámara Virtual** en una carpeta temporal de Windows, leído desde WSL2 | Webcam USB física (`cv2.VideoCapture` + V4L2) |
| Rangos de color | `RANGOS_HSV_VIRTUAL` (`vision.py`) | `RANGOS_HSV_REAL` (`vision.py`) |
| Controlador | Controlador Virtual de RobotStudio | Controlador ABB OmniCore físico |
| Reconstrucción previa a resolver | No aplica | El estado escaneado se reproduce primero como giros **virtuales** sobre el gemelo digital (si está abierto) antes de iniciar el giro físico, marcado por el índice de comando `99` |

---

## Detalle de los nodos ROS 2

### `orquestador` (`main.py`)

Nodo `rubik_orchestrator`. Coordina el ciclo completo:

1. Genera (o no, en modo real) el scramble y lo publica en `comando_scramble`.
2. Espera confirmación del robot (`scan_state == 98`) para iniciar el escaneo.
3. Llama a `vision.escanear_cubo()` pasándole callbacks para consultar/mandar telemetría al robot (`scan_cmd` / `scan_state`) sin acoplarse a la implementación EGM.
4. Valida el estado del cubo y llama a `solver.resolver_cubo()`.
5. Publica la solución en `comando_movimiento` y espera a que `robot_node` confirme el fin de la ejecución física (aparición del CSV de telemetría).
6. Genera el **reporte técnico** (`datos_resolucion.txt`) con timings, cadencia de giro, uso de CPU/RAM/GPU y, si el nodo de métricas está activo, anexa un resumen de latencia EGM y cinemática por eje.

### `robot_node` (`robot.py`)

Nodo `gofa_controller_node`. Es el cliente del driver EGM del profesor:

- Traduce las letras de movimiento del cubo (`U`, `U'`, `U2`, …) a códigos numéricos (`DICCIONARIO_RUBIK`) que entiende el RAPID.
- Publica/suscribe el array `Float64MultiArray` de 40 elementos (`command/data` / `state/data`) que constituye el protocolo EGM completo, además de `state/joint` para la telemetría articular.
- Gestiona el **handshake** comando-a-comando: solo avanza al siguiente índice cuando el controlador confirma (`ACK == índice_actual`) haber ejecutado el anterior.
- Aplica una pausa configurable (`PAUSA_GIRO_VIRTUAL`) entre giros **virtuales** (mezclado y reconstrucción previa a la resolución) para dar tiempo a la animación del Smart Component; los giros **físicos** no la necesitan porque el propio brazo marca el ritmo.
- Vuelca a CSV la cinemática completa de la resolución física (`~/rubik_ros2_ws/historial/latest_telemetria.csv`).

### `vision.py`

- Detecta automáticamente la cara del cubo en la imagen (máscara por saturación/brillo + contorno más grande "cuadrado-ish"), con suavizado temporal (EMA) para no perder la posición entre frames.
- Muestrea el color de cada una de las 9 pegatinas con la **mediana HSV** de un pequeño parche alrededor de su centro (robusto a ruido).
- Sigue el **orden de escaneo optimizado** `L, R, F, B, D, U` (3 viajes × 2 caras opuestas por agarre), en correspondencia exacta con `Rutina_Escanear_Cubo` en RAPID.
- Permite corrección de orientación por cara (`ROTACION_CARA`) para compensar el giro de muñeca al presentar cada cara.
- Expone `validar_cubo()` (comprobación de centros/conteo de colores antes de pasar al solver) y `mostrar_mapa_2d()` (visualización tipo *cube net*).

### `solver.py`

Envoltorio directo sobre la librería `kociemba`, que dado el string de 54 caracteres (orden estándar U-R-F-D-L-B) devuelve la solución óptima (máx. 20 movimientos, algoritmo de dos fases de Kociemba).

### `metricas.py`

Nodo **100 % pasivo** (`rubik_metricas`) que no publica nada en los topics de control; solo escucha lo que ya existe en el sistema:

- **Latencia del handshake EGM** por comando (envío del índice → ACK recibido).
- **Cinemática por eje**: velocidad, aceleración y *jerk* máximos, derivados numéricamente de `state/joint`.
- **Distancia TCP recorrida** (total de sesión y solo durante la resolución), a partir de `state/pose`.
- **Salud del sistema**: frecuencia real de cada topic, medida en ventanas de 5 s.

Salida en `~/rubik_ros2_ws/historial/metricas/metricas_<fecha>/` (`resumen.txt`, `latencias.csv`, `cinematica.csv`, `salud_topics.csv`, `metricas.json`). Se puede desactivar (`metricas:=false`) o cerrar en cualquier momento sin afectar al pipeline principal.

---

## Comunicación EGM y protocolo de comandos

La comunicación de bajo nivel con el controlador ABB se realiza mediante **EGM (Externally Guided Motion) sobre UDP**, a través del driver ROS 2 del tutor (`abb_egm_driver`). Cada paquete es un **snapshot autocontenido** de 40 elementos (`Float64MultiArray`), no un lote acumulativo — lo que permite descartar con seguridad paquetes antiguos y quedarse siempre con el más reciente (ver *drain loop* más abajo).

Mapeo relevante del array (índices fijos, documentados también en la memoria del TFG):

| Índice | Uso |
|---|---|
| `[0]` | ACK del robot / índice de comando confirmado |
| `[1]` | Comando a ejecutar (código numérico de `DICCIONARIO_RUBIK`, o marcadores especiales) |
| `[2]` | Comando de escaneo (`scan_cmd`, en modo escaneo) |

Marcadores especiales del código de comando:

- `98` — fin del mezclado / salto al escaneo.
- `99` — inicio de la resolución física (todo lo anterior son giros virtuales de reconstrucción del gemelo digital).
- `999` — el controlador confirma que el cubo está resuelto.
- `998` — señal de aborto de emergencia (`ABORTAR`).

### Diccionario de movimientos (`DICCIONARIO_RUBIK`)

```
U=1  U'=2   U2=13
F=3  F'=4   F2=14
D=5  D'=6   D2=15
B=7  B'=8   B2=16
R=9  R'=10  R2=17
L=11 L'=12  L2=18
```

---

## Programación RAPID

El controlador ABB ejecuta tres módulos (`rapid/`):

- **`Module_Rubik.modx`** — una rutina `Girar_X` / `Girar_X_Prima` / `Girar_X_Doble` por cada cara del cubo (`U`, `D`, `F`, `B`, `L`, `R`), que combinan el movimiento articular necesario, la activación del Smart Component (señal digital `DO_Girar_*`) y una pausa de animación (`t_anim`).
- **`Module_Movimientos.modx`** — movimientos cartesianos/articulares del brazo: posicionamiento en cada agarre, volteos entre caras (p. ej. `Mov_Voltear_F_a_U` / `Mov_Devolver_U_a_F`) y la rutina de presentación a cámara (`Presentar_Y_Fotografiar`).
- **`Module1.modx`** — módulo principal / máquina de estados EGM que interpreta el array de 40 elementos recibido por UDP y despacha las rutinas anteriores.

---

## Add-in de Cámara Virtual (RobotStudio)

`VirtualCameraAddin/` es una extensión de **ABB RobotStudio** en C# que añade una cámara virtual dentro de la simulación, de forma que el gemelo digital pueda "fotografiarse" igual que lo haría una webcam real:

- Sincroniza la posición de la cámara con un **frame de RobotStudio buscado por nombre** en `Station.ActiveStation.Frames` (únicamente de primer nivel, no en frames anidados).
- Orienta la vista a lo largo del eje **Z local**, con **Y como "arriba"** de la imagen.
- Genera capturas (`snapshot_yolo.png`) en una carpeta temporal de Windows, que `vision.py` localiza automáticamente desde WSL2 traduciendo `%TEMP%` a una ruta `/mnt/c/...`.
- Aplica una pequeña compensación lateral (~2%) cuando la dirección de vista está casi perfectamente vertical, para evitar la singularidad del *LookAt*.

> ⚠️ El add-in **absorbe silenciosamente cualquier excepción interna** — si la cámara "se selecciona pero no llega a activarse", no se ve ningún error en pantalla; hay que revisar que el frame de referencia exista en el nivel superior de la estación y no esté anidado.

---

## Visión por computador y calibración de color

- Los rangos HSV de cada color (`RANGOS_HSV_REAL` / `RANGOS_HSV_VIRTUAL`) están definidos en `vision.py` y **son distintos para el modo real y el virtual**, ya que la iluminación y el renderizado difieren sustancialmente.
- `calibrador_colores.py` abre una ventana interactiva con *trackbars* de OpenCV para ajustar en vivo los umbrales H/S/V mientras se visualiza la máscara resultante — el flujo recomendado antes de operar en real es recalibrar aquí y trasladar los valores a `RANGOS_HSV_REAL`.
- La detección de la cara del cubo (posición y tamaño de la rejilla 3×3) es **automática y adaptativa**: no depende de una posición fija en el frame, por lo que tolera variaciones en la distancia cámara-cubo.

---

## Métricas y reportes técnicos

Cada resolución completa genera automáticamente, en `~/rubik_ros2_ws/historial/<timestamp>/`:

- `mapa_2d.png` — visualización tipo *cube net* del estado escaneado.
- `telemetria_ejes.csv` — posición articular durante toda la ejecución física.
- `datos_resolucion.txt` — informe legible con:
  1. Estado detectado por visión y solución generada.
  2. Desglose de tiempos (percepción, cómputo del solver, ejecución mecánica, total).
  3. Métricas de eficiencia mecánica (cadencia de giro, velocidad de respuesta del solver).
  4. Diagnóstico de carga computacional del PC (RAM, CPU, GPU/VRAM).
  5. *(si el nodo de métricas está activo)* Latencia del handshake EGM, distancia TCP recorrida y cinemática (velocidad/aceleración/jerk máximos) por eje.
- `metricas/` — copia de los CSV detallados del nodo observador, si estaba en marcha.

`generar_dashboard.py` toma la carpeta de historial más reciente y genera gráficas (matplotlib) a partir de estos datos.

---

## Herramientas auxiliares

| Script | Propósito |
|---|---|
| `rubik_control/calibrador_colores.py` | Calibración interactiva de rangos HSV con trackbars de OpenCV. |
| `rubik_control/generar_dashboard.py` | Genera gráficas a partir del último historial de ejecución. |
| `lanzar_rubik.sh` | Lanzador alternativo basado en `tmux` con panel 2×2, pensado para Windows Terminal. |
| `test/` | Tests de estilo del paquete ROS 2 (`ament_copyright`, `ament_flake8`, `ament_pep257`). |

---

## Problemas conocidos y decisiones de diseño

Estas son algunas de las soluciones técnicas más relevantes adoptadas durante el desarrollo:

- **QoS incompatible entre publisher/subscriber (`BEST_EFFORT` vs `RELIABLE`)** provocaba pérdida silenciosa de mensajes en DDS → todos los topics de control usan perfiles `RELIABLE` explícitos, y los de comando (`comando_movimiento`, `comando_scramble`) añaden `DurabilityPolicy.TRANSIENT_LOCAL` para que un nodo que se suscribe tarde no pierda el último comando publicado.
- **Saltos del reloj de WSL2** podían congelar temporizadores basados en tiempo de sistema → todos los `Timer` de ROS 2 usan `ClockType.STEADY_TIME` y las medidas de tiempo internas usan `time.monotonic()`.
- **Retardo por acumulación de paquetes UDP** en el driver EGM → cada paquete EGM es un snapshot autocontenido; se usa un *drain loop* (lecturas con `timeout=0`) para procesar siempre el paquete más reciente y descartar los antiguos con seguridad.
- **Desbordamiento del eje 6 (error 50027, fuera de rango articular)** por acumulación de vueltas de muñeca durante el escaneo → sistema de "postura auto-aprendida": la primera llegada cartesiana a cada posición memoriza los ángulos articulares (`CJointT()`); las siguientes llegadas usan `MoveAbsJ` con el delta del eje 6 normalizado (acotado a ±270°). La segunda cara de cada pareja de agarre usa un volteo de muñeca puro (`MoveAbsJ`, ±180° en `rax_6`) para evitar reconfiguraciones de cinemática inversa.
- **Ritmo de los giros virtuales** — se introduce una pausa (`PAUSA_GIRO_VIRTUAL`) tras cada giro virtual para dar tiempo a la animación del Smart Component del cubo; aplica al mezclado virtual y a la reconstrucción previa a la resolución en modo real, pero no a los movimientos físicos reales (marcados por el índice `99`), donde el propio brazo marca el ritmo.
- **`wobj0` en RAPID es el frame base del robot, no el origen del mundo de RobotStudio** — las coordenadas leídas desde la GUI de RobotStudio no son directamente utilizables en RAPID; la posición de la cámara se calibró empíricamente respecto a la base del robot.

---

## Hoja de ruta

Trabajo pendiente hacia la validación completa sobre el **robot físico**:

- [ ] Conectar la webcam física a WSL2 mediante `usbipd-win` (`list` → `bind --busid` → `attach --wsl --busid`) y verificar el soporte del módulo de kernel UVC.
- [ ] Configurar el enrutado UDP entre el controlador físico y WSL2 mediante `networkingMode=mirrored` en `.wslconfig`.
- [ ] Hacer que `PAUSA_GIRO_VIRTUAL` sea condicional al modo (el robot físico ya tiene latencia de movimiento inherente y no la necesita).
- [ ] Recalibrar los umbrales HSV de `vision.py` para las condiciones de iluminación reales.

---

## Licencia

Este proyecto está distribuido bajo licencia **MIT** — ver el fichero [LICENSE](LICENSE) para más detalles.

© 2026 Antonio Balboa
