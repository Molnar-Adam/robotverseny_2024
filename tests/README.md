# Tesztdokumentáció

Ez az oldal a `megoldas_gyor24` projekt összes tesztjét összefoglalja: a Python alapú unit teszteket, a C++ unit tesztet és a ROS integrációs teszteket.

## Tesztkategóriák

### 1. Python unit tesztek

Ezek a tesztek a `src/` könyvtár Python moduljainak tiszta logikáját ellenőrzik mockolt ROS környezetben.

#### [tests/test_control.py](tests/test_control.py)

**Tesztelt modul:** [src/control.py](src/control.py)

**Fókusz:** PID szabályzó viselkedése, sebesség- és szögkorlátok, simple és non-simple mód.

**Mit ellenőriz:**
- A kimeneti `Twist` üzenet helyes lineáris és szögsebessége.
- A simple mód fix sebességlogikája.
- A non-simple mód sebességszabályzása.
- A `prev_error` állapot frissülése.
- Szélső esetek, például `NaN` bemenet.

**Tesztek száma:** 13

**Futtatás:**
```bash
python -m pytest tests/test_control.py -v
```

---

#### [tests/test_pid_error.py](tests/test_pid_error.py)

**Tesztelt modul:** [src/pid_error.py](src/pid_error.py)

**Fókusz:** LIDAR-adatok feldolgozása, `getRange`, `followSimple`, `followCenter`, callback viselkedés.

**Mit ellenőriz:**
- A szög-indexelés és a határérték-kezelés helyessége.
- `inf` és `nan` értékek fallback kezelése.
- Fal-követési logika szimmetrikus és aszimmetrikus helyzetekben.
- A callback megfelelő `PidState` üzenetet publikál.

**Tesztek száma:** 16

**Futtatás:**
```bash
python -m pytest tests/test_pid_error.py -v
```

---

#### [tests/test_simple_pursuit.py](tests/test_simple_pursuit.py)

**Tesztelt modul:** [src/simple_pursuit.py](src/simple_pursuit.py)

**Fókusz:** Pure Pursuit alapgeometria, távolságszámítás, szögszámítás, callback logika.

**Mit ellenőriz:**
- `calcPointPos` koordináta-transzformáció.
- `calcPursuitAngle` kormányzási szög számítása.
- `getDistance` és `getAngle` működése.
- `followSimple` logika transformált és fallback útvonalon.
- A `callbackLaser` helyes `cmd_vel` publikálása.

**Tesztek száma:** 16

**Futtatás:**
```bash
python -m pytest tests/test_simple_pursuit.py -v
```

---

### 2. C++ unit teszt

Ez a teszt a `path_and_steering` C++ node determinisztikus segédlogikáját ellenőrzi.

#### [tests/test_path_and_steering_utils.cpp](tests/test_path_and_steering_utils.cpp)

**Tesztelt kód:** [src/path_and_steering_utils.cpp](src/path_and_steering_utils.cpp)

**Fókusz:** pályapont-generálás, értéktérképezés, path-trimmelés.

**Mit ellenőriz:**
- `MapValue` helyes interpolációja.
- `BuildSteeringPoints` előre és hátra haladó ágai.
- A generált pontok numerikus stabilitása.
- `TrimPath` a megfelelő számú `PoseStamped` elemet tartja meg.

**Tesztek száma:** 6

**Futtatás:**
```bash
catkin_make run_tests_megoldas_gyor24_gtest_path_and_steering_utils_test
```

Ha csak a binárist akarod futtatni:
```bash
./devel/lib/megoldas_gyor24/path_and_steering_utils_test
```

---

### 3. ROS integrációs tesztek

Ezek a tesztek valódi ROS node-okat indítanak, és topic-szinten ellenőrzik a működést.

Részletes leírás: [tests/integration/README.md](tests/integration/README.md)

#### [tests/integration/test_control_node.py](tests/integration/test_control_node.py) + [tests/integration/test_control_node_integration.test](tests/integration/test_control_node_integration.test)

**Tesztelt komponens:** [src/control.py](src/control.py)

**Pipeline:**
`TESZT -> /error -> control.py -> /cmd_vel -> TESZT`

**Mit ellenőriz:**
- A node elindul és feliratkozik az `/error` topicra.
- A publikált `/cmd_vel` helyes irányú és korlátolt.
- Többszöri üzenetre is stabilan reagál.

**Tesztek száma:** 10

**Futtatás:**
```bash
rostest megoldas_gyor24 test_control_node_integration.test
```

---

#### [tests/integration/test_pid_error_node.py](tests/integration/test_pid_error_node.py) + [tests/integration/test_pid_error_integration.test](tests/integration/test_pid_error_integration.test)

**Tesztelt komponens:** [src/pid_error.py](src/pid_error.py)

**Pipeline:**
`TESZT -> /scan -> pid_error.py -> /error -> TESZT`

**Mit ellenőriz:**
- A node elindul és feliratkozik a `/scan` topicra.
- A `PidState` hibaérték megfelelő szimmetrikus és aszimmetrikus szkennelésnél.
- A callback a várt üzenetet publikálja.

**Tesztek száma:** 6

**Futtatás:**
```bash
rostest megoldas_gyor24 test_pid_error_integration.test
```

---

#### [tests/integration/test_control_pipeline.py](tests/integration/test_control_pipeline.py) + [tests/integration/test_control_pipeline.test](tests/integration/test_control_pipeline.test)

**Tesztelt komponensek:** [src/pid_error.py](src/pid_error.py) + [src/control.py](src/control.py)

**Pipeline:**
`TESZT -> /scan -> pid_error.py -> /error -> control.py -> /cmd_vel -> TESZT`

**Mit ellenőriz:**
- A teljes falkövető vezérlési lánc működik.
- A két node együtt helyes hibát és parancsot állít elő.
- A pipeline stabil több egymást követő bemenetre.

**Tesztek száma:** 6

**Futtatás:**
```bash
rostest megoldas_gyor24 test_control_pipeline.test
```

---

#### [tests/integration/test_simple_pursuit_node.py](tests/integration/test_simple_pursuit_node.py) + [tests/integration/test_simple_pursuit_integration.test](tests/integration/test_simple_pursuit_integration.test)

**Tesztelt komponens:** [src/simple_pursuit.py](src/simple_pursuit.py)

**Pipeline:**
`TESZT -> /scan -> simple_pursuit.py -> /cmd_vel -> TESZT`

**Mit ellenőriz:**
- A node publikál `cmd_vel` választ egy `scan` bemenetre.
- Reverse-zone és akadályhelyzet kezelése.
- Többszöri bemenetre is reagál.

**Tesztek száma:** 4

**Futtatás:**
```bash
rostest megoldas_gyor24 test_simple_pursuit_integration.test
```

---

#### [tests/integration/test_path_and_steering_node.py](tests/integration/test_path_and_steering_node.py) + [tests/integration/test_path_and_steering_integration.test](tests/integration/test_path_and_steering_integration.test)

**Tesztelt komponens:** [src/path_and_steering.cpp](src/path_and_steering.cpp)

**Pipeline:**
`TESZT -> /odom + /cmd_vel -> path_and_steering.cpp -> /marker_path + /marker_text + /marker_steering -> TESZT`

**Mit ellenőriz:**
- A node publikálja a path és vizualizációs marker topicokat.
- A sebességszöveg formázása helyes.
- A path hossza a konfigurált `path_size` korláton belül marad.

**Tesztek száma:** 2

**Futtatás:**
```bash
rostest megoldas_gyor24 test_path_and_steering_integration.test
```

---

### 4. Rendszerteszt

A rendszerteszt a teljes, több node-ból álló láncot egyszerre indítja el és vizsgálja. Ez már nem csak egyetlen node vagy egyetlen kapcsolat helyességét ellenőrzi, hanem azt is, hogy a teljes megoldás együtt, valós ROS kommunikáció mellett működik-e.

#### [tests/integration/test_system_pipeline.py](tests/integration/test_system_pipeline.py) + [tests/integration/test_system_pipeline.test](tests/integration/test_system_pipeline.test)

**Tesztelt komponensek:** [src/pid_error.py](src/pid_error.py), [src/control.py](src/control.py), [src/path_and_steering.cpp](src/path_and_steering.cpp)

**Pipeline:**
`TESZT -> /scan -> pid_error.py -> /error -> control.py -> /cmd_vel -> path_and_steering.cpp`

`TESZT -> /odom -> path_and_steering.cpp -> /marker_path + /marker_text + /marker_steering -> TESZT`

**Mit ellenőriz:**
- A `pid_error.py` helyesen számít hibát a szimulált LIDAR-adatokból.
- A `control.py` helyesen alakítja át az error értéket `cmd_vel` parancsokká.
- A `path_and_steering.cpp` fogadja a vezérlési parancsot és publikálja a vizualizációs markereket.
- A teljes lánc egyszerre, egymással együttműködve működik.

**Tesztek száma:** 1

**Futtatás:**
```bash
rostest megoldas_gyor24 test_system_pipeline.test
```

---

## Teljes futtatás

Ha minden tesztet egyszerre szeretnél futtatni:

```bash
catkin_make -DCATKIN_ENABLE_TESTING=ON
catkin_make run_tests
catkin_test_results build/test_results
```

Ha csak a `megoldas_gyor24` csomag tesztjei kellenek:

```bash
catkin_make run_tests_megoldas_gyor24
catkin_test_results build/test_results
```

## Összesítés

- Python unit tesztek: 13 + 16 + 16 = 45
- C++ unit teszt: 6
- Integrációs tesztek: 10 + 6 + 6 + 4 + 2 = 28
- Rendszerteszt: 1
- Összes teszt: 80
