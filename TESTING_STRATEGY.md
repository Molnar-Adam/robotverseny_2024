# 🧪 Tesztelési Stratégia - Robotverseny 2024

Ez a dokumentum a teljes tesztelési infrastruktúrát mutatja be a robotverseny projekthez.

---

## 📊 Tesztelési Piramis

```
                    ╱╲
                   ╱  ╲
                  ╱ E2E╲            1-2 teszt (manuális/Gazebo)
                 ╱──────╲
                ╱        ╲
               ╱Integrációs╲       8 teszt (rostest)
              ╱─────────────╲
             ╱               ╲
            ╱   Unit Tesztek  ╲    31 teszt (pytest/GTest)
           ╱───────────────────╲
          ╱_____________________╲
```

**Elv:** Minél lejjebb, annál több teszt kell!

- **Unit tesztek** (alap): Gyorsak, sok teszt
- **Integrációs tesztek** (közép): Közepes sebesség, kevesebb teszt
- **E2E tesztek** (csúcs): Lassúak, nagyon kevés teszt

---

## 🗂️ Projekt Struktúra

```
robotverseny_2024/
├── src/                          # Forráskód
│   ├── pid_error.py              # LIDAR → error számítás
│   ├── control.py                # PID kontroller
│   ├── simple_pursuit.py         # Pure Pursuit algoritmus
│   └── path_and_steering.cpp     # Vizualizáció (C++)
│
├── tests/                        # TESZTEK
│   ├── conftest.py               # Pytest konfig (ROS mock)
│   │
│   ├── test_simple_pursuit.py    # Unit: simple_pursuit függvények
│   ├── test_pid_error.py         # Unit: pid_error függvények
│   ├── test_control.py           # Unit: control függvények
│   │
│   └── integration/              # Integrációs tesztek
│       ├── README.md             # Integrációs teszt dokumentáció
│       ├── test_pid_error_node.py           # Teszt: pid_error.py node
│       ├── test_pid_error_integration.test  # Launch fájl
│       ├── test_control_pipeline.py         # Teszt: teljes pipeline
│       └── test_control_pipeline.test       # Launch fájl
│
├── CMakeLists.txt                # Build konfiguráció
├── package.xml                   # ROS csomag metaadatok
├── Teszt_Dokumentacio.docx       # Word dokumentáció (unit tesztek)
└── generate_test_docs.py         # Dokumentáció generáló szkript
```

---

## 🎯 Tesztelési Lefedettség

### 1️⃣ Python Unit Tesztek (pytest)

| Modul | Tesztelt függvények | Tesztek száma | Státusz |
|-------|-------------------|--------------|---------|
| **simple_pursuit.py** | `calcPointPos`, `calcPursuitAngle` | 10 | ✅ 100% |
| **pid_error.py** | `getRange`, `followSimple`, `followCenter` | 11 | ✅ 100% |
| **control.py** | `control` (PID loop) | 10 | ✅ 100% |
| **Összesen** | | **31** | ✅ |

**Futtatás:**
```bash
python -m pytest tests/ -v
```

**Jellemzők:**
- ⚡ Gyors (~0.2s)
- 🔄 ROS mock használata (offline működés)
- 📊 100% success rate

---

### 2️⃣ C++ Unit Tesztek (GTest)

| Modul | Tesztelt függvények | Tesztek száma | Státusz |
|-------|-------------------|--------------|---------|
| **path_and_steering.cpp** | `mapval` | 7 | 📝 Elkészítve |

**Futtatás:**
```bash
catkin_make run_tests
```

**Jellemzők:**
- 🧮 Matematikai függvények (koordináta transzformáció)
- ⚡ Natív C++ sebesség
- 🔗 catkin integráció

---

### 3️⃣ ROS Integrációs Tesztek (rostest)

| Teszt | Tesztelt komponensek | Tesztek száma | Státusz |
|-------|---------------------|--------------|---------|
| **test_pid_error_node** | `pid_error.py` node | 4 | ✅ Kész |
| **test_control_pipeline** | `pid_error.py` + `control.py` | 4 | ✅ Kész |
| **Összesen** | | **8** | ✅ |

**Futtatás:**
```bash
# Egyenként
rostest robotverseny_2024 test_pid_error_integration.test
rostest robotverseny_2024 test_control_pipeline.test

# Összes
catkin_make run_tests
```

**Jellemzők:**
- 🤖 Valós ROS node-ok futnak
- 🔗 Topic kommunikáció tesztelése
- 🐢 Lassabb (~8-10s per teszt)

---

## 🔬 Tesztelési Lefedettség Táblázat

| Réteg | Technológia | Tesztek | Futási idő | ROS kell? |
|-------|------------|---------|-----------|----------|
| **Unit** (Python) | pytest | 31 | ~0.2s | ❌ Nem |
| **Unit** (C++) | GTest | 7 | ~0.1s | ❌ Nem |
| **Integration** | rostest | 8 | ~20s | ✅ Igen |
| **Összesen** | | **46** | ~20s | |

---

## 🧩 Tesztelési Folyamat

### Fejlesztési Workflow

```
1. Kód írása
   ↓
2. Unit teszt írása (pytest/GTest)
   ↓
3. Unit teszt futtatása (gyors feedback)
   ↓
4. Integrációs teszt frissítése (ha kell)
   ↓
5. Integrációs teszt futtatása (lassabb, de valósághű)
   ↓
6. Commit + Push
```

---

### CI/CD Pipeline (javaslat)

```yaml
# .github/workflows/test.yml
name: ROS Tests

on: [push, pull_request]

jobs:
  unit-tests:
    runs-on: ubuntu-20.04
    steps:
      - uses: actions/checkout@v2
      - name: Install Python dependencies
        run: pip install pytest pytest-mock
      - name: Run Python unit tests
        run: python -m pytest tests/ -v
  
  integration-tests:
    runs-on: ubuntu-20.04
    steps:
      - uses: actions/checkout@v2
      - name: Install ROS
        run: |
          sudo apt-get install ros-noetic-desktop-full
          source /opt/ros/noetic/setup.bash
      - name: Build workspace
        run: |
          cd ~/catkin_ws
          catkin_make
      - name: Run integration tests
        run: |
          source devel/setup.bash
          catkin_make run_tests
```

---

## 📚 Tesztelési Technikák

### 1. Mock Pattern (Unit Tesztek)

**Probléma:** ROS függőségek tesztelés nélkül nem elérhetők.

**Megoldás:** `conftest.py` automatikusan mockolja a ROS modulokat.

```python
# conftest.py
import sys
from unittest.mock import MagicMock

sys.modules['rospy'] = MagicMock()
sys.modules['sensor_msgs'] = MagicMock()
# ... stb
```

**Előny:** Tesztek futnak ROS nélkül is (gyors fejlesztés).

---

### 2. Callback Pattern (Integrációs Tesztek)

**Probléma:** ROS aszinkron → nem tudjuk, mikor érkezik üzenet.

**Megoldás:** Callback függvények + timeout loop.

```python
def error_callback(self, msg):
    self.received_msg = msg
    self.msg_received = True

# Várakozás
timeout = rospy.Time.now() + rospy.Duration(10.0)
while not self.msg_received and rospy.Time.now() < timeout:
    rospy.sleep(0.1)
```

**Előny:** Nem blokkolódik, ha üzenet nem érkezik.

---

### 3. Fixture Pattern (Setup/Teardown)

**Python unittest:**
```python
def setUp(self):
    # Minden teszt ELŐTT fut
    self.data = initialize_data()

def tearDown(self):
    # Minden teszt UTÁN fut (cleanup)
    cleanup()
```

**Előny:** Minden teszt tiszta állapotból indul.

---

## 🛠️ Tesztelési Eszközök

| Eszköz | Nyelv | Célja |
|--------|-------|-------|
| **pytest** | Python | Unit tesztek |
| **pytest-mock** | Python | Mock objektumok |
| **GTest** | C++ | Unit tesztek |
| **rostest** | Python/C++ | Integrációs tesztek |
| **unittest** | Python | Teszt keretrendszer (rostest alatt) |

---

## 📖 Dokumentáció

### Elérhető Dokumentumok

1. **`TESTING.md`** (unit tesztek) - Markdown formátum
2. **`Teszt_Dokumentacio.docx`** - Word dokumentum (31 unit teszt)
3. **`tests/integration/README.md`** - Integrációs tesztek dokumentációja
4. **Ez a fájl** - Teljes tesztelési stratégia

### Dokumentáció Generálása

```bash
# Word dokumentum generálása
python generate_test_docs.py
```

---

## 🔍 Debugging Tippek

### Unit Teszt Debug

**Verbose kimenet:**
```bash
python -m pytest tests/test_pid_error.py -v -s
```

**Egyetlen teszt futtatása:**
```bash
python -m pytest tests/test_pid_error.py::TestGetRange::test_inf_value -v
```

---

### Integrációs Teszt Debug

**Részletes logok:**
```bash
rostest robotverseny_2024 test_pid_error_integration.test --text
```

**ROS node-ok státusza:**
```bash
# Másik terminálban (teszt futása közben)
rosnode list
rostopic list
rostopic echo /error
```

---

## ✅ Best Practices

### ✅ DO (Ajánlott)

- ✅ Írj unit tesztet **minden** új függvényhez
- ✅ Használj `pytest.approx()` lebegőpontos számokhoz
- ✅ Adj értelmes neveket a teszteknek (`test_zero_degree` ❌ `test_1`)
- ✅ Használj docstringet minden teszt függvényben
- ✅ Futtass teszteket **MINDEN commit előtt**

### ❌ DON'T (Kerülendő)

- ❌ Ne használj `assert x == 0.1` lebegőpontos számokra
- ❌ Ne hagyj "TODO" teszteket commitolva
- ❌ Ne ignoráld a teszt failure-öket
- ❌ Ne írj 500 soros teszt függvényt (bontsd kisebbre!)

---

## 📈 Jövőbeli Fejlesztések

### Rövid távú (1-2 hét)

- [ ] C++ unit tesztek futtatása CI-ben
- [ ] Coverage jelentés generálás (`pytest-cov`)
- [ ] Integrációs tesztek bővítése (simple_pursuit.py)

### Közép távú (1 hónap)

- [ ] Gazebo szimulációs tesztek
- [ ] Property-based testing (`hypothesis`)
- [ ] Performance benchmarking

### Hosszú távú (2+ hónap)

- [ ] Hardveres HIL (Hardware-in-the-Loop) tesztek
- [ ] Valós robot tesztelés
- [ ] Verseny pálya replay tesztek (rosbag)

---

## 📞 Kapcsolat & Segítség

**Kérdés van?** Nézd meg a README fájlokat:
- Unit tesztek: `tests/README.md` (ha létezik)
- Integrációs tesztek: `tests/integration/README.md`

**Hibajelentés:** Nyiss GitHub Issue-t vagy küldd el a teszt kimenetét.

---

## 🎓 Összegzés

✅ **46 automatizált teszt** biztosítja a kód megbízhatóságát  
✅ **3 rétegű tesztelés**: Unit (Python/C++) + Integrációs + (jövő: E2E)  
✅ **Mock pattern** gyors fejlesztéshez ROS nélkül  
✅ **rostest** valós ROS környezetben  
✅ **Teljes dokumentáció** (MD + Word formátumban)  

**A tesztek futtatása:**
```bash
# Gyors ellenőrzés (unit tesztek, ~0.2s)
python -m pytest tests/ -v

# Teljes ellenőrzés (minden teszt, ~20s)
source devel/setup.bash
catkin_make run_tests
```

🚀 **Boldog tesztelést!**
