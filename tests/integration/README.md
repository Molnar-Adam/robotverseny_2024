# ROS Integrációs Tesztek

Ez a könyvtár a `robotverseny_2024` projekt ROS integrációs tesztjeit tartalmazza.

## 📚 Mi az integrációs teszt?

Az **integrációs tesztek** valós ROS node-ok együttműködését ellenőrzik, ellentétben a unit tesztekkel, amelyek csak izolált függvényeket tesztelnek mock objektumokkal.

### Különbségek Unit és Integrációs Tesztek között

| **Unit Test** | **Integration Test** |
|---------------|---------------------|
| Egyetlen függvény izoláltan | Több node együttműködése |
| Mock objektumok (nincs ROS) | Valós ROS node-ok |
| Gyors (milliszekundumok) | Lassabb (másodpercek) |
| pytest keretrendszer | rostest keretrendszer |

---

## 📂 Tesztek Áttekintése

### 1. `test_pid_error_node.py` + `test_pid_error_integration.test`

**Tesztelt komponens:** `pid_error.py` node

**Pipeline:**
```
TESZT → /scan → pid_error.py → /error → TESZT
```

**Tesztesetek:**
- ✅ Node inicializáció (elindult-e a node?)
- ✅ Szimmetrikus fal-követés (bal = jobb távolság → error ≈ 0)
- ✅ Bal fal közelebb (error < 0)
- ✅ Jobb fal közelebb (error > 0)

**Összes teszt:** 4 db

---

### 2. `test_control_pipeline.py` + `test_control_pipeline.test`

**Tesztelt komponensek:** `pid_error.py` + `control.py` (teljes pipeline)

**Pipeline:**
```
TESZT → /scan → pid_error.py → /error → control.py → /cmd_vel → TESZT
```

**Tesztesetek:**
- ✅ Teljes pipeline működése (mindkét node válaszol-e)
- ✅ Szimmetrikus eset (error ≈ 0 → megfelelő sebesség/kormányzás)
- ✅ Jobbra fordulás (bal fal közelebb)
- ✅ Balra fordulás (jobb fal közelebb)

**Összes teszt:** 4 db

**Összesen:** 8 integrációs teszt

---

## 🚀 Futtatási Útmutató

### Előfeltételek

1. **ROS telepítve van** (ezek a tesztek valós ROS-t igényelnek!)
2. **Workspace buildelve:**
   ```bash
   cd ~/catkin_ws
   catkin_make
   source devel/setup.bash
   ```

3. **Python szkriptek futtathatók:**
   ```bash
   chmod +x src/robotverseny_2024/src/pid_error.py
   chmod +x src/robotverseny_2024/src/control.py
   chmod +x src/robotverseny_2024/tests/integration/*.py
   ```

---

### Egyetlen Teszt Futtatása

**1. pid_error.py node teszt:**
```bash
rostest robotverseny_2024 test_pid_error_integration.test
```

**2. Teljes pipeline teszt:**
```bash
rostest robotverseny_2024 test_control_pipeline.test
```

---

### Verbose Kimenet (Részletes Logok)

```bash
rostest robotverseny_2024 test_pid_error_integration.test --text
```

A `--text` flag minden log üzenetet kiír a konzolra.

---

### Minden Integrációs Teszt Futtatása

Ha a `CMakeLists.txt`-ben beállítottad a `catkin_add_rostest()` sorokat:

```bash
catkin_make run_tests
```

Ez lefuttatja az **összes** rostest fájlt a workspace-ben.

---

## 📊 Teszt Kimenet Értelmezése

### Sikeres Teszt Példa

```
[ROSTEST] ============================================================
[ROSTEST] test_pid_error_integration_test
[ROSTEST] ============================================================

test_1_node_initialization ... ok
test_2_symmetric_wall_following ... ok
test_3_left_closer_wall_following ... ok
test_4_right_closer_wall_following ... ok

----------------------------------------------------------------------
Ran 4 tests in 8.234s

OK

[ROSTEST] SUMMARY
 * RESULT: SUCCESS
```

✅ **SUCCESS** = minden teszt átment

---

### Sikertelen Teszt Példa

```
test_2_symmetric_wall_following ... FAIL

======================================================================
FAIL: test_2_symmetric_wall_following
----------------------------------------------------------------------
AssertionError: Nem érkezett PidState üzenet az /error topicra!

----------------------------------------------------------------------
Ran 4 tests in 10.000s

FAILED (failures=1)

[ROSTEST] SUMMARY
 * RESULT: FAIL
```

❌ **FAIL** = legalább egy teszt elbukott

**Debugging:** Nézd meg a részletes logokat:
```bash
rostest robotverseny_2024 test_pid_error_integration.test --text
```

---

## 🧪 Teszt Anatómia

### .test Fájl Struktúra

```xml
<launch>
    <!-- 1. Teszt alatt álló node-ok -->
    <node pkg="robotverseny_2024" type="pid_error.py" name="pid_error"/>
    
    <!-- 2. Teszt szkript -->
    <test test-name="my_test" pkg="robotverseny_2024" type="test_script.py"/>
</launch>
```

**Fontos:**
- `<node>` = normál ROS node (mint `roslaunch`)
- `<test>` = teszt szkript (automatikusan leáll teszt után)

---

### Python Teszt Szkript Struktúra

```python
import unittest
import rospy
import rostest

class MyTest(unittest.TestCase):
    def setUp(self):
        # Minden teszt ELŐTT fut
        rospy.init_node('test_node')
        self.subscriber = rospy.Subscriber('/topic', Msg, self.callback)
    
    def callback(self, msg):
        # Automatikusan hívódik, ha üzenet érkezik
        self.received_msg = msg
    
    def test_something(self):
        # A tényleges teszt
        self.assertEqual(self.received_msg.data, 42)

if __name__ == '__main__':
    rostest.rosrun('pkg_name', 'test_name', MyTest)
```

---

## 🔧 Hibaelhárítás

### Probléma 1: `rostest: command not found`

**Ok:** ROS nincs telepítve vagy nincs source-olva.

**Megoldás:**
```bash
source /opt/ros/noetic/setup.bash
source ~/catkin_ws/devel/setup.bash
```

---

### Probléma 2: `Timeout - Nem érkezett üzenet`

**Ok:** A node nem indult el vagy nem publikál.

**Debug:**
1. Ellenőrizd, hogy a node fut-e:
   ```bash
   rosnode list
   ```
2. Ellenőrizd a topic-okat:
   ```bash
   rostopic list
   rostopic echo /error
   ```

---

### Probléma 3: `ImportError: No module named control_msgs`

**Ok:** ROS függőségek hiányoznak.

**Megoldás:**
```bash
sudo apt-get install ros-noetic-control-msgs
sudo apt-get install ros-noetic-sensor-msgs
```

---

## 📈 Tesztek Futtatása CI/CD-ben

GitHub Actions vagy GitLab CI példa:

```yaml
test:
  script:
    - source /opt/ros/noetic/setup.bash
    - cd ~/catkin_ws
    - catkin_make
    - source devel/setup.bash
    - catkin_make run_tests
    - catkin_test_results
```

---

## 🎓 További Olvasnivaló

- [ROS Testing Documentation](http://wiki.ros.org/rostest)
- [unittest Python módszer](https://docs.python.org/3/library/unittest.html)
- Unit tesztek: `../tests/` könyvtár (pytest-based)

---

## ✅ Összegzés

- 8 integrációs teszt fut valós ROS node-okkal
- Pipeline szintű tesztelés (LIDAR → PID → motor)
- Timeout-ok biztosítják, hogy a tesztek nem ragadnak be végtelenül
- Callback pattern: aszinkron üzenet fogadás
