# Használati útmutató

# `megoldas_gyor24` ROS package
🤖 Autonóm robotverseny helyszíni fordulójának megoldás mintája.

## Tesztdokumentáció

A teljes tesztleírás és a futtatási parancsok itt találhatók:
- [tests/README.md](tests/README.md)
- [tests/integration/README.md](tests/integration/README.md)

# A versenyen használt robot
Wheeltec / Roboworks Rosbot mini Ackermann robot
![main](etc/wheeltec_roboworks_ack01.png)

Fedélzeti számítógép
- Nvidia Jetson Nano

Érzékelők
- Orbbec Depth Camera
- LSN10 LIDAR

# Használat offline (javasolt)

Beüzemelés

1. Csatlakoztassuk a biztosított képernyő HDMI csatlakozóját a robot fedélzeti számítógépén található HDMI csatlakozóhoz. Hasonló módon, csatlakoztassuk a biztosított egér és billentyűzet USB csatlakozóját a robot fedélzeti számítógépéhez.

![main](etc/wheeltec_connectors.png)

2. Kapcsoljuk be a robotot a megjelölt kapcsoló megnyomásával. Sikeres bekapcsolás esetén a kapcsoló kéken világít.

![main](etc/wheeltec_switch.png#center)

3. Nyissuk meg az Ubuntu terminált a Ctrl + Alt + T billentyűk együttes lenyomásával. 

Az önvezető megoldás indítása

1. Győződjünk meg róla, hogy a robotplatform a biztosított állványra van helyezve, kerekei a levegőben vannak, szabadon foroghatnak. 

2. A korábban megnyitott terminálablakban adjuk ki az alábbi parancsot:

```
rosrun megoldas_gyor24 verseny_start.sh
```
Az Enter billentyű leütését követően, kis várakozás után elindul a megoldás. 

3. Csatlakoztassuk le a HDMI, valamint USB kábeleket, majd helyezzük a robotot a pályára. 

Az önvezető megoldás leállítása

1. Óvatosan, körültekintően helyezzük vissza a robotot az állványra.

2. Csatlakoztassuk a biztosított képernyő HDMI csatlakozóját a robot fedélzeti számítógépén található HDMI csatlakozóhoz. Hasonló módon, csatlakoztassuk a biztosított egér és billentyűzet USB csatlakozóját a robot fedélzeti számítógépéhez.

3. A korábban megnyitott terminálablakban adjuk ki az alábbi parancsot: 
```
rosrun megoldas_gyor24 stop_all.sh
```

# Az önvezető megoldás szerkesztése

1. Hajtsuk végre a beüzemelési folyamatot, győződjünk meg róla, hogy minden vezeték (HDMI, USB) csatlakoztatva van. 

2. A megnyitott terminálablakban navigáljunk a megoldást tartalmazó mappába az alábbi parancs kiadásával: 
```
cd ~/megoldas_ws/src/megoldas_gyor24 
```
3. Nyissuk meg a kódokat Visual Studio Code segítségével, az alábbi parancs kiadásával:
```
code .
```

4. Mentsük a módosításokat (mentés gomb, vagy Ctrl + s billentyűkombináció).

5. Lépjünk vissza a munkakönyvtárba, majd a build folyamat végrehajtásához adjuk ki az alábbi parancsokat: 

```
cd ~/megoldas_ws/ 
catkin build megoldas_gyor24 
cd .. 
source ~/.bashrc 
```
6. A módosítások végrehajtásra kerültek, a megoldás kipróbálható az önvezető megoldás indításának lépéseivel. 


# Használat WiFi elérés esetén (nem javasolt)
Előkövetelmények:
- WiFi kapcsolatra képes számítógép, Ubuntu 18.04 operációs rendszerrel és ROS Melodic telepítéssel
- Internet hozzáférés (Ethernet kábel vagy WiFi)

1. Kapcsoljuk be a robotplatformot. 
2. A számítógéppel csatlakozzunk a robot által létrehozott WiFi hálózatra. A WiFi hálózat neve minden robotplatform esetében egyedi, az SSID végén a # a robotplatform száma szerint változik:
```
SSID: WHEELTEC_CAR_5.5_#
Jelszó: dongguan
```
3. SSH-val csatlakozzunk a robotplatform fedélzeti számítógépére az alábbi terminálutasítással:

```
ssh wheeltec@192.168.0.100
```
Az utasítás kiadását követően jelszóra lesz szükség. Az alapértelmezett jelszó: `dongguan` 

## Internet hozzáférés a robotplatformon

A robotplatform fedélzeti számítógépére szoftvercsomagok tölthetők le, ehhez internet hozzáférés biztosítása szükséges.
Megjegyzés: a példamegoldás telepítése már megtörtént, interent hozzáférés biztosítása opcionális.

1. Ethernet: csatlakoztassuk az Ethernet kábelt a robotplatform fedélzeti számítógépének Ethernet portjába.

vagy

2. WiFi: nmtui terminálutasítás kiadását követően csatlakozzunk az elérhető WiFi hálózatra.
```
nmtui
```

<img src="https://user-images.githubusercontent.com/11504709/160778891-0c06e338-405f-43c6-8aac-928af33c057e.png" width="50%" />


## ROS példamegoldás telepítése

Beüzemelést követően ROS segítségével érhetőek el a robotplatform funkciói. A verseny példamegoldása is ROS által üzemelhető be. A verseny során használt robotok esetében a telepítés már megtörtént, ez a lépés kihagyható! Folytatás: `ROS példamegoldás beüzemelése`.

1. Workspace létrehozása és megoldás telepítése
```
mkdir -p ~/megoldas_ws/src
cd ~/megoldas_ws/
catkin init
cd ~/megoldas_ws/src/
git clone https://github.com/robotverseny/megoldas_zala23
cd ~/megoldas_ws/
catkin build megoldas_zala23
echo "source /home/wheeltec/megoldas_ws/devel/setup.bash" >> ~/.bashrc 
source ~/.bashrc
```
2. `screen` telepítése
```
sudo apt install mc screen
```
# ROS példamegoldás beüzemelése

## Megoldás indítása screen segítségével (javasolt)


```
rosrun megoldas_zala23 verseny_start.sh
```

A `verseny_start.sh` shell script gyszerre több viruális terminalt indít, úgy, mint: `roscore`, `turn_on_wheeltec_robot`, `lsn10_lidar`, `megoldas1.launch`. A megoldás minden komponense az alábbi paranccsal állítható le:
```
rosrun megoldas_zala23 stop_all.sh
```

További kezelés:

- list screen: `screen -ls`
- restore screen:  `screen -r roscore` / `screen -r turn_on_wheeltec_robot` /  `screen -r megoldas1 `
- detach: `Ctrl-a` + `Ctrl-d`

# ROS csatlakozás 

A robotplatform által hirdetett ROS topicok a platformhoz csatlakozó számítógépen is elérhetőek, a `ROS_MASTER_URI` változó megfelelő beállításával:
```
export ROS_MASTER_URI=http://192.168.0.100:11311
```
A változó megfelelő beállítását követően a topicok listázhatóak, Rviz segítségével vizualizálhatóak:
```
rostopic list
```
```
rosrun rviz rviz
```
# Példamegoldás működésének szemléltetése
```
roslaunch megoldas_zala23 rviz1.launch
```

![](etc/left_right01.gif)
![](etc/trajectory01.gif)
![](etc/angles01.svg)

## Megoldás indítása komponensenként

A megoldás komponensenként is elindítható. Ehhez a robotplatform fedélzeti számítógépén négy terminálablakra, és terminálonként az alábbi utasítások kiadására van szükség:

```
roscore
roslaunch turn_on_wheeltec_robot turn_on_wheeltec_robot.launch
roslaunch lsn10 lsn10.launch
roslaunch megoldas_zala23 megoldas1.launch
```

# Kiegészítő információk

## Workspace-ek
```
~/wheeltec_robot/src
~/catkin_workspace/src
~/megoldas_ws/src/
```



## Topic kezelés

```
rostopic hz /scan
rostopic echo /scan -n1
rostopic type /scan
```

```
sensor_msgs/LaserScan
```

## Robotplatform nyelvi beállításai
```
sudo dpkg-reconfigure locales
```

![](etc/locales.png)

`reboot`

## Rosbag kezelése
```
cd ~/rosbags
rosbag record -a -o teszt1
```
```
rsync -avzh --progress wheeltec@192.168.0.100:/home/wheeltec/rosbags/ /mnt/c/bag/wheeltec/
rosbag info teszt1_2023-03-30-12-37-22.bag
rosbag play teszt1_2023-03-30-12-37-22.bag
```



- [További magyarázat](https://github.com/robotverseny/megoldas_zala23/blob/main/etc/explain.ipynb)
- [Verseny kezdőoldal](https://robotverseny.github.io/)

