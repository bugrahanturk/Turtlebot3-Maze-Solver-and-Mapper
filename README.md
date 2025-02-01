# BLM191 Robotlar Ödev 1


## Kurulum Derleme
- Öncelikle bir catkin çalışma alanı oluşturun ve `src` dizinine geçin
- Repoyu klonlayın:
```
git clone https://gitlab.com/blm6191_2425b/members/23501021/blm6191-robotlar-odev-1.git
```
- Paketi derleme:
```
catkin build
```
- Yeni paketi ROS ortamınıza ekleyin:
```
- source ~/<catkin_ws>/devel/setup.bash
```

## Çalıştırma
Labirent dünyası ve turtlebot3'ü oluşturan paketlerin kurulu olduğu varsayılmaktadır.

- Labirent dünyası çalıştırılır(maze4):
```
roslaunch micromouse_maze micromouse_maze4.launch
```
- my_solver nod'unun çalıştırılması:
```
rosrun solve_maze my_solver
```

- my_mapper nod'unun çalıştırılması:
```
rosrun solve_maze my_mapper
```

## Sonuçlar
### my_solver
- Robot labirent duvarlarını soldan paralel olarak takip edecek şekilde ayarlanmıştır.
- Robotun duvara olan mesafesi **d = 0.5**, Paralelband mesafesi **r = 0.1** olarak ayarlanmıştır.

![](img/DuvaraOlanMesafe.png)

- Robotun Başlangıç Durum Görüntüsü:

![](img/solver_start.png)

- Robotun Merkeze Ulaşmış Görüntüsü:

![](img/solver_finish.png)


### my_mapper
- Harita görüntüsü:

![](img/map.png)

- Gazebo ortamından, rqt_gui kullanılarak yönlendirilen robotun haritalama görüntüsü:

![](img/map_olusturma.png)
