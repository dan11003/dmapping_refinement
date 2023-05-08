### Dmapping refinement

## Calibration

#### Requirements

git@github.com:dan11003/floam.git branch refinement

git@github.com:dan11003/SC-LIO-SAM.git branch refinement

#### Running

roslaunch dmapping_refinement calib.launch

Or simply follow the steps below:



Hej. Nu ska vi kalibrera!

(1) Se till att vi har rätt branches: För dmapping: master, lio_sam: refinement, floam: refinement. Har du itne rätt branch, git checkout <namn på branch>

(2) Kör igång kalibreringen med  roslaunch dmapping_refinement calib.launch
(3) klicka på "calib", då kommer kalibreringsparametrarna upp.
(4) starta bag fil, eller se till att alla sensorer snurrar i ros,
Detta sätter igång en bag fil, kör 10 sekunder, och loopar allt från början rosbag play LIOSAM_test_kali.bag --clock -r1 --duration=10 -l

(5) Nu är det dags att kalibrera.
Punktmolnet är färgkodat. blått = längre upp, grönt = längre ner. Väggarna är raka, därför borde grönt och blått ligga ovanpå varandra. Om de INTE gör det, börja justera parametrarna.
Lite bättre? Nu verkar det vara bättre. Det räcker att justera ex och ey, ez borde vara ungefär 180, vi struntar i t=tiden.
(6) Se till att ta ett screenshot på parametrarna.
(7) Fyll i parametrarna i floam/launch/structor_slam.launch.

Nu har structor_slam de nya parametrarna och borde ge lite bättre prestanda :)


