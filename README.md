1. cartographer를 통해 pbstream으로 매핑을 한다.
2. QR코드의 절대 좌표를 구한다.
	- turtlebot3_navigation2을 켜서 amcl_pose 값의 Position값과 Orientation값을 통해 로봇의 절대 좌표를 가져온다.
	-로봇의 orientation을 euler angle로 변환한다.
	- webcam과 2D_LiDAR sensor fusion 코드에서 QR코드를 인식했을때의 angle_min, angle_max, range(거리값) 을 받아온다. 
	- angle_min과 angle_max의 평균 angle_avg를 구해 webcam-> QR 각도를 가져온다.
	- 로봇의 orientation과 angle_avg를 합쳐서 QR 코드의 절대 각도를 계산한다.
	- QR 코드의 절대 좌표 계산하고 계산된 절대 좌표가 txt파일로 저장이 된다. (angle, range, amcl_pose 값이 다 나와야 출력됨)
		- 문제점 : 동시에 여러개를 돌리다보니 amcl_pose와 camera/image가 나오는데 버퍼가 생겨 좌표를 계산할 때 잘못된 값으로 계산된다.
		
		
turtlebot에서 실행시킬 것 : /scan 토픽, /camera/image , QR코드 index 값을 새로운 토픽으로 발행

QR코드 index가 같은것끼리 모아서 일정 오차 범위 내의 값들끼리 평균을 내어 QR 코드 좌표 확정

