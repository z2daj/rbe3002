In order to impliment this A* star search node:

1. run roscore

2. rosron rviz rviz

3. file -> open Lab3Config.rviz

4. open a map server for example:
	rosrun map_server map_server AK220Map.yaml

5. rosrun cmkeaneLab3 Lab3.py

6. Place start and goal locations using the "2D Pose Estimate" and "2D Nav Goal" buttons



Map Key:

green: 	expanded obsticals
red: 	closelist of nodes
blue: 	frontier nodes
yellow: path
black:	obsticals
