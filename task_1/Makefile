# Author: Damian Sova
# Date: 25.9.2022

TASKS = tickets sky_links holidays wander company communication distribution travel
TASK_NUM=0

all:
	for task in $(TASKS) ; do \
		((TASK_NUM = TASK_NUM + 1)) ; \
		echo "python3.6 src/task$$TASK_NUM.py" > $$task; \
		chmod +x $$task; \
	done


# all:
# 	echo "python3.6 src/task1.py" > tickets
# 	chmod +x tickets
# 	echo "python3.6 src/task2.py" > sky_links
# 	chmod +x sky_links
# 	echo "python3.6 src/task3.py" > holidays
# 	chmod +x holidays
# 	echo "python3.6 src/task4.py" > wander
# 	chmod +x wander
# 	echo "python3.6 src/task5.py" > company
# 	chmod +x company
# 	echo "python3.6 src/task6.py" > communication
# 	chmod +x communication
# 	echo "python3.6 src/task7.py" > distribution
# 	chmod +x distribution
# 	echo "python3.6 src/task8.py" > travel
# 	chmod +x travel