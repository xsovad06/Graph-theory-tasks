# Author: Damian Sova
# Date: 6.10.2022

TASKS = evacuation teams parking
TASK_NUM=0

all:
	for task in $(TASKS) ; do \
		((TASK_NUM = TASK_NUM + 1)) ; \
		echo "python3.6 src/task$$TASK_NUM.py" > $$task; \
		chmod +x $$task; \
	done
