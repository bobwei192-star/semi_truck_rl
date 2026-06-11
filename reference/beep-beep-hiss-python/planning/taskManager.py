class TaskManager:
    def __init__(self):
        self.taskType = 1
        self.taskStatus = 0  # 1 on, 0 done
    
    def set_task_type(self, tasktype):
        if tasktype == 1:
            self.taskType = 1
        elif tasktype == -1:
            self.taskType = -1
        else:
            raise ValueError('wrong task type')
    
    def update_task_status(self):
        pass


if __name__ == "__main__":
    # Test the class
    tm = TaskManager()
    print(f"Initial task type: {tm.taskType}")
    
    tm.set_task_type(-1)
    print(f"Task type after setting to -1: {tm.taskType}")
