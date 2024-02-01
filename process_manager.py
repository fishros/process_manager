import os
import subprocess
import time
import atexit
import rclpy
from rclpy.node import Node
from rcl_interfaces.srv import SetParameters
from rcl_interfaces.msg import Parameter, ParameterValue, ParameterType,SetParametersResult
import psutil  
  
def is_process_running(pid):  
    try:  
        process = psutil.Process(pid)  
        if process.status()==psutil.STATUS_ZOMBIE:
            process.wait()
        return process.status() != psutil.STATUS_DEAD  
    except (psutil.NoSuchProcess, psutil.AccessDenied, psutil.ZombieProcess):  
        return False
  
def kill_process_and_child(pid):  
    try:  
        proc = psutil.Process(pid)  
        print(proc.children())
        for child in proc.children():  
            if len(child.children())==0:
                os.kill(child.pid, 9)
            else:
                kill_process_and_child(child.pid)
        os.kill(pid, 9)
    except Exception as e:  
        print(e)
    
class ProcessManagerNode(Node):
    def __init__(self, file_path):
        super().__init__('process_manager')
        self.file_path = file_path
        self.processes = {}
        self._register_cleanup()
        self.create_timer(5.0,self.process_status_check)
        self.declare_parameter("current_status","")
        
    def _read_process_list(self):
        process_list = {}
        with open(self.file_path, 'r') as file:
            for line in file.readlines():
                pinfo = line.strip().split(",")
                name = pinfo[0]
                command = pinfo[1]
                pname = f'{name}'
                process_list[name] = {
                    'command':command,
                    'process':None,
                    'pname':pname
                }
                if not self.has_parameter(pname):
                    self.declare_parameter(pname,"run")
        return process_list

    def _register_cleanup(self):
        atexit.register(self._cleanup_processes)

    def _cleanup_processes(self):
        for process in self.processes.values():
            try:
                # self.get_logger().info(f"Stop {process.pid}")
                os.kill(process.pid, 9)
            except ProcessLookupError:
                pass

    def process_status_check(self):
        current_status = {}
        process_list = self._read_process_list()
        for name,process_info in process_list.items():
            current_status[name] = self.get_parameter(name).value
            # if name in self.processes.keys():
                # self.get_logger().info(f"is_process_running:{is_process_running(self.processes[name].pid)}-{name}-{self.processes[name].pid}")
            if self.get_parameter(name).value == 'run':
                if name not in self.processes or is_process_running(self.processes[name].pid) is False:
                    self.processes[name] = subprocess.Popen(process_info['command'], shell=True)
                    self.get_logger().info(f"Started process: {name} command={process_info['command']} With ID:{self.processes[name].pid}")
            elif self.get_parameter(name).value == 'stop':
                if name in self.processes and self.processes[name].poll() is None:
                    try:
                        kill_process_and_child(self.processes[name].pid)
                        self.get_logger().info(f"Stop {name}: PID:{ self.processes[name].pid}")
                    except ProcessLookupError:
                        pass
        self.set_parameters([rclpy.Parameter("current_status", rclpy.Parameter.Type.STRING, str(current_status))])

def main(args=None):
    rclpy.init(args=args)
    file_path = "ProcessList.txt"
    process_manager_node = ProcessManagerNode(file_path)
    rclpy.spin(process_manager_node)
    process_manager_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
