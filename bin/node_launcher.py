#!/usr/bin/env python

""" 
    A service to run ROS nodes and launch files
  
    Created for the Pi Robot Project: http://www.pirobot.org
    Copyright (c) 2013 Patrick Goebel.  All rights reserved.

    This program is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 2 of the License, or
    (at your option) any later version.
    
    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details at:
    
    http://www.gnu.org/licenses/gpl.html
"""

import rospy
from rbx2_utils.srv import *
import shlex, subprocess
            
class NodeLauncher():
    def __init__(self):
        rospy.init_node("node_launcher")
        
        rospy.on_shutdown(self.shutdown)
        
        # A dictionary to map processes to timestamps.
        self.processes = dict()
        
        # The node launcher service
        rospy.Service('~launch_process', LaunchProcess, self.LaunchProcessHandler)
        
        # Terminate a process
        rospy.Service('~kill_process', KillProcess, self.KillProcessHandler)
        
    def LaunchProcessHandler(self, req):  
        command = req.run_type + ' ' + req.package_name + ' ' + req.filename + ' ' + req.params
        args = shlex.split(command)
        process = subprocess.Popen(args)
        now = rospy.Time.now()
        self.processes[str(now)] = process
        
        return LaunchProcessResponse(str(now))
    
    def KillProcessHandler(self, req):
        try:
            self.processes[req.process_id].poll()
            if not self.processes[req.process_id].poll():
                self.processes[req.process_id].terminate()
                self.processes = removekey(self.processes, req.process_id)
                return KillProcessResponse(True)
        except:
            return KillProcessResponse(False)
        
    def shutdown(self):
        rospy.loginfo("Shutting down node_launcher.")
        for process in self.processes.itervalues():
            process.poll()
            if not process.poll():
                process.terminate()
            
def removekey(d, key):
    r = dict(d)
    del r[key]
    return r
        

if __name__ == '__main__':
    NodeLauncher()
    rospy.spin()
    
    

