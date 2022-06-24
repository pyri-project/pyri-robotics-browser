from typing import List, Dict, Callable, Any
from pyri.webui_browser.plugins.panel import PyriWebUIBrowserPanelBase
from pyri.webui_browser import PyriWebUIBrowser
import importlib_resources
import js
import traceback
from RobotRaconteur.Client import *

from pyri.webui_browser import util
from pyri.webui_browser.util import to_js2
from pyodide import create_proxy, to_js

import numpy as np

def R2rpy(R):
    assert np.linalg.norm(R[0:2,0]) > np.finfo(float).eps * 10.0, "Singular rpy requested"
    
    r=np.arctan2(R[2,1],R[2,2])
    y=np.arctan2(R[1,0], R[0,0])
    p=np.arctan2(-R[2,0], np.linalg.norm(R[2,1:3]))
        
    return (r,p,y)

def q2R(q):
   
    I = np.identity(3)
    qhat = hat(q[1:4])
    qhat2 = qhat.dot(qhat)
    return I + 2*q[0]*qhat + 2*qhat2

def hat(k):
    khat=np.zeros((3,3))
    khat[0,1]=-k[2]
    khat[0,2]=k[1]
    khat[1,0]=k[2]
    khat[1,2]=-k[0]
    khat[2,0]=-k[1]
    khat[2,1]=k[0]    
    return khat

class PyriJogPanel(PyriWebUIBrowserPanelBase):
    def __init__(self, device_manager, core):
        self.vue = None
        self.mousedown = False
        self.device_manager = device_manager
        self.core = core
        self.jog_connected = False
        
    def init_vue(self,vue):
        self.vue = vue
   
    def watch_current_robot_options(self, new_value, *args):
        if new_value.length > 0:
            if getattr(self.vue,"$data").current_robot is None:
                getattr(self.vue,"$data").current_robot = new_value[0].value
        else:
            getattr(self.vue,"$data").current_robot = None

    def joint_state(self):
        
        current_robot = getattr(self.vue,"$data").current_robot
        if current_robot is None:
            return [], []

        ret = []
        ret2 = []
        joint_info = None
        joint_position = None
        try:
            joint_info = self.core.device_infos[current_robot]["extended_info"]["com.robotraconteur.robotics.robot.RobotInfo"].data.joint_info
        except AttributeError:            
            pass            
        except KeyError:            
            pass

        if joint_info is None:
            return [], []

        try:
            e_state = self.core.devices_states.devices_states[current_robot].state
            if e_state is not None:
                for e in e_state:
                    if e.type == "com.robotraconteur.robotics.robot.RobotState":
                        joint_position = e.state_data.data.joint_position
        except AttributeError:
            traceback.print_exc()
        except KeyError:
            traceback.print_exc()        

        for i in range(len(joint_info)):
            v = dict()
            v["joint_number"] = i
            if joint_info is not None:
                v["lower"]= f"{np.rad2deg(joint_info[i].joint_limits.lower):.2f}"
                v["upper"]= f"{np.rad2deg(joint_info[i].joint_limits.upper):.2f}"
            else:
                v["lower"] = "N/A"
                v["upper"] = "N/A"

            if joint_position is not None:
                try:
                    ret2.append(f"{np.rad2deg(joint_position[i]):.2f}")
                except KeyError:
                    ret2.append("N/A")
            else:
                ret2.append("N/A")
            
            ret.append(v)        
        return ret, ret2

    def current_robot_mode(self):
        current_robot = getattr(self.vue,"$data").current_robot
        if current_robot is None:
            return "Invalid"

        mode = "Invalid"

        try:
            e_state = self.core.devices_states.devices_states[current_robot].state
            if e_state is not None:
                for e in e_state:
                    if e.type == "com.robotraconteur.robotics.robot.RobotState":
                        command_mode = e.state_data.data.command_mode

                        if command_mode < 0:
                            mode = "Error"
                        elif command_mode == 0:
                            mode = "Halt"
                        elif command_mode == 1:
                            mode = "Jog"
                        elif command_mode == 2:
                            mode = "Trajectory"
                        elif command_mode == 3:
                            mode = "Position Command"
                        elif command_mode == 4:
                            mode = "Velocity Command"
                        elif command_mode == 5:
                            mode = "Homing"
                        else:
                            mode = f"Unknown ({command_mode})"
        except AttributeError:
            traceback.print_exc()
        except KeyError:
            traceback.print_exc()

        return mode

    def current_robot_connected(self, vue, *args):
        try:
            current_robot = getattr(vue,"$data").current_robot
            return self.core.devices_states.devices_states[current_robot].connected
        except:
            return False

    def current_robot_error(self, vue, *args):
        try:
            current_robot = getattr(vue,"$data").current_robot
            return self.core.devices_states.devices_states[current_robot].error
        except:
            return True

    def current_robot_ready(self, vue, *args):
        try:
            current_robot = getattr(vue,"$data").current_robot
            return self.core.devices_states.devices_states[current_robot].ready
        except:
            return False

    def current_tool_connected(self, vue, *args):
        try:
            current_tool = getattr(vue,"$data").current_tool
            return self.core.devices_states.devices_states[current_tool].connected
        except:
            return False

    def current_tool_error(self, vue, *args):
        try:
            current_tool = getattr(vue,"$data").current_tool
            return self.core.devices_states.devices_states[current_tool].error
        except:
            return True

    def current_tool_ready(self, vue, *args):
        try:
            current_tool = getattr(vue,"$data").current_tool
            return self.core.devices_states.devices_states[current_tool].ready
        except:
            return False

    async def get_jog(self):
        
        #TODO: Fix connect_device("robotics_jog")
        if not self.jog_connected:
            self.device_manager.connect_device("robotics_jog")            
            self.jog_connected = True
            
        current_robot = getattr(self.vue,"$data").current_robot
        if current_robot is None:
            return None, None
        try:
            jog_service = await self.device_manager.get_device_subscription("robotics_jog").AsyncGetDefaultClient(None,timeout=1)
            jog =  await jog_service.async_get_jog(current_robot,None)
            return jog
        except:
            traceback.print_exc()
            return None

    def jog_joints(self,q_i, sign):
        # @burakaksoy RR-Client-WebBrowser-Robot.py:380
        self.core.loop.create_task(self.async_jog_joints(q_i, sign))

    async def async_jog_joints(self, q_i, sign):
        try:
            # @burakaksoy RR-Client-WebBrowser-Robot.py:391
            jog = await self.get_jog()
            while (self.mousedown): 
                # Call Jog Joint Space Service funtion to handle this jogging
                # await plugin_jogJointSpace.async_jog_joints2(q_i, sign, None)
                speed_perc = float(getattr(self.vue,"$data").selected_jog_speed)
                await jog.async_jog_joints(q_i, sign, speed_perc, None)
                await RRN.AsyncSleep(0.01,None)

            #await plugin_jogJointSpace.async_stop_joints(None)
        except:
            traceback.print_exc()
        
    def jog_decrement_mousedown(self, joint_index):
        self.jog_joints(joint_index+1,-1)

    def jog_increment_mousedown(self, joint_index):
        self.jog_joints(joint_index+1,+1)

    async def do_set_jog_mode(self):
        try:
            jog = await self.get_jog()
            if jog is None:
                return
            await jog.async_setf_jog_mode(None)
        except:
            traceback.print_exc()

    def set_jog_mode(self, evt):
        self.core.create_task(self.do_set_jog_mode())

    def set_halt_mode(self, evt):
        self.core.create_task(self.do_set_halt_mode())
    
    async def do_set_halt_mode(self):
        try:
            jog = await self.get_jog()
            if jog is None:
                return
            await jog.async_setf_halt_mode(None)
        except:
            traceback.print_exc()

    def mousedown_evt(self,evt):
        self.mousedown = True

    def mouseup_evt(self,evt):
        self.mousedown = False

    def mouseleave_evt(self,evt):
        self.mousedown = False
        
    def jog_cart_decrement_mousedown(self, index):
        #self.jog_joints(joint_index+1,-1)
        if index == 0:
            self.jog_cartesian(np.array(([-1.,0.,0.])), np.array(([0.,0.,0.])))
            return
        if index == 1:
            self.jog_cartesian(np.array(([0.,-1.,0.])), np.array(([0.,0.,0.])))
            return
        if index == 2:
            self.jog_cartesian(np.array(([0.,0.,-1.])), np.array(([0.,0.,0.])))
            return
        if index == 3:
            self.jog_cartesian(np.array(([0.,0.,0.])), np.array(([-1.,0.,0.])))
            return
        if index == 4:
            self.jog_cartesian(np.array(([0.,0.,0.])), np.array(([0.,-1.,0.])))
            return
        if index == 5:
            self.jog_cartesian(np.array(([0.,0.,0.])), np.array(([0.,0.,-1.])))
            return

    def jog_cart_increment_mousedown(self, index):
        #self.jog_joints(joint_index+1,+1)
        if index == 0:
            self.jog_cartesian(np.array(([+1.,0.,0.])), np.array(([0.,0.,0.])))
            return
        if index == 1:
            self.jog_cartesian(np.array(([0.,+1.,0.])), np.array(([0.,0.,0.])))
            return
        if index == 2:
            self.jog_cartesian(np.array(([0.,0.,+1.])), np.array(([0.,0.,0.])))
            return
        if index == 3:
            self.jog_cartesian(np.array(([0.,0.,0.])), np.array(([+1.,0.,0.])))
            return
        if index == 4:
            self.jog_cartesian(np.array(([0.,0.,0.])), np.array(([0.,+1.,0.])))
            return
        if index == 5:
            self.jog_cartesian(np.array(([0.,0.,0.])), np.array(([0.,0.,+1.])))
            return

    def jog_cartesian(self,P_axis, R_axis):
        # @burakaksoy RR-Client-WebBrowser-Robot.py:508
        self.core.loop.create_task(self.async_jog_cartesian(P_axis, R_axis))

    async def async_jog_cartesian(self, P_axis, R_axis):
        # @burakaksoy RR-Client-WebBrowser-Robot.py:520
        try:
            jog = await self.get_jog()
            #await jog.async_prepare_jog(None)
                            
            while (self.mousedown):
                # Call Jog Cartesian Space Service funtion to handle this jogging
                # await plugin_jogCartesianSpace.async_jog_cartesian(P_axis, R_axis, None)
                spatial_velocity_dtype = RRN.GetNamedArrayDType("com.robotraconteur.geometry.SpatialVelocity",jog)
                vel = RRN.ArrayToNamedArray(np.concatenate((R_axis,P_axis)),spatial_velocity_dtype)
                speed_perc = float(getattr(self.vue,"$data").selected_jog_speed)
                await jog.async_jog_cartesian(vel, speed_perc, "robot", None)
                await RRN.AsyncSleep(0.01,None)

            #await plugin_jogCartesianSpace.async_stop_joints(None)
        except:
            traceback.print_exc()
        
    def cur_ZYX_angles(self):
        current_robot = getattr(self.vue,"$data").current_robot
        if current_robot is None:
            return ""

        current_rpy = ""
        try:
            e_state = self.core.devices_states.devices_states[current_robot].state
            if e_state is not None:
                for e in e_state:
                    if e.type == "com.robotraconteur.robotics.robot.RobotState":
                        
                        current_rpy = np.array2string(np.rad2deg(R2rpy(q2R(np.array([e.state_data.kin_chain_tcp[0][0][x] for x in range(4)],dtype=np.float64)))),formatter={'float_kind':lambda x: "%.2f" % x})
        except AttributeError:
            #traceback.print_exc()
            pass
        except KeyError:
            #traceback.print_exc()
            pass

        return current_rpy

    def cur_position(self):

        current_robot = getattr(self.vue,"$data").current_robot
        if current_robot is None:
            return ""

        current_position = ""
        try:
            e_state =self.core.devices_states.devices_states[current_robot].state
            if e_state is not None:
                for e in e_state:
                    if e.type == "com.robotraconteur.robotics.robot.RobotState":
                        current_position = np.array2string(np.array([e.state_data.kin_chain_tcp[0][1][x] for x in range(3)],dtype=np.float64), formatter={'float_kind':lambda x: "%.2f" % x})
        except AttributeError:
            #traceback.print_exc()
            pass
        except KeyError:
            #traceback.print_exc()
            pass

        return current_position

    async def do_move_to_standby(self,joint_angles):
        try:
            jog = await self.get_jog()
            speed_perc = float(getattr(self.vue,"$data").selected_jog_speed)
            await jog.async_jog_joints_to_angles(joint_angles,speed_perc,None)
        except:
            traceback.print_exc()

    def get_target_joint_angles(self):
        try:
            current_robot = getattr(self.vue,"$data").current_robot
            joint_info = self.core.device_infos[current_robot]["extended_info"]["com.robotraconteur.robotics.robot.RobotInfo"].data.joint_info
        except:
            traceback.print_exc()
            js.alert("Robot not selected!")
            return None

        n_joints = len(joint_info)
        target_angles = [0.0]*n_joints
        try:
            target_angles1 = getattr(self.vue,"$data").joint_standby
            for i in range(len(target_angles1)):
                target_angles[i] = target_angles1[i]

            target_angles = np.deg2rad(np.array(target_angles))
        except:
            traceback.print_exc()
            js.alert("Invalid joint angle entries")
            return
        return target_angles

    def set_standby_joint_angles(self,target_angles):
        # TODO: Verify the length and boards of target_angles
        try:
            for i in range(len(target_angles)):
                js.jQuery.find(f"#standby_j{i}_angle_in")[0].value = f"{target_angles[i]:.4f}"
            # getattr(self.vue,"$data").joint_standby = to_js2(list(target_angles))
        except:
            traceback.print_exc()
            js.alert("Invalid joint angle entries")
            return
        return target_angles

    def joint_jog_standby_show(self, *args):
        try:
            standby = getattr(self.vue,"$data").joint_standby
            for i in range(len(standby)):
                js.jQuery.find(f"#standby_j{i}_angle_in")[0].value = f"{standby[i]:.4f}"            
        except:
            traceback.print_exc()

    def current_edit_standby(self):
        joint_state = getattr(self.vue,"$data").joint_state
        current_standby = [0.0]*len(joint_state)
        for i in range(len(joint_state)):
            current_standby[i] = float(js.jQuery.find(f"#standby_j{i}_angle_in")[0].value)
        return current_standby

    def joint_jog_standby_ok(self, *args):
        try:
            current_standby = self.current_edit_standby()
            getattr(self.vue,"$data").joint_standby = to_js2(list(current_standby))
        except:
            traceback.print_exc()

    def move_to_standby(self,evt):
        target_angles = self.get_target_joint_angles()
        if target_angles is None:
            return

        self.core.create_task(self.do_move_to_standby(target_angles))

    async def do_load_joint_pose(self):
        try:
            selected_pose = getattr(self.vue,"$data").load_joint_pose_selected
            var_storage = self.device_manager.get_device_subscription("variable_storage").GetDefaultClient()
            joint_pose = await var_storage.async_getf_variable_value("globals",selected_pose,None)
            self.set_standby_joint_angles(list(joint_pose.data))
            
        except:
            traceback.print_exc()
            js.alert(f"Refresh joint pose failed:\n\n{traceback.format_exc()}")

    def load_joint_pose(self,evt):
        self.core.create_task(self.do_load_joint_pose())

    def set_standby_current(self,*args):
        current_robot = getattr(self.vue,"$data").current_robot
        joint_angles = None
        e_state = self.core.devices_states.devices_states[current_robot].state
        if e_state is not None:
            for e in e_state:
                if e.type == "com.robotraconteur.robotics.robot.RobotState":
                    joint_angles = np.rad2deg([j for j in e.state_data.data.joint_position])
        if joint_angles is None:
            js.alert("Could not determine robot pose")
            return
        self.set_standby_joint_angles(joint_angles)
        
    async def do_refresh_joint_pose_options(self):
        try:
            var_storage = self.device_manager.get_device_subscription("variable_storage").GetDefaultClient()
            joint_pose_names = await var_storage.async_filter_variables("globals",".*",["joint_pose"],None)
            getattr(self.vue,"$data").load_joint_pose_options = to_js2(joint_pose_names)
        except:
            traceback.print_exc()
            js.alert(f"Refresh joint pose list failed:\n\n{traceback.format_exc()}")

    def refresh_joint_pose_options(self,evt):
        self.core.create_task(self.do_refresh_joint_pose_options())

    async def do_save_standby(self, name, joint_angles):
        try:
            var_storage = self.device_manager.get_device_subscription("variable_storage").GetDefaultClient()
            var_consts = RRN.GetConstants('tech.pyri.variable_storage', var_storage)
            variable_persistence = var_consts["VariablePersistence"]
            variable_protection_level = var_consts["VariableProtectionLevel"]
                                    
            await var_storage.async_add_variable2("globals", name ,"double[]", \
                RR.VarValue(np.array(joint_angles,dtype=np.float64),"double[]"), ['joint_pose'], {}, variable_persistence["const"], None, variable_protection_level["read_write"], \
                    [], "Saved joint pose", False, None)
        except:
            traceback.print_exc()
            js.alert(f"Save joint pose failed:\n\n{traceback.format_exc()}")

    def save_standby(self,evt):
        
        joint_pose_name = js.prompt("Joint Pose Name")
        joint_angles = None
        try:
            joint_angles = self.current_edit_standby()
        except:
            traceback.print_exc()
        if joint_angles is None or len(joint_angles) == 0:
            js.alert("Joint position standby not set!")
            return
        self.core.create_task(self.do_save_standby(joint_pose_name,joint_angles))

    async def do_delete_joint_pose(self):
        try:
            selected_pose = getattr(self.vue,"$data").load_joint_pose_selected
            var_storage = self.device_manager.get_device_subscription("variable_storage").GetDefaultClient()
            await var_storage.async_delete_variable("globals",selected_pose,None)
            
        except:
            traceback.print_exc()
            js.alert(f"Delete joint pose failed:\n\n{traceback.format_exc()}")

    def delete_joint_pose(self,evt):
        self.core.create_task(self.do_delete_joint_pose())

    def watch_current_tool_options(self, new_value, *args):
        if new_value.length > 0:
            if getattr(self.vue,"$data").current_tool is None:
                getattr(self.vue,"$data").current_tool = new_value[0].value
        else:
            getattr(self.vue,"$data").current_tool = None

    async def get_tool(self):
        
        #TODO: Fix connect_device("jog_joint")
        if not self.jog_connected:
            self.device_manager.connect_device("robotics_jog")            
            self.jog_connected = True
            
        current_tool = getattr(self.vue,"$data").current_tool
        if current_tool is None:
            return None
        try:
            jog_service = await self.device_manager.get_device_subscription("robotics_jog").AsyncGetDefaultClient(None,timeout=1)
        except:
            return None
        return  await jog_service.async_get_tool(current_tool,None)        

    def tool_open(self):
        self.core.loop.create_task(self.async_tool_open())

    async def async_tool_open(self):
        try:
            
            tool = await self.get_tool()
            await tool.async_open(None)            
        except:
            traceback.print_exc()

    def tool_close(self):
        self.core.loop.create_task(self.async_tool_close())

    async def async_tool_close(self):
        try:
            
            tool = await self.get_tool()
            await tool.async_close(None)            
        except:
            traceback.print_exc()

    async def async_selected_joystick_enable_changed(self,e):
        try:

            if e == "disable":
                jog = await self.get_jog()
                await jog.async_disable_jog_joints_joystick(None)
                return
            if e == "group1":
                group = 0,
            elif e == "group2":
                group = 1
            elif e == "cartesian":
                group = 1000
            else:
                assert False, "Invalid joint jog group"
            
            jog = await self.get_jog()
            while getattr(self.vue,"$data").selected_joystick_enable == e: 
                # Call Jog Joint Space Service funtion to handle this jogging
                # await plugin_jogJointSpace.async_jog_joints2(q_i, sign, None)
                speed_perc = float(getattr(self.vue,"$data").selected_jog_speed)
                if e != "cartesian":
                    await jog.async_enable_jog_joints_joystick(group, speed_perc, None)
                else:
                    await jog.async_enable_jog_cartesian_joystick(speed_perc, "robot", None)
                await RRN.AsyncSleep(0.01,None)

            #await plugin_jogJointSpace.async_stop_joints(None)
        except:
            getattr(self.vue,"$data").selected_joystick_enable = "disable"
            traceback.print_exc()

    def selected_joystick_enable_changed(self,e):
        self.core.loop.create_task(self.async_selected_joystick_enable_changed(e))

    def current_robot_changed(self,e):
        getattr(self.vue,"$data").selected_joystick_enable = "disable"
        getattr(self.vue,"$data").joint_standby = []

    def joint_standby_disp(self, j):        
        try:
            p = getattr(self.vue,"$data").joint_standby[j]            
            return f"{p:.2f}"
        except:
            return ""

    async def run(self):
             
        last_robots = set()
        last_tools = set()
        last_joint_state = []

        joint_pos_els = []
        joint_pos_els2 = []
        
        while True:
            try:
                devices_states = self.core.devices_states
                
                new_robots = util.get_devices_with_type(self.core,"com.robotraconteur.robotics.robot.Robot")
                new_tools = util.get_devices_with_type(self.core,"com.robotraconteur.robotics.tool.Tool")
                if set(new_robots) != last_robots:
                    getattr(self.vue,"$data").current_robot_options = to_js2(util.device_names_to_dropdown_options(new_robots))                    
                    last_robots = set(new_robots)

                if set(new_tools) != last_tools:
                    getattr(self.vue,"$data").current_tool_options = to_js2(util.device_names_to_dropdown_options(new_tools))
                    last_tools = set(new_tools)
                
                try:
                    getattr(self.vue,"$data").current_robot_status = util.device_status_name(devices_states,getattr(self.vue,"$data").current_robot)
                    getattr(self.vue,"$data").current_robot_mode = self.current_robot_mode()
                except:
                    traceback.print_exc()

                try:
                    getattr(self.vue,"$data").current_tool_status = util.device_status_name(devices_states,getattr(self.vue,"$data").current_tool)
                except:
                    traceback.print_exc()

                
                try:
                    joint_state, joint_pos = self.joint_state()
                    if last_joint_state != joint_state:
                        getattr(self.vue,"$data").joint_state = to_js2(joint_state)
                        last_joint_state = joint_state
                        joint_pos_els = []
                    
                    if len(joint_pos_els) != len(joint_pos):
                        for i in range(len(joint_pos)):
                            jpos_el = js.document.getElementById(f"joint_pos_{i}")
                            if jpos_el is None:
                                joint_pos_els = []
                                break
                            joint_pos_els.append(jpos_el)

                    if getattr(self.vue,"$data").joint_jog_standby_visible:

                        if len(joint_pos_els2) != len(joint_pos):
                            for i in range(len(joint_pos)):
                                jpos_el2 = js.document.getElementById(f"standby_joint_pos_{i}")
                                if jpos_el2 is None:
                                    joint_pos_els2 = []
                                    break                            
                                joint_pos_els2.append(jpos_el2)
                        else:
                            joint_pos_els2 = []
                    
                    if joint_pos_els is not None:
                        for i in range(len(joint_pos_els)):
                            joint_pos_els[i].textContent = joint_pos[i]
                    if joint_pos_els2 is not None:
                        for i in range(len(joint_pos_els2)):
                            joint_pos_els2[i].textContent = joint_pos[i]
                    
                except:
                    traceback.print_exc()

                try:
                    getattr(self.vue,"$data").cur_ZYX_angles = self.cur_ZYX_angles()
                    getattr(self.vue,"$data").cur_position = self.cur_position()
                except:
                    traceback.print_exc()

            except:
                traceback.print_exc()
            

            #getattr(self.vue,"$forceUpdate")()
            await RRN.AsyncSleep(0.1,None)



async def add_jog_panel(panel_type: str, core: PyriWebUIBrowser, parent_element: Any):

    assert panel_type == "jog"

    
    jog_panel_html = importlib_resources.read_text(__package__,"jog_panel.html")

    panel_config = {
        "type": "component",
        "componentName": "jog",
        "componentState": {},
        "title": "Jogging",
        "id": "jog",
        "isClosable": False
    }

    gl = core.layout.layout

    def register_jog_panel(container, state):
        container.getElement().html(jog_panel_html)

    core.layout.register_component("jog",register_jog_panel)

    core.layout.add_panel(panel_config)

    core.layout.add_panel_menu_item("jog", "Jogging")

    jog_panel_obj = PyriJogPanel(core.device_manager,core)

    jog_panel = js.Vue.new(to_js2({
        "el": "#jog_panel_component",
        "data":
        {
            "current_robot": None,
            "current_tool": None,
            "current_robot_options": [],
            "current_robot_status": "disconnected",
            "current_tool_options": [],
            "current_tool_status": "disconnected",
            "current_robot_mode": "Invalid",
            "load_joint_pose_selected": "",
            "load_joint_pose_options": [],
            "selected_jog_speed": 10,
            "selected_joystick_enable": "disable",
            "joint_state": [],
            "cur_ZYX_angles": "",
            "cur_position": "jog_panel_obj.cur_position",
            "joint_pos": [],
            "joint_standby": [],
            "joint_jog_standby_visible": False

        },
        "methods":
        {
            "current_robot_changed": jog_panel_obj.current_robot_changed,
            "jog_decrement_mousedown": jog_panel_obj.jog_decrement_mousedown,
            "jog_increment_mousedown": jog_panel_obj.jog_increment_mousedown,
            "set_jog_mode": jog_panel_obj.set_jog_mode,
            "set_halt_mode": jog_panel_obj.set_halt_mode,
            "mousedown": jog_panel_obj.mousedown_evt,
            "mouseup": jog_panel_obj.mouseup_evt,
            "mouseleave": jog_panel_obj.mouseleave_evt,
            "jog_cart_decrement_mousedown": jog_panel_obj.jog_cart_decrement_mousedown,
            "jog_cart_increment_mousedown": jog_panel_obj.jog_cart_increment_mousedown,
            "move_to_standby": jog_panel_obj.move_to_standby,
            "load_joint_pose": jog_panel_obj.load_joint_pose,
            "set_standby_current": jog_panel_obj.set_standby_current,
            "refresh_joint_pose_options": jog_panel_obj.refresh_joint_pose_options,
            "save_standby": jog_panel_obj.save_standby,
            "delete_joint_pose": jog_panel_obj.delete_joint_pose,
            "tool_open": jog_panel_obj.tool_open,
            "tool_close": jog_panel_obj.tool_close,
            "selected_joystick_enable_changed": jog_panel_obj.selected_joystick_enable_changed,
            "joint_standby_disp": jog_panel_obj.joint_standby_disp,
            "joint_jog_standby_show": jog_panel_obj.joint_jog_standby_show,
            "joint_jog_standby_ok": jog_panel_obj.joint_jog_standby_ok

        },
        "watch":
        {
            "current_robot_options": jog_panel_obj.watch_current_robot_options,
            "current_tool_options": jog_panel_obj.watch_current_tool_options

        }
    }))

    jog_panel_obj.init_vue(jog_panel)

    core.create_task(jog_panel_obj.run())

    return jog_panel_obj