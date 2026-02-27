#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import math
import argparse
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from std_msgs.msg import String

# ===================================================================
# ===== CLASES NODE Y BVHREADER =====================================
# ===================================================================

class Node:
    def __init__(self, root=False):
        self.name = None
        self.channels = []
        self.offset = (0,0,0)
        self.children = []
        self._is_root = root

    def isRoot(self):
        return self._is_root

    def isEndSite(self):
        return len(self.children)==0

class BVHReader:
    def __init__(self, filename):
        self.filename = filename
        self.tokenlist = []
        self.linenr = 0
        self._root = None
        self._nodestack = []
        self._numchannels = 0
        self.status_pub = rospy.Publisher('/animation/status', String, queue_size=1, latch=True)
        
        # --- ELIMINADO ---
        # self.tick_pub = rospy.Publisher('/animation/tick', Empty, queue_size=10)

    def onHierarchy(self, root):
        pass

    def onMotion(self, frames, dt):
        pass

    def onFrame(self, values):
        pass

    def read(self):
        try: #Python 2
            self.fhandle = file(self.filename)
        except NameError: #Python 3
            self.fhandle = open(self.filename)

        self.readHierarchy()
        self.onHierarchy(self._root)
        self.readMotion()

    def readMotion(self):
        try:
            tok = self.token()
        except StopIteration:
            return
        
        if tok!="MOTION":
            raise SyntaxError("Syntax error in line %d: 'MOTION' expected, got '%s' instead"%(self.linenr, tok))

        tok = self.token()
        if tok!="Frames:":
            raise SyntaxError("Syntax error in line %d: 'Frames:' expected, got '%s' instead"%(self.linenr, tok))
        frames = self.intToken()

        tok = self.token()
        if tok!="Frame":
            raise SyntaxError("Syntax error in line %d: 'Frame Time:' expected, got '%s' instead"%(self.linenr, tok))
        tok = self.token()
        if tok!="Time:":
            raise SyntaxError("Syntax error in line %d: 'Frame Time:' expected, got 'Frame %s' instead"%(self.linenr, tok))
        dt = self.floatToken()
        self.onMotion(frames, dt)

        for i in range(frames):
            s = self.readLine()
            a = s.split()
            if len(a)!=self._numchannels:
                raise SyntaxError("Syntax error in line %d: %d float values expected, got %d instead"%(self.linenr, self._numchannels, len(a)))
            values = map(lambda x: float(x), a)
            self.onFrame(values)

    def readHierarchy(self):
        tok = self.token()
        if tok!="HIERARCHY":
            raise SyntaxError("Syntax error in line %d: 'HIERARCHY' expected, got '%s' instead"%(self.linenr, tok))

        tok = self.token()
        if tok!="ROOT":
            raise SyntaxError("Syntax error in line %d: 'ROOT' expected, got '%s' instead"%(self.linenr, tok))

        self._root = Node(root=True)
        self._nodestack.append(self._root)
        self.readNode()

    def readNode(self):
        name = self.token()
        self._nodestack[-1].name = name
        
        tok = self.token()
        if tok!="{":
            raise SyntaxError("Syntax error in line %d: '{' expected, got '%s' instead"%(self.linenr, tok))
    
        while 1:
            tok = self.token()
            if tok=="OFFSET":
                x = self.floatToken()
                y = self.floatToken()
                z = self.floatToken()
                self._nodestack[-1].offset = (x,y,z)
            elif tok=="CHANNELS":
                n = self.intToken()
                channels = []
                for i in range(n): 
                    tok = self.token()
                    if tok not in ["Xposition", "Yposition", "Zposition", "Xrotation", "Yrotation", "Zrotation"]:
                        raise SyntaxError("Syntax error in line %d: Invalid channel name: '%s'"%(self.linenr, tok))
                    channels.append(tok)
                self._numchannels += len(channels)
                self._nodestack[-1].channels = channels
            elif tok=="JOINT":
                node = Node()
                self._nodestack[-1].children.append(node)
                self._nodestack.append(node)
                self.readNode()
            elif tok=="End":
                node = Node()
                self._nodestack[-1].children.append(node)
                self._nodestack.append(node)
                self.readNode()
            elif tok=="}":
                if self._nodestack[-1].isEndSite():
                    self._nodestack[-1].name = "End Site"
                self._nodestack.pop()
                break
            else:
                raise SyntaxError("Syntax error in line %d: Unknown keyword '%s'"%(self.linenr, tok))

    def intToken(self):
        tok = self.token()
        try:
            return int(tok)
        except ValueError:
            raise SyntaxError("Syntax error in line %d: Integer expected, got '%s' instead"%(self.linenr, tok))

    def floatToken(self):
        tok = self.token()
        try:
            return float(tok)
        except ValueError:
            raise SyntaxError("Syntax error in line %d: Float expected, got '%s' instead"%(self.linenr, tok))

    def token(self):
        if self.tokenlist!=[]:
            tok = self.tokenlist[0]
            self.tokenlist = self.tokenlist[1:]
            return tok

        s = self.readLine()
        self.createTokens(s)
        return self.token()

    def readLine(self):
        self.tokenlist = []
        while 1:
            s = self.fhandle.readline()
            self.linenr += 1
            if s=="":
                raise StopIteration
            return s

    def createTokens(self, s):
        s = s.strip()
        a = s.split()
        self.tokenlist = a

# ===================================================================

class BvhToPepper(BVHReader):
    def __init__(self, filename):
        BVHReader.__init__(self, filename)
        
        self.right_arm_pub = rospy.Publisher('/pepper/RightArm_controller/command', JointTrajectory, queue_size=1)
        self.left_arm_pub = rospy.Publisher('/pepper/LeftArm_controller/command', JointTrajectory, queue_size=1)
        self.head_pub = rospy.Publisher('/pepper/Head_controller/command', JointTrajectory, queue_size=1)
        self.pelvis_pub = rospy.Publisher('/pepper/Pelvis_controller/command', JointTrajectory, queue_size=1)
        
        self.right_arm_joint_names = ['RShoulderPitch', 'RShoulderRoll', 'RElbowYaw', 'RElbowRoll', 'RWristYaw']
        self.left_arm_joint_names = ['LShoulderPitch', 'LShoulderRoll', 'LElbowYaw', 'LElbowRoll', 'LWristYaw']
        self.head_joint_names = ['HeadPitch', 'HeadYaw']
        self.pelvis_joint_names = ['HipPitch', 'HipRoll', 'KneePitch']

        self.bvh_indices_map = {
            'RShoulderPitch': 179, 'RShoulderRoll': 179, 'RElbowYaw': 184, 'RElbowRoll': 189,
            'LShoulderPitch': 321, 'LShoulderRoll': 323, 'LElbowYaw': 328, 'LElbowRoll': 333,
            'HeadPitch': 46, 'HeadYaw': 47,
            'HipPitch': 22,
            'HipRoll': 21,
            'KneePitch': 10,
        }
        
        self.all_motions = []
        self.dt = 1.0 / 30.0

    def onHierarchy(self, root): rospy.loginfo("Jerarquía leída.")
    def onMotion(self, frames, dt): self.num_motions, self.dt = frames, dt
    def onFrame(self, values): self.all_motions.append(values)

    def start_retargeting(self, loop=False):
        self.read()
        rospy.loginfo("Iniciando retargeting...")

        rospy.loginfo("Enviando señal de inicio...")
        self.status_pub.publish("START")
        rospy.sleep(0.5)

        rate = rospy.Rate(1/self.dt)
        
        while not rospy.is_shutdown():
            for frame_values in self.all_motions:
                if rospy.is_shutdown(): break

                positions_map = {}
                for joint_name, bvh_index in self.bvh_indices_map.items():
                    positions_map[joint_name] = math.radians(frame_values[bvh_index])

                pitch_r = (positions_map.get('RShoulderPitch', 0.0) * 0.5) * -1.0 + (math.pi/2)
                roll_r = (positions_map.get('RShoulderRoll', 0.0) * 0.5) * -1.0
                elbow_yaw_r = 1.57
                elbow_roll_r = positions_map.get('RElbowRoll', 0.0) * -1.0
                
                pitch_l = ((positions_map.get('LShoulderPitch', 0.0) * 0.5) * 1.0) + (math.pi/2)
                roll_l = (positions_map.get('LShoulderRoll', 0.0) * 0.5) * -1.0
                elbow_yaw_l = -1.57
                elbow_roll_l = positions_map.get('LElbowRoll', 0.0) * -1.0

                head_pitch = positions_map.get('HeadPitch', 0.0) * 0.5
                head_yaw = positions_map.get('HeadYaw', 0.0) * 0.5
                
                hip_pitch_raw = positions_map.get('HipPitch', 0.0)
                hip_roll_raw = positions_map.get('HipRoll', 0.0)
                knee_pitch_raw = positions_map.get('KneePitch', 0.0)
                
                hip_pitch = hip_pitch_raw * 0.5
                hip_roll = hip_roll_raw * 0.5
                knee_pitch = knee_pitch_raw * 0.5
                
                final_r_positions = [pitch_r, roll_r, elbow_yaw_r, elbow_roll_r, 0.0]
                final_l_positions = [pitch_l, roll_l, elbow_yaw_l, elbow_roll_l, 0.0]
                final_h_positions = [head_pitch, head_yaw]
                final_p_positions = [hip_pitch, hip_roll, knee_pitch]

                traj_r = JointTrajectory(); traj_r.header.stamp = rospy.Time.now(); traj_r.joint_names = self.right_arm_joint_names; traj_r.points.append(JointTrajectoryPoint(positions=final_r_positions, time_from_start=rospy.Duration(self.dt)))
                traj_l = JointTrajectory(); traj_l.header.stamp = rospy.Time.now(); traj_l.joint_names = self.left_arm_joint_names; traj_l.points.append(JointTrajectoryPoint(positions=final_l_positions, time_from_start=rospy.Duration(self.dt)))
                traj_h = JointTrajectory(); traj_h.header.stamp = rospy.Time.now(); traj_h.joint_names = self.head_joint_names; traj_h.points.append(JointTrajectoryPoint(positions=final_h_positions, time_from_start=rospy.Duration(self.dt)))
                traj_p = JointTrajectory(); traj_p.header.stamp = rospy.Time.now(); traj_p.joint_names = self.pelvis_joint_names; traj_p.points.append(JointTrajectoryPoint(positions=final_p_positions, time_from_start=rospy.Duration(self.dt)))
                
                self.right_arm_pub.publish(traj_r)
                self.left_arm_pub.publish(traj_l)
                self.head_pub.publish(traj_h)
                self.pelvis_pub.publish(traj_p)
                
                # --- ELIMINADO: Ya no enviamos el pulso de sincronización ---
                # self.tick_pub.publish(Empty())

                rate.sleep()
            
            if not loop: break
            rospy.loginfo("Reiniciando animación.")

        rospy.loginfo("Animación completada. Enviando señal de STOP.")
        self.status_pub.publish("STOP")
        rospy.sleep(0.5)

def main():
    parser = argparse.ArgumentParser(description="Retarget BVH motion to Pepper robot.")
    parser.add_argument('bvh_file', help="Path to the BVH file.")
    parser.add_argument('-l', '--loop', action="store_true", help="Loop the animation.")
    args = parser.parse_args()
    rospy.init_node("bvh_to_pepper_retargeter")
    retargeter = BvhToPepper(args.bvh_file)
    retargeter.start_retargeting(loop=args.loop)

if __name__ == "__main__":
    main()