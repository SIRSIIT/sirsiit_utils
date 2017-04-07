#!/usr/bin/env python
import sys
import types
import time
import csv
import inspect
from IPython import embed

import rospy
import tf
import tf2_ros
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Pose, PoseStamped, Wrench, WrenchStamped, PoseArray, Vector3
from qb_interface.msg import handRef
from manipulation_msgs.msg import ClusterBoundingBox

class TFPrinter(object):
    def __init__(self):
        rospy.init_node('tf_printer')
        if not rospy.has_param('tfprint'):
            rospy.logerr('Parameters not found')
            exit()

        self.m_setts = rospy.get_param("tfprint")
        self.tfBuffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tfBuffer)
        self.started=False;
        self.subs=[];
        self.others=dict()
        self.poses=dict()
        self.transforms=dict()
        self.wrenches=dict()
        self.t_start=rospy.Time.now()
        self.filename=self.m_setts['file_prefix']+'_'+time.strftime("%Y-%m-%d_%H-%M-%S")+'.csv';
        self.csvfile=open(self.filename,'wb')
        for i in self.m_setts['others']:
            self.others[i['topic']]=None
            self.subs.append(rospy.Subscriber(i['topic'], eval(i['type']), self.update_others,i['topic']))
        for i in self.m_setts['transforms']:
            self.transforms[i]=None
        for i in self.m_setts['poses']:
            self.poses[i]=None
            self.subs.append(rospy.Subscriber(i, PoseStamped, self.update_poses,(i)))
        for i in self.m_setts['wrenches']:
            self.wrenches[i]=None
            self.subs.append(rospy.Subscriber(i, WrenchStamped, self.update_wrench,(i)))
        #embed()
        #self.writer.writeheader()


    def up_and_running(self):
        started=True;
        topic=str()
        for i in self.poses:
            if(self.poses[i]==None):
                topic=i;
                started=False;
        for i in self.transforms:
            if(self.transforms[i]==None):
                topic=i;
                started=False;
        for i in self.wrenches:
            if(self.wrenches[i]==None):
                topic=i;
                started=False;
        for i in self.others:
            if(self.others[i]==None):
                topic=i;
                started=False;
        return started,topic



    def xyz_to_str(self,v):
        return str(v.x) + ',' + str(v.y) + ',' + str(v.z);
    def to_str(self,pin):
        tmp=str()
        vtype=str(pin.__class__.__name__);
        if(vtype != 'tuple'):
            if(vtype=='Wrench'):
                tmp=tmp+self.xyz_to_str(pin.force);
                tmp=tmp+','
                tmp=tmp+self.xyz_to_str(pin.torque);
            elif(vtype=='Pose'):
                tmp=tmp+self.xyz_to_str(pin.position);
                tmp=tmp+','
                tmp=tmp+self.xyz_to_str(pin.orientation);
                tmp=tmp+','+str(pin.orientation.w);
            elif(vtype=='Vector3'):
                tmp=tmp+self.xyz_to_str(pin);
                tmp=tmp+','
        else:
            for i in pin:
                tmp=tmp+','+str(i)
        return tmp


    def update_wrench(self,data,arg):
        self.wrenches[arg]=data.wrench;
    def update_poses(self,data,arg):
        self.poses[arg]=data.pose
    def update_others(self,data,arg):
        for i in self.m_setts['others']:
            if i['type']==data.__class__.__name__:
                self.others[i['topic']]=eval('data'+i['field']);

    def trans_to_pose(self,T):
        p=Pose();
        p.position=T.transform.translation;
        p.orientation=T.transform.rotation;
        return p

    def get_transforms(self):
        for i in self.transforms:
            try:
                trans = self.tfBuffer.lookup_transform(self.m_setts['base_frame'],i, rospy.Time())
                self.transforms[i]=self.trans_to_pose(trans)
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
                rospy.Rate(100).sleep()
                continue

    def get_string(self):
        s=str();
        for i in self.poses:
            s=s+self.to_str(self.poses[i]);
            s=s+','
        for i in self.transforms:
            s=s+self.to_str(self.transforms[i]);
            s=s+','
        for i in self.wrenches:
            s=s+self.to_str(self.wrenches[i]);
            s=s+','
        for i in self.others:
            s=s+self.to_str(self.others[i]);
        return(s)

    def str_add_xyz(self,str_in):
        return(str_in + 'x' + ',' + str_in + 'y' + ',' + str_in + 'z')


    def get_header(self):
        s=str();
        s=s+'Time,'
        for i in self.poses:
            s=s+self.str_add_xyz(str(i)+'_')+',';
            s=s+self.str_add_xyz(str(i)+'_q')+',';
            s=s+str(i)+'_qw'+','
        for i in self.transforms:
            s=s+self.str_add_xyz(str(i)+'_')+',';
            s=s+self.str_add_xyz(str(i)+'_q')+',';
            s=s+str(i)+'_qw'+','
        for i in self.wrenches:
            s=s+self.str_add_xyz(str(i)+'_f')+',';
            s=s+self.str_add_xyz(str(i)+'_m')+',';
        for i in self.others:
            s=s+i+',';
        return(s)


    def run(self,print_file):
        r=rospy.Rate(5)
        self.csvfile.write('%'+self.get_header()+'\n')
        #embed()
        self.started, waiting_topic=self.up_and_running();
        while not self.started:
            self.get_transforms()
            rospy.loginfo_throttle(1,('waiting for '+waiting_topic + ' to become available'))
            self.started, waiting_topic=self.up_and_running();
            rospy.sleep(0.1)

        while not rospy.is_shutdown():
            self.get_transforms()
            self.csvfile.write(str((rospy.Time.now()-self.t_start).to_sec())+','+self.get_string()+'\n')
            r.sleep();
            #embed()

if __name__ == '__main__':
    print_file=True
    if(len(sys.argv)>1):
        if(sys.argv[1]=='-noprint'):
            print_file=False;
            print("Not printing to file")
    tfp= TFPrinter()
    tfp.run(print_file);
