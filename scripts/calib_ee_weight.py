#!/usr/bin/env python
import sys
import types
import time
import csv
import inspect
from IPython import embed
from scipy.optimize import minimize
import numpy as np
import rospy
import tf
import tf2_ros
from geometry_msgs.msg import PoseStamped, WrenchStamped

class CalibEEWeight(object):
    def __init__(self):
        rospy.init_node('calib_weight')
        self.tfBuffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tfBuffer)
        self.robot_base_frame='world'
        self.robot_ee_frame='iiwa_opto_ft_link'        
        #self.E2FT=[[0.0,   1.0,   0.0, 0.0], [0, -0.0, -01.0, -0.036], [-1.0,-0.0,0.0,  0.064], [0,0,0,1]]
        self.E2FT=tf.transformations.identity_matrix()
        self.sub_ft=rospy.Subscriber("netft_data", WrenchStamped, self.ft_callback)
        self.marker_pose_pub = rospy.Publisher('ee_marker_pose', PoseStamped, queue_size=10)

        self.i=0
        self.finished=False
        self.write_to_file=False
        self.T1=[]
        self.T2=[]
        self.x0=[1.031, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        #X = [m, rx, ry, rz, fbx, fby, fbz, tbx, tby, tbz]

    def ft_callback(self,msg):
        self.ft_cur=msg;
        #rospy.loginfo("I heard %s", data.data)

    def skew(self,r):
        return [[0.0, -r[2], r[1]], [r[2], 0.0, -r[0]], [-r[1], r[0], 0.0] ]

    def transform_to_matrix(self,tra):
        #embed()
        T=tf.transformations.quaternion_matrix([tra.rotation.x,tra.rotation.y,tra.rotation.z,tra.rotation.w])
        T[:,3]=[tra.translation.x,tra.translation.y,tra.translation.z,1]
        return T

    def get_transforms(self):
        try:
            self.b2m=trans = self.tfBuffer.lookup_transform(self.robot_base_frame,self.robot_ee_frame, rospy.Time())
            self.B2E=self.transform_to_matrix(self.b2m.transform);
            self.Fm=np.array([self.ft_cur.wrench.force.x,self.ft_cur.wrench.force.y,self.ft_cur.wrench.force.z,
                                self.ft_cur.wrench.torque.x,self.ft_cur.wrench.torque.y,self.ft_cur.wrench.torque.z]);
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            rospy.Rate(100).sleep()

    def cost_function(self,x):
        rx=self.skew([x[1], x[2], x[3]]);
        cost=0;
        for i in range(0,len(self.T1)):
            c=np.zeros(6)
            g=np.dot(tf.transformations.inverse_matrix(self.T1[i]),[[0] ,[0], [-9.8], [0]]) #gravity vector in t
            #print("====")
            #embed()
            mg=np.dot(g[0:3],x[0]) #mass * gravity vector
            f = np.subtract(self.T2[i][0:3], x[4:7])  # subtract bias force
            t=np.subtract(self.T2[i][3:6],x[7:10]) #subtract bias torque
            #print(t)
            c[0:3] = np.subtract(mg.transpose(), f)
            c[3:6] = np.subtract(np.dot(rx, mg).transpose(), t)
            cost=cost+np.sum(np.abs(c));
        cost=cost/len(self.T1);
        print(cost);
        return cost;
            

    def optimise(self):
        print("optimising")
        res = minimize(self.cost_function, self.x0, method='nelder-mead',options={'xtol': 1e-8, 'disp': True, 'maxiter':10000, 'maxfev':150000})
        #self.cost_function();
        #self.cost_function([0,0,0,0,0,0]);
        print ("=== Solution ===")
        print (res.x)
        return res

    def run(self):
        #embed()
        r=rospy.Rate(5)
        if self.write_to_file:
            self.filename='calib_w'+'_'+time.strftime("%Y-%m-%d_%H-%M-%S")+'.csv';
            self.csvfile=open(self.filename,'wb')

        while not self.finished:
            t=raw_input('Go? (press "e" to finish) \n')
            if t == 'e':
                res=self.optimise()
                self.finished=True
                embed()
            else:                       
                self.get_transforms()
                #self.csvfile.write(str((rospy.Time.now()-self.t_start).to_sec())+','+self.get_string()+'\n')
                self.B2FT=self.B2E.dot(self.E2FT)
                print(self.B2FT)
                print(self.Fm)
                ee_pose=PoseStamped();
                #embed()
                ee_pose.header.stamp=rospy.Time.now()
                ee_pose.header.frame_id=self.robot_base_frame
                q=tf.transformations.quaternion_from_matrix(self.B2FT)
                t=tf.transformations.translation_from_matrix(self.B2FT)
                ee_pose.pose.orientation.w=q[3]
                ee_pose.pose.orientation.x=q[0]
                ee_pose.pose.orientation.y=q[1]
                ee_pose.pose.orientation.z=q[2]
                ee_pose.pose.position.x=t[0]
                ee_pose.pose.position.y=t[1]
                ee_pose.pose.position.z=t[2]
                self.marker_pose_pub.publish(ee_pose)
                self.T1.append(self.B2FT)
                self.T2.append(self.Fm)
                if self.write_to_file:
                    self.csvfile.write(str(self.B2FT) + "\n")
                    self.csvfile.write(str(self.Fm) + "\n\n")
                r.sleep();
                #embed()

if __name__ == '__main__':
    print_file=False
    if(len(sys.argv)>1):
        if(sys.argv[1]=='-print'):
            print_file=True;
            print("Printing to file")
    tfp= CalibEEWeight()
    tfp.write_to_file=print_file;
    tfp.run();
