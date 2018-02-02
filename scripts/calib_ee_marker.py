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
from geometry_msgs.msg import PoseStamped

class CalibEEMarker(object):
    def __init__(self):
        rospy.init_node('calib_ee')        
        self.tfBuffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tfBuffer)
        self.camera_frame='kinect2_rgb_optical_frame'
        self.marker_frame='ar_marker_4';
        self.robot_base_frame='world'
        self.robot_ee_frame='iiwa_opto_ft_link'        
        #UR5
        #self.E2M=[[-1.0,   0.0,   0.0, 0.07], [0, -0.707, -0.707, -0.025], [0,-0.707,0.707,  0.025], [0,0,0,1]]
        #KUKA
        self.E2M=[[0.0,   1.0,   0.0, 0.0], [0, -0.0, -01.0, -0.036], [-1.0,-0.0,0.0,  0.064], [0,0,0,1]]
        self.marker_pose_pub = rospy.Publisher('ee_marker_pose', PoseStamped, queue_size=10)
        self.i=0;        
        self.finished=False;
        self.write_to_file=False;
        self.T1=[];
        self.T2=[];        
        self.x0=[1.383, -0.31, 1.0, 0.0, 0.737, 0.004];


    def transform_to_matrix(self,tra):
        #embed()
        T=tf.transformations.quaternion_matrix([tra.rotation.x,tra.rotation.y,tra.rotation.z,tra.rotation.w])
        T[:,3]=[tra.translation.x,tra.translation.y,tra.translation.z,1]
        return T;
    def get_transforms(self):
        try:
            self.c2m=trans = self.tfBuffer.lookup_transform(self.camera_frame,self.marker_frame, rospy.Time())            
            self.b2m=trans = self.tfBuffer.lookup_transform(self.robot_base_frame,self.robot_ee_frame, rospy.Time())
            self.C2M=self.transform_to_matrix(self.c2m.transform);
            self.B2E=self.transform_to_matrix(self.b2m.transform);
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            rospy.Rate(100).sleep()

    def cost_function(self,x):
        M=tf.transformations.euler_matrix(x[3],x[4],x[5],'syxz');
        M[:,3]=[x[0],x[1],x[2],1];
        cost=0;
        for i in range(0,len(self.T1)):
            A=self.T1[i].dot(tf.transformations.inverse_matrix(self.T2[i]));
            C=np.subtract(A,M);
            cost=cost+np.linalg.norm(C,'fro')
            #embed();
        cost=cost/len(self.T1);
        print(cost);
        return cost;
            

    def optimise(self):
        print("optimising")
        res = minimize(self.cost_function, self.x0, method='nelder-mead',options={'xtol': 1e-8, 'disp': True, 'maxiter':3000, 'maxfev':15000})
        #self.cost_function();
        #self.cost_function([0,0,0,0,0,0]);
        print ("=== Solution ===")
        print (res.x)
        M=tf.transformations.euler_matrix(res.x[3],res.x[4],res.x[5],'syxz');
        M[:,3]=[res.x[0],res.x[1],res.x[2],1];
        print (M)
        print (str(res.x[0:3]) + " " +  str(tf.transformations.quaternion_from_matrix(M)));
        embed()


    def run(self):
        r=rospy.Rate(5)
        if self.write_to_file:
            self.filename='calib'+'_'+time.strftime("%Y-%m-%d_%H-%M-%S")+'.csv';
            self.csvfile=open(self.filename,'wb')

        while not self.finished:
            t=raw_input('Go? (press "e" to finish) \n')
            if t == 'e':
                T=self.optimise()
                self.finished=True
            else:                       
                self.get_transforms()
                #self.csvfile.write(str((rospy.Time.now()-self.t_start).to_sec())+','+self.get_string()+'\n')
                print('C2M=')
                print(self.C2M);
                print('B2M=')

                self.B2M=self.B2E.dot(self.E2M)
                print(self.B2M)
                ee_pose=PoseStamped();
                #embed()
                ee_pose.header.stamp=rospy.Time.now()
                ee_pose.header.frame_id=self.robot_base_frame
                q=tf.transformations.quaternion_from_matrix(self.B2M)
                t=tf.transformations.translation_from_matrix(self.B2M)
                ee_pose.pose.orientation.w=q[3]
                ee_pose.pose.orientation.x=q[0]
                ee_pose.pose.orientation.y=q[1]
                ee_pose.pose.orientation.z=q[2]
                ee_pose.pose.position.x=t[0]
                ee_pose.pose.position.y=t[1]
                ee_pose.pose.position.z=t[2]
                self.marker_pose_pub.publish(ee_pose)
                self.T1.append(self.B2M)
                self.T2.append(self.C2M)
                if self.write_to_file:
                    self.csvfile.write(str(self.B2M) + "\n")
                    self.csvfile.write(str(self.C2M) + "\n\n")
                r.sleep();
                #embed()

if __name__ == '__main__':
    print_file=False
    if(len(sys.argv)>1):
        if(sys.argv[1]=='-print'):
            print_file=True;
            print("Printing to file")
    tfp= CalibEEMarker()
    tfp.write_to_file=print_file;
    tfp.run();
