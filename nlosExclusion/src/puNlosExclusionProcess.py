#!/usr/bin/env python
# license removed for brevity
"""
    Nlos exclusion show
    Welson Wen, Ph.D.
    https://sites.google.com/view/weisongwen/about-me
"""
import sys
from PyQt4.QtCore import *
from PyQt4.QtGui import *

import matplotlib
from matplotlib.backends.backend_qt4agg import FigureCanvasQTAgg as FigureCanvas
from matplotlib.backends.backend_qt4agg import NavigationToolbar2QTAgg as  NavigationToolbar
from matplotlib.figure import Figure
from mpl_toolkits.mplot3d import Axes3D
import rospy
from sensor_msgs.msg import LaserScan
import matplotlib
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd # pandas to pd
from PyQt4.QtCore import *
from PyQt4.QtGui import *
import sys
import  math
from matplotlib.patches import Circle
import csv # csv reading needed library
from nlosExclusion.msg import GNSS_Raw,GNSS_Raw_Array # ros msg
from geometry_msgs.msg import Quaternion, Point, Pose, Twist,PoseArray # ros message needed
from PyQt4 import QtCore, QtGui
import puNlosExclusion
import puGNSSPosCal
import time
from novatel_msgs.msg import INSPVAX
from novatel_msgs.msg import BESTPOS
from sensor_msgs.msg import PointCloud2
import llh2ecef # llh to ecef
import ecef2llh #ecef coordinate to llh coordinate
from sensor_msgs.msg   import NavSatFix # standard message type for GNSSs
from geographic_msgs.msg import GeoPointStamped
from nlosExclusion.msg import DOP,Error
import pugeomath



class nlosExclusionProcess():
    def __init__(self, ):
        self.reserve = 0
        self.GrouLat_ = 0.0
        self.GrouLon_ = 0.0
        self.GrouAlt_ = 0.0
        self.calAngNNLOS = 0.0
        self.SPANUpdate = 0.0
        self.busUpdate  = 0.0
        self.GNSSUpdate = 0.0 # Ublox
        self.SPANYawUpdate = 0.0
        self.GNSSNRAWlosDelTimeFlag = 0
        self.posArr = []

        self.initialLLH = [114.19289862,22.3181226456,0]

       
        self.HDOPProp = 0
        # self.excluSatLis = [17,28,8,3,22,9,92]  # Not blocked: 11,1,93,94,96,89,7  blocked: 17,28,8,3,22,9,92 
        self.excluSatLis = [9,14,31]  # 9,14,31,94,90,87
        self.GNSS_ = GNSS_Raw_Array()
        self.GNSSNRAWlosDel = GNSS_Raw_Array()
        self.puGNSSPosCalF_ = puGNSSPosCal.GNSSPosCal()
        self.puGNSSPosCalF_prop = puGNSSPosCal.GNSSPosCal()
        self.nlosExclusionF_ = puNlosExclusion.nlosExclusion(self.calAngNNLOS)
        rospy.Subscriber('/novatel_data/bestpos', BESTPOS, self.callcptBestPos_) # Ground truth
        rospy.Subscriber('double_decker_parameters', PoseArray, self.callDouDeckBus) # bus
        rospy.Subscriber('GNSS_', GNSS_Raw_Array, self.CallGNSS_) # Ublox
        rospy.Subscriber('novatel_data/inspvax', INSPVAX, self.callINSPVAXNLOS_) # yaw
        rospy.Subscriber('velodyne_points_0', PointCloud2, self.PointCloudCall) # velodyne
        rospy.Subscriber('GNSSRAWNlosDel_', GNSS_Raw_Array, self.CallGNSSRAWNlosDel_)  # Ublox
        rospy.Subscriber('DOP_', DOP, self.dopsubscribe)  # Ublox
        self.GNSS_Navfix_pub = rospy.Publisher('GNSS_NavFix', NavSatFix,queue_size=100)  #
        self.propGNSS_Navfix_pub = rospy.Publisher('GNSS_NavFix_pro', NavSatFix, queue_size=100)  #
        self.graphslam_Navfix_pub = rospy.Publisher('/gps/fix', NavSatFix, queue_size=100)  #
        self.graphslam_GeoPoint_pub = rospy.Publisher('/gps/geopoint', GeoPointStamped, queue_size=100)  #
        self.graphslam_PointCloud_pub = rospy.Publisher('/velodyne_points', PointCloud2, queue_size=100)  #


    def dopsubscribe(self,data): # subscribe data after exclusion
        dop_ = DOP()
        dop_ = data

        self.HDOPProp = data.HDOP

    def CallGNSSRAWNlosDel_(self,data): # subscribe data after exclusion
        self.GNSSNRAWlosDel = GNSS_Raw_Array()
        self.GNSSNRAWlosDel = data

    def correctNLOS(self, data):
        correctedGNSS_ = GNSS_Raw_Array()


    def callcptBestPos_(self,data): # 1Hz
        self.bestPos_ = BESTPOS()
        self.bestPos_ = data
        self.GrouLat_ = data.latitude
        self.GrouLon_ = data.longitude
        self.GrouAlt_ = data.altitude

    def callDouDeckBus(self, data):  # double-decker bus data frequency(subjected to availibility of bus)
        self.posArr = []
        self.posArr = data.poses  # save double-decker bus boundary information

    def CallGNSS_(self, data):  # GNSS data # 1Hz
        self.GNSS_ = data
        self.puGNSSPosCalF_ = puGNSSPosCal.GNSSPosCal()
        self.puGNSSPosCalF_.iterPosCal(self.GNSS_, 'WLSGNSS')
        navfix_ = NavSatFix()
        llh_ = []
        llh_ = ecef2llh.xyz2llh(self.puGNSSPosCalF_.ecef_)
        navfix_.header.stamp.secs = self.GNSS_.GNSS_Raws[-1].GNSS_time
        navfix_.latitude  = float(llh_[0])
        navfix_.longitude = float(llh_[1])
        navfix_.altitude  = float(llh_[2])
        self.GNSS_Navfix_pub.publish(navfix_) # print 'ecef',self.puGNSSPosCalF_.ecef_,'llh',llh_

    def callINSPVAXNLOS_(self,data): # 1Hz
        inspvax_ = INSPVAX()
        inspvax_ = data
        self.calAngNNLOS = (float(360.0 - inspvax_.azimuth))

    def PointCloudCall(self,data): # 20 Hz As this function has highest frequency, good for update
        self.pointCloud_ = PointCloud2()
        self.pointCloud_ = data

        self.graphslam_PointCloud_pub.publish(pointCloud_) # for Graph slam

        self.nlosExclusionF_ = puNlosExclusion.nlosExclusion(self.calAngNNLOS)
        self.nlosExclusionF_.nlosExclusion_(self.posArr, self.GNSS_, 'statManu_Exclusion', self.excluSatLis) # statManu
        self.puGNSSPosCalF_prop = puGNSSPosCal.GNSSPosCal()
        if((len(self.GNSSNRAWlosDel.GNSS_Raws) > 1) and (self.GNSSNRAWlosDel.GNSS_Raws[-1].GNSS_time != self.GNSSNRAWlosDelTimeFlag)): # only if there is a change, there will conduct the calculation
            # print 'self.GNSSNRAWlosDel',len(self.GNSSNRAWlosDel.GNSS_Raws)
            self.puGNSSPosCalF_prop.iterPosCal(self.GNSSNRAWlosDel, 'WLSGNSS')
            self.GNSSNRAWlosDelTimeFlag = self.GNSSNRAWlosDel.GNSS_Raws[-1].GNSS_time
            navfix_ = NavSatFix()
            llh_ = []
            llh_ = ecef2llh.xyz2llh(self.puGNSSPosCalF_prop.ecef_)
            navfix_.header.stamp.secs = self.GNSSNRAWlosDel.GNSS_Raws[-1].GNSS_time
            navfix_.latitude = float(llh_[0])
            navfix_.longitude = float(llh_[1])
            navfix_.altitude = float(llh_[2])
            self.propGNSS_Navfix_pub.publish(navfix_)

            # for Graph slam
            graphNavfix_ = NavSatFix()
            graphNavfix_ = navfix_
            graphNavfix_.header = pointCloud_.header
            self.graphslam_Navfix_pub.publish(graphNavfix_)
            # for Graph slam
            geopoint = GeoPointStamped()
            geopoint.header = graphNavfix_.header

            # calculate the ENU coordiantes  initialLLH
            enu_ = pugeomath.transformEcefToEnu(self.initialLLH,self.puGNSSPosCalF_prop.ecef_)
            print 'enu_ ->: ',enu_

            geopoint.position.latitude = float(enu_[0])
            geopoint.position.longitude = float(enu_[1])
            # geopoint.position.altitude = float(llh_[2]) # use this to save the covariance in altitude component 
            geopoint.position.altitude = self.GNSSNRAWlosDel.GNSS_Raws[-1].snr * self.HDOPProp # save the covariance
            self.graphslam_GeoPoint_pub.publish(geopoint)

        # print 'self.puGNSSPosCalF_prop.ecef_',self.puGNSSPosCalF_prop.ecef_


if __name__ == '__main__':
    app = QApplication(sys.argv)
    rospy.init_node('puNlosExclusionProcess', anonymous=True)
    print 'puNlosExclusionProcess.......'
    # nlosExclusion_=nlosExclusionS_()
    # nlosExclusion_.show()
    nlosExclusionProcess_ = nlosExclusionProcess()
    app.exec_()