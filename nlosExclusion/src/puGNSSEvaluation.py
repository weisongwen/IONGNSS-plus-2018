#!/usr/bin/env python
# license removed for brevity
"""
    GNSS evaluation
    input: subscribe multi topics
    output: publish some topic and show show it in plot at the same time
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
from nlosExclusion.msg import DOP,Error
from geometry_msgs.msg import Quaternion, Point, Pose, Twist,PoseArray # ros message needed
from PyQt4 import QtCore, QtGui
import puNlosExclusion
import puGNSSPosCal
import time
from nav_msgs.msg import Odometry
from tf import TransformListener
from tf.transformations import quaternion_from_euler
from tf.transformations import quaternion_multiply
import tf
import math
from novatel_msgs.msg import INSPVAX
from novatel_msgs.msg import BESTPOS
from sensor_msgs.msg import PointCloud2
from sensor_msgs.msg   import NavSatFix # standard message type for GNSSs
import llh2ecef # llh to ecef
import ecef2llh #ecef coordinate to llh coordinate
import time
import datetime
from lxml import etree
import xlrd
from pykml.factory import KML_ElementMaker as KML
import puBusDetectionLabelling1
import puBusDetectionLabelling3

class GNSSEvaluation(QMainWindow): # by timer   For paper NLOS exclusion caused by double-decekr bus

    def __init__(self, parent=None):
        QMainWindow.__init__(self, parent)
        self.setWindowTitle('GNSS Evaluation')
        self.create_main_frame()
        self.bestPos_ = BESTPOS()
        self.navfixTruth_ = NavSatFix()
        self.navfix_      = NavSatFix()
        self.navfixprop_ = NavSatFix()
        self.GNSS_ = GNSS_Raw_Array()
        self.GNSSNRAWlosDel = GNSS_Raw_Array()
        self.SatNum = []
        self.HDOP   = []
        self.error  = []
        self.errorEv = [] # error for further eveluation
        self.busAvail = [] # actual bus availibility
        self.GNSSTime = []

        self.SatNumProp = []
        self.HDOPProp   = []
        self.errorProp  = []
        self.Covariance = []
        self.CovarianceFixed = []
        self.errorPropEv = []  # error for further eveluation
        self.busAvailProp = [] # proposed bus availibility which is subjected to the performance of object detection
        self.GNSSTimeProp = []

        self.lat_ = []
        self.lon_ = []
        self.latProp_ = []
        self.lonProp_ = []

        self.firstTime_ = 0.0
        self.firstTimeProp_ = 0.0
        self.preTime = 0.0
        self.checkTimes = 0.0
        self.finishSaving = 0.0
        self.calAngNNLOS = 0.0
        self.navfixAvailable = 0
        self.navfixpropAvailable = 0

        self.timer = QtCore.QTimer()
        QtCore.QObject.connect(self.timer, QtCore.SIGNAL("timeout()"), self.checkEnd)
        self.timer.start(3000)

        rospy.Subscriber('/novatel_data/bestpos', BESTPOS, self.callcptBestPos_)  # Ground truth
        rospy.Subscriber('GNSS_', GNSS_Raw_Array, self.CallGNSS_)  # Ublox Raw data
        rospy.Subscriber('novatel_data/inspvax', INSPVAX, self.callINSPVAXNLOS_)  # yaw
        rospy.Subscriber('GNSS_NavFix', NavSatFix, self.GNSSNavCall)  # conventional GNSS positioning result
        rospy.Subscriber('GNSS_NavFix_pro', NavSatFix, self.GNSSNavPropCall)  # proposed GNSS positioning result
        rospy.Subscriber('GNSSRAWNlosDel_', GNSS_Raw_Array, self.CallGNSSRAWNlosDel_1)  # Ublox
        rospy.Subscriber('velodyne_points_0', PointCloud2, self.PointCloudCall)  # velodyne

        self.DOP_Pub = rospy.Publisher('DOP_', DOP, queue_size=100)  #
        self.Error_Pub = rospy.Publisher('ErrorGNSSXYZ_', Error, queue_size=100)  #
        self.DOPProp_Pub = rospy.Publisher('DOPProp_', DOP, queue_size=100)  #
        self.ErrorProp_Pub = rospy.Publisher('ErrorGNSSXYZProp_', Error, queue_size=100)  #

    def checkEnd(self):

        # print 'ckecking......'
        if(self.preTime == self.navfixTruth_.header.stamp.secs):
            self.checkTimes = self.checkTimes + 1
            if(self.checkTimes > 1 and (self.finishSaving == 0)):
                print 'bag file finished, you can save the llh into .kml file now'
                # save the conventional positioning result to .kml file
                fold = KML.Folder(KML.Placemark(
                    KML.Point(KML.coordinates(str(self.lon_[0]) + ',' + str(self.lat_[0]) + ',0'))
                )
                )
                for i in range(1, len(self.lon_)):
                    fold.append(KML.Placemark(
                        KML.Point(
                            KML.coordinates(str(self.lon_[i]) + ',' + str(self.lat_[i]) + ',0')))
                    )
                content = etree.tostring(etree.ElementTree(fold), pretty_print=True)
                with open('/home/wenws/20180612/small_round/conventional_kml.kml', 'w') as fp:
                    fp.write(content)

                # save the proposed positioning result into .kml file
                fold = KML.Folder(KML.Placemark(
                    KML.Point(KML.coordinates(str(self.lonProp_[0]) + ',' + str(self.latProp_[0]) + ',0'))
                )
                )
                for i in range(1, len(self.lonProp_)):
                    fold.append(KML.Placemark(
                        KML.Point(
                            KML.coordinates(str(self.lonProp_[i]) + ',' + str(self.latProp_[i]) + ',0')))
                    )
                content = etree.tostring(etree.ElementTree(fold), pretty_print=True)
                with open('/home/wenws/20180612/small_round/Proposed_kml.kml',
                          'w') as fp:
                    fp.write(content)
                self.finishSaving = 1
                print 'finishing saving two .kml files '

        self.preTime = self.navfixTruth_.header.stamp.secs

    def callcptBestPos_(self,data): # 1Hz
        self.bestPos_ = BESTPOS()
        self.bestPos_ = data
        self.navfixTruth_ = NavSatFix()
        self.navfixTruth_.header.stamp.secs = float(data.header.gps_week_seconds)
        self.navfixTruth_.latitude  = float(data.latitude)
        self.navfixTruth_.longitude = float(data.longitude)
        self.navfixTruth_.altitude  = float(data.altitude)


    def CallGNSS_(self, data):  # GNSS data # 1Hz
        self.GNSS_ = GNSS_Raw_Array()
        self.GNSS_ = data


    def callINSPVAXNLOS_(self,data): # 1Hz
        inspvax_ = INSPVAX()
        inspvax_ = data
        self.calAngNNLOS = (float(360.0 - inspvax_.azimuth))

    def GNSSNavCall(self,data): # llh from coventional method
        reserve_ = 0.0
        self.navfix_ = NavSatFix()
        self.navfix_ = data
        self.lat_.append(self.navfix_.latitude)
        self.lon_.append(self.navfix_.longitude)
        self.navfixAvailable = 1

    def GNSSNavPropCall(self,data): # llh from proposed method  # about 20 Hz
        self.navfixprop_ = NavSatFix()
        self.navfixprop_ = data
        self.latProp_.append(self.navfixprop_.latitude)
        self.lonProp_.append(self.navfixprop_.longitude)
        self.navfixpropAvailable = 1
        # ---sometimes, self.GNSSNRAWlosDel will be initialized by function CallGNSSRAWNlosDel_1
        # ---In this case, save this value for further processing
        self.GNSSNRAWlosDelSave = GNSS_Raw_Array()
        self.GNSSNRAWlosDelSave = self.GNSSNRAWlosDel
        if((self.navfixprop_.header.stamp.secs == self.navfixTruth_.header.stamp.secs) and (len(self.GNSSNRAWlosDel.GNSS_Raws))):
            # print 'self.navfixprop_.header.stamp.secs',self.navfixprop_.header.stamp.secs
            if(self.firstTimeProp_ == 0):
                self.firstTimeProp_ = float(self.navfixprop_.header.stamp.secs)
            self.GNSSTimeProp.append((float(self.navfixprop_.header.stamp.secs)- float(self.firstTimeProp_)) / 1000) # get GNSS Time
            self.SatNumProp.append(float(len(self.GNSSNRAWlosDel.GNSS_Raws))) # get SatNum
            puGNSSPosCalProp_ = puGNSSPosCal.GNSSPosCal()

            self.HDOPProp.append(float(puGNSSPosCalProp_.DopCalculation(self.GNSSNRAWlosDelSave)))
            self.errorProp.append(self.PosErrorCal(self.navfixTruth_, self.navfixprop_))
            hdop_temp =self.HDOPProp[-1]
            if(hdop_temp>3):
                hdop_temp =3
            self.Covariance.append(self.GNSSNRAWlosDelSave.GNSS_Raws[-1].snr * hdop_temp) # calculate the final covariance
            self.CovarianceFixed.append(7.5 * hdop_temp) # calculate the final covariance
            print 'covariance ------',self.GNSSNRAWlosDelSave.GNSS_Raws[-1].snr
            errorxyz_ = Error()
            errorxyz_.header.stamp.secs = self.navfixprop_.header.stamp.secs
            errorxyz_.errorxyz = float(self.errorProp[-1])
            self.ErrorProp_Pub.publish(errorxyz_)

            Dop_ = DOP()
            Dop_.header.stamp.secs = self.navfixprop_.header.stamp.secs
            Dop_.HDOP = float(self.HDOPProp[-1])
            self.DOPProp_Pub.publish(Dop_)


    def CallGNSSRAWNlosDel_1(self,data):
        self.GNSSNRAWlosDel = GNSS_Raw_Array()
        self.GNSSNRAWlosDel = data

    def PointCloudCall(self,data): # 20 Hz As this function has highest frequency, good for update
        pointCloud_ = PointCloud2()
        pointCloud_ = data
        if((self.navfixAvailable) and (len(self.GNSS_.GNSS_Raws)) and (self.navfixTruth_.header.stamp.secs == self.navfix_.header.stamp.secs) and (self.navfixTruth_.header.stamp.secs == self.GNSS_.GNSS_Raws[-1].GNSS_time)):
            # print 'three conventional source ok at GPS_Week_Second=',self.navfix_.header.stamp.secs
            self.navfixAvailable = 0

            if(self.firstTime_ == 0): # save first epoch
                self.firstTime_ = self.navfix_.header.stamp.secs
            self.GNSSTime.append((float(self.navfix_.header.stamp.secs) - float(self.firstTime_)) / 1000)
            self.SatNum.append(float(len(self.GNSS_.GNSS_Raws)))
            puGNSSPosCal_ = puGNSSPosCal.GNSSPosCal()
            self.HDOP.append(float(puGNSSPosCal_.DopCalculation(self.GNSS_)))
            self.error.append(self.PosErrorCal(self.navfixTruth_,self.navfix_))
            errorxyz = Error()
            errorxyz.header.stamp.secs = self.navfix_.header.stamp.secs
            errorxyz.errorxyz = float(self.error[-1])
            self.Error_Pub.publish(errorxyz)
            Dop_ = DOP()
            Dop_.header.stamp.secs = self.navfix_.header.stamp.secs
            Dop_.HDOP = float(self.HDOP[-1])
            self.DOP_Pub.publish(Dop_)

            if ((len(self.SatNum)) and (len(self.SatNumProp))):
                print 'NLOS exclusion available',self.SatNum[-1],self.SatNumProp[-1],self.SatNum[-1] - self.SatNumProp[-1]
                self.errorEv.append(float(self.error[-1]))
                self.errorPropEv.append(float(self.errorProp[-1]))
                meanEr_ = []
                std_ = np.std(self.errorEv)  # stdandard deviation
                print 'meanEr_', sum(self.errorEv) / (len(self.errorEv)), 'std_----', std_
                self.PercentCal(self.errorEv)

                stdProp_ = np.std(self.errorPropEv)
                print 'meanErPros_', sum(self.errorPropEv) / (len(self.errorPropEv)), 'stdProp_', stdProp_
                self.PercentCal(self.errorPropEv)
                print '----------------------------------------'
            #     self.busAvailProp.append(float(30))
            # else:
            #     self.busAvailProp.append(float(10))
            # if(len(self.SatNum) < 15):
            #     self.busAvail.append(10)
            # if ((len(self.SatNum) >= 15) and (len(self.SatNum) <87)):
            #     self.busAvail.append(30)
            # if (len(self.SatNum) >= 87):
            #     self.busAvail.append(10)
            self.busAvail.append(1 * (puBusDetectionLabelling1.truth[len(self.SatNum)] * 1)+0)
            self.busAvailProp.append(1 * (puBusDetectionLabelling1.detection[len(self.SatNum)] * 1) + 0)
            # self.axesCurv_1.tick_params(axis='both', labelsize=25)
            self.axesCurv_2.tick_params(axis='both', labelsize=25)
            # self.axesCurv_2.set_ylim([-2, 2])
            self.axesCurv_3.tick_params(axis='both', labelsize=25)
            # plot satellite number
            # plot satellite number
            self.axesCurv_2.plot(self.GNSSTime, self.SatNum, '-*', linewidth='1', color='red', label='SatNum')
            if (len(self.GNSSTimeProp) == len(self.SatNumProp)):
                self.axesCurv_2.plot(self.GNSSTimeProp, self.SatNumProp, '-*', linewidth='1', color='blue',
                                     label='SatNumProp')
            # plot HDOP or exclusion accuracy
            # self.axesCurv_2.plot(self.GNSSTime, self.HDOP, '-*', linewidth='1', color='dodgerblue', label='HDOP')
            # if (len(self.GNSSTimeProp) == len(self.HDOPProp)):
            #     self.axesCurv_2.plot(self.GNSSTimeProp, self.HDOPProp, '-*', linewidth='1', color='lime',label='HDOPProp')


            # self.axesCurv_1.plot(self.GNSSTime, self.busAvail, '-*', linewidth='1', color='red', label='busAvail')
            # self.axesCurv_1.plot(self.GNSSTime, self.busAvailProp, '-*', linewidth='1', color='blue',
            #                      label='busAvailProp')

            # plot error
            # self.axesCurv_3.plot(self.GNSSTime, self.error, '-*', linewidth='1', color='red', label='HDOP')
            if (len(self.GNSSTimeProp) == len(self.errorProp)):
                self.axesCurv_3.plot(self.GNSSTimeProp, self.errorProp, '-*', linewidth='1', color='black',
                                     label='errorProp') 
                self.axesCurv_3.plot(self.GNSSTimeProp, self.Covariance, '-*', linewidth='1', color='blue',
                                     label='Covariance') # self.Covariance
                # CovarianceFixed
                self.axesCurv_3.plot(self.GNSSTimeProp, self.CovarianceFixed, '-*', linewidth='1', color='Red',
                                     label='Covariance') # self.Covariance






            self.canvas.draw()
            # self.axesCurv_1.clear()
            self.axesCurv_2.clear()
            self.axesCurv_3.clear()

    def PercentCal(self, data_):
        # self.axes_1.grid(True)
        error_ = []
        error_ = data_
        percent_1 = 0.0  # >15
        percent_2 = 0.0  # >25
        percent_3 = 0.0  # >40
        for perCal in range(len(error_)):
            if (error_[perCal] <= 20):
                percent_1 = percent_1 + 1
            if (error_[perCal] <= 35):
                percent_2 = percent_2 + 1
            if (error_[perCal] >= 45):
                percent_3 = percent_3 + 1
        percent_1 = percent_1 / len(error_)
        percent_2 = percent_2 / len(error_)
        percent_3 = percent_3 / len(error_)
        print 'percent_1=', percent_1, 'percent_2=', percent_2, 'percent_3=', percent_3

    def PosErrorCal(self, truth, llh):
        truth_ = NavSatFix()
        llh_   = NavSatFix()
        truth_ = truth
        llh_   = llh
        truthList = []
        curllh_   = []
        error_ = 0.0
        truthList.append(float(truth_.latitude))
        truthList.append(float(truth_.longitude))
        truthList.append(float(truth_.altitude))

        curllh_.append(float(llh_.latitude))
        curllh_.append(float(llh_.longitude))
        curllh_.append(float(llh_.altitude))

        ecef   = llh2ecef.llh2xyz(curllh_)
        xyzTru_ = llh2ecef.llh2xyz(truthList)

        if (len(ecef) == 3):
            error_ = math.sqrt(
                (ecef[0] - xyzTru_[0]) * (ecef[0] - xyzTru_[0]) + (ecef[1] - xyzTru_[1]) * (ecef[1] - xyzTru_[1]) + (
                        ecef[2] - xyzTru_[2]) * (ecef[2] - xyzTru_[2]))
        if (error_ > 100):
            error_ = 100
        return error_

    def save_plot(self):
        pass

    def on_about(self):
        pass

    def on_pick(self, event):
        pass

    def on_draw(self):
        # self.axes_1.clear()
        # self.axes_1.grid(True)
        self.canvas.draw()

    def create_main_frame(self):
        self.main_frame = QWidget()
        self.dpi = 100
        self.fig = Figure((30.0, 20.0), dpi=self.dpi,facecolor='none') # 5.0 4.0
        self.canvas = FigureCanvas(self.fig)
        self.canvas.setParent(self.main_frame)
        # self.axes = self.fig.add_subplot(111, projection='3d')
        # self.axesCurv_1 = self.fig.add_subplot(311)
        self.axesCurv_2 = self.fig.add_subplot(211)
        self.axesCurv_3 = self.fig.add_subplot(212)
        # self.axes.get_zaxis().set_visible(False)
        self.canvas.mpl_connect('pick_event', self.on_pick)
        self.mpl_toolbar = NavigationToolbar(self.canvas, self.main_frame)
        vbox = QVBoxLayout()
        vbox.addWidget(self.canvas)
        vbox.addWidget(self.mpl_toolbar)
        self.main_frame.setLayout(vbox)
        self.setCentralWidget(self.main_frame)

if __name__ == '__main__':
    app = QApplication(sys.argv)
    rospy.init_node('puGNSSEvaluation_', anonymous=True)
    puGNSSEvaluation=GNSSEvaluation()
    print 'process GNSS evaluation...'
    print '--------------------------'
    puGNSSEvaluation.show()
    app.exec_()