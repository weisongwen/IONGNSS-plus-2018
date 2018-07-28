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
from math import cos
from math import sin
from math import pi
from matplotlib.patches import Circle
import csv # csv reading needed library
from nlosExclusion.msg import GNSS_Raw,GNSS_Raw_Array,exclusionSatNum # ros msg
from geometry_msgs.msg import Quaternion, Point, Pose, Twist,PoseArray # ros message needed
from PyQt4 import QtCore, QtGui
from novatel_msgs.msg import INSPVAX


class nlosExclusion():

    def __init__(self,calAngle):
        self.nlExcMode = 'statManu' # 'statAuto' 'dynaAuto' 'statManu' 'dynaManu'
        self.GNSSNlos_pub = rospy.Publisher('GNSSRAWNNlos_', GNSS_Raw_Array,queue_size=10)  # customerized GNSS raw data ros message
        self.GNSSNlosDel_pub = rospy.Publisher('GNSSRAWNlosDel_', GNSS_Raw_Array,queue_size=10)  # customerized GNSS raw data ros message
        rospy.Subscriber('novatel_data/inspvax', INSPVAX, self.callINSPVAX_)
        self.exclusionSatnum_Pub = rospy.Publisher('exclusionSatNum', exclusionSatNum, queue_size=100)  #
        self.calAngN = calAngle
        self.bouRang = 160
        self.GNSSTime = 0
        self.azim_ = []
        self.elev_ = []
        self.azimIni_ = []
        self.elevIni_ = []
        self.satIdx = []
        self.bouSide1X = []
        self.bouSide1Y = []
        self.bouSide2X = []
        self.bouSide2Y = []
        self.bouSide1IniX = []
        self.bouSide2IniX = []
        self.satExcl_ = []
        self.mannuSat = []
        self.GPSNum_ = 0
        self.BeiNum_ = 0
        self.GPSExclNum = 0
        self.BeidExclNum = 0
        self.IniGNSS_ = GNSS_Raw_Array()
        self.nlosGNSS_ = GNSS_Raw_Array()


        self.posArr = []  # create a list to save double-decker bus boundary information

    def callINSPVAX_(self,data):
        inspvax_ = INSPVAX()
        inspvax_ = data
        self.calAngN = (float(360.0 - inspvax_.azimuth))
        # print 'self.calAngN in puNLOSExclusion.py',self.calAngN

    def IsTrangleOrArea(self,x1, y1, x2, y2, x3, y3):
        return math.fabs((x1 * (y2 - y3) + x2 * (y3 - y1) + x3 * (y1 - y2)) / 2.0)

    def IsInside(self,x1, y1, x2, y2, x3, y3, x, y):
        # rectangle ABC area
        ABC = self.IsTrangleOrArea(x1, y1, x2, y2, x3, y3)

        # rectangle PBC area
        PBC = self.IsTrangleOrArea(x, y, x2, y2, x3, y3)

        # rectangle ABC area
        PAC = self.IsTrangleOrArea(x1, y1, x, y, x3, y3)

        # rectangle ABCarea
        PAB = self.IsTrangleOrArea(x1, y1, x2, y2, x, y)
        Area_error_temp = float(ABC - (PBC + PAC + PAB))
        return float(ABC - (PBC + PAC + PAB))

    def nlosExclusion_(self, dataPB, dataNG, mode, manuSatLis):
        self.posArr   = PoseArray()
        self.IniGNSS_ = GNSS_Raw_Array()
        self.nlosGNSS_ = GNSS_Raw_Array()
        self.nlosGNSSDel_ = GNSS_Raw_Array()
        self.mannuSat= manuSatLis
        self.nlExcMode = mode
        self.posArr   = dataPB
        self.IniGNSS_ = dataNG
        self.nlosGNSS_ = dataNG
        self.lateralDis =5.8 # for Transaction on Vehicle Technology
        self.GNSSCovariance = 0
        self.getSatNum(self.IniGNSS_)
        lenPosArr_ = 0.0
        lenPosArr_ = len(self.posArr) # print 'len(self.posArr)------',len(self.posArr)
        if(lenPosArr_ > 2):
            lenPosArr_ = 2
        for i in range(lenPosArr_):
            # x--azimuth y--elevation
            self.bouSide1X.append(1 * (self.posArr[i].orientation.y * (-0.55556) + 50.0) * np.cos((self.posArr[i].orientation.x - 90.0 + self.calAngN) * 3.14159 / 180.0))  # start point azimuth
            self.bouSide1Y.append(1 * (self.posArr[i].orientation.y * (-0.55556) + 50.0) * np.sin((self.posArr[i].orientation.x - 90.0 + self.calAngN) * 3.14159 / 180.0))  # start point elevation
            azi_temp = 0.0
            azi_temp = (math.atan2(self.bouSide1Y[-1] , self.bouSide1X[-1])) * 180.0/(math.pi)
            if (azi_temp < 0.0):
                azi_temp = azi_temp + 360
            self.bouSide1IniX.append(float(azi_temp))  # initial azimuth   start point
            # print 'start point azimuth IN PUNLOSEXCLUSION===', azi_temp,self.bouSide1IniX[-1],len(self.bouSide1IniX)

            self.bouSide2X.append(1 * (self.posArr[i].orientation.w * (-0.55556) + 50.0) * np.cos((self.posArr[i].orientation.z - 90.0 + self.calAngN) * 3.14159 / 180.0))  # end point azimuth
            self.bouSide2Y.append(1 * (self.posArr[i].orientation.w * (-0.55556) + 50.0) * np.sin((self.posArr[i].orientation.z - 90.0 + self.calAngN) * 3.14159 / 180.0))  # end point elevation
            azi_temp = (math.atan2(self.bouSide2Y[-1] , self.bouSide2X[-1])) * 180.0 / (math.pi)
            if (azi_temp < 0.0):
                azi_temp = azi_temp + 360
            self.bouSide2IniX.append(float(azi_temp))  # initial azimuth   start point
            # print 'start point azimuth===', azi_temp

        for index_1 in range(len(self.IniGNSS_.GNSS_Raws)):  # index all the satellite information in one epoch
            self.azim_.append( (self.IniGNSS_.GNSS_Raws[index_1].elevation*(-0.55556)+50.0)* np.cos(-1*(self.IniGNSS_.GNSS_Raws[index_1].azimuth-90.0)*3.14159/180.0))
            self.elev_.append( (self.IniGNSS_.GNSS_Raws[index_1].elevation*(-0.55556)+50.0)* np.sin(-1*(self.IniGNSS_.GNSS_Raws[index_1].azimuth-90.0)*3.14159/180.0))
            sateazi_ = (360 - (self.IniGNSS_.GNSS_Raws[index_1].azimuth)) + 90 # change azimuth value into the coordinate that the same with the bus one (bouSide2X)
            if(sateazi_ > 360):
                sateazi_ = sateazi_ -360
            self.azimIni_.append(sateazi_)
            self.elevIni_.append(self.IniGNSS_.GNSS_Raws[index_1].elevation)
            self.satIdx.append(self.IniGNSS_.GNSS_Raws[index_1].prn_satellites_index)
            self.GNSSTime = self.IniGNSS_.GNSS_Raws[index_1].GNSS_time

        if(self.nlExcMode=='statAuto'):
            for idx in range(len(self.bouSide1X)):  # index all the boundary line (usually only one double-decker bus, sometime 2)
                for index_sat in range(len(self.IniGNSS_.GNSS_Raws)):  # index all the satellite information in one epoch
                    point0_x = 0.0  # yuanxin x
                    point0_y = 0.0  # yuanxin y
                    point1_x = float(self.bouSide1X[idx])  # point1 coordinate x
                    point1_y = float(self.bouSide1Y[idx])  # point1 coordinate y
                    point2_x = float(self.bouSide2X[idx])  # point2 coordinate x
                    point2_y = float(self.bouSide2Y[idx])  # point2 coordinate y
                    point_x = float(self.azim_[index_sat])  # point_x (satellite azimuth in skyplot)
                    point_y = float(self.elev_[index_sat])  # point_y (satellite elevation in skyplot)
                    self.DistanceThres_ = 10
                    area_error = math.fabs(self.IsInside(point1_x, point1_y, point2_x, point2_y, point0_x, point0_y, point_x, point_y))
                    if (self.bouSide1IniX[idx] > self.bouSide2IniX[idx]) and len(self.bouSide2IniX) > 0:  # make sure that bouSide1IniX is smaller
                        azi_temp_ = self.bouSide2IniX[idx]
                        self.bouSide2IniX[idx] = self.bouSide1IniX[idx]
                        self.bouSide1IniX[idx] = azi_temp_
                    ang_ran_ = float(self.bouSide2IniX[idx] - self.bouSide1IniX[idx])
                    # print 'ang_ran_----------------------', ang_ran_, 'from', self.bouSide1IniX[idx], 'to', self.bouSide2IniX[idx]
                    angDif1 = 0.0
                    angDif2 = 0.0
                    if (ang_ran_ < self.bouRang):
                        angDif1 = float(math.fabs(self.azimIni_[index_sat] - self.bouSide1IniX[idx]))
                        angDif2 = float(math.fabs(self.azimIni_[index_sat] - self.bouSide2IniX[idx]))
                        # print 'angDif1= ',angDif1,'angDif2= ',angDif2
                        if (self.azimIni_[index_sat] > self.bouSide1IniX[idx]) and (
                                self.azimIni_[index_sat] < self.bouSide2IniX[idx]) and (
                                area_error > self.DistanceThres_) and (angDif1 > 3) and (angDif2 > 3):  #
                            self.nlosGNSS_.GNSS_Raws[index_sat].visable = 2  # for demonstration: this element will be deleted later
                            self.satExcl_.append(self.satIdx[index_sat])
                    elif (ang_ran_ > self.bouRang):
                        if (self.azimIni_[index_sat] > 0) and (self.azimIni_[index_sat] < self.bouSide1IniX[idx]) and (area_error > self.DistanceThres_):  # and (math.fabs(area_error)>3) :
                            self.nlosGNSS_.GNSS_Raws[index_sat].visable = 2  # for demonstration: this element will be deleted later
                            self.satExcl_.append(self.satIdx[index_sat])
                        if (self.azimIni_[index_sat] < 360) and (
                                self.azimIni_[index_sat] > self.bouSide2IniX[idx]) and (
                                area_error > self.DistanceThres_):  # and (math.fabs(area_error)>3) :
                            # print 'detect one NLOS signals at satellite-', self.satIdx[
                            #     index_sat], 'area_error', area_error, 'satellite_initial_azimuth_list', self.azimIni_[index_sat]
                            # print 'point1_x point1_y point2_x point2_y point_x point_y', point1_x, point1_y, point2_x, point2_y, point_x, point_y
                            self.nlosGNSS_.GNSS_Raws[index_sat].visable = 2  # for demonstration: this element will be deleted later
                            self.satExcl_.append(self.satIdx[index_sat])
            self.GNSSNlos_pub.publish(self.nlosGNSS_)
            exclusionSatNum_ = exclusionSatNum()
            exclusionSatNum_.satlist = self.satExcl_
            self.exclusionSatnum_Pub.publish(exclusionSatNum_)
            for saIdx in range(len(self.nlosGNSS_.GNSS_Raws)):
                if (self.nlosGNSS_.GNSS_Raws[saIdx].visable != 2):  # if satellite is none Light of sight (save)
                    self.nlosGNSSDel_.GNSS_Raws.append(self.nlosGNSS_.GNSS_Raws[saIdx])

            self.GNSSNlosDel_pub.publish(self.nlosGNSSDel_)
            self.getExclSatNum()
        if (self.nlExcMode == 'statManu_Correction'):
            for saIdx in range(len(self.nlosGNSS_.GNSS_Raws)):
                if ( (self.nlosGNSS_.GNSS_Raws[saIdx].prn_satellites_index in self.mannuSat) and (len(self.nlosGNSS_.GNSS_Raws) > 9)):  # if satellite is none Light of sight (save)
                    # GNSS_Raw_Correction = GNSS_Raw()
                    # afa = 4.0
                    # GNSS_Raw_Correction = self.nlosGNSS_.GNSS_Raws[saIdx]
                    # kama1 = afa * ((1 / (math.cos(GNSS_Raw_Correction.elevation * math.pi / 180.0))) * (1 + math.cos(2 * GNSS_Raw_Correction.elevation * math.pi / 180.0)))
                    # kama2 = afa * ((1 / (math.cos(GNSS_Raw_Correction.azimuth * math.pi / 180.0))) * (1 + math.cos(2 * GNSS_Raw_Correction.azimuth * math.pi / 180.0)))
                    # kama = float(math.fabs(kama1) + math.fabs(kama2) )
                    # # GNSS_Raw_Correction.pseudorange = GNSS_Raw_Correction.pseudorange - kama
                    # print 'kama1----', type(kama1),'    kama2----', kama2,'    kama----', kama, '    pseudorange----', GNSS_Raw_Correction.pseudorange
                    self.nlosGNSSDel_.GNSS_Raws.append(self.nlosGNSS_.GNSS_Raws[saIdx])
                    ele = self.nlosGNSSDel_.GNSS_Raws[-1].elevation * 3.14/180.0
                    azi = self.nlosGNSSDel_.GNSS_Raws[-1].azimuth * 3.14/180.0
                    cor_1 = self.lateralDis * cos(float(ele))
                    cor_1 = math.fabs(cor_1* (1.0 + cos(2.0*ele)))
                    cor_2 = self.lateralDis * cos(float(azi))
                    cor_2 = math.fabs(cor_2 * (1.0 + cos(2.0 * azi)))
                    print 'cor_1  ',cor_1,'cor_2  ',cor_2
                    self.nlosGNSSDel_.GNSS_Raws[-1].pseudorange = self.nlosGNSSDel_.GNSS_Raws[-1].pseudorange - (
                            cor_1 + cor_2)
                    # if( (self.nlosGNSSDel_.GNSS_Raws[-1].elevation >=54 ) and (self.nlosGNSSDel_.GNSS_Raws[-1].elevation <= 72)):
                    #     self.nlosGNSSDel_.GNSS_Raws[-1].pseudorange = self.nlosGNSSDel_.GNSS_Raws[-1].pseudorange - (cor_1 + cor_2)
                    #     print 'self.nlosGNSSDel_.GNSS_Raws[-1].prn_satellites_index',self.nlosGNSSDel_.GNSS_Raws[-1].prn_satellites_index
                elif( (self.nlosGNSS_.GNSS_Raws[saIdx].prn_satellites_index not in self.mannuSat) or (len(self.nlosGNSS_.GNSS_Raws) <= 9)):  #no correction
                    # self.nlosGNSS_.GNSS_Raws[saIdx].visable =2
                    self.nlosGNSSDel_.GNSS_Raws.append(self.nlosGNSS_.GNSS_Raws[saIdx])
            self.satExcl_ = self.mannuSat
            self.GNSSNlos_pub.publish(self.nlosGNSS_)
            self.GNSSNlosDel_pub.publish(self.nlosGNSSDel_)
            exclusionSatNum_ = exclusionSatNum()
            exclusionSatNum_.satlist = self.satExcl_
            self.exclusionSatnum_Pub.publish(exclusionSatNum_)
            self.getExclSatNum()
        if (self.nlExcMode == 'statManu_Exclusion'):
            for saIdx in range(len(self.nlosGNSS_.GNSS_Raws)):
                dangerArea = 1; # if danger area, exclusion is needed
                if( (self.nlosGNSS_.GNSS_Raws[-1].GNSS_time >   176810000) and (self.nlosGNSS_.GNSS_Raws[-1].GNSS_time <   176834000)):
                    # print 'safe area '
                    dangerArea = 0
                if( (self.nlosGNSS_.GNSS_Raws[-1].GNSS_time >   176875000) and (self.nlosGNSS_.GNSS_Raws[-1].GNSS_time <   176908000)):
                    dangerArea = 0
                    # print 'safe area '
                # corner start: 176810000
                #corner end: 176834000.0

                # corner start: 176875000
                # corner end: 176908000
                cor_1 =0
                cor_2 =0
                # this satellites should be exlude if satellite is NLOS and the elevation angle is low 
                if ( (self.nlosGNSS_.GNSS_Raws[saIdx].prn_satellites_index in self.mannuSat) and (dangerArea ==1)):  # if satellite is Light of sight (save)
                    
                    print 'cor_1  ',cor_1,'cor_2  ',cor_2, 'GNSS time --',self.nlosGNSS_.GNSS_Raws[-1].GNSS_time
                                        
                elif((dangerArea ==0)):  #no exclusion
                    # self.nlosGNSS_.GNSS_Raws[saIdx].visable =2
                    self.nlosGNSSDel_.GNSS_Raws.append(self.nlosGNSS_.GNSS_Raws[saIdx])
                    self.GNSSCovariance = 4.3+ self.GNSSCovariance
                elif((self.nlosGNSS_.GNSS_Raws[saIdx].prn_satellites_index not in self.mannuSat) and (dangerArea ==1)):  #no exclusion
                    # self.nlosGNSS_.GNSS_Raws[saIdx].visable =2
                    self.nlosGNSSDel_.GNSS_Raws.append(self.nlosGNSS_.GNSS_Raws[saIdx])
                    ele = self.nlosGNSS_.GNSS_Raws[saIdx].elevation * 3.14/180.0
                    azi = self.nlosGNSS_.GNSS_Raws[saIdx].azimuth * 3.14/180.0
                    cor_1 = self.lateralDis * cos(float(ele))
                    cor_1 = math.fabs(cor_1* (1.0 + cos(2.0*ele)))
                    cor_2 = self.lateralDis * cos(float(azi))
                    cor_2 = math.fabs(cor_2 * (1.0 + cos(2.0 * azi)))
                    self.GNSSCovariance = self.GNSSCovariance + (cor_1 + cor_2)
                    
            self.satExcl_ = self.mannuSat
            self.GNSSNlos_pub.publish(self.nlosGNSS_)
            self.GNSSCovariance = self.GNSSCovariance + 4.5
            self.GNSSCovariance =self.GNSSCovariance * 0.43
            print 'self.GNSSCovariance== ',self.GNSSCovariance
            self.nlosGNSSDel_.GNSS_Raws[-1].snr = self.GNSSCovariance
            self.GNSSNlosDel_pub.publish(self.nlosGNSSDel_)
            exclusionSatNum_ = exclusionSatNum()
            exclusionSatNum_.satlist = self.satExcl_
            self.exclusionSatnum_Pub.publish(exclusionSatNum_)
            self.getExclSatNum()
        # print 'self.satExcl_=',self.satExcl_,len(self.nlosGNSSDel_.GNSS_Raws),len(self.nlosGNSS_.GNSS_Raws)
        

    def getSatNum(self,GNSS_one_epoch): # get number of GPS and Beidou satellites in one epoch and save all the satellite number in list
        for index_1 in range(len(GNSS_one_epoch.GNSS_Raws)):  # index all the satellite information in one epoch
            if ((GNSS_one_epoch.GNSS_Raws[index_1].prn_satellites_index >= 1) and (
                GNSS_one_epoch.GNSS_Raws[index_1].prn_satellites_index <= 32)): # GPS satellites index range
                self.GPSNum_ = self.GPSNum_ + 1
            if ((GNSS_one_epoch.GNSS_Raws[index_1].prn_satellites_index >= 88) and (
                GNSS_one_epoch.GNSS_Raws[index_1].prn_satellites_index <= (87 + 37))): # Beidou satellites index range
                self.BeiNum_ = self.BeiNum_+ 1

    def getExclSatNum(self):
        GPSN_ = 0
        BeiN_ = 0
        for idx_1 in range(len(self.satExcl_)):
            if ((self.satExcl_[idx_1] >= 1) and (self.satExcl_[idx_1] <= 32)):  # GPS satellites index range
                GPSN_ = GPSN_ + 1
            if ((self.satExcl_[idx_1] >= 88) and (self.satExcl_[idx_1] <= (87 + 37))):  # Beidou satellites index range
                BeiN_ = BeiN_ + 1
            self.GPSExclNum = GPSN_
            self.BeidExclNum = BeiN_