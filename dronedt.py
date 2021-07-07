#!/usr/bin/env python
# -*- coding: utf-8 -*-

import roslib; roslib.load_manifest('rviz_python_tutorial')
import sys
from python_qt_binding.QtGui import *
from python_qt_binding.QtCore import *
try:
    from python_qt_binding.QtWidgets import *
except ImportError:
    pass
import rviz

import rospy
from std_msgs.msg import String
from sensor_msgs.msg import BatteryState
from geometry_msgs.msg import PoseStamped

import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
import numpy as np

class MyViz( QWidget ):
    # 생성자를 통해 frame, button 등의 레이아웃을 추가한다.
    def __init__(self):
        QWidget.__init__(self)
        #rospy.Subscriber('commandar_line', String, self.handle_sub_hello)
        rospy.Subscriber('mavros/local_position/pose', PoseStamped, self.pos_callback)
        rospy.Subscriber('mavros/battery', BatteryState, self.batt_callback)
        self.frame = rviz.VisualizationFrame()
        self.frame.setSplashPath( "" )
        self.frame.initialize()
        # rviz의 기본 프레임을 렌더링 창만 표기하도록 함

        reader = rviz.YamlConfigReader()
        config = rviz.Config()
        reader.readFile( config, "f450_urdf.rviz" )
        self.frame.load( config )

        self.frame.resize(500, 500)
        # .rviz 파일을 읽어 환경으로 구성함
        self.setWindowTitle( config.mapGetChild( "Title" ).getValue() )

        self.frame.setMenuBar( None )
        self.frame.setStatusBar( None )
        self.frame.setHideButtonVisibility( False )

        self.manager = self.frame.getManager()

        self.grid_display = self.manager.getRootDisplayGroup().getDisplayAt(1)
        
        #b_layout = QHBoxLayout()
        layout = QVBoxLayout()
        layout.addWidget( self.frame )
        
        h_layout = QHBoxLayout()


        left_button = QPushButton( "Battery" )
        left_button.clicked.connect( self.onLeftButtonClick )
        h_layout.addWidget( left_button )
        
        center_button = QPushButton( "Orientation" )
        center_button.clicked.connect( self.onCenterButtonClick )
        h_layout.addWidget( center_button )

        right_button = QPushButton( "Position" )
        right_button.clicked.connect( self.onRightButtonClick )
        h_layout.addWidget( right_button )

        
        #layout.addLayout( h_layout )
        
        t_layout = QHBoxLayout()
        self.tablewidget = QTableWidget(self)
        self.tablewidget.resize(200, 400)
        self.tablewidget.setRowCount(3) # 행 개수
        self.tablewidget.setColumnCount(1) # 열 개수
        for i in range(0,3):
            self.tablewidget.setRowHeight(i, self.tablewidget.height() / 3)
        self.tablewidget.setColumnWidth(0,self.tablewidget.width())
        #self.tablewidget.setColumnWidth(1,self.tablewidget.width()/2)

        self.tablewidget.setHorizontalHeaderLabels(['POSITION'])
        self.tablewidget.setVerticalHeaderLabels(['X','Y','Z'])
        for i in range(0, 3):
            for j in range(0, 1):
                self.tablewidget.setItem(i, j, QTableWidgetItem())
                self.tablewidget.item(i, j).setTextAlignment(Qt.AlignVCenter | Qt.AlignHCenter)

        t_layout.addWidget(self.tablewidget)

        self.tablewidget2 = QTableWidget(self)
        self.tablewidget2.resize(200, 400)
        self.tablewidget2.setRowCount(4) # 행 개수
        self.tablewidget2.setColumnCount(1) # 열 개수
        for i in range(0,4):
            self.tablewidget2.setRowHeight(i, self.tablewidget2.height() / 4)
        self.tablewidget2.setColumnWidth(0,self.tablewidget.width())
        #self.tablewidget.setColumnWidth(1,self.tablewidget.width()/2)

        self.tablewidget2.setHorizontalHeaderLabels(['ORIENTATION'])
        self.tablewidget2.setVerticalHeaderLabels(['X','Y','Z','W'])
        for i in range(0, 4):
            for j in range(0, 1):
                self.tablewidget2.setItem(i, j, QTableWidgetItem())
                self.tablewidget2.item(i, j).setTextAlignment(Qt.AlignVCenter | Qt.AlignHCenter)

        t_layout.addWidget(self.tablewidget2)

        layout.addLayout(t_layout)

        #b_layout.addLayout(layout)
        #layout.addWidget(self.tablewidget)

        #self.setLayout(b_layout)

        self.pbar = QProgressBar(self)
        #self.pbar.setGeometry()
        layout.addWidget(self.pbar)
        
        self.setLayout(layout)

    def batt_callback(self, msg):
        #self.tablewidget.item(0, 1).setText(str(msg.percentage*100)+"%")
        self.pbar.setValue(int(msg.percentage*100))
        print("battery check")

    def pos_callback(self, msg):
        self.tablewidget.item(0, 0).setText(str(msg.pose.position.x))
        self.tablewidget.item(1, 0).setText(str(msg.pose.position.y))
        self.tablewidget.item(2, 0).setText(str(msg.pose.position.z))
        self.tablewidget2.item(0, 0).setText(str(msg.pose.orientation.x))
        self.tablewidget2.item(1, 0).setText(str(msg.pose.orientation.y))
        self.tablewidget2.item(2, 0).setText(str(msg.pose.orientation.z))
        self.tablewidget2.item(3, 0).setText(str(msg.pose.orientation.w))
        print("position receive")

    
        

    def onLeftButtonClick( self ):
        if self.grid_display != None:
            self.grid_display.subProp( "Marker Topic" ).setValue("/group_localization")

        
    def onCenterButtonClick( self ):
        if self.grid_display != None:
            self.grid_display.subProp( "Marker Topic" ).setValue("/fire")

    def onRightButtonClick( self ):
        if self.grid_display != None:
            self.grid_display.subProp( "Marker Topic" ).setValue("/path")

    def switchToView( self, view_name ):
        view_man = self.manager.getViewManager()
        for i in range( view_man.getNumViews() ):
            if view_man.getViewAt( i ).getName() == view_name:
                view_man.setCurrentFrom( view_man.getViewAt( i ))
                return
        print( "Did not find view named %s." % view_name )

if __name__ == '__main__':
    rospy.init_node('drone_dt_node', anonymous=False)
    app = QApplication(sys.argv)

    myviz = MyViz()
    myviz.resize(500, 800)
    myviz.show()

    app.exec_()
