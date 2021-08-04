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
cnt = 0

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

        #layout.addLayout(t_layout)

        po_layout = QHBoxLayout()
        ho0_layout = QHBoxLayout()
        ho1_layout = QVBoxLayout()
        ho2_layout = QVBoxLayout()
        ho1x_layout = QHBoxLayout()
        ho2x_layout = QHBoxLayout()
        ho1y_layout = QHBoxLayout()
        ho2y_layout = QHBoxLayout()
        ho1z_layout = QHBoxLayout()
        ho2z_layout = QHBoxLayout()
        ho2w_layout = QHBoxLayout()
        #b_layout.addLayout(layout)
        #layout.addWidget(self.tablewidget)

        #self.setLayout(b_layout)

        self.position = QLabel()
        self.position.setText("POSITION")
        self.position.setFont(QFont("Times", 15, QFont.Bold))
        self.orientation = QLabel()
        self.orientation.setText("ORIENTATION")
        self.orientation.setFont(QFont("Times", 15, QFont.Bold))
        ho0_layout.addWidget(self.position)
        ho0_layout.addWidget(self.orientation)
        layout.addLayout(ho0_layout)

        self.label_px = QLabel()
        self.px = QLabel()
        self.px.setText("X")
        #self.px.setAlignment(Qt.AlignCenter)
        self.px.setFont(QFont('Arial', 12, QFont.Bold))
        ho1x_layout.addWidget(self.px)
        ho1x_layout.addWidget(self.label_px)
        self.empty_px = QLabel()
        ho1x_layout.addWidget(self.empty_px)
        ho1_layout.addLayout(ho1x_layout)

        self.label_py = QLabel()
        self.py = QLabel()
        self.py.setText("Y")
        #self.py.setAlignment(Qt.AlignCenter)
        self.py.setFont(QFont('Arial', 12, QFont.Bold))
        ho1y_layout.addWidget(self.py)
        ho1y_layout.addWidget(self.label_py)
        self.empty_py = QLabel()
        ho1y_layout.addWidget(self.empty_py)
        ho1_layout.addLayout(ho1y_layout)

        self.label_pz = QLabel()
        self.pz = QLabel()
        self.pz.setText("Z")
        #self.pz.setAlignment(Qt.AlignCenter)
        self.pz.setFont(QFont('Arial', 12, QFont.Bold))
        ho1z_layout.addWidget(self.pz)
        ho1z_layout.addWidget(self.label_pz)
        self.empty_pz = QLabel()
        ho1z_layout.addWidget(self.empty_pz)
        ho1_layout.addLayout(ho1z_layout)

        po_layout.addLayout(ho1_layout)

        self.label_ox = QLabel()
        self.ox = QLabel()
        self.ox.setText("X")
        #self.ox.setAlignment(Qt.AlignCenter)
        self.ox.setFont(QFont('Arial', 12, QFont.Bold))
        ho2x_layout.addWidget(self.ox)
        ho2x_layout.addWidget(self.label_ox)
        self.empty_ox = QLabel()
        ho2x_layout.addWidget(self.empty_ox)
        ho2_layout.addLayout(ho2x_layout)

        self.label_oy = QLabel()
        self.oy = QLabel()
        self.oy.setText("Y")
        #self.oy.setAlignment(Qt.AlignCenter)
        self.oy.setFont(QFont('Arial', 12, QFont.Bold))
        ho2y_layout.addWidget(self.oy)
        ho2y_layout.addWidget(self.label_oy)
        self.empty_oy = QLabel()
        ho2y_layout.addWidget(self.empty_oy)
        ho2_layout.addLayout(ho2y_layout)

        self.label_oz = QLabel()
        self.oz = QLabel()
        self.oz.setText("Z")
        #self.oz.setAlignment(Qt.AlignCenter)
        self.oz.setFont(QFont('Arial', 12, QFont.Bold))
        ho2z_layout.addWidget(self.oz)
        ho2z_layout.addWidget(self.label_oz)
        self.empty_oz = QLabel()
        ho2z_layout.addWidget(self.empty_oz)
        ho2_layout.addLayout(ho2z_layout)

        self.label_ow = QLabel()
        self.ow = QLabel()
        self.ow.setText("W")
        #self.ow.setAlignment(Qt.AlignCenter)
        self.ow.setFont(QFont('Arial', 12, QFont.Bold))
        ho2w_layout.addWidget(self.ow)
        ho2w_layout.addWidget(self.label_ow)
        self.empty_ow = QLabel()
        ho2w_layout.addWidget(self.empty_ow)
        ho2_layout.addLayout(ho2w_layout)
        
        po_layout.addLayout(ho2_layout)
        layout.addLayout(po_layout)
        self.setLayout(layout)

        self.batterylabel = QLabel()
        self.batterylabel.setText("Battery")

        self.empty = QLabel()
        self.pbar = QProgressBar(self)
        #self.pbar.setGeometry()
        layout.addWidget(self.empty)
        layout.addWidget(self.pbar)

    def batt_callback(self, msg):
        #self.tablewidget.item(0, 1).setText(str(msg.percentage*100)+"%")
        self.pbar.setValue(int(msg.percentage*100))
        print("battery check")

    def pos_callback(self, msg):
        #label.setText(str(msg.pose.orientation.w))
        self.label_px.setText(str(msg.pose.position.x))
        self.label_py.setText(str(msg.pose.position.y))
        self.label_pz.setText(str(msg.pose.position.z))

        self.label_ox.setText(str(msg.pose.orientation.x))
        self.label_oy.setText(str(msg.pose.orientation.y))
        self.label_oz.setText(str(msg.pose.orientation.z))
        self.label_ow.setText(str(msg.pose.orientation.w))
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
