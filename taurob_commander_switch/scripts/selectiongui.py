#!/usr/bin/env python

from PyQt5 import QtCore
from PyQt5 import QtWidgets

import signal
import sys

import rospy
import taurob_commander_switch.srv

class MainForm(QtWidgets.QWidget):
    def __init__(self):
        super(MainForm, self).__init__()
        self.resize(780, 576)
        self.setWindowTitle("Selection GUI")

        self._currently_controlling = None

        self._layout = QtWidgets.QGridLayout(self)

        self._lbl_controlling = QtWidgets.QLabel()
        self._btn_select = QtWidgets.QPushButton("Switch between ROS/Commander")
        self._btn_select.setStyleSheet("padding: 30px;")
        self._btn_select.clicked.connect(self._on_btn_select_clicked)

        self._layout.addWidget(QtWidgets.QLabel("Controlling:"), 1, 1)
        self._layout.addWidget(self._lbl_controlling, 1, 2, 1, 1, QtCore.Qt.AlignRight | QtCore.Qt.AlignVCenter)
        self._layout.addWidget(self._btn_select, 2, 1, 1, 2)

        self._layout.setRowStretch(0, 1)
        self._layout.setRowStretch(3, 1)
        self._layout.setColumnStretch(0, 1)
        self._layout.setColumnStretch(3, 1)
        self._layout.setSpacing(20)

        self._switch_state_service = rospy.ServiceProxy(
        				'commander_switch_state',
        				taurob_commander_switch.srv.State)
        self._switch_change_service = rospy.ServiceProxy(
        				'commander_switch_change',
        				taurob_commander_switch.srv.Change)

        self._timer = QtCore.QTimer(self)
        self._timer.timeout.connect(self._on_timer_timeout)
        self._timer.start(1000)

    def _on_btn_select_clicked(self, ev):
        if self._currently_controlling is None:
            return

        new_controller = 'commander' if self._currently_controlling == 'ros' else 'ros'
        self._switch_change_service(new_controller)

    def _on_timer_timeout(self):
        state = self._switch_state_service()
	if state.str not in ('ros', 'commander'):
            print('unknown result "{}" from "commander_switch_state" service call'.format(state.str))
            return
        
        self._currently_controlling = state.str
        if self._currently_controlling == 'ros':
            self._lbl_controlling.setText('ROS')
        else:
            self._lbl_controlling.setText('Commander')


if __name__ == '__main__':
    rospy.init_node('selectiongui')

    # Make it possible to kill this program even when
    # it is inside the QApplication.exec_() call.
    signal.signal(signal.SIGINT, signal.SIG_DFL)

    print('waiting for ROS services...')
    rospy.wait_for_service('commander_switch_state')
    rospy.wait_for_service('commander_switch_change')

    app = QtWidgets.QApplication(sys.argv)
    form = MainForm()
    form.show()
 
    sys.exit(app.exec_())
