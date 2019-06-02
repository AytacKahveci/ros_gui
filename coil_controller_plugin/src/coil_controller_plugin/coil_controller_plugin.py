import os
import rospy
import rospkg

from PyQt5.QtCore import *
from PyQt5.QtGui import *

from qt_gui.plugin import Plugin
from python_qt_binding import loadUi
from python_qt_binding.QtWidgets import QWidget
from std_msgs.msg import String
from std_msgs.msg import Float64MultiArray
from std_msgs.msg import Float64
from coil_controller.msg import CoilStates



class Gui(Plugin):
    currents = Float64MultiArray()
    currentAct = Float64MultiArray()
    current1 = 0.0
    current2 = 0.0

    def __init__(self, context):
        super(Gui, self).__init__(context)
        # Give QObjects reasonable names
        self.setObjectName('Gui')

        self.sub = rospy.Subscriber("/coil_pid_controller/coil_states", CoilStates, self.callback)
        self.pubCoils = rospy.Publisher('/coil_pid_controller/CoilController/command', Float64MultiArray, queue_size=1)
        self.pubKuka = rospy.Publisher('/joint_position_controller/command', Float64MultiArray, queue_size=1)

        # Process standalone plugin command-line arguments
        from argparse import ArgumentParser
        parser = ArgumentParser()
        # Add argument(s) to the parser.
        parser.add_argument("-q", "--quiet", action="store_true",
                      dest="quiet",
                      help="Put plugin in silent mode")
        args, unknowns = parser.parse_known_args(context.argv())
        if not args.quiet:
            print 'arguments: ', args
            print 'unknowns: ', unknowns

        # Create QWidget
        self._widget = QWidget()
        # Get path to UI file which should be in the "resource" folder of this package
        ui_file = os.path.join(rospkg.RosPack().get_path('coil_controller_plugin'), 'resource', 'gui.ui')
        # Extend the widget with all attributes and children from UI file
        loadUi(ui_file, self._widget)
        # Give QObjects reasonable names
        self._widget.setObjectName('gui')
        # Show _widget.windowTitle on left-top of each plugin (when
        # it's set in _widget). This is useful when you open multiple
        # plugins at once. Also if you open multiple instances of your
        # plugin at once, these lines add number to make it easy to
        # tell from pane to pane.
        if context.serial_number() > 1:
            self._widget.setWindowTitle(self._widget.windowTitle() + (' (%d)' % context.serial_number()))
        # Add widget to the user interface
        context.add_widget(self._widget)
        self._widget.verticalSlider.valueChanged.connect(self.fun)
        self._widget.verticalSlider_2.valueChanged.connect(self.fun2)
        self._widget.lineEdit.editingFinished.connect(self.lineCb)
        self._widget.lineEdit_2.editingFinished.connect(self.lineCb2)
        self._widget.coils_check_box.stateChanged.connect(self.coilsCheckBoxCb)
        self._widget.kuka_publish_button.clicked.connect(self.kukaPublish)


        Gui.current1 = self._widget.verticalSlider.value()
        Gui.current2 = self._widget.verticalSlider_2.value()

        self.dualFlag = True
        self._widget.coils_check_box.setChecked(True)
        self._widget.lineEdit.setValidator(QIntValidator())
        self._widget.lineEdit_2.setValidator(QIntValidator())

        validator = QDoubleValidator()
        validator.setRange(-50, 50, 2)
        self._widget.kuka_x.setValidator(validator)
        self._widget.kuka_y.setValidator(QDoubleValidator(-50, 50, 3))
        self._widget.kuka_z.setValidator(QDoubleValidator(-50, 50, 3))
        self._widget.kuka_a.setValidator(QDoubleValidator(-350, 350, 3))
        self._widget.kuka_b.setValidator(QDoubleValidator(-30, 30, 3))
        self._widget.kuka_c.setValidator(QDoubleValidator(-30, 30, 3))

    def fun(self):
        if(self.dualFlag):
            Gui.current1 = self._widget.verticalSlider.value()
            Gui.current2 = Gui.current1
            self._widget.verticalSlider_2.setValue(self._widget.verticalSlider.value())
        else:
            Gui.current1 = self._widget.verticalSlider.value()
            Gui.current2 = self._widget.verticalSlider_2.value()
        Gui.currents.data = [Gui.current1, Gui.current2]
        self.pubCoils.publish(Gui.currents)
        self._widget.lineEdit.setText(str(Gui.current1))

    def fun2(self):
        if(self.dualFlag):
            Gui.current2 = self._widget.verticalSlider_2.value()
            Gui.current1 = Gui.current2
            self._widget.verticalSlider.setValue(self._widget.verticalSlider_2.value())
        else:
            Gui.current1 = self._widget.verticalSlider.value()
            Gui.current2 = self._widget.verticalSlider_2.value()
        Gui.currents.data = [Gui.current1, Gui.current2]
        self.pubCoils.publish(Gui.currents)
        self._widget.lineEdit_2.setText(str(Gui.current2))

    def lineCb(self):
        strVal = "lineEdit item val: %s" % self._widget.lineEdit.text()
        print strVal
        try:
            print int(self._widget.lineEdit.text())
            self._widget.verticalSlider.setValue(int(self._widget.lineEdit.text()))
        except:
            print "It is not a number"

    def lineCb2(self):
        strVal = "lineEdit item val: %s" % self._widget.lineEdit_2.text()
        print strVal
        try:
            print int(self._widget.lineEdit_2.text())
            self._widget.verticalSlider_2.setValue(int(self._widget.lineEdit_2.text()))
        except:
            print "It is not a number"

    def callback(self,msg):
        if len(msg.current) == 2:
            self._widget.lcdNumber.display(msg.current[0])
            self._widget.lcdNumber_2.display(msg.current[1])
            print("Message data[0]:%f ,Message data[1]:%f" % (msg.current[0], msg.current[1]) )
        else:
            print "Error in CurrentAct"
    
    def coilsCheckBoxCb(self):
        if self._widget.coils_check_box.isChecked():
            self.dualFlag = True
        else:
            self.dualFlag = False

    def kukaPublish(self):
        x = self._widget.kuka_x.text()
        y = self._widget.kuka_y.text()
        z = self._widget.kuka_z.text()
        a = self._widget.kuka_a.text()
        b = self._widget.kuka_b.text()
        c = self._widget.kuka_c.text()

        if(x < -50 or x > 50):
            rospy.logerr("X val is exceeded limits")

        command = Float64MultiArray()
        command.data = [x, y, z, a, b, c]
        self.pubKuka.publish(command)

        

    def shutdown_plugin(self):
        # TODO unregister all publishers here
        pass

    def save_settings(self, plugin_settings, instance_settings):
        # TODO save intrinsic configuration, usually using:
        # instance_settings.set_value(k, v)
        pass

    def restore_settings(self, plugin_settings, instance_settings):
        # TODO restore intrinsic configuration, usually using:
        # v = instance_settings.value(k)
        pass

    #def trigger_configuration(self):
        # Comment in to signal that the plugin has a way to configure
        # This will enable a setting button (gear icon) in each dock widget title bar
        # Usually used to open a modal configuration dialog
