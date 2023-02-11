#!/usr/bin/env python

import rospy
from mcr_messages.msg import mcr_predictions, mcr_control_monit, mcr_intention, mcr_datastream
import numpy as np

class GUIDataServer:
    '''
    GUI Data Server for mcr system.

    PL:
    Klasa odpowiadajaca za agregowanie wiadomosci z podsystemu mcr. 
    GUIboi. 

    ARA:
    Klass som gör det möjligt att samla in data från delsystemet mcr. 
    För att - du gissade det - visa dem i ett grafiskt gränssnitt.



    '''

    def __init__(self, ui_pointer):
        
        # ui pointer, zostaw go tutej :3 on jest po to zeby nie podawac go w argu przy wywolaniach funckcji
        self.ui = ui_pointer

        # 

        # datastream
        self.datastream = {'electrode_1': np.zeros((640,)),
                           'electrode_2': np.zeros((640,)), 
                           'electrode_3': np.zeros((640,)), 
                           'electrode_4': np.zeros((640,)),
                           'signal_purity': None, 
                           'signal_block_lenght': None,
                           'intention_interpretation': None}

        
        # control monit
        self.control_monit = {'predicted_intention': None,
                              'confidence': 0.0,
                              'all_predictions': None}

        # predictions 
        self.predictions = {'predictions': None}
        self.prediction_accuracy = 0.0
        self.tp = 0
        self.cum = 0

        # intention
        self.intention = {'intention': None,
                          'semantic': 'STOP'}


        self.prediction_accuracy = 1.0

        # self explanatory
        self._initialize_subscribers()

    
    def __accuracy_eval(self):
        self.tp +=1
    
        pred = self.control_monit['predicted_intention']
        actu = self.intention['semantic']

        if pred == actu:
            self.cum += 1.0
        else:
            self.cum = self.cum

        self.prediction_accuracy = self.cum/self.tp
    
    # SZPACHLA SZPACHLA SZPACHLAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA
    # XDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDD
    def __change_color_bar(self, data, ui):
        if data >= 90:
            ui.confidence.setStyleSheet("QProgressBar {\n"
"\n"
"    border-style: solid;\n"
"    border-color: #dae3ff;\n"
"    border-radius: 7px;\n"
"    border-width: 2px;\n"
"    text-align:center;\n"
"\n"
"    font:  large \"Verdana\";\n"
"    color: #dae3ff;\n"
"\n"
"}\n"
"\n"
"QProgressBar::chunk {\n"
"width: 2px;\n"
"background-color : #178f15;\n"
"margin: 1px\n"
"}")
        if data >= 40 and data <90:
            ui.confidence.setStyleSheet("QProgressBar {\n"
"\n"
"    border-style: solid;\n"
"    border-color: #dae3ff;\n"
"    border-radius: 7px;\n"
"    border-width: 2px;\n"
"    text-align:center;\n"
"\n"
"    font:  large \"Verdana\";\n"
"    color: #dae3ff;\n"
"\n"
"}\n"
"\n"
"QProgressBar::chunk {\n"
"width: 2px;\n"
"background-color : #b5b24e;\n"
"margin: 1px\n"
"}")

        if data < 40:
            ui.confidence.setStyleSheet("QProgressBar {\n"
"\n"
"    border-style: solid;\n"
"    border-color: #dae3ff;\n"
"    border-radius: 7px;\n"
"    border-width: 2px;\n"
"    text-align:center;\n"
"\n"
"    font:  large \"Verdana\";\n"
"    color: #dae3ff;\n"
"\n"
"}\n"
"\n"
"QProgressBar::chunk {\n"
"width: 2px;\n"
"background-color : #178f15;\n"
"margin: 1px\n"
"}")


    def __update_data_viz(self, ax, data, c):
        ax.clear()
        t = np.linspace(0,4,640)
        ax.plot(t, data, color = c)
        ax.grid()
        ax.figure.canvas.draw()



    def update_ui(self, event):
        # TODO: Update of GUI's datafields.
        
        ui = self.ui 
        
        # Update intention and prediction      
        ui.command.setText(self.intention['semantic']) # Intention
        ui.prediction.setText(self.control_monit['predicted_intention']) 

        # Update confidence monit
        confidence = int(self.control_monit['confidence'] * 100)
        ui.confidence.setProperty("value", confidence)
        self.__change_color_bar(confidence, ui)

        # Update accuracy        
        self.__accuracy_eval()
        ui.accuracy.setText("{:.2f}%".format(self.prediction_accuracy*100.0))

        # TODO: display data

        self.__update_data_viz(self.ui._dynamic_ax_1, self.datastream['electrode_1'], c = 'r')
        self.__update_data_viz(self.ui._dynamic_ax_2, self.datastream['electrode_2'], c = 'g')
        self.__update_data_viz(self.ui._dynamic_ax_3, self.datastream['electrode_3'], c = 'b')
        self.__update_data_viz(self.ui._dynamic_ax_4, self.datastream['electrode_4'], c = 'y')

    # MESSAGE COLLECTING

    def _predictions_callback(self, data : mcr_predictions):
        self.predictions['predictions'] = list(data.predictions)


    def _control_monit_callback(self, data):
        self.control_monit['predicted_intention'] = data.predicted_intention
        self.control_monit['confidence'] = data.confidence
        self.control_monit['all_predictions'] = list(data.all_predictions)

    def _datastream_callback(self, data):
        self.datastream['electrode_1'] = list(data.electrode_1)
        self.datastream['electrode_2'] = list(data.electrode_2)
        self.datastream['electrode_3'] = list(data.electrode_3)
        self.datastream['electrode_4'] = list(data.electrode_4)
        self.datastream['signal_purity'] = data.signal_purity
        self.datastream['signal_block_lenght'] = data.signal_block_lenght
        self.datastream['intention_interpretation'] = data.intention_interpretation

    def _intention_callback(self, data):
        self.intention['intention'] = data.intention
        self.intention['semantic'] = data.semantic

    def _initialize_subscribers(self):
        rospy.Subscriber('/mcr_mlclass_predictions', mcr_predictions, callback= self._predictions_callback)
        rospy.Subscriber('/mcr_control_monit', mcr_control_monit,  callback= self._control_monit_callback)
        rospy.Subscriber("/mcr_bci_data_feed", mcr_datastream,  callback= self._datastream_callback)
        rospy.Subscriber("/mcr_bci_intention", mcr_intention,  callback= self._intention_callback)


