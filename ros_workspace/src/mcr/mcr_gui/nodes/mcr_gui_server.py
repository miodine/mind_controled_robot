#!/usr/bin/env python

import rospy
from mcr_messages.msg import mcr_predictions, mcr_control_monit, mcr_intention, mcr_datastream


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
        self.datastream = {'electrode_1': None,
                           'electrode_2': None, 
                           'electrode_3': None, 
                           'electrode_4': None,
                           'signal_purity': None, 
                           'signal_block_lenght': None,
                           'intention_interpretation': None}

        
        # control monit
        self.control_monit = {'predicted_intention': None,
                              'confidence': None,
                              'all_predictions': None}

        # predictions 
        self.predictions = {'predictions': None}

        # intention
        self.intention = {'intention': None,
                          'semantic': None}

        # self explanatory
        self._initialize_subscribers()

    
    def update_ui(self, event):
        # Update of GUI's datafields.
        
        ui = self.ui 
        
        # Update       
        ui.command.setText(self.intention['semantic']) # Intention
        ui.prediction.setText(self.control_monit['predicted_intention']) 
        ui.confidence.setProperty("value", int(self.control_monit['confidence'] * 100))
        
        # TODO accuracy and graphview
        ui.accuracy.setText(self.)

    

    # MESSAGE COLLECTING

    def _predictions_callback(self, data : mcr_predictions):
        self.predictions['predictions'] = list(data.predictions)


    def _control_monit_callback(self, data):
        self.control_monit['predicted_intention'] = data.predicted_intention
        self.control_monit['confidence'] = data.confidence
        self.control_monit['all_predictions'] = list(data.all_predictions)

    def _datastream_callback(self, data):
        self.datastream['electrdode_1'] = list(data.electrode_1)
        self.datastream['electrdode_2'] = list(data.electrode_2)
        self.datastream['electrdode_3'] = list(data.electrode_3)
        self.datastream['electrdode_4'] = list(data.electrode_4)
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


