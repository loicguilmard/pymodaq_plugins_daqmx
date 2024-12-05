import datetime
import numpy as np
import traceback
from qtpy import QtCore
from .daqmxni import DAQmx
from pymodaq_plugins_daqmx.hardware.national_instruments.daq_NIDAQmx import DAQ_NIDAQmx_base
from pymodaq.control_modules.viewer_utility_classes import DAQ_Viewer_base, comon_parameters as viewer_params
from pymodaq.utils.daq_utils import ThreadCommand
from pymodaq.utils.data import DataFromPlugins, DataToExport
from pymodaq.utils.logger import set_logger, get_module_name
logger = set_logger(get_module_name(__file__))


class DAQ_NIDAQmx_Viewer(DAQ_Viewer_base, DAQ_NIDAQmx_base):
    """
        ==================== ========================
        **Attributes**         **Type**
        *data_grabed_signal*   instance of Signal
        *params*               dictionnary list
        *task*
        ==================== ========================

        See Also
        --------
        refresh_hardware
    """

    live_mode_available = True
    params = viewer_params + DAQ_NIDAQmx_base.params

    def __init__(self, parent=None, params_state=None, control_type="0D"):
        DAQ_Viewer_base.__init__(self, parent, params_state)  # defines settings attribute and various other methods
        DAQ_NIDAQmx_base.__init__(self)

        self.Naverage = None
        self.live = False
        self.control_type = control_type  # could be "0D", "1D" or "Actuator"
        if self.control_type == "0D":
            self.settings.child('NIDAQ_type').setLimits(
                ['Analog_Input', 'Counter', 'Digital_Input'])  # analog input and counter
        elif self.control_type == "1D":
            self.settings.child('NIDAQ_type').setLimits(['Analog_Input'])
        elif self.control_type == "Actuator":
            self.settings.child('NIDAQ_type').setLimits(['Analog_Output'])

        self.settings.child('ao_settings').hide()
        self.settings.child('ao_channels').hide()

        # timer used for the counter
        self.timer = QtCore.QTimer()
        self.timer.setSingleShot(True)
        self.timer.timeout.connect(self.counter_done)

    def stop(self):
        """Stop the current grab hardware wise if necessary"""
        self.controller.stop()
        self.live = False
        try:
            self.controller.task.StopTask()
            logger.info("StopTask() done")
        except Exception:
            pass
        ##############################

        self.emit_status(ThreadCommand('Update_Status', ['Acquisition stopped.']))
        return ''

    def commit_settings(self, param):
        """
            Activate the parameters changes in the hardware.

            =============== ================================ ===========================
            **Parameters**   **Type**                        **Description**
            *param*         instance of pyqtgraph.parameter   the parameter to activate
            =============== ================================ ===========================

            See Also
            --------
            update_NIDAQ_channels, update_task, DAQ_NIDAQ_source, refresh_hardware
        """

        if param.parent() is not None:
            if param.parent().name() == 'ai_channels':
                device = param.opts['title'].split('/')[0]
                self.settings.child('clock_settings', 'frequency').setOpts(max=self.getAIMaxRate(device))

                ranges = self.controller.getAIVoltageRange(device)
                param.child('voltage_settings', 'volt_min').setOpts(limits=[r[0] for r in ranges])
                param.child('voltage_settings', 'volt_max').setOpts(limits=[r[1] for r in ranges])

        DAQ_NIDAQmx_base.commit_settings(self, param)

    def ini_detector(self, controller=None):
        """
            Initialisation procedure of the detector.

            See Also
            --------
            daq_utils.ThreadCommand
        """
        try:
            self.controller = self.ini_detector_init(controller, DAQmx())
            self.update_task()

            # actions to perform in order to set properly the settings tree options
            self.commit_settings(self.settings.child('NIDAQ_type'))

            info = "Plugin Initialized"
            initialized = True
            return info, initialized

        except Exception as e:
            logger.info(traceback.format_exc())
            self.emit_status(ThreadCommand('Update_Status', [str(e), 'log']))
            info = str(e)
            initialized = False
            return info, initialized

    def grab_data(self, Naverage=1, **kwargs):
        """
            | grab the current values with NIDAQ profile procedure.
            |
            | Send the data_grabed_signal once done.

            =============== ======== ===============================================
            **Parameters**  **Type**  **Description**
            *Naverage*      int       Number of values to average
            =============== ======== ===============================================

            See Also
            --------
            DAQ_NIDAQ_source
        """
        logger.info("grab_data at: {}".format(datetime.datetime.now()))

        update = False

        if 'live' in kwargs:
            if kwargs['live'] != self.live:
                update = True
            self.live = kwargs['live']

        if Naverage != self.Naverage:
            self.Naverage = Naverage
            update = True
        if update:
            self.update_task()

        if self.controller.task is None:
            self.update_task()

        self.controller.register_callback(self.emit_data, "Nsamples", self.clock_settings.Nsamples)
        self.controller.start()

    def emit_data(self, task_handle, every_n_samples_event_type, number_of_samples, callback_data):
        channels_names = [ch.name for ch in self.channels]
        # channels_ai_names = [ch.name for ch in self.channels if ch.source == 'Analog_Input']
        data_from_task = self.controller.task.read(self.settings['nsamplestoread'], timeout=20.0)
        if self.control_type == "0D":
            if not len(self.controller.task.channels.channel_names) != 1:
                data_dfp = [np.array(data_from_task)]
            else:
                data_dfp = list(map(np.array, data_from_task))
            dte = DataToExport(name='NIDAQmx',
                               data=[DataFromPlugins(name='NI Analog Input',
                                                     data=data_dfp,
                                                     dim=f'Data{self.settings.child("display").value()}',
                                                     labels=channels_names
                                                     ),
                                     ])
        self.dte_signal.emit(dte)
        return 0  # mandatory for the NIDAQmx callback

    def counter_done(self):
        channels_name = [ch.name for ch in self.channels]
        data_counter = self.readCounter(len(self.channels),
                                        self.settings['counter_settings', 'counting_time'] * 1e-3)
        self.data_grabed_signal.emit([DataFromPlugins(name='NI Counter', data=[data_counter / 1e-3], dim='Data0D',
                                                      labels=channels_name, )])
        # y_axis=Axis(label='Count Number', units='1/s'))])
        self.task.StopTask()
