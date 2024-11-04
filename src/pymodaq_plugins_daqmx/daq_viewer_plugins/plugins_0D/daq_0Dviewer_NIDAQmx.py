import numpy as np
import traceback
from pymodaq.utils.daq_utils import ThreadCommand
from pymodaq.utils.data import DataWithAxes, DataToExport, DataSource, DataFromPlugins
from pymodaq.control_modules.viewer_utility_classes import DAQ_Viewer_base, comon_parameters, main
from pymodaq.utils.parameter import Parameter
from pymodaq_plugins_daqmx import config
from pymodaq_plugins_daqmx.hardware.national_instruments.daqmxni import *
from pymodaq.utils.logger import set_logger, get_module_name
logger = set_logger(get_module_name(__file__))

class DAQ_0DViewer_NIDAQmx(DAQ_Viewer_base):
    """
    Plugin for a 0D data visualization & acquisition with various NI modules plugged in a NI cDAQ.
    """
    channels_ai: list
    clock_settings_ai: str
    dict_device_input: dict
    live: bool

    params = comon_parameters+[
        {'title': 'Display type:', 'name': 'display', 'type': 'list', 'limits': ['0D', '1D', '2D']},
        # {'title': 'Module ref. :', 'name': 'module', 'type': 'list', 'limits': DAQmx.get_NIDAQ_devices(),
        #  'value': DAQmx.get_NIDAQ_devices()[0]
        #  },
        {'title': 'Devices', 'name': 'devices', 'type': 'text', 'value': ''},
        {'title': 'Channels', 'name': 'channels', 'type': 'text', 'value': ''},
        {'title': 'Analog channel :', 'name': 'ai_channel', 'type': 'list',
         'limits': DAQmx.get_NIDAQ_channels(source_type='Analog_Input'),
         'value': DAQmx.get_NIDAQ_channels(source_type='Analog_Input')[0]
         },
        {'title': 'Analog Source :', 'name': 'ai_srce', 'type': 'list',
         'limits': DAQ_NIDAQ_source.names(),
         'value': DAQ_NIDAQ_source.names()[0]
         },
        {'title': 'Analog Type :', 'name': 'ai_type', 'type': 'list',
         'limits': DAQ_analog_types.names(),
         'value': DAQ_analog_types.names()[0]
         },
        # {'title': 'Min. value:', 'name': 'ai_min', 'type': 'float', 'value': -80e-3, 'min': -1e4},
        # {'title': 'Max. value:', 'name': 'ai_max', 'type': 'float', 'value': 80e-3, 'max': 1e4},
        {'title': 'Clock settings:', 'name': 'clock_settings', 'type': 'group', 'children': [
            {'title': 'Frequency Acq.:', 'name': 'frequency', 'type': 'int', 'value': 100, 'min': 1},
            {'title': 'Nsamples:', 'name': 'Nsamples', 'type': 'int', 'value': 100, 'default': 100, 'min': 1},
            {'title': 'Acquisition mode:', 'name': 'acqmode', 'type': 'list',
             'limits': ['Continuous', 'Finite'], 'value': 'Continous'},
            {"title": "Counting channel:", "name": "counter_channel", "type": "list",
             "limits": DAQmx.get_NIDAQ_channels(source_type="Counter")},
        ]},
        ]

    def __init__(self, parent=None, params_state=None):
        super().__init__(parent, params_state)

    def ini_attributes(self):
        self.controller: DAQmx = None
        self.channels_ai = []
        self.clock_settings_ai = None
        self.dict_device_input = None
        self.live = False  # True during a continuous grab

    def commit_settings(self, param: Parameter):
        """Apply the consequences of a change of value in the detector settings

        Parameters
        ----------
        param: Parameter
            A given parameter (within detector_settings) whose value has been changed by the user
        """
        # if param.name() == "frequency":
        self.update_tasks()

    def ini_detector(self, controller=None):
        """Detector communication initialization

        Parameters
        ----------
        controller: (object)
            custom object of a PyMoDAQ plugin (Slave case). None if only one actuator/detector by controller
            (Master case)

        Returns
        -------
        info: str
        initialized: bool
            False if initialization failed otherwise True
        """
        logger.info("Detector 0D initialized")

        try:
            self.controller = self.ini_detector_init(controller, DAQmx())
            self.update_tasks()
            initialized = True
            info = "DAQ_0D initialized"
            self.controller.update_NIDAQ_devices()
            self.controller.update_NIDAQ_channels()
            logger.info("Detected devices: {}".format(self.controller.devices))
            self.settings.child('devices').setValue(str(self.controller.devices))
            logger.info("Detected channels: {}".format(self.controller.channels))
            self.settings.child('channels').setValue(str(self.controller.channels))
        except Exception as err:
            logger.error("Exception catched {}".format(err))
            logger.error(traceback.format_exc())
            initialized = False
            info = "Error"
        return info, initialized

    def close(self):
        """Terminate the communication protocol"""
        self.controller.close()
        pass

    def grab_data(self, Naverage=1, **kwargs):
        """Start a grab from the detector

        Parameters
        ----------
        Naverage: int
            Number of hardware averaging not relevant here.
        kwargs: dict
            others optionals arguments
        """
        if self.controller._task is None:
            self.update_tasks()

        data_from_task = self.controller._task.read(self.settings.child('clock_settings', 'Nsamples').value(), timeout = 20.0)
        self.emit_data(data_from_task)

    def emit_data(self, data_measurement):
        logger.info("Channels names {}".format(self.controller._task.channels.channel_names))
        logger.info("Channels names {}".format(len(self.controller._task.channels.channel_names)))
        if not len(self.controller._task.channels.channel_names) != 1:
            data_dfp = [np.array(data_measurement)]
        else:
            data_dfp = list(map(np.array, data_measurement))

        logger.info("data_dfp: {}" .format(len(data_dfp)))
        dte = DataToExport(name='NIDAQmx',
                           data=[DataFromPlugins(name='NI Analog Input 01',
                                                 data=data_dfp,
                                                 dim=f'Data{self.settings.child("display").value()}',
                                                 labels=[self.channels_ai[0].name],
                                                 ),
                                 ])
        self.dte_signal.emit(dte)

    def stop(self):
        """Stop the current grab hardware wise if necessary"""
        self.close()
        self.emit_status(ThreadCommand('Update_Status', ['Acquisition stopped.']))
        return ''

    def update_tasks(self):
        """Set up the tasks in the NI card."""
        # Create channels
        #####################
        '''
        Idée:
        for Device in Devices:
            self.channels_from_device(Device01) = [AIchannel(),AIchannel()......]
            self.channels_from_device(Device02) = [AIthermochannel(),AIthermochannel()......]
        Emit by mode (as for keithley) === by device
        
        '''
        config_ai = config["NIDAQ_Devices", "cDAQ9174", "NI9205", "ai"]
        for AI in config_ai.keys():
            self.channels_ai.append(AIChannel(name=config_ai[AI].get("name"),
                                              source=config_ai[AI].get("source"),
                                              analog_type=config_ai[AI].get("analog_type"),
                                              value_min=float(config_ai[AI].get("value_min")),
                                              value_max=float(config_ai[AI].get("value_max")),
                                              termination=DAQ_termination.__getitem__(config_ai[AI].get("termination")),
                                              ))

        self.clock_settings_ai = ClockSettings(frequency=self.settings.child('clock_settings', 'frequency').value(),
                                               Nsamples=self.settings.child('clock_settings', 'Nsamples').value(),
                                               edge=Edge.Rising,
                                               repetition=self.live)

        self.controller.update_task(self.channels_ai, self.clock_settings_ai)


if __name__ == '__main__':
    """Main section used during development tests"""
    main_file = True
    if main_file:
        main(__file__)
    else:
        try:
            print("In main")
            import nidaqmx

            logger.info("DAQ Sources names{}".format(DAQ_NIDAQ_source.names()))
            logger.info("DAQ Sources members{}".format(DAQ_NIDAQ_source.members()))
            channels = [AIThermoChannel(name="cDAQ1Mod1/ai0",
                                        source='Analog_Input',
                                        analog_type='Thermocouple',
                                        value_min=-80.0e-3,
                                        value_max=80.0e-3,
                                        termination='Diff',
                                        thermo_type=DAQ_thermocouples.K),
                       ]
            # Create Task
            print("DAQ_thermocouples.names ", DAQ_thermocouples.names())
            print("DAQ_thermocouples.K ", nidaqmx.constants.ThermocoupleType.K)
            print("DAQ_thermocouples.K type", type(nidaqmx.constants.ThermocoupleType.K))
            print("DAQ_thermocouples.K ", DAQ_thermocouples.K)
            print("DAQ_thermocouples.K type ", type(DAQ_thermocouples.K))
            print("channel.thermo_type ", channels[0].thermo_type)
            task = nidaqmx.Task()
            for channel in channels:
                task.ai_channels.add_ai_thrmcpl_chan(physical_channel=channel.name,
                                                     # name_to_assign_to_channel="Channel 01",
                                                     min_val=channel.value_min,
                                                     max_val=channel.value_max,
                                                     units=TemperatureUnits.DEG_C,
                                                     thermocouple_type=channel.thermo_type,
                                                     # cjc_source=CJCSource.BUILT_IN,
                                                     # cjc_val="",
                                                     # cjc_channel="",
                                                     )
            NIDAQ_Devices = nidaqmx.system.System.local().devices

            print("NIDAQ devices ", NIDAQ_Devices)
            print("NIDAQ devices names ", NIDAQ_Devices.device_names)

            Chassis = NIDAQ_Devices[0]
            Module01 = NIDAQ_Devices[1]
            Module02 = NIDAQ_Devices[2]
            print("Chassis={}, Module01={}, Module02={}" .format(Chassis, Module01, Module02))

            # Test resources
            try:
                Chassis.self_test_device()
                Module01.self_test_device()
                Module02.self_test_device()
            except Exception as e:
                print("Resources test failed: {}" .format(e))

            print("Chassis: name={}, Num={}".format(Chassis.name, Chassis.product_type))
            print("Module01: name={}, Num={}".format(Module01.name, Module01.product_type))
            print("Module02: name={}, Num={}".format(Module02.name, Module02.product_type))

            print("channel 01 name : ", channels[0].name)
            data = task.read()
            print("data = ", data)
            print("type(data) = ", type(data))
            print("type(data[0]) = ", type(data[0]))

            task.close()

        except Exception as e:
            print("Exception ({}): {}".format(type(e), str(e)))
