import numpy as np
import traceback
from pymodaq.utils.daq_utils import ThreadCommand
from pymodaq.utils.data import DataToExport, DataFromPlugins
from pymodaq.control_modules.viewer_utility_classes import DAQ_Viewer_base, comon_parameters
from pymodaq.utils.parameter import Parameter
from pymodaq_plugins_daqmx import config
from pymodaq_plugins_daqmx.hardware.national_instruments.daqmxni import AIChannel, AIThermoChannel, ClockSettings,\
    DAQmx, nidaqmx, DAQ_analog_types, DAQ_NIDAQ_source, DAQ_termination, DAQ_thermocouples, Edge
from pymodaq.utils.logger import set_logger, get_module_name

logger = set_logger(get_module_name(__file__))


class DAQ_0DViewer_NIDAQmx_Synchronous(DAQ_Viewer_base):
    """
    Plugin for a 0D data visualization & acquisition with various NI modules plugged in a NI cDAQ.
    """
    config_channels: list
    channels_ai: list
    clock_settings_ai: ClockSettings
    config: config
    controller: DAQmx
    config_devices: list
    config_modules: list
    current_device: nidaqmx.system.Device
    live: bool

    param_devices = DAQmx.get_NIDAQ_devices()[1]
    params = comon_parameters + [
        {'title': 'Display type:', 'name': 'display', 'type': 'list', 'limits': ['0D', '1D']},
        {'title': 'Devices :', 'name': 'devices', 'type': 'list', 'limits': param_devices,
         'value': param_devices[0]
         },
        {'title': 'Device To Use:', 'name': 'dev_to_use', 'type': 'list', 'limits': param_devices,
         },
        {'title': 'Source :', 'name': 'source', 'type': 'list',
         'limits': DAQ_NIDAQ_source.names(),
         'value': DAQ_NIDAQ_source.names()[0]
         },
        {'title': 'Analog Types :', 'name': 'ai_type', 'type': 'list',
         'limits': DAQ_analog_types.names(),
         'value': DAQ_analog_types.names()[0]
         },
        {'title': 'Channels:', 'name': 'channels', 'type': 'group', 'children': [
            {'title': 'Analog Input channels :', 'name': 'ai_channel', 'type': 'list',
             'limits': DAQmx.get_NIDAQ_channels(param_devices, source_type='Analog_Input'),
             },
            {'title': 'Analog Output channels :', 'name': 'ao_channel', 'type': 'list',
             'limits': DAQmx.get_NIDAQ_channels(param_devices, source_type='Analog_Output'),
             },
            {'title': 'Counter channels :', 'name': 'co_channel', 'type': 'list',
             'limits': DAQmx.get_NIDAQ_channels(param_devices, source_type='Counter'),
             },
            {'title': 'Digital Input channels :', 'name': 'di_channel', 'type': 'list',
             'limits': DAQmx.get_NIDAQ_channels(param_devices, source_type='Digital_Input'),
             },
            {'title': 'Digital Output channels :', 'name': 'do_channel', 'type': 'list',
             'limits': DAQmx.get_NIDAQ_channels(param_devices, source_type='Digital_Output'),
             },
            {'title': 'Terminals channels :', 'name': 'te_channel', 'type': 'list',
             'limits': DAQmx.get_NIDAQ_channels(param_devices, source_type='Terminals'),
             },
        ]},
        {'title': 'NSampleToRead', 'name': 'nsampletoread', 'type': 'int', 'value': 1, 'default': 1, 'min': 1},
        {'title': 'Clock settings:', 'name': 'clock_settings', 'type': 'group', 'children': [
            {'title': 'Frequency Acq.:', 'name': 'frequency', 'type': 'int', 'value': 1, 'min': 1},
            {'title': 'Nsamples:', 'name': 'Nsamples', 'type': 'int', 'value': 1, 'default': 1, 'min': 1},
            {'title': 'Acquisition mode:', 'name': 'acqmode', 'type': 'list',
             'limits': ['Continuous', 'Finite'], 'value': 'Continuous'},
            {"title": "Counting channel:", "name": "counter_channel", "type": "list",
             "limits": DAQmx.get_NIDAQ_channels(param_devices, source_type="Counter")},
        ]},
    ]

    def __init__(self, parent=None, params_state=None):
        super().__init__(parent, params_state)

    def ini_attributes(self):
        self.config_channels = []
        self.channels_ai = []
        self.config = config
        self.config_devices = []
        self.live = False
        self.config_modules = []

    def commit_settings(self, param: Parameter):
        """Apply the consequences of a change of value in the detector settings

        Parameters
        ----------
        param: Parameter
            A given parameter (within detector_settings) whose value has been changed by the user
        """
        self.update_tasks()

    def configuration_sequence(self, daqmx_controller, current_device):
        """Configure each  / modules / channels as giver by the user in the configuration file

        Read the .toml file to get the desired hardware configuration,
        and send the nidaqmx a sequence which set up each channel.
        """
        logger.info("********** CONFIGURATION SEQUENCE INITIALIZED **********")
        devices_info = [dev.name + ': ' + dev.product_type for dev in daqmx_controller.devices]
        logger.info("Detected devices: {}".format(devices_info))
        try:
            self.config_devices = [config["NIDAQ_Devices", dev].get('name') for dev in self.config["NIDAQ_Devices"]
                                   if "Mod" not in config["NIDAQ_Devices", dev].get('name')]
            logger.info(self.config_devices)
            for dev in config["NIDAQ_Devices"]:
                if not isinstance(config["NIDAQ_Devices", dev], dict):
                    continue
                try:
                    device_name = config["NIDAQ_Devices", dev].get('name')
                    if not device_name == current_device.name:
                        continue
                    device_product = config["NIDAQ_Devices", dev].get('product')
                    device = nidaqmx.system.device.Device(device_name)
                    assert device in daqmx_controller.devices and device.product_type == device_product, device.name
                except AssertionError as err:
                    logger.error("Device {} not detected: {}".format(device_name, err))
                    continue
                for mod in config["NIDAQ_Devices", dev]:
                    if not isinstance(config["NIDAQ_Devices", dev, mod], dict):
                        continue
                    try:
                        module_name = config["NIDAQ_Devices", dev, mod].get('name')
                        module_product = config["NIDAQ_Devices", dev, mod].get('product')
                        module = nidaqmx.system.device.Device(module_name)
                        assert module in daqmx_controller.devices and module.product_type == module_product, module.name
                        self.config_modules.append(config["NIDAQ_Devices", dev, mod].get('name'))
                    except AssertionError as err:
                        logger.error("Module {} not detected: {}".format(module_name, err))
                        continue
                    for source in config["NIDAQ_Devices", dev, mod]:
                        if not isinstance(config["NIDAQ_Devices", dev, mod, source], dict):
                            continue
                        if source == "ai":
                            ai = config["NIDAQ_Devices", dev, mod, source]
                            for ch in ai.keys():
                                name = module_name + "/" + str(ch)
                                term = ai[ch].get("termination")
                                if ai[ch].get("analog_type") == "Voltage":
                                    self.config_channels.append(AIChannel
                                                                (name=name,
                                                                 source=ai[ch].get("source"),
                                                                 analog_type=ai[ch].get("analog_type"),
                                                                 value_min=float(ai[ch].get("value_min")),
                                                                 value_max=float(ai[ch].get("value_max")),
                                                                 termination=DAQ_termination.__getitem__(term),
                                                                 ))
                                elif ai[ch].get("analog_type") == "Current":
                                    self.config_channels.append(AIChannel
                                                                (name=name,
                                                                 source=ai[ch].get("source"),
                                                                 analog_type=ai[ch].get("analog_type"),
                                                                 value_min=float(ai[ch].get("value_min")),
                                                                 value_max=float(ai[ch].get("value_max")),
                                                                 termination=DAQ_termination.__getitem__(term),
                                                                 ))
                                elif ai[ch].get("analog_type") == "Thermocouple":
                                    th = ai[ch].get("thermo_type")
                                    self.config_channels.append(AIThermoChannel
                                                                (name=name,
                                                                 source=ai[ch].get("source"),
                                                                 analog_type=ai[ch].get("analog_type"),
                                                                 value_min=float(ai[ch].get("value_min")),
                                                                 value_max=float(ai[ch].get("value_max")),
                                                                 thermo_type=DAQ_thermocouples.__getitem__(th),
                                                                 ))
            self.clock_settings_ai = ClockSettings(frequency=self.settings.child('clock_settings', 'frequency').value(),
                                                   Nsamples=self.settings.child('clock_settings', 'Nsamples').value(),
                                                   edge=Edge.Rising,
                                                   repetition=self.live)
            daqmx_controller.update_task(self.config_channels, self.clock_settings_ai)
            logger.info("Devices from config: {}".format(self.config_devices))
            logger.info("Current device: {}".format(self.current_device))
            logger.info("Current device modules from config: {}".format(self.config_modules))
            logger.info("Current device channels from config: {}".format([ch.name for ch in self.config_channels]))
        except AssertionError as err:
            logger.error("Configuration entries <{}> does not match the hardware ".format(err))
        except Exception as err:
            logger.info("Configuration sequence error, verify if your config matches the hardware: {}".format(err))
            pass
        logger.info("       ********** CONFIGURATION SEQUENCE SUCCESSFULLY ENDED **********")

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
        try:
            self.current_device = nidaqmx.system.Device(self.settings["devices"])
            self.controller = self.ini_detector_init(controller, DAQmx())
            self.configuration_sequence(self.controller, self.current_device)
            initialized = True
            info = "DAQ_0D initialized"
            logger.info("Detector 0D initialized")
        except Exception as err:
            logger.error("Exception caught {}".format(err))
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
        if self.controller.task is None:
            self.update_tasks()
        data_from_task = self.controller.task.read(self.settings.child('nsampletoread').value(), timeout=20.0)
        self.emit_data(data_from_task)

    def emit_data(self, data_measurement):
        """Emit the data

        Parameters
        ----------
        data_measurement: scalar, list or list of lists
            Since data comes from nidaqmx.task.read, it's type depends on what the task contains & how many channels.
        """
        channels_name = [ch.name for ch in self.channels_ai]
        if not len(self.controller.task.channels.channel_names) != 1:
            data_dfp = [np.array(data_measurement)]
        else:
            data_dfp = list(map(np.array, data_measurement))
        dte = DataToExport(name='NIDAQmx',
                           data=[DataFromPlugins(name='NI Analog Input 01',
                                                 data=data_dfp,
                                                 dim=f'Data{self.settings.child("display").value()}',
                                                 labels=channels_name,
                                                 ),
                                 ])
        self.dte_signal.emit(dte)

    def stop(self):
        """Stop the current grab hardware wise if necessary"""
        self.controller.stop()
        self.emit_status(ThreadCommand('Update_Status', ['Acquisition stopped.']))
        return ''

    def update_tasks(self):
        """Set up the tasks in the NI card."""
        self.clock_settings_ai = ClockSettings(frequency=self.settings.child('clock_settings', 'frequency').value(),
                                               Nsamples=self.settings.child('clock_settings', 'Nsamples').value(),
                                               edge=Edge.Rising,
                                               repetition=self.live)
        chan_to_use = [ch for ch in self.config_channels if self.settings.child("dev_to_use").value() in ch.name]
        logger.info(chan_to_use)
        self.controller.update_task(chan_to_use, self.clock_settings_ai)
