import numpy as np
from pymodaq.utils.daq_utils import ThreadCommand
from pymodaq.utils.data import DataWithAxes, DataToExport, DataSource, DataFromPlugins
from pymodaq.control_modules.viewer_utility_classes import DAQ_Viewer_base, main
from pymodaq.control_modules.viewer_utility_classes import comon_parameters as viewer_params
from pymodaq.utils.parameter import Parameter
from pymodaq_plugins_daqmx.hardware.national_instruments.daqmxni import *
from pymodaq_plugins_daqmx.hardware.national_instruments.daq_NIDAQmx import DAQ_NIDAQmx_base, DAQ_NIDAQmx_Viewer
from pymodaq.utils.logger import set_logger, get_module_name
logger = set_logger(get_module_name(__file__))


class DAQ_0DViewer_NIDAQmx_2(DAQ_NIDAQmx_Viewer):
    """
    Plugin for a 0D data visualization & acquisition with various NI modules plugged in a NI cDAQ.
    """
    channels_ai: str
    clock_settings_ai: str
    dict_device_input: dict
    live: bool

    params = viewer_params + [
        {'title': 'Display type:', 'name': 'display', 'type': 'list', 'limits': ['0D', '1D']},
        {'title': 'Module ref. :', 'name': 'module', 'type': 'list', 'limits': DAQmx.get_NIDAQ_devices(),
         'value': DAQmx.get_NIDAQ_devices()[0]
         },
        ] + DAQ_NIDAQmx_base.params

    def __init__(self, parent=None, params_state=None):
        super().__init__(parent, params_state)

    def ini_attributes(self):
        super().ini_attributes()
        self.controller: DAQmx = None
        # self.channels_ai = None
        self.clock_settings_ai = None
        self.dict_device_input = None
        self.live = False  # True during a continuous grab

    # def commit_settings(self, param: Parameter):
    #     """Apply the consequences of a change of value in the detector settings
    #
    #     Parameters
    #     ----------
    #     param: Parameter
    #         A given parameter (within detector_settings) whose value has been changed by the user
    #     """
    #     # self.update_task()
    #     super().commit_settings(self, param)

    # def ini_detector(self, controller=None):
    #     """Detector communication initialization
    #
    #     Parameters
    #     ----------
    #     controller: (object)
    #         custom object of a PyMoDAQ plugin (Slave case). None if only one actuator/detector by controller
    #         (Master case)
    #
    #     Returns
    #     -------
    #     info: str
    #     initialized: bool
    #         False if initialization failed otherwise True
    #     """
    #     logger.info("Detector 0D initialized")
    #     # try:
    #     #     self.controller = self.ini_detector_init(controller, DAQmx())
    #     #     self.update_task()
    #     #     initialized = True
    #     #     info = "DAQ_0D initialized"
    #     # except Exception as err:
    #     #     logger.error(err)
    #     #     initialized = False
    #     #     info = "Error"
    #     # return info, initialized
    #     # info, initialized = super().ini_detector(controller)
    #     # info, initialized = DAQ_NIDAQmx_Viewer.ini_detector(self, controller)
    #     self.controller = self.ini_detector_init(controller, DAQmx())
    #     logger.info(self.controller._task)
    #     # info, initialized = self.ini
    #     logger.info("self.controller devices {}".format(self.controller.devices))
    #     info, initialized = "Plugin initialized", True
    #     return info, initialized

    # def close(self):
    #     """Terminate the communication protocol"""
    #     self.controller.close()
    #     pass

    # def grab_data(self, Naverage=1, **kwargs):
    #     """Start a grab from the detector
    #
    #     Parameters
    #     ----------
    #     Naverage: int
    #         Number of hardware averaging not relevant here.
    #     kwargs: dict
    #         others optionals arguments
    #     """
    #     logger.info("Grab data \n")
    #     logger.info("TASK NAME WHEN GRAB: {}".format(self._task))
    #     # self.update_task()
    #     # logger.info("TASK UPDATED")
    #     logger.info("SELF sans channel: {}".format(self))
    #     logger.info("Channels in task when grab requested: {}"
    #                 .format(self._task.ai_channels.channel_names))
    #     data_from_task = self._task.read(self.settings.child('clock_settings', 'Nsamples').value())
    #     logger.info("LEN DATA = {}".format(len(data_from_task)))
    #
    #     self.emit_datas(data_from_task)
    #
    # def emit_datas(self, data_measurement):
    #     logger.info("Emit data")
    #     logger.info("Channel names {}".format(self._task.ai_channels.channel_names))
    #     dte = DataToExport(name='NIDAQmx',
    #                        data=[DataFromPlugins(name='NI-9205: Analog Input 01',
    #                                              data=[np.array([data_measurement])],
    #                                              # dim=f'Data{self.settings.child("display").value()}',
    #                                              dim=f'Data0D',
    #                                              labels=self._task.ai_channels.channel_names,
    #                                              ),
    #                              ])
    #     self.dte_signal.emit(dte)
        # dte = DataToExport(name='NIDAQmx',
        #                    data=[DataFromPlugins(name='NI Analog Input 01',
        #                                          data=[np.array([data_measurement])],
        #                                          # dim=f'Data{self.settings.child("display").value()}',
        #                                          dim=f'Data0D',
        #                                          labels=["aaaaaaaaa"],
        #                                          ),
        #                          DataFromPlugins(name='NI Analog Input 02',
        #                                          data=[np.array([data_measurement[1]])],
        #                                          # dim=f'Data{self.settings.child("display").value()}',
        #                                          dim=f'Data0D',
        #                                          labels=[self.channels_ai[0].name],
        #                                          ),
        #                          ])

        # # Dictionary linking devices to channel's physical quantities
        # self.dict_device_input = {self.controller.devices[1]: [self.channels_ai[0].name, self.channels_ai[1].name],
        #                           self.controller.devices[3]: self.channels_ai[2].name}
        # # self.controller.devices #ci_meas_types
        # logger.info("DICT DEVICE LABEL {}".format(self.dict_device_input))
        # logger.info(self.dict_device_input.get(self.controller.devices[1]))
        # a = 0
        # b = 0
        # gen = [[ai for ai in self.dict_device_input.get(device)]
        #        for device in self.dict_device_input if not isinstance(self.dict_device_input.get(device), str)] + \
        #       [self.dict_device_input.get(device)
        #        for device in self.dict_device_input if isinstance(self.dict_device_input.get(device), str)]
        #
        # logger.info("DATA TO EXPORT = {}".format(gen))
        #
        #
        #
        # dte = DataToExport(name='NIDAQmx',
        #                    data=[DataFromPlugins(name=self.dict_device_input[device],
        #                                          data=[np.array([data_measurement[i]])
        #                                                for i in range(len(data_measurement))],
        #                                          dim='Data0D',
        #                                          labels=[ai for ai in self.dict_device_input.get(device)]
        #                                          ) for device in self.dict_device_input])


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

            channels = [AIChannel(name="cDAQ1Mod3/ai0",
                                  source='Analog_Input',
                                  analog_type='Voltage',
                                  value_min=-10.0,
                                  value_max=10.0,
                                  termination=DAQ_termination.RSE,
                                  ),
                        ]
            # Create Task
            task = nidaqmx.Task()
            for channel in channels:
                if channel.analog_type == "Thermocouple":
                    task.ai_channels.add_ai_thrmcpl_chan(physical_channel=channel.name,
                                                         name_to_assign_to_channel="Channel 01",
                                                         min_val=channel.value_min,
                                                         max_val=channel.value_max,
                                                         units=TemperatureUnits.DEG_C,
                                                         thermocouple_type=channel.thermo_type,
                                                         )
                elif channel.analog_type == "Voltage":
                    task.ai_channels.add_ai_voltage_chan(physical_channel=channel.name,
                                                         name_to_assign_to_channel="",
                                                         terminal_config=channel.termination,
                                                         min_val=-10.0,
                                                         max_val=10.0,
                                                         units=VoltageUnits.VOLTS,
                                                         custom_scale_name="")
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

            for i in range(10):
                print("channel 01 name : ", channels[0].name)
                data = task.read()
                print("data = ", data)
                print("type(data) = ", type(data))

            task.close()

        except Exception as e:
            print("Exception ({}): {}".format(type(e), str(e)))