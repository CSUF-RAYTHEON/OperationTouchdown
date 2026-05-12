from .core import (BackendType, Padding, PoolType, ActivationType, LayerType, HwVersion, NP,
                   MapConstraints, Model, Layer, AkidaUnsupervised, Device,
                   HardwareDevice, devices, NSoC_v1, NSoC_v2,
                   TwoNodesIP_v1, AKD1500_v1, FPGA_v2, PowerMeter, PowerEvent, Sequence,
                   Pass, soc, LayerParams, Optimizer, __version__,
                   get_program_memory_infos, nn, evaluate_bitwidth, IpVersion, boardAkd1500,
                   graph_utils)

from .layer import *
from .model import *
from .layers import *
from .statistics import Statistics
from .np import *
from .sequence import *
from .virtual_devices import *

Model.__str__ = model_str
Model.__repr__ = model_repr
Model.statistics = statistics
Model.summary = summary
Model.predict_classes = predict_classes
Model.to_dict = model_to_dict
Model.from_dict = staticmethod(model_from_dict)
Model.to_json = model_to_json
Model.from_json = staticmethod(model_from_json)
Model.map = map

Layer.__str__ = layer_str
Layer.__repr__ = layer_repr
Layer.set_variable = set_variable
Layer.get_variable = get_variable
Layer.get_variable_names = get_variable_names
Layer.get_learning_histogram = get_learning_histogram
Layer.to_dict = layer_to_dict

Sequence.__repr__ = sequence_repr
Pass.__repr__ = pass_repr

NP.Info.__repr__ = np_info_repr
NP.Mesh.__repr__ = np_mesh_repr
NP.Component.__repr__ = np_component_repr
