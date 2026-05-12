import numpy as np
import akida

from akida.generate.test.test_tools import set_constant_weights


class TestGenerator(akida.generate.EngineTestGenerator):

    def model(self):
        """
        This test hangs the first time it is run on an NSoC_v2 after a cold boot.
        """
        filters = 1
        kernel_size = (3, 3)
        input_shape = (8, 8, 1)
        weights_bits = 8
        act_bits = 1
        padding = akida.Padding.Same
        pool_type = akida.PoolType.Max
        pool_size = (2, 2)
        kernel_stride = (2, 2)
        # Build akida model
        input_layer = akida.InputConvolutional(filters=filters,
                                               kernel_size=kernel_size,
                                               input_shape=input_shape,
                                               padding=padding,
                                               act_bits=act_bits,
                                               weights_bits=weights_bits,
                                               activation=True,
                                               pool_type=pool_type,
                                               pool_size=pool_size,
                                               kernel_stride=kernel_stride)
        model = akida.Model()
        model.add(input_layer)
        set_constant_weights(model, 1)
        return model

    def device(self):
        return akida.AKD1000()

    def inputs(self):
        return np.ones((1, 8, 8, 1), dtype=np.uint8)

    def outputs(self):
        # Forward inputs in software
        self.model().map(None)
        return self.model().forward(self.inputs())
