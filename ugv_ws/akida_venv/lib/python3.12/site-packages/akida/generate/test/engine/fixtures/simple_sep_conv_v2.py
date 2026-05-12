import akida

from akida.generate.test.test_tools import get_cyclic_input, set_cyclic_weights


class TestGenerator(akida.generate.EngineTestGenerator):

    def model(self):
        input_shape = (6, 6, 3)
        kernel_size = (3, 3)
        filters = 10
        model = akida.Model()
        model.add(akida.InputData(input_shape))
        model.add(
            akida.SeparableConvolutional(kernel_size, filters,
                                         activation=False))
        # Set cyclic weights
        set_cyclic_weights(model)
        return model

    def device(self):
        return akida.AKD1000()

    def inputs(self):
        # Generate cyclic inputs
        return get_cyclic_input(self.model(), 2)

    def outputs(self):
        # Forward inputs in software
        self.model().map(None)
        return self.model().forward(self.inputs())
