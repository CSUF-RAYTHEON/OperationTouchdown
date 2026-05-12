import akida

from akida.generate.test.test_tools import get_cyclic_input, set_cyclic_weights


class TestGenerator(akida.generate.EngineTestGenerator):

    def model(self):
        model = akida.Model()
        model.add(akida.InputData((1, 1, 2048), 4))
        # Add FullyConnected layer
        fc = akida.FullyConnected(1000, weights_bits=4, activation=False)
        model.add(fc)
        # Set cyclic weights
        set_cyclic_weights(model)
        return model

    def device(self):
        return akida.AKD1000()

    def inputs(self):
        # Generate cyclic inputs
        return get_cyclic_input(self.model(), 10)

    def outputs(self):
        # Forward inputs in software
        self.model().map(None)
        return self.model().forward(self.inputs())
