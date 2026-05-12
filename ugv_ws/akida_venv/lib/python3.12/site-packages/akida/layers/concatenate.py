from akida.core import (Layer, LayerParams, LayerType)


class Concatenate(Layer):
    """Layer that concatenates two or more inputs from incoming layers, along
    the last dimensions.

    The operation is equivalent to this numpy operation

        >>>  # Inputs are a and b
        >>>  output = np.concatenate((a, b), axis=-1)

    All inbound layers should have the same output dimensions on the first two
    axis. All inbound layers should have the same output bitwidth and output
    sign.

    Args:
        name (str, optional): name of the layer. Defaults to empty string.
    """

    def __init__(self, name=""):
        try:
            params = LayerParams(
                LayerType.Concatenate, {})
            # Call parent constructor to initialize C++ bindings
            # Note that we invoke directly __init__ instead of using super, as
            # specified in pybind documentation
            Layer.__init__(self, params, name)
        except BaseException:
            self = None
            raise
