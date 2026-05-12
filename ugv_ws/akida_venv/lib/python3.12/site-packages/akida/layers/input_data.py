from akida.core import (Layer, LayerParams, LayerType)


class InputData(Layer):
    """This layer is used to specify the Model input dimensions and bitwidth.

    It specifically targets Models accepting signed or low bitwidth inputs, or if
    the channel number is neither 1 nor 3.
    For images input, model must start instead with an image-specific input layer.

    Args:
        input_shape (tuple): the 3D input shape.
        input_bits (int, optional): input bitwidth. Defaults to 4.
        name (str, optional): name of the layer. Defaults to empty string.

    """

    def __init__(self, input_shape, input_bits=4, name=""):
        try:
            params = LayerParams(
                LayerType.InputData, {
                    "input_width": input_shape[0],
                    "input_height": input_shape[1],
                    "input_channels": input_shape[2],
                    "input_signed": 0 if input_bits <= 4 else 1,
                    "input_bits": input_bits
                })

            # Call parent constructor to initialize C++ bindings
            # Note that we invoke directly __init__ instead of using super, as
            # specified in pybind documentation
            Layer.__init__(self, params, name)
        except BaseException:
            self = None
            raise
