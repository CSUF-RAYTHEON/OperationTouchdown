from akida.core import Layer, LayerParams, LayerType


class Quantizer(Layer):
    """Layer capable of Quantizing an input tensor.

    This quantizes an input tensor, following the equation:

        output = round(input x scales) + zero_points

    Args:
        input_shape (tuple): 3D input shape.
        output_bits (int): output bitwidth.
        output_signed (bool): True if the output is signed, False if unsigned.
        channels_first (bool, optional): True if input shape has channels first. Defaults to False.
        name (str, optional): name of the layer. Defaults to empty string.

    """

    def __init__(self, input_shape, output_bits, output_signed, channels_first=False, name=""):
        try:
            params = LayerParams(LayerType.Quantizer, {
                "input_width": input_shape[0],
                "input_height": input_shape[1],
                "input_channels": input_shape[2],
                "output_bits": output_bits,
                "output_signed": output_signed,
                "channels_first": channels_first
            })

            # Call parent constructor to initialize C++ bindings.
            # Note that we invoke directly __init__ instead of using super, as
            # specified in pybind documentation.
            Layer.__init__(self, params, name)
        except BaseException:
            self = None
            raise
