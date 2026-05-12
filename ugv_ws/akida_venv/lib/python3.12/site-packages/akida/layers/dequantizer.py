from akida.core import Layer, LayerParams, LayerType


class Dequantizer(Layer):
    """Layer capable of dequantizing an input tensor.

    This resolves the scales of an input tensor, following the equation:

        output = input x scales

    Args:
         name (str, optional): name of the layer. Defaults to empty string.

    """

    def __init__(self, name=""):
        try:
            params = LayerParams(LayerType.Dequantizer, {})
            Layer.__init__(self, params, name)
        except BaseException:
            self = None
            raise
