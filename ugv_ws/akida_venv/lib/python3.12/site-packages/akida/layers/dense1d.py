from akida.core import Layer, LayerParams, LayerType, ActivationType


class Dense1D(Layer):
    """Dense layer capable of working on 1D inputs.

    This is a simple dotproduct between an input of shape (1, 1, X) and a kernel
    of shape (X, F) to output a tensor of shape (1, 1, F). Being the result of a
    quantized operation, it is possible to apply some shifts to adjust the
    inputs/outputs scales to the equivalent operation performed on floats, while
    maintaining a limited usage of bits and performing the operations on integer
    values.

    The 1D Dense operation can be described as follows:

        >>> inputs = inputs << input_shift
        >>> prod = matmul(inputs, weights)
        >>> output = prod + (bias << bias_shift)
        >>> output = output * output_scale >> output_shift

    Inputs shape must be (1, 1, X), if not it's reshaped automatically at the
    beginning. Note that output values will be saturated on the range that can be
    represented with output_bits.

    Args:
        units (int): Positive integer, dimensionality of the output space.
        output_bits (int, optional): output bitwidth. Defaults to 8.
        buffer_bits (int, optional): buffer bitwidth. Defaults to 28.
        post_op_buffer_bits (int, optional): internal bitwidth for post operations.
            Defaults to 32.
        activation (:obj:`ActivationType`, optional): activation type.
            Defaults to `ActivationType.NoActivation`.
        name (str, optional): name of the layer. Defaults to empty string.

    """

    def __init__(self,
                 units,
                 output_bits=8,
                 buffer_bits=28,
                 post_op_buffer_bits=32,
                 activation=ActivationType.NoActivation,
                 weights_bits=8,
                 name=""):
        try:
            params = LayerParams(
                LayerType.Dense1D, {
                    "units": units,
                    "output_bits": output_bits,
                    "buffer_bits": buffer_bits,
                    "post_op_buffer_bits": post_op_buffer_bits,
                    "activation": activation,
                    "weights_bits": weights_bits
                })
            # Call parent constructor to initialize C++ bindings
            # Note that we invoke directly __init__ instead of using super, as
            # specified in pybind documentation
            Layer.__init__(self, params, name)
        except BaseException:
            self = None
            raise
