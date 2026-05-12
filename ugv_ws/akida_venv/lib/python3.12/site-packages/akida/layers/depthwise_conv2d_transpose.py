from akida.core import Layer, LayerType, LayerParams, ActivationType


class DepthwiseConv2DTranspose(Layer):
    """This represents the Akida V2 DepthwiseConv2DTranspose layer.

    It applies a transposed depthwise convolution (also called deconvolution) optionally followed
    by a bias addition and a ReLU activation.
    This is like a standard transposed convolution, except it acts on each input channel
    separately.
    Inputs shape must be in the form (X, Y, C). Being the result of a quantized operation, it is
    possible to apply some shifts to adjust the inputs/outputs scales to the equivalent operation
    performed on floats, while maintaining a limited usage of bits and performing the operations on
    integer values.
    The order of the input spatial dimensions is preserved, but their values may change according
    to the layer parameters.
    Note that the layer performs only transpose depthwise convolution with a "Same" padding and a
    kernel stride equal to 2.

    The DepthwiseConv2DTranspose operation can be described as follows:

        >>> inputs = inputs << input_shift
        >>> prod = depthwise_conv2d_transpose(inputs, weights)
        >>> output = prod + (bias << bias_shift) #optional
        >>> output = ReLU(output) #optional
        >>> output = output * output_scale >> output_shift

    Note that output values will be saturated on the range that can be represented with
    output_bits.

    Args:
        kernel_size (int): Integer representing the spatial dimensions of the depthwise kernel.
        activation (:obj:`ActivationType`, optional): activation type.
            Defaults to `ActivationType.ReLU`.
        output_bits (int, optional): output bitwidth. Defaults to 8.
        buffer_bits (int, optional): buffer bitwidth. Defaults to 28.
        post_op_buffer_bits (int, optional): internal bitwidth for post operations. Defaults to 32.
        name (str, optional): name of the layer. Defaults to empty string.

    """

    def __init__(self,
                 kernel_size,
                 activation=ActivationType.ReLU,
                 output_bits=8,
                 buffer_bits=28,
                 post_op_buffer_bits=32,
                 weights_bits=8,
                 name=""):
        try:
            params = LayerParams(
                LayerType.DepthwiseConv2DTranspose, {
                    "kernel_size": kernel_size,
                    "activation": activation,
                    "output_bits": output_bits,
                    "buffer_bits": buffer_bits,
                    "post_op_buffer_bits": post_op_buffer_bits,
                    "weights_bits": weights_bits
                })
            # Call parent constructor to initialize C++ bindings
            # Note that we invoke directly __init__ instead of using super, as
            # specified in pybind documentation
            Layer.__init__(self, params, name)
        except BaseException:
            self = None
            raise
