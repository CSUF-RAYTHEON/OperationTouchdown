from akida.core import Layer, LayerType, ActivationType, LayerParams


class Conv2DTranspose(Layer):
    """This represents the Akida V2 Conv2DTranspose layer.

    It applies a transposed convolution (also called deconvolution) optionally followed by a bias
    addition and a ReLU activation.
    Inputs shape must be in the form (X, Y, C). Being the result of a quantized operation, it is
    possible to apply some shifts to adjust the inputs/outputs scales to the equivalent operation
    performed on floats, while maintaining a limited usage of bits and performing the operations on
    integer values.
    The order of the input spatial dimensions is preserved, but their values may change according
    to the layer parameters.
    Note that the layer performs only transpose convolution with a "Same" padding and a kernel
    stride equal to 2.

    The Conv2DTranspose operation can be described as follows:

        >>> inputs = inputs << input_shift
        >>> prod = conv2d_transpose(inputs, weights)
        >>> output = prod + (bias << bias_shift) #optional
        >>> output = ReLU(output) #optional
        >>> output = output * output_scale >> output_shift

    Note that output values will be saturated on the range that can be represented with
    output_bits.

    Args:
        filters (int): number of filters.
        kernel_size (int): integer value specifying the height and width of the 2D convolution
            window.
        activation (:obj:`ActivationType`, optional): activation type.
            Defaults to `ActivationType.ReLU`.
        output_bits (int, optional): output bitwidth. Defaults to 8.
        buffer_bits (int, optional): buffer bitwidth. Defaults to 28.
        post_op_buffer_bits (int, optional): internal bitwidth for post operations. Defaults to 32.
        name (str, optional): name of the layer. Defaults to empty string.

    """

    def __init__(self,
                 filters,
                 kernel_size,
                 activation=ActivationType.ReLU,
                 output_bits=8,
                 buffer_bits=28,
                 post_op_buffer_bits=32,
                 weights_bits=8,
                 name=""):
        try:
            params = LayerParams(
                LayerType.Conv2DTranspose, {
                    "filters": filters,
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
