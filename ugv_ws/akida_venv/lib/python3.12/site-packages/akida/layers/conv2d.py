from akida.core import Layer, Padding, PoolType, ActivationType, LayerType, LayerParams


class Conv2D(Layer):
    """This represents the Akida V2 Conv2D layer.

    It applies a convolution optionally followed by a bias addition, a
    pooling operation and a ReLU activation.
    Inputs shape must be in the form (X, Y, C). Being the result of a quantized
    operation, it is possible to apply some shifts to adjust the inputs/outputs
    scales to the equivalent operation performed on floats, while maintaining
    a limited usage of bits and performing the operations on integer values.
    The order of the input spatial dimensions is preserved, but their values may
    change according to the convolution and pooling parameters.

    The Conv2D operation can be described as follows:

        >>> inputs = inputs << input_shift
        >>> prod = conv2d(inputs, weights)
        >>> output = prod + (bias << bias_shift) (optional)
        >>> output = pool(output) (optional)
        >>> output = ReLU(output) (optional)
        >>> output = output * output_scale >> output_shift

    Note that output values will be saturated on the range that can be represented with
    output_bits.

    Args:
        filters (int): number of filters.
        kernel_size (int): integer value specifying the height and width of the 2D convolution
            window.
        kernel_stride (int, optional): integer representing the convolution stride across both
            spatial dimensions.
            Defaults to 1.
        padding (:obj:`Padding`, optional): type of convolution rather Padding.Same or
            Padding.Valid.
            Defaults to Padding.Same.
        pool_type (:obj:`PoolType`, optional): pooling type. Defaults to PoolType.NoPooling.
        pool_size (int, optional): integer value specifying the height and width of the window
            over which to take the maximum or the average (depending on pool_type parameter).
            Defaults to -1.
        pool_stride (int, optional): integer representing the stride across both dimensions.
            Defaults to -1.
        output_bits (int, optional): output bitwidth. Defaults to 8.
        buffer_bits (int, optional): buffer bitwidth. Defaults to 28.
        activation (:obj:`ActivationType`, optional): activation type.
            Defaults to `ActivationType.ReLU`.
        post_op_buffer_bits (int, optional): internal bitwidth for post operations. Defaults to 32.
        name (str, optional): name of the layer. Defaults to empty string.

    """

    def __init__(self,
                 filters,
                 kernel_size,
                 kernel_stride=1,
                 padding=Padding.Same,
                 pool_type=PoolType.NoPooling,
                 pool_size=-1,
                 pool_stride=-1,
                 output_bits=8,
                 buffer_bits=28,
                 activation=ActivationType.ReLU,
                 post_op_buffer_bits=32,
                 weights_bits=8,
                 name=""):
        try:
            params = LayerParams(
                LayerType.Conv2D, {
                    "filters": filters,
                    "kernel_size": kernel_size,
                    "kernel_stride": kernel_stride,
                    "padding": padding,
                    "pool_type": pool_type,
                    "pool_size": pool_size,
                    "pool_stride": pool_stride,
                    "output_bits": output_bits,
                    "buffer_bits": buffer_bits,
                    "activation": activation,
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
