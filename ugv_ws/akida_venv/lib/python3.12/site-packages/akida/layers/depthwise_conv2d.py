from akida.core import Layer, LayerParams, LayerType, Padding, PoolType, ActivationType


class DepthwiseConv2D(Layer):
    """This represents a depthwise convolutional layer.

    This is like a standard convolution, except it acts on each input channel separately.
    There is a single filter per input channel, so weights shape is (X, Y, F).
    Being the result of a quantized operation, it is possible to apply some shifts to adjust the
    inputs/outputs scales to the equivalent operation performed on floats, while maintaining a
    limited usage of bits and performing the operations on integer values.

    Note: this layer applies a real convolution, and not a cross-correlation. It can optionally
    apply a step-wise ReLU activation to its outputs.
    The layer expects a 4D tensor whose first dimension is the sample index as input.

    It returns a 4D tensor whose first dimension is the sample index and the last dimension is the
    number of convolution filters, so the same as input channels.
    The order of the input spatial dimensions is preserved, but their value may change according to
    the convolution and pooling parameters.

    Args:
        kernel_size (int): Integer representing the spatial dimensions of the depthwise kernel.
        kernel_stride (int, optional): Integer representing the spatial convolution stride.
            Defaults to 1.
        padding (:obj:`Padding`, optional): type of convolution. Defaults to Padding.Same.
        pool_type (:obj:`PoolType`, optional): pooling type (NoPooling, or Max). Defaults to
            PoolType.NoPooling.
        pool_size (int, optional): Integer representing the window size over which to take the
            maximum. Defaults to -1.
        pool_stride (int, optional): Integer representing the pooling stride dimensions. A value of
            -1 means same as pool_size. Defaults to -1.
        output_bits (int, optional): output bitwidth. Defaults to 8.
        buffer_bits (int, optional): buffer bitwidth. Defaults to 28.
        post_op_buffer_bits (int, optional): internal bitwidth for post operations. Defaults to 32.
        activation (:obj:`ActivationType`, optional): activation type.
            Defaults to `ActivationType.ReLU`.
        name (str, optional): name of the layer. Defaults to empty string.

    """

    def __init__(self,
                 kernel_size,
                 kernel_stride=1,
                 padding=Padding.Same,
                 pool_type=PoolType.NoPooling,
                 pool_size=-1,
                 pool_stride=-1,
                 output_bits=8,
                 buffer_bits=28,
                 post_op_buffer_bits=32,
                 activation=ActivationType.ReLU,
                 weights_bits=8,
                 name=""):
        try:
            params = LayerParams(
                LayerType.DepthwiseConv2D, {
                    "kernel_size": kernel_size,
                    "kernel_stride": kernel_stride,
                    "padding": padding,
                    "pool_type": pool_type,
                    "pool_size": pool_size,
                    "pool_stride": pool_stride,
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
