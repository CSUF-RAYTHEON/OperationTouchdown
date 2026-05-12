from akida.core import (Layer, Padding, PoolType, LayerType, LayerParams)


class SeparableConvolutional(Layer):
    """This represents a separable convolution layer.

    A standard Separable Convolution Layer in a network having an input with
    arbitrary number of channels is converted to SeparableConvolutional Layer
    on Akida.
    This layer optionally executes Pooling and ReLU operation to the outputs of
    Separable Convolution.

    This layer accepts 1-bit, 2-bit or 4-bit 3D input tensors.
    It can be configured with 1-bit, 2-bit or 4-bit weights.
    Separable convolutions consist in first performing a depthwise spatial
    convolution (which acts on each input channel separately) followed by a
    pointwise convolution which mixes together the resulting output channels.
    Note: this layer applies a real convolution, and not a cross-correlation.
    It can optionally apply a step-wise ReLU activation to its outputs.
    The layer expects a 4D tensor whose first dimension is the sample index
    as input.
    It returns a 4D tensor whose first dimension is the sample index and the
    last dimension is the number of convolution filters.
    The order of the input spatial dimensions is preserved, but their value may
    change according to the convolution and pooling parameters.

    Args:
        kernel_size (list): list of 2 integers representing the spatial
            dimensions of the convolutional kernel.
        filters (int): number of pointwise filters.
        name (str, optional): name of the layer. Defaults to empty string.
        padding (:obj:`Padding`, optional): type of convolution. Defaults to
            Padding.Same.
        kernel_stride (list, optional): list of 2 integers representing the
            convolution stride (X, Y). Defaults to (1, 1).
        weights_bits (int, optional): number of bits used to quantize weights.
            Defaults to 2.
        pool_size (list, optional): list of 2 integers, representing the window
            size over which to take the maximum or the average (depending on
            pool_type parameter). Defaults to (-1, -1).
        pool_type (:obj:`PoolType`, optional): pooling type (NoPooling, Max or
            Average). Defaults to PoolType.NoPooling.
        pool_stride (list, optional): list of 2 integers representing
            the stride dimensions. Defaults to (-1, -1).
        activation (bool, optional): enable or disable activation
            function. Defaults to True.
        act_bits (int, optional): number of bits used to quantize
            the neuron response. Defaults to 1.

    """

    def __init__(self,
                 kernel_size,
                 filters,
                 name="",
                 padding=Padding.Same,
                 kernel_stride=(1, 1),
                 weights_bits=2,
                 pool_size=(-1, -1),
                 pool_type=PoolType.NoPooling,
                 pool_stride=(-1, -1),
                 activation=True,
                 act_bits=1):
        try:
            pooling_stride_x = pool_stride[0]
            if pool_stride[0] < 0:
                pooling_stride_x = pool_size[0]
            pooling_stride_y = pool_stride[1]
            if pool_stride[1] < 0:
                pooling_stride_y = pool_size[1]
            params = LayerParams(
                LayerType.SeparableConvolutional, {
                    "kernel_width": kernel_size[0],
                    "kernel_height": kernel_size[1],
                    "padding": padding,
                    "filters": filters,
                    "stride_x": kernel_stride[0],
                    "stride_y": kernel_stride[1],
                    "weights_bits": weights_bits,
                    "pooling_width": pool_size[0],
                    "pooling_height": pool_size[1],
                    "pool_type": pool_type,
                    "pooling_stride_x": pooling_stride_x,
                    "pooling_stride_y": pooling_stride_y,
                    "activation": activation,
                    "act_bits": act_bits
                })

            # Call parent constructor to initialize C++ bindings
            # Note that we invoke directly __init__ instead of using super, as
            # specified in pybind documentation
            Layer.__init__(self, params, name)
        except BaseException:
            self = None
            raise
