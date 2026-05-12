from akida.core import (Layer, Padding, PoolType, LayerType, LayerParams)


class Convolutional(Layer):
    """This represents a standard Convolutional layer.

    A standard Convolution Layer in a network having an input with arbitrary
    number of channels is converted to Convolutional Layer on Akida.
    This layer optionally executes Pooling and ReLU operation to the outputs of
    Convolution.

    The Convolutional layer accepts 1-bit, 2-bit or 4-bit 3D input tensors with
    an arbitrary number of channels.
    The Convolutional layer can be configured with 1-bit, 2-bit or 4-bit weights.
    It applies a convolution (not a cross-correlation) optionally followed by a
    pooling operation to the input tensors.
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
        filters (int): number of filters.
        name (str, optional): name of the layer. Defaults to empty string
        padding (:obj:`Padding`, optional): type of convolution. Defaults to
            Padding.Same.
        kernel_stride (list, optional): list of 2 integers representing the
            convolution stride (X, Y). Defaults to (1, 1).
        weights_bits (int, optional): number of bits used to quantize weights.
             Defaults to 1.
        pool_size (list, optional): list of 2 integers, representing the window
            size over which to take the maximum or the average (depending on
            pool_type parameter). Defaults to (-1, -1).
        pool_type (:obj:`PoolType`, optional): pooling type. Defaults to Pooling.NoPooling.
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
                 weights_bits=1,
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
                LayerType.Convolutional, {
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
