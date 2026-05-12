from akida.core import Layer, LayerType, ActivationType, LayerParams


class BufferTempConv(Layer):
    """This represents the Akida BufferTempConv layer.

    To mitigate the memory requirements of a real 3D spatiotemporal convolution,
    this spatiotemporal layer can store a FIFO buffer caching the past inputs and
    perform a standard 2-D convolution between the buffered inputs and the layer
    kernel.
    The FIFO has a limited storage capacity (T), determined by its parameter
    fifo_size.
    Inputs shape must be in the form (X, Y, C). But FIFO has a shape of (X, Y, T, C).

    The BufferTempConv convolution operation can be described as follows:

        >>> fifo = fifo_op(inputs)
        >>> fifo = fifo << input_shift
        >>> output = conv2d(fifo, weights)
        >>> output = output + (bias << bias_shift) (optional)
        >>> output = ReLU(output) (optional)
        >>> output = output * output_scale >> output_shift

    Where fifo_op pushes a new sample to the FIFO and pop out the oldest one (The storage
    occurs on the axis T of the FIFO where FIFO shape is (X, Y, T, C))

    Note that output values will be saturated on the range that can be represented with
    output_bits.

    Args:
        filters (int): number of filters.
        fifo_size (int): integer value specifying the FIFO capacity.
        activation (:obj:`ActivationType`, optional): activation type.
            Defaults to `ActivationType.ReLU`.
        output_bits (int, optional): output bitwidth. Defaults to 8.
        buffer_bits (int, optional): buffer bitwidth. Defaults to 32.
        post_op_buffer_bits (int, optional): internal bitwidth for post operations. Defaults to 32.
        name (str, optional): name of the layer. Defaults to empty string.

    """

    def __init__(self,
                 filters,
                 fifo_size,
                 activation=ActivationType.ReLU,
                 output_bits=8,
                 buffer_bits=32,
                 post_op_buffer_bits=32,
                 weights_bits=8,
                 name=""):
        try:
            params = LayerParams(
                LayerType.BufferTempConv, {
                    "filters": filters,
                    "fifo_size": fifo_size,
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
