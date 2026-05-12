from akida.core import Layer, LayerParams, LayerType


class Add(Layer):
    """Layer that adds two inputs from incoming layers.

    It takes as input the output tensors from the input layers, all of the same
    shape, and returns a single tensor (also of the same shape).
    Add layers require Incoming input layers to produce output tensors of the
    same type.
    The Add layer will create three variables, `a_shift`, `b_shift` and
    `output_shift`.
    The operation it will perform on each couple of integer values on input
    tensors (a, b) is equivalent to:

        >>>  a1 = a << a_shift
        >>>  b1 = b << b_shift
        >>>  intermediate_output = a1 + b1
        >>>  for i, shift in enumerate(output_shift):
        >>>      if shift > 0:
        >>>          output[i] = intermediate_output[i] << |shift|
        >>>      else:
        >>>          output[i] = intermediate_output[i] >> |shift|

    Note that output values will be saturated on the range that can be
    represented with output_bits.

    Args:
        output_bits (int, optional): output bitwidth. Defaults to 8.
        buffer_bits (int, optional): internal bitwidth. Defaults to 32.
        post_op_buffer_bits (int, optional): internal bitwidth for post operations. Defaults to 32.
        name (str, optional): name of the layer. Defaults to empty string.

    """

    def __init__(self,
                 output_bits=8,
                 buffer_bits=32,
                 post_op_buffer_bits=32,
                 name=""):
        try:
            params = LayerParams(
                LayerType.Add, {
                    "output_bits": output_bits,
                    "buffer_bits": buffer_bits,
                    "post_op_buffer_bits": post_op_buffer_bits,
                })
            # Call parent constructor to initialize C++ bindings
            # Note that we invoke directly __init__ instead of using super, as
            # specified in pybind documentation
            Layer.__init__(self, params, name)
        except BaseException:
            self = None
            raise
