import argparse
import os
import glob
import numpy as np

from .core import devices, Model, __version__
from .deploy import deploy_engine
from .generate.model import deploy_cmake
from .generate.application_generator import generate_files


def list_devices():
    devices_list = devices()
    if len(devices_list) == 0:
        print("No devices detected")
    else:
        print("Available devices:")
        for device in devices_list:
            print(device.desc)


def print_metrics(model):
    print("\nModel metrics:")
    for name in ["inference_frames", "inference_clk", "program_clk"]:
        print(f"  {name}: {model.metrics[name]}")
    print("")


def default_fixture_path():
    current_dir = os.path.abspath(os.path.dirname(__file__))
    fixture_path = current_dir + "/generate/test/engine/fixtures/*.py"
    return glob.glob(fixture_path)


def run(model_path, input_data):
    # Try to load the model and retrieve details for testing
    try:
        model = Model(model_path)
        (x, y, z) = model.input_shape
        input_layer = model.get_layer(0)
        bitwidth = input_layer.input_bits
        max_value = 2**bitwidth
        input_layer_idata = input_layer.name == "InputData"
        if input_layer_idata and input_layer.output_signed:
            if bitwidth > 8:
                dtype = np.int16
            else:
                dtype = np.int8
        else:
            dtype = np.uint8
    except Exception as e:
        raise type(e)("Error while loading {model_path}: " + str(e))

    # Try to map the model on the first available device
    try:
        model.map(devices()[0])
        model.summary()
        print("")
    except Exception as e:
        if len(devices()) == 0:
            err_msg = "No devices detected..."
        else:
            err_msg = f"Error while mapping {model_path} on {devices()[0]}: " + str(e)
        raise type(e)(err_msg)

    # Try to load the given input file as a numpy array then as an image
    # Use random data if no input is provided
    if input_data is not None:
        try:
            inputs = np.load(input_data)
        except Exception:
            try:
                from PIL import Image
                image = Image.open(input_data)
                image = image.resize((x, y))
                inputs = np.asarray(image)
                inputs = np.expand_dims(inputs, 0)
            except Exception:
                raise ValueError("Impossible to load the input. \
                                  An image or a numpy array is expected.")
    else:
        print("No input provided, using random data.")
        np.random.seed(0)
        size = (1, x, y, z)
        if dtype == np.uint8:
            inputs = np.random.randint(max_value, size=size, dtype=dtype)
        else:
            inputs = np.random.randint(-max_value/2, max_value/2, size=size, dtype=dtype)

    # Try to enable power measurement
    try:
        from time import sleep
        devices()[0].soc.power_measurement_enabled = True
        # A delay is required for the power logs to be properly populated
        sleep(1)
        floor_power = devices()[0].soc.power_meter.floor
    except Exception:
        floor_power = None
        print("Power measurement disabled...")

    # Send data for inference and provide metrics
    try:
        model.forward(inputs)
        print(f"\nFloor power (mW): {floor_power:.2f}") if floor_power else print("")
        print(f"{model.statistics}")
        print_metrics(model)
    except Exception as e:
        print("Error during inference: " + str(e))


def main():
    parser = argparse.ArgumentParser()
    sp = parser.add_subparsers(dest="action")
    sp.add_parser("devices", help="List available devices")
    sp.add_parser("version", help="Return akida version")
    run_parser = sp.add_parser("run", help="Perform an inference on the available device")
    run_parser.add_argument("-m",
                            "--model",
                            type=str,
                            required=True,
                            help="The source model path")
    run_parser.add_argument("-i",
                            "--input",
                            type=str,
                            default=None,
                            help="Input image or a numpy array")
    gen_parser = sp.add_parser("generate",
                               help="Generate application(s) from fixture file(s).")
    gen_parser.add_argument("--fixture-files",
                            help="A list of python fixture files",
                            nargs="+",
                            type=str,
                            required=True)
    gen_parser.add_argument("--modules-paths",
                            help="Path to additional modules required for fixtures",
                            nargs="+",
                            type=str,
                            default=None)
    gen_parser.add_argument("--dest-path",
                            type=str,
                            default=None,
                            required=True,
                            help="The destination path.")
    engine_parser = sp.add_parser("engine", help="Deploy engine sources and applications")
    # Create parent subparser for arguments shared between engine methods
    engine_parent = argparse.ArgumentParser(add_help=False)
    engine_parent.add_argument(
        "--dest-path",
        type=str,
        default=None,
        required=True,
        help="The destination path.")
    engine_action_parser = engine_parser.add_subparsers(
        dest="engine_action",
        help="Action: deploy or generate.")
    deploy_parser = engine_action_parser.add_parser(
        "deploy",
        help="Deploy the engine library.",
        parents=[engine_parent])
    deploy_parser.add_argument("--with-host-examples", action='store_true',
                               help="Deploy host examples development files")
    args = parser.parse_args()
    if args.action == "devices":
        list_devices()
    elif args.action == "version":
        print(__version__)
    elif args.action == "run":
        run(args.model, args.input)
    elif args.action == "engine":
        if args.engine_action == "deploy":
            deploy_engine(args.dest_path)
            fixture_files = default_fixture_path()
            dest_path = os.path.join(args.dest_path, "engine/test/akd1000")
            generate_files(fixture_files, dest_path, None, args.with_host_examples)
            if args.with_host_examples:
                deploy_cmake(dest_path)
    elif args.action == "generate":
        deploy_cmake(args.dest_path)
        generate_files(args.fixture_files, args.dest_path, args.modules_paths)
