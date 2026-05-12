import os
import sys
import importlib.util
import inspect
from pathlib import Path
import glob


class ApplicationGenerator():
    """This class is an interface for `akida generate` cli command

    In order to generate a sample application with `akida generate` commands,
    one must create a class that inherits from this class, and override methods
    to return the desired value.
    """

    def generate(self, name, dest_path):
        """Generate application files

        Override this method to generate the application.
        """
        raise NotImplementedError()


def _get_generator(fixture_file):
    # Add the fixture file directory to the system path to allow relative
    # imports form there
    path = os.path.split(fixture_file)[0]
    sys.path.insert(0, path)
    # Dynamically load the module corresponding to the fixture file
    name = Path(fixture_file).stem
    spec = importlib.util.spec_from_file_location(name, fixture_file)
    if spec is None:
        return None, None
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    # Loop over module class symbols
    for _, obj in inspect.getmembers(module, inspect.isclass):
        # Only check for members that are defined in the current module, and not imported ones
        if obj.__module__ == name:
            # Identify children of ApplicationGenerator
            if ApplicationGenerator in obj.mro():
                return name, obj()

    print(f"No ApplicationGenerator children class found in {fixture_file}")
    return None, None


def generate_files(fixtures, dest_path, modules_paths=None, host_example_files=False):
    # Add additional modules path to the system path to allow the model files
    # to import their dependencies
    if modules_paths is not None:
        for path in modules_paths:
            sys.path.insert(0, path)
    # Evaluate fixtures as globbing patterns
    fixture_files = []
    for fixture in fixtures:
        fixture_files.extend(glob.glob(fixture))
    # Iterate over each fixture file
    for fixture_file in fixture_files:
        if not os.path.isfile(fixture_file):
            continue
        # Each fixture file should contain an ApplicationGenerator class
        name, generator = _get_generator(fixture_file)
        if name is not None:
            # Generate application
            generator.generate(name, dest_path)
            if host_example_files:
                generator.generate_host_example_files(name, dest_path)
