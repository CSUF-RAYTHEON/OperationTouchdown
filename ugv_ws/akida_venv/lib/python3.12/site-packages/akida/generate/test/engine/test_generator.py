import os
from pathlib import Path

from ...application_generator import ApplicationGenerator
from ...array_to_cpp import array_to_cpp
from ..template import test_file_from_template


class EngineTestGenerator(ApplicationGenerator):

    def program(self):
        model = self.model()
        device = self.device()
        if model is not None and device is not None:
            model.map(device, hw_only=True)
            return model.sequences[0].program
        return None

    def generate(self, test_name, dest_path):
        """Generate engine test application files
        """
        program = self.program()
        inputs = self.inputs()
        outputs = self.outputs()
        dest_dir = os.path.join(dest_path, test_name)
        # Generate source arrays
        if program is not None:
            array_to_cpp(dest_dir, program, "program")
            tpl_path = os.path.join(Path(__file__).parent, "app_templates")
            # Write test files
            test_file_from_template(test_name, tpl_path, dest_path, "test.h")
            test_file_from_template(
                test_name, tpl_path, dest_path, "test.cpp")
        if inputs is not None:
            array_to_cpp(dest_dir, inputs, "inputs")
        if outputs is not None:
            array_to_cpp(dest_dir, outputs, "outputs")

    def generate_host_example_files(self, test_name, dest_path):
        # Write host main files
        tpl_path = os.path.join(Path(__file__).parent, "app_templates")
        # Add template main.cpp
        test_file_from_template(test_name, tpl_path, dest_path, "main.cpp")
        # Add template CMakeLists.txt
        test_file_from_template(test_name, tpl_path, dest_path, "CMakeLists.txt")
