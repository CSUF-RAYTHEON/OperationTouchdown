#include "akida/dense.h"
#include "akida/hardware_device.h"
#include "akida/program_info.h"

#include "akida/input_conversion.h"
#include "infra/system.h"

#include "TEST_NAME/inputs.h"
#include "TEST_NAME/outputs.h"
#include "TEST_NAME/program.h"
#include "TEST_NAME/test.h"

static std::vector<akida::TensorUniquePtr> to_sparse(
    const std::vector<akida::TensorConstPtr>& dense_inputs,
    const akida::ProgramInfo& program_info) {
  std::vector<akida::TensorUniquePtr> result;
  result.reserve(dense_inputs.size());
  for (const auto& input : dense_inputs) {
    result.push_back(akida::conversion::to_sparse(
        *static_cast<const akida::Dense*>(input.get()), program_info));
  }
  return result;
}

bool TEST_NAME(akida::HardwareDriver* driver,
               on_engine_event_t on_engine_event) {
  // Instantiate the device for the corresponding driver
  auto device = akida::HardwareDevice::create(driver);
  on_engine_event(EngineProgramStart);
  // Program device
  const auto program_info = device->program(program, program_len);
  on_engine_event(EngineProgramSuccess);
  // Wrap inputs inside a Dense
  auto input_tensor = akida::Dense::create_view(
      reinterpret_cast<const char*>(inputs), inputs_type, inputs_shape,
      akida::Dense::Layout::RowMajor);
  // Split inputs in sub-tensors because generated inputs are 4D tensors, and
  // engine enqueue only accepts 3D tensor
  auto input_vector = akida::Dense::split(*input_tensor);
  // Wrap expected outputs inside a Dense
  auto output_tensor = akida::Dense::create_view(
      reinterpret_cast<const char*>(outputs), outputs_type, outputs_shape,
      akida::Dense::Layout::RowMajor);
  auto expected_vector = akida::Dense::split(*output_tensor);

  // set batch size to number of inputs (allocate inputs if no memory is visible
  // from akida)
  device->set_batch_size(input_vector.size(),
                         driver->akida_visible_memory() == 0);

  // convert inputs if required
  std::vector<akida::TensorUniquePtr> sparse_inputs;
  bool conversion_required = !program_info.input_is_dense();
  if (conversion_required) {
    sparse_inputs = to_sparse(input_vector, program_info);
  }

  // used to detect eventual timeout (should not happen)
  auto last_output_read = time_ms();
  static constexpr int32_t timeout = 5000;  // 5s timeout

  // loop until we read all outputs
  std::vector<akida::TensorUniquePtr> obtained_vector;
  size_t nb_inputs_queued = 0;
  while (obtained_vector.size() < input_vector.size()) {
    // keep system alive
    kick_watchdog();

    // enqueue as many jobs as current pipeline allow us
    bool pipeline_ready = true;
    while (nb_inputs_queued < input_vector.size() && pipeline_ready) {
      const auto& input = conversion_required ? *sparse_inputs[nb_inputs_queued]
                                              : *input_vector[nb_inputs_queued];
      // try to enqueue
      on_engine_event(EngineEnqueueStart);
      pipeline_ready = device->enqueue(input);
      // if input was inserted, increment counter
      if (pipeline_ready) {
        on_engine_event(EngineEnqueueSuccess);
        ++nb_inputs_queued;
      } else {
        on_engine_event(EngineEnqueueFailed);
      }
    }

    // now try to fetch outputs
    bool output_ready = true;
    while (output_ready) {
      on_engine_event(EngineFetchStart);
      auto output = device->fetch();
      output_ready = output != nullptr;
      // if an output was ready, increment counter
      if (output_ready) {
        on_engine_event(EngineFetchSuccess);
        // if expected outputs were float, we need to dequantize potentials
        if (outputs_type == akida::TensorType::float32) {
          obtained_vector.push_back(
              device->dequantize(*akida::conversion::as_dense(*output)));
        } else {
          obtained_vector.push_back(std::move(output));
        }
        last_output_read = time_ms();
      } else if (time_ms() - last_output_read > timeout) {
        panic("Fatal error: timed out while fetching output");
      } else {
        on_engine_event(EngineFetchFailed);
      }
    }
  }

  // check the  number of output is as expected
  if (obtained_vector.size() != expected_vector.size()) {
    return false;
  }
  // Compare each individual output
  for (size_t i = 0; i < obtained_vector.size(); ++i) {
    if (!(*obtained_vector[i] == *expected_vector[i])) {
      return false;
    }
  }
  return true;
}
