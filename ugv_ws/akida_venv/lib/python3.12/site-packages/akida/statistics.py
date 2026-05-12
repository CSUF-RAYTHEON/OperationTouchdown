from statistics import stdev


class Statistics():
    """Provides inference statistics.

    Args:
        model (:obj:`Model`, optional): get statistics from model. Defaults to None.
        device (:obj:`Device`, optional): get statistics from device. Defaults to None.
    """

    def __init__(self, model=None, device=None):
        if model:
            metrics = model.metrics
            power_events = model.power_events
        elif device:
            # Check if we have a soc device to get power events
            soc = device.soc
            power_events = None
            if soc and soc.power_measurement_enabled:
                power_events = device.inference_power_events
            metrics = device.metrics
        self._fps = None
        self._powers = {}
        self._energy = None
        self._inference_clk = None
        self._program_clk = None
        if "inference_start" in metrics.names:
            inf_start = metrics["inference_start"]
            inf_end = metrics["inference_end"]
            frames = metrics["inference_frames"]
            duration = (inf_end - inf_start) / 1000
            if duration > 0:
                self._fps = frames / duration
            if power_events:
                # get power events between inference start & end
                powers = []
                for event in power_events:
                    if event.ts >= inf_start and event.ts <= inf_end:
                        powers.append(event.power)
                num_powers = len(powers)
                if num_powers > 1:
                    # Remove first value
                    powers = powers[1:]
                    # get avg/min/max
                    self._powers["Avg"] = sum(powers) / len(powers)
                    self._powers["Min"] = min(powers)
                    self._powers["Max"] = max(powers)
                    if len(powers) > 1:
                        self._powers["Std"] = stdev(powers)
                    # evaluate the energy consumed by frame
                    # It is average power * duration / frame
                    self._energy = self._powers["Avg"] * duration / frames
        if "inference_clk" in metrics.names:
            self._inference_clk = metrics["inference_clk"]
        if "program_clk" in metrics.names:
            self._program_clk = metrics["program_clk"]

    def __repr__(self):
        fps = "N/A" if self.fps is None else "%.2f" % self._fps
        data = "fps: " + fps
        if self._powers:
            data += ", powers: " + str(self._powers)
        if self._energy:
            data += ", energy: " + str(self._energy)
        if self._inference_clk:
            data += ", inference clock: " + str(self._inference_clk)
        if self._program_clk:
            data += ", program clock: " + str(self._program_clk)
        return data

    def __str__(self):
        fps = "N/A" if self.fps is None else "%.2f" % self._fps + " fps"
        data = "Average framerate = " + fps
        if self._powers:
            data += "\nLast inference power range (mW): "
            num_powers = len(self._powers)
            for index, (key, value) in enumerate(self._powers.items()):
                data += " {} {:.2f} ".format(key, value)
                if index != num_powers - 1:
                    data += "/"
        if self._energy:
            data += "\nLast inference energy consumed (mJ/frame): {:.2f}".format(
                self._energy)
        if self._inference_clk:
            data += "\nLast inference clock: {}".format(
                self._inference_clk)
        if self._program_clk:
            data += "\nLast program clock: {}".format(
                self._program_clk)
        return data

    @property
    def fps(self):
        """Returns the frames per seconds for the last inference batch.

        Returns:
            a float value in frames/s.
        """
        return self._fps

    @property
    def powers(self):
        """Returns the power ranges during the last inference batch.

        Note that the power measurements must be enabled for the device.

        Note also that the inference must last long enough to provide meaningful
        power measurements: try increasing the number of samples and/or batch
        size if power ranges are missing.

        Returns:
            a dictionary of float power values in mW indexed by name (where
            names are in ['Avg', 'Min', 'Max', 'Std']).
        """
        return self._powers

    @property
    def energy(self):
        """Returns the energy consumed during the last inference batch.

        This corresponds to the average amount of energy consumed to process one
        frame.

        Returns:
            a float value in mJ/frame.
        """
        return self._energy

    @property
    def inference_clk(self):
        """Returns the clock count during the last inference batch.

        Returns:
            an int value.
        """
        return self._inference_clk

    @property
    def program_clk(self):
        """Returns the clock count during the last programming.

        Returns:
            an int value.
        """
        return self._program_clk
