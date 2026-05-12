def np_info_repr(self):
    data = "<akida.NP.Info"
    data += ", ident=" + str(self.ident)
    data += ", types=" + str(self.types)
    if self.ident.channel > 0:
        data += ", num_channels=" + str(self.ident.channel)
    else:
        data += ", has_lut=" + str(self.has_lut)
    data += ">"
    return data


def np_mesh_repr(self):
    data = "<akida.NP.Mesh"
    data += ", dma_event=" + str(self.dma_event)
    data += ", dma_conf=" + str(self.dma_conf)
    if (self.skip_dmas):
        data += ", skip_dmas=" + str(self.skip_dmas)
    data += ", nps=" + str(self.nps) + ">"
    return data


def np_component_repr(self):
    return "<akida.NP.Component" + \
        ", ident=" + str(self.ident) + \
        ", type=" + str(self.type) + \
        ", filters=" + str(self.filters) + \
        ", single_buffer=" + str(self.single_buffer) + ">"
