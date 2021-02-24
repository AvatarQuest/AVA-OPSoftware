def reboot(self):
    dxl_comm_result, dxl_error = self.packet_handler.reboot(self.port_handler, self.id)
    if dxl_comm_result != 0:
        print("%s" % self.packet_handler.getTxRxResult(dxl_comm_result))
    elif dxl_error != 0:
        print("%s" % self.packet_handler.getRxPacketError(dxl_error))

    print("[ID:%03d] reboot Succeeded\n" % self.id)