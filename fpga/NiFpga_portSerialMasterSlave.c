/*
 * Generated with the FPGA Interface C API Generator 24.3
 * for NI-RIO 24.3 or later.
 */
#include "NiFpga_portSerialMasterSlave.h"

void NiFpga_portSerialMasterSlave_IndicatorCluster_FPGAErrorOut_UnpackCluster(
   const uint8_t* const packedData,
   NiFpga_portSerialMasterSlave_IndicatorCluster_FPGAErrorOut_Type* const destination)
{
   (*destination).status = 0;
   (*destination).status |= ((packedData[0] >> 7) & 0x1);
   (*destination).code = 0;
   (*destination).code |= (packedData[0] & 0x7FULL) << 25;
   (*destination).code |= (packedData[1] & 0xFF) << 17;
   (*destination).code |= (packedData[2] & 0xFF) << 9;
   (*destination).code |= (packedData[3] & 0xFF) << 1;
   (*destination).code |= ((packedData[4] >> 7) & 0x1);
}

void NiFpga_portSerialMasterSlave_IndicatorCluster_FPGAErrorOut_PackCluster(
   uint8_t* const packedData,
   const NiFpga_portSerialMasterSlave_IndicatorCluster_FPGAErrorOut_Type* const source)
{
   packedData[0] = (uint8_t)(((*source).status & 0x1) << 7);
   packedData[0] |= (uint8_t)(((*source).code >> 25) & 0x7F);
   packedData[1] = (uint8_t)(((*source).code >> 17) & 0xFF);
   packedData[2] = (uint8_t)(((*source).code >> 9) & 0xFF);
   packedData[3] = (uint8_t)(((*source).code >> 1) & 0xFF);
   packedData[4] = (uint8_t)(((*source).code & 0x1) << 7);
}

void NiFpga_portSerialMasterSlave_ControlCluster_SerialConfig_UnpackCluster(
   const uint8_t* const packedData,
   NiFpga_portSerialMasterSlave_ControlCluster_SerialConfig_Type* const destination)
{
   (*destination).BaudRate = 0;
   (*destination).BaudRate |= (packedData[0] & 0xFF) << 8;
   (*destination).BaudRate |= (packedData[1] & 0xFF);
   (*destination).Parity = 0;
   (*destination).Parity |= (packedData[2] & 0xFF);
   (*destination).FlowControl = 0;
   (*destination).FlowControl |= (packedData[3] & 0xFF);
   (*destination).DataBits = 0;
   (*destination).DataBits |= (packedData[4] & 0xFF);
   (*destination).StopBits = 0;
   (*destination).StopBits |= (packedData[5] & 0xFF);
}

void NiFpga_portSerialMasterSlave_ControlCluster_SerialConfig_PackCluster(
   uint8_t* const packedData,
   const NiFpga_portSerialMasterSlave_ControlCluster_SerialConfig_Type* const source)
{
   packedData[0] = (uint8_t)(((*source).BaudRate >> 8) & 0xFF);
   packedData[1] = (uint8_t)((*source).BaudRate & 0xFF);
   packedData[2] = (uint8_t)((*source).Parity & 0xFF);
   packedData[3] = (uint8_t)((*source).FlowControl & 0xFF);
   packedData[4] = (uint8_t)((*source).DataBits & 0xFF);
   packedData[5] = (uint8_t)((*source).StopBits & 0xFF);
}

void NiFpga_portSerialMasterSlave_ControlCluster_errorin_UnpackCluster(
   const uint8_t* const packedData,
   NiFpga_portSerialMasterSlave_ControlCluster_errorin_Type* const destination)
{
   (*destination).status = 0;
   (*destination).status |= ((packedData[0] >> 7) & 0x1);
   (*destination).code = 0;
   (*destination).code |= (packedData[0] & 0x7FULL) << 25;
   (*destination).code |= (packedData[1] & 0xFF) << 17;
   (*destination).code |= (packedData[2] & 0xFF) << 9;
   (*destination).code |= (packedData[3] & 0xFF) << 1;
   (*destination).code |= ((packedData[4] >> 7) & 0x1);
}

void NiFpga_portSerialMasterSlave_ControlCluster_errorin_PackCluster(
   uint8_t* const packedData,
   const NiFpga_portSerialMasterSlave_ControlCluster_errorin_Type* const source)
{
   packedData[0] = (uint8_t)(((*source).status & 0x1) << 7);
   packedData[0] |= (uint8_t)(((*source).code >> 25) & 0x7F);
   packedData[1] = (uint8_t)(((*source).code >> 17) & 0xFF);
   packedData[2] = (uint8_t)(((*source).code >> 9) & 0xFF);
   packedData[3] = (uint8_t)(((*source).code >> 1) & 0xFF);
   packedData[4] = (uint8_t)(((*source).code & 0x1) << 7);
}

void NiFpga_portSerialMasterSlave_IndicatorCluster_errorout_UnpackCluster(
   const uint8_t* const packedData,
   NiFpga_portSerialMasterSlave_IndicatorCluster_errorout_Type* const destination)
{
   (*destination).status = 0;
   (*destination).status |= ((packedData[0] >> 7) & 0x1);
   (*destination).code = 0;
   (*destination).code |= (packedData[0] & 0x7FULL) << 25;
   (*destination).code |= (packedData[1] & 0xFF) << 17;
   (*destination).code |= (packedData[2] & 0xFF) << 9;
   (*destination).code |= (packedData[3] & 0xFF) << 1;
   (*destination).code |= ((packedData[4] >> 7) & 0x1);
}

void NiFpga_portSerialMasterSlave_IndicatorCluster_errorout_PackCluster(
   uint8_t* const packedData,
   const NiFpga_portSerialMasterSlave_IndicatorCluster_errorout_Type* const source)
{
   packedData[0] = (uint8_t)(((*source).status & 0x1) << 7);
   packedData[0] |= (uint8_t)(((*source).code >> 25) & 0x7F);
   packedData[1] = (uint8_t)(((*source).code >> 17) & 0xFF);
   packedData[2] = (uint8_t)(((*source).code >> 9) & 0xFF);
   packedData[3] = (uint8_t)(((*source).code >> 1) & 0xFF);
   packedData[4] = (uint8_t)(((*source).code & 0x1) << 7);
}
