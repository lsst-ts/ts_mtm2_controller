/*
 * Generated with the FPGA Interface C API Generator 24.3
 * for NI-RIO 24.3 or later.
 */
#ifndef __NiFpga_portSerialMasterSlave_h__
#define __NiFpga_portSerialMasterSlave_h__

#ifndef NiFpga_Version
   #define NiFpga_Version 243
#endif

#include "NiFpga.h"

/**
 * The filename of the FPGA bitfile.
 *
 * This is a #define to allow for string literal concatenation. For example:
 *
 *    static const char* const Bitfile = "C:\\" NiFpga_portSerialMasterSlave_Bitfile;
 */
#define NiFpga_portSerialMasterSlave_Bitfile "NiFpga_portSerialMasterSlave.lvbitx"

/**
 * The signature of the FPGA bitfile.
 */
static const char* const NiFpga_portSerialMasterSlave_Signature = "BCA48EBD4CE9E2D0FA6A1B6BC2636EC6";

#if NiFpga_Cpp
extern "C"
{
#endif

typedef enum
{
   NiFpga_portSerialMasterSlave_IndicatorBool_DataFIFOFull = 0x18042,
   NiFpga_portSerialMasterSlave_IndicatorBool_SensorNotResponding = 0x1801E,
   NiFpga_portSerialMasterSlave_IndicatorBool_TimedOut = 0x1800A
} NiFpga_portSerialMasterSlave_IndicatorBool;

typedef enum
{
   NiFpga_portSerialMasterSlave_IndicatorU8_BytesatPort = 0x18002,
   NiFpga_portSerialMasterSlave_IndicatorU8_Functioncode = 0x18062,
   NiFpga_portSerialMasterSlave_IndicatorU8_Receivedaddress = 0x1805E
} NiFpga_portSerialMasterSlave_IndicatorU8;

typedef enum
{
   NiFpga_portSerialMasterSlave_IndicatorI16_Writebufferoverflows = 0x18036
} NiFpga_portSerialMasterSlave_IndicatorI16;

typedef enum
{
   NiFpga_portSerialMasterSlave_ControlBool_EnableCapture = 0x1803E,
   NiFpga_portSerialMasterSlave_ControlBool_ILC_Comm_Power_On = 0x18056,
   NiFpga_portSerialMasterSlave_ControlBool_ILC_Motor_Power_On = 0x1805A,
   NiFpga_portSerialMasterSlave_ControlBool_Mod4CH6 = 0x18076,
   NiFpga_portSerialMasterSlave_ControlBool_Mod4CH7 = 0x1807A,
   NiFpga_portSerialMasterSlave_ControlBool_Reset_Comm_Power_Breakers = 0x1806E,
   NiFpga_portSerialMasterSlave_ControlBool_Reset_Motor_Power_Breakers = 0x1806A,
   NiFpga_portSerialMasterSlave_ControlBool_cRIO_Interlock_Enable = 0x18052,
   NiFpga_portSerialMasterSlave_ControlBool_closedLoopControlOn = 0x18072,
   NiFpga_portSerialMasterSlave_ControlBool_stop = 0x18046
} NiFpga_portSerialMasterSlave_ControlBool;

typedef enum
{
   NiFpga_portSerialMasterSlave_ControlU8_Port = 0x18032
} NiFpga_portSerialMasterSlave_ControlU8;

typedef enum
{
   NiFpga_portSerialMasterSlave_ControlU16_CmdforFPGA = 0x1802E,
   NiFpga_portSerialMasterSlave_ControlU16_ReceiveStates = 0x18022,
   NiFpga_portSerialMasterSlave_ControlU16_WriteFIFOpaceticks = 0x1803A
} NiFpga_portSerialMasterSlave_ControlU16;

typedef enum
{
   NiFpga_portSerialMasterSlave_ControlI32_ByteTimeout = 0x1800C,
   NiFpga_portSerialMasterSlave_ControlI32_NumberofBytes = 0x18028
} NiFpga_portSerialMasterSlave_ControlI32;

typedef enum
{
   NiFpga_portSerialMasterSlave_ControlU32_CharTimeoutcount = 0x18024,
   NiFpga_portSerialMasterSlave_ControlU32_Latencycount = 0x18018,
   NiFpga_portSerialMasterSlave_ControlU32_LoopRateuS = 0x18048,
   NiFpga_portSerialMasterSlave_ControlU32_characterserialtimeticks = 0x18064
} NiFpga_portSerialMasterSlave_ControlU32;

typedef enum
{
   NiFpga_portSerialMasterSlave_TargetToHostFifoU8_Inbound_FIFO = 3
} NiFpga_portSerialMasterSlave_TargetToHostFifoU8;

typedef enum
{
   NiFpga_portSerialMasterSlave_TargetToHostFifoU32_NiRioScanInterfaceDmaInput = 2
} NiFpga_portSerialMasterSlave_TargetToHostFifoU32;

typedef enum
{
   NiFpga_portSerialMasterSlave_HostToTargetFifoU8_Outbound_FIFO = 0
} NiFpga_portSerialMasterSlave_HostToTargetFifoU8;

typedef enum
{
   NiFpga_portSerialMasterSlave_HostToTargetFifoU32_NiRioScanInterfaceDmaOutput = 1
} NiFpga_portSerialMasterSlave_HostToTargetFifoU32;

/* Indicator: FPGAErrorOut */
/* Use NiFpga_ReadArrayU8() to access FPGAErrorOut */
static const uint32_t NiFpga_portSerialMasterSlave_IndicatorCluster_FPGAErrorOut_Resource = 0x1804C;
static const uint32_t NiFpga_portSerialMasterSlave_IndicatorCluster_FPGAErrorOut_PackedSizeInBytes = 5;

typedef struct NiFpga_portSerialMasterSlave_IndicatorCluster_FPGAErrorOut_Type{
   NiFpga_Bool status;
   int32_t code;
}NiFpga_portSerialMasterSlave_IndicatorCluster_FPGAErrorOut_Type;


void NiFpga_portSerialMasterSlave_IndicatorCluster_FPGAErrorOut_UnpackCluster(
   const uint8_t* const packedData,
   NiFpga_portSerialMasterSlave_IndicatorCluster_FPGAErrorOut_Type* const destination);

void NiFpga_portSerialMasterSlave_IndicatorCluster_FPGAErrorOut_PackCluster(
   uint8_t* const packedData,
   const NiFpga_portSerialMasterSlave_IndicatorCluster_FPGAErrorOut_Type* const source);

/* Control: SerialConfig */
/* Use NiFpga_WriteArrayU8() to access SerialConfig */
static const uint32_t NiFpga_portSerialMasterSlave_ControlCluster_SerialConfig_Resource = 0x18004;
static const uint32_t NiFpga_portSerialMasterSlave_ControlCluster_SerialConfig_PackedSizeInBytes = 6;

typedef struct NiFpga_portSerialMasterSlave_ControlCluster_SerialConfig_Type{
   uint16_t BaudRate;
   uint8_t Parity;
   uint8_t FlowControl;
   uint8_t DataBits;
   uint8_t StopBits;
}NiFpga_portSerialMasterSlave_ControlCluster_SerialConfig_Type;


void NiFpga_portSerialMasterSlave_ControlCluster_SerialConfig_UnpackCluster(
   const uint8_t* const packedData,
   NiFpga_portSerialMasterSlave_ControlCluster_SerialConfig_Type* const destination);

void NiFpga_portSerialMasterSlave_ControlCluster_SerialConfig_PackCluster(
   uint8_t* const packedData,
   const NiFpga_portSerialMasterSlave_ControlCluster_SerialConfig_Type* const source);

/* Control: errorin */
/* Use NiFpga_WriteArrayU8() to access errorin */
static const uint32_t NiFpga_portSerialMasterSlave_ControlCluster_errorin_Resource = 0x18010;
static const uint32_t NiFpga_portSerialMasterSlave_ControlCluster_errorin_PackedSizeInBytes = 5;

typedef struct NiFpga_portSerialMasterSlave_ControlCluster_errorin_Type{
   NiFpga_Bool status;
   int32_t code;
}NiFpga_portSerialMasterSlave_ControlCluster_errorin_Type;


void NiFpga_portSerialMasterSlave_ControlCluster_errorin_UnpackCluster(
   const uint8_t* const packedData,
   NiFpga_portSerialMasterSlave_ControlCluster_errorin_Type* const destination);

void NiFpga_portSerialMasterSlave_ControlCluster_errorin_PackCluster(
   uint8_t* const packedData,
   const NiFpga_portSerialMasterSlave_ControlCluster_errorin_Type* const source);

/* Indicator: errorout */
/* Use NiFpga_ReadArrayU8() to access errorout */
static const uint32_t NiFpga_portSerialMasterSlave_IndicatorCluster_errorout_Resource = 0x18014;
static const uint32_t NiFpga_portSerialMasterSlave_IndicatorCluster_errorout_PackedSizeInBytes = 5;

typedef struct NiFpga_portSerialMasterSlave_IndicatorCluster_errorout_Type{
   NiFpga_Bool status;
   int32_t code;
}NiFpga_portSerialMasterSlave_IndicatorCluster_errorout_Type;


void NiFpga_portSerialMasterSlave_IndicatorCluster_errorout_UnpackCluster(
   const uint8_t* const packedData,
   NiFpga_portSerialMasterSlave_IndicatorCluster_errorout_Type* const destination);

void NiFpga_portSerialMasterSlave_IndicatorCluster_errorout_PackCluster(
   uint8_t* const packedData,
   const NiFpga_portSerialMasterSlave_IndicatorCluster_errorout_Type* const source);

/* FIFO: DAQ_FIFO */
static const NiFpga_FxpTypeInfo NiFpga_portSerialMasterSlave_TargetToHostFifoFxp_DAQ_FIFO_TypeInfo =
{
   1,
   27,
   16
};

/* Use NiFpga_ReadFifoU64() to access DAQ_FIFO */
static const uint32_t NiFpga_portSerialMasterSlave_TargetToHostFifoFxp_DAQ_FIFO_Resource = 4;



#if NiFpga_Cpp
}
#endif

#endif
