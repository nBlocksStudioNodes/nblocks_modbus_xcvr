#ifndef __NB_MODBUS_XCVR
#define __NB_MODBUS_XCVR

#include "nworkbench.h"
#include "tinymal_modbus_c.h"

class nBlock_MODBUS_XCVR: public nBlockSimpleNode<1> {
public:
    nBlock_MODBUS_XCVR(PinName pinTX, PinName pinRX, PinName pinDir, uint32_t baud,
				uint32_t instrument_address, 
				uint32_t start_register_address, 
				uint32_t num_registers);
    void triggerInput(nBlocks_Message message);
    void endFrame(void);
private:
	TinyMod _tinymod;
	nBlocks_MappedValue * _modified_regs;
   
};

#endif