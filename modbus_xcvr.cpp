#include "modbus_xcvr.h"

nBlock_MODBUS_XCVR::nBlock_MODBUS_XCVR(PinName pinTX, PinName pinRX, PinName pinDir, uint32_t baud,
				uint32_t instrument_address, 
				uint32_t start_register_address, 
				uint32_t num_registers): 
					_tinymod(pinTX, pinRX, pinDir, baud, 
					instrument_address, start_register_address, num_registers) {
    // Output type is array of nBlocks_MappedValue    
	outputType[0] = OUTPUT_TYPE_ARRAY;
	// Allocate memory for the output array
	_modified_regs = (nBlocks_MappedValue *)malloc(num_registers * sizeof(nBlocks_MappedValue));
	
    return;
}
void nBlock_MODBUS_XCVR::triggerInput(nBlocks_Message message) {
	//_modbus.printf(message.stringValue);
    // tiny_regs[1] = reg_data
}
void nBlock_MODBUS_XCVR::endFrame(void) {
	TinyMod_AffectedRegs result = _tinymod.check();
	
	// if last operation was MODBUS_FUNC_WRITE_MULT, the master has just
	// modified the registers. Emit output messages for the ones which
	// have just changed
	if (result.operation == MODBUS_FUNC_WRITE_MULT) {
		// Populate _modified_regs
		for (int i=0; i<result.count; i++) {
			uint32_t offset = result.start_index + i;
			_modified_regs[i].index = _tinymod.registerNumberFromIndex(offset);
			_modified_regs[i].value = _tinymod.getRegister(offset);
		}
		
        output[0] = (uint32_t)(_modified_regs); 
        available[0] = result.count;
	}
    return;
}
