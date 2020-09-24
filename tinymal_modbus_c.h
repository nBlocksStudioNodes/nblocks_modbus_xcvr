/** @file tinymal_modbus.h
 *  Library to implement a ModBus slave in mbed. 
 *  This version is modified to work in a class
 *  
 *  Original is at os.mbed.com/users/fbcosentino/code/Tinymal_Modbus/
 * 
 *  @author Fernando Cosentino
 *  os.mbed.com/users/fbcosentino
 */
#ifndef __TINYMOD
#define __TINYMOD

#include "nworkbench.h"
 
#define MODBUS_IDLE                 0
#define MODBUS_FUNC_READ            3
#define MODBUS_FUNC_WRITE_MULT      16

typedef struct TinyMod_AffectedRegs {
	uint32_t operation;
	uint16_t start_index;
	uint16_t count;
} TinyMod_AffectedRegs;

class TinyMod {
public:
	TinyMod(PinName pinTX, PinName pinRX, PinName pinDir, uint32_t baud,
				uint32_t instrument_address, 
				uint32_t start_register_address, 
				uint32_t num_registers);
	TinyMod_AffectedRegs check();
	void setRegister(int index, uint16_t value);
	uint16_t getRegister(int index);
	uint32_t registerNumberFromIndex(int index);
private:
	// Constructor arguments
	uint32_t _inst_address;
	uint32_t _reg_start;
	uint32_t _reg_length;
	uint32_t _reg_end;
	// Internal storage of holding registers
	// Pointer to array created during construction
	nBlocks_MappedValue *_regs;
	// Hardware peripherals
	Serial _ser;
	DigitalOut _dir_out;
	Ticker _ticker;
	
	// Serial buffers - they are not fifos
	// TX buffer is populated at once with the desired transmitting 
	// message, and is traversed byte by byte via TX interrupt
	char _buffer_tx[257];
	uint32_t _buffer_tx_cursor; // cursor to next byte to be sent
	uint32_t _buffer_tx_length; // used size of buffer
	uint32_t _tx_completed;     // flags when all bytes in buffer are sent
	// RX buffer is filled by each incoming byte as a stack, and on
	// silence timeout is processed as a whole
	char _buffer_rx[257];
	uint32_t _buffer_rx_cursor; // current data length, also cursor
	
	// Transmission timeout downcounter
	uint32_t _timeout_counter;
	// Flag to process the message
	uint32_t _rx_message_ready;
	
	// Serial callbacks
	void _serial_TX();
	void _serial_RX();
	void _timeoutTick();
	// Internal methods
	void _startTransmission(void);
	TinyMod_AffectedRegs _readRegisters(uint8_t * buf, uint32_t buflen);
	TinyMod_AffectedRegs _writeRegisters(uint8_t * buf, uint32_t buflen);
	TinyMod_AffectedRegs _processMessage(void);

};





#endif