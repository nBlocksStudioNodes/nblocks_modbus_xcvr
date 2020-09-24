#include "tinymal_modbus_c.h"

uint16_t tinymod_chksum(char * buf, int buflen) {
    uint16_t crc = 0xFFFF;
    int i, j;
    for (i=0; i<buflen; i++) {
        crc ^= (uint16_t)buf[i];
        
        for (j=8; j>0; j--) {
            if ((crc & 0x0001) != 0) {
                crc >>= 1;
                crc ^= 0xA001;
            }
            else crc >>= 1;
        }
    }
    return crc;
}

TinyMod::TinyMod(PinName pinTX, PinName pinRX, PinName pinDir, uint32_t baud,
					uint32_t instrument_address, 
					uint32_t start_register_address, 
					uint32_t num_registers):
					_ser(pinTX, pinRX), _dir_out(pinDir) {
	// Store construction arguments
	_inst_address = instrument_address;
	_reg_start = start_register_address;
	_reg_length = num_registers;
	_reg_end = _reg_start + _reg_length - 1;
	// Initialize the holding register storage array
	_regs = (nBlocks_MappedValue *)malloc(num_registers * sizeof(nBlocks_MappedValue));

	// Initialize peripherals
    
    _dir_out.write(0);
    
    _ser.attach(this, &TinyMod::_serial_TX, Serial::TxIrq);
    _ser.attach(this, &TinyMod::_serial_RX, Serial::RxIrq);

    _ticker.attach(this, &TinyMod::_timeoutTick, 0.0001);
    
    for (int i=0; i<num_registers; i++) {
        _regs[i].index = _reg_start + i;
        _regs[i].value = 0x0000;
    }
	
	// Explicit variable reset for readability
	_buffer_tx_cursor = 0;
	_buffer_tx_length = 0;
	_tx_completed = 0;
	_buffer_rx_cursor = 0;
	_timeout_counter = 0;
	_rx_message_ready = 0;
}

/**
 *  \brief Called when serial data transmittion is finished. In this
 *  library, it is called per char.
 */
void TinyMod::_serial_TX() {
    if (_buffer_tx_cursor < _buffer_tx_length) {
        // Send one more
        _startTransmission();
    }
    else {
        wait(0.000050);
        _dir_out.write(0);
        _tx_completed = 1;
    }
}

/**
 *  \brief Called when characters are received from the serial port.
 */
void TinyMod::_serial_RX() {
    char c;
    c = _ser.getc();
    if (_buffer_rx_cursor < 256) {
       // Adds this char to buffer
       _buffer_rx[ _buffer_rx_cursor ] = c;
       _buffer_rx_cursor++;
       // reset countdown to 9 (900us)
       _timeout_counter = 9; 
    }
}

// Runs every 100us. Decrements _timeout_counter. If reaches zero, processes buffer
// Every time a byte is received, _timeout_counter is set to 9 -- that is, a 900us coundown
// It will only reach zero when there is a 900us silence in RX -- that is, end of packet
void TinyMod::_timeoutTick() {
    if (_timeout_counter > 0) {
        _timeout_counter--;
        if (_timeout_counter <= 0) _rx_message_ready = 1; 
    }
}

void TinyMod::_startTransmission(void) {
    if (_buffer_tx_cursor < _buffer_tx_length) {
        _dir_out.write(1);
        _tx_completed = 0;
        _ser.putc(_buffer_tx[_buffer_tx_cursor]);
        _buffer_tx_cursor++;
    }
}






/**
 *  \brief Process incoming message as READ HOLDING REGISTERS
 *  
 *  \param [in] buf Pointer to the start of the data field in a buffer
 *  \param [in] buflen Length of the data field in the buffer
 */
TinyMod_AffectedRegs TinyMod::_readRegisters(uint8_t * buf, uint32_t buflen) {
    uint16_t start_addr, reg_count, val, crc;
    int i, len;

	TinyMod_AffectedRegs result;
	result.operation = MODBUS_IDLE;

	// Expected data in buf is: 
	// [ START_ADDRESS_MSB | START_ADDRESS_LSB | NUM_REGS_MSB | NUM_REGS_LSB ]
	// Extract data from buffer:
	if (buflen < 4) return result;
	
	start_addr = ((uint16_t)buf[0] << 8) | buf[1];
	reg_count = ((uint16_t)buf[2] << 8) | buf[3];
	uint32_t end_addr = start_addr + reg_count - 1;
	// start_index is the index to start in the internal array
	// E.g. if first register is 12 (index 0) and the range 
	// requested is 16-18, start_index is 4
	uint32_t start_index = start_addr - _reg_start;
	
	// Integrity check - only proceed if the requested registers are
	// all within the available range
	if ( (start_addr >= _reg_start) && (end_addr <= _reg_end) && (reg_count <= 125)) {
		_buffer_tx[0] = _inst_address;
		_buffer_tx[1] = MODBUS_FUNC_READ;
		_buffer_tx[2] = reg_count*2; 
		len = 3;
		for (i = 0; i<reg_count; i++) {
			val = _regs[start_index + i].value;
			_buffer_tx[3 + 2*i] = (val >> 8) & 0xFF;
			_buffer_tx[4 + 2*i] = (val     ) & 0xFF;
			len += 2;
		}
		crc = tinymod_chksum(_buffer_tx, 3 + reg_count*2);
		_buffer_tx[3 + reg_count*2] = (crc     ) & 0xFF; // CRC IS LITTLE ENDIAN
		_buffer_tx[4 + reg_count*2] = (crc >> 8) & 0xFF;
		len += 2;
		
		_buffer_tx_cursor = 0;
		_buffer_tx_length = len;
		
		_dir_out.write(1);
		
		wait(0.0019);

		_startTransmission();
		while (_tx_completed == 0) wait(0.000010);
		
		wait(0.0009);
		
		result.operation = MODBUS_FUNC_READ;
		result.start_index = start_index;
		result.count = reg_count;
	}
	
	return result;
}


TinyMod_AffectedRegs TinyMod::_writeRegisters(uint8_t * buf, uint32_t buflen) {
    uint16_t start_addr, reg_count, byte_count, val, crc;
    int i;
    
	TinyMod_AffectedRegs result;
	result.operation = MODBUS_IDLE;
	
	if (buflen < 5) return result;
	start_addr = ((uint16_t)buf[0] << 8) | buf[1];
	reg_count  = ((uint16_t)buf[2] << 8) | buf[3];
	uint32_t end_addr = start_addr + reg_count - 1;
	byte_count = ((uint8_t )buf[4]);
	// start_index is the index to start in the internal array
	// E.g. if first register is 12 (index 0) and the range 
	// requested is 16-18, start_index is 4
	uint32_t start_index = start_addr - _reg_start;
	
	if (byte_count != (reg_count*2)) return;
	// Integrity check - only proceed if the requested registers are
	// all within the available range
	if ( (start_addr >= _reg_start) && (end_addr <= _reg_end) && (reg_count <= 124)) {
		// Write registers
		for (i=0; i<reg_count; i++) {
			val = ((uint16_t)buf[5 + 2*i] << 8) | (uint8_t)buf[6 + 2*i];
			_regs[start_index + i].value = val;
		}
		
		// Response
		_buffer_tx[0] = _inst_address;
		_buffer_tx[1] = MODBUS_FUNC_WRITE_MULT;
		_buffer_tx[2] = (start_addr >> 8) & 0xFF;
		_buffer_tx[3] = (start_addr     ) & 0xFF;
		_buffer_tx[4] = (reg_count >> 8) & 0xFF;
		_buffer_tx[5] = (reg_count     ) & 0xFF;
		
		crc = tinymod_chksum(_buffer_tx, 6);
		_buffer_tx[6] = (crc     ) & 0xFF; // CRC IS LITTLE ENDIAN
		_buffer_tx[7] = (crc >> 8) & 0xFF;

		_buffer_tx_cursor = 0;
		_buffer_tx_length = 8;

		_dir_out.write(1);
		wait(0.0019);
		
		_startTransmission();
		while (_tx_completed == 0) wait(0.000010);
		
		wait(0.0009);

		result.operation = MODBUS_FUNC_WRITE_MULT;
		result.start_index = start_index;
		result.count = reg_count;
	}
	
	return result;

}



TinyMod_AffectedRegs TinyMod::_processMessage(void) {
    int dest_addr;
    int func_num;
    uint16_t rc_chksum, cl_chksum;
	

    // Ignore if less than 4 chars (wee need at least address, function, checksum)
    if (_buffer_rx_cursor >= 4) {
        dest_addr = _buffer_rx[0];
        // if the destination address for this packet is corresponds
		// to our internal address
        if (dest_addr == _inst_address) {
            // received checksum:
            rc_chksum = 0x0000 | _buffer_rx[ _buffer_rx_cursor-1 ];
            rc_chksum = (rc_chksum << 8) | _buffer_rx[ _buffer_rx_cursor-2 ];
            // calculated chksum:
            cl_chksum = tinymod_chksum(_buffer_rx, _buffer_rx_cursor-2 );
            if (rc_chksum == cl_chksum) {
                // We have a valid Modbus message
                func_num = _buffer_rx[1];
				switch (func_num) {
					
					case MODBUS_FUNC_READ:
						return _readRegisters(    (uint8_t *)&_buffer_rx[2], _buffer_rx_cursor-4) ;
					
					case MODBUS_FUNC_WRITE_MULT:
						return _writeRegisters(   (uint8_t *)&_buffer_rx[2], _buffer_rx_cursor-4) ;
				}
				
            }
            
        }
    }
    
    _buffer_rx_cursor = 0;

	// If conditions are not met, returns idle
	TinyMod_AffectedRegs result;
	result.operation = MODBUS_IDLE;
	return result;
}



TinyMod_AffectedRegs TinyMod::check() {
	if (_rx_message_ready) {
		_rx_message_ready = 0;

		return _processMessage();
	}
	else {
		TinyMod_AffectedRegs blank_result;
		blank_result.operation = MODBUS_IDLE;
		return blank_result;
	}
}


void TinyMod::setRegister(int index, uint16_t value) {
	_regs[index] = value;
}

uint16_t TinyMod::getRegister(int index) {
	return _regs[index];
}

uint32_t TinyMod::registerNumberFromIndex(int index) {
	return _reg_start + index;
}
