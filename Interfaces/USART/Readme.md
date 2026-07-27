USART IP Core – User Guide Manual
Document Version: 1.0
Core Version: Production‑Ready (ATmega‑compatible)
Based On: Atmel AVR USART (ATmega16) Datasheet

1. Overview
This USART (Universal Synchronous and Asynchronous serial Receiver and Transmitter) IP core is a hardware‑accurate, production‑ready implementation of the AVR USART peripheral. It is designed for FPGA/ASIC integration and offers full compatibility with the ATmega16 USART, including all advanced features such as buffered error flags, multi‑processor communication, synchronous master/slave operation, and double‑speed asynchronous mode.

The core provides a simple register‑bus interface (address, data, read/write strobes) and separate serial I/O pins (rx_in, tx_out, xck_in/out). All internal timers, parity generation, majority‑vote sampling, and FIFO management operate independently of the CPU, minimising software overhead.

2. Key Features
Feature	Support
Operation Modes	Asynchronous (normal & double‑speed), Synchronous (master & slave)
Frame Formats	5, 6, 7, 8, or 9 data bits; 1 or 2 stop bits; no/even/odd parity
Baud Rate	12‑bit programmable prescaler (UBRR), up to fosc/2 (sync master)
Receiver	Majority‑vote noise filtering, false start bit detection, 2‑level FIFO + 3rd buffer level
Error Detection	Frame Error (FE), Data OverRun (DOR), Parity Error (PE) – buffered with data
Interrupts	TX Complete (TXC), Data Register Empty (UDRE), RX Complete (RXC)
Multi‑Processor	Address/Data frame filtering (MPCM) using 9th bit or first stop bit
Synchronous Clock	Master mode generates XCK; Slave mode accepts external XCK; UCPOL controls edge
Register Sharing	UBRRH and UCSRC share same address with URSEL select bit
3. Port Descriptions
3.1 Global Signals
Port	Direction	Description
clk	Input	System clock (all internal logic synchronous to this)
reset	Input	Asynchronous active‑high reset (initialises all registers and state machines)
3.2 Serial Interface
Port	Direction	Description
rx_in	Input	Serial data input (asynchronous or synchronous)
tx_out	Output	Serial data output (idles high)
xck_in	Input	External clock input (synchronous slave mode)
xck_out	Output	Generated clock output (synchronous master mode)
xck_oe	Output	Output enable for XCK pin (1 = drive xck_out, 0 = high‑Z)
ddr_xck	Input	Data direction control (1 = master/output, 0 = slave/input)
3.3 Register Bus
Port	Direction	Description
addr[3:0]	Input	Register address (see Register Map)
din[7:0]	Input	Write data
dout[7:0]	Output	Read data
we	Input	Write enable (active high, single‑cycle)
re	Input	Read enable (active high, single‑cycle)
3.4 Interrupts
Port	Direction	Description
rx_irq	Output	RX Complete interrupt (RXC = 1 & RXCIE = 1)
tx_irq	Output	TX Complete interrupt (TXC = 1 & TXCIE = 1)
udre_irq	Output	Data Register Empty interrupt (UDRE = 1 & UDRIE = 1)
tx_irq_ack	Input	Transmit Complete interrupt acknowledge (clears TXC when asserted)
Note: tx_irq_ack is optional; drive to '0' if unused. TXC can also be cleared by writing a ‘1’ to bit 6 of UCSRA.

4. Register Map
Address	Name	Access	Description
0000	UDR	R/W	Data Register (TX buffer write / RX buffer read)
0001	UCSRA	R/W (partial)	Control & Status Register A
0010	UCSRB	R/W	Control & Status Register B
0011	Shared	R/W	UBRRH (write URSEL=0) / UCSRC (write URSEL=1)
0100	UBRRL	R/W	Baud Rate Register Low Byte
Others	–	–	Reserved (reads zero, writes ignored)
4.1 UCSRA – Address 0x01
Bit	Name	R/W	Description
7	RXC	R	Receive Complete (1 = unread data in FIFO)
6	TXC	R/W	Transmit Complete (set when frame sent and buffer empty). Clear by writing ‘1’ or via interrupt ack.
5	UDRE	R	Data Register Empty (1 = transmit buffer ready)
4	FE	R	Frame Error (first stop bit of head character = 0)
3	DOR	R	Data OverRun (data lost due to full FIFO)
2	PE	R	Parity Error (mismatch for head character)
1	U2X	R/W	Double Speed (async only; 0 = normal, 1 = 2×)
0	MPCM	R/W	Multi‑Processor Communication Mode enable
Important: Read UCSRA before reading UDR to correctly capture error flags for that character.

4.2 UCSRB – Address 0x02
Bit	Name	R/W	Description
7	RXCIE	R/W	RX Complete Interrupt Enable
6	TXCIE	R/W	TX Complete Interrupt Enable
5	UDRIE	R/W	Data Register Empty Interrupt Enable
4	RXEN	R/W	Receiver Enable (set to 1 to enable)
3	TXEN	R/W	Transmitter Enable (set to 1 to enable)
2	UCSZ2	R/W	Character Size bit 2 (combines with UCSRC bits)
1	RXB8	R	9th data bit of received character (read before UDR)
0	TXB8	R/W	9th data bit to be transmitted (write before UDR)
4.3 UCSRC – Shared Address 0x03 (write URSEL=1)
Bit	Name	R/W	Description
7	URSEL	W	Must be 1 when writing to UCSRC
6	UMSEL	R/W	0 = Asynchronous, 1 = Synchronous
5:4	UPM[1:0]	R/W	00 = No parity, 01 = Reserved, 10 = Even, 11 = Odd
3	USBS	R/W	0 = 1 stop bit, 1 = 2 stop bits
2:1	UCSZ[1:0]	R/W	Character size bits (combined with UCSZ2 in UCSRB)
0	UCPOL	R/W	Clock Polarity (sync mode only)
Character Size Encoding:

UCSZ2	UCSZ1	UCSZ0	Data Bits
0	0	0	5
0	0	1	6
0	1	0	7
0	1	1	8
1	1	1	9
Others	–	–	Reserved (treated as 8‑bit safe default)
4.4 UBRRL and UBRRH – Addresses 0x04 and 0x03 (URSEL=0)
UBRRL (0x04) contains bits 7:0 of the 12‑bit baud rate register.

UBRRH (0x03, URSEL=0) contains bits 11:8 in its lower nibble (bits 3:0). Bits 7:4 are reserved (write 0).

Baud Rate Equations:

Mode	BAUD =	UBRR =
Async Normal (U2X=0)	fosc / (16·(UBRR+1))	fosc/(16·BAUD) – 1
Async Double Speed (U2X=1)	fosc / (8·(UBRR+1))	fosc/(8·BAUD) – 1
Sync Master (UMSEL=1)	fosc / (2·(UBRR+1))	fosc/(2·BAUD) – 1
Writing UBRRL triggers an immediate update of the baud rate prescaler (counter reset). Writes to UBRRH alone do not cause an update until UBRRL is written.

5. Initialisation Sequence
A typical setup for asynchronous, polling‑based operation:

Disable global interrupts (if interrupt‑driven, handled outside the core).

Set baud rate – write UBRRH (URSEL=0) and UBRRL.

Set frame format – write UCSRC with URSEL=1.

Enable TX and/or RX – write TXEN/RXEN in UCSRB.

Enable interrupts (optional) – set RXCIE, TXCIE, UDRIE in UCSRB.

Warning: Changing baud rate or frame format while a transmission is ongoing will corrupt communication. Wait for TXC=1 and RXC=0 (all data flushed) before re‑initialising.

6. Transmitter Usage
6.1 Basic Transmission (5–8 Data Bits)
c
void usart_transmit(uint8_t data) {
    while (!(UCSRA & (1<<UDRE)));   // wait for buffer empty
    UDR = data;                     // write data – starts transmission
}
6.2 9‑Bit Transmission
c
void usart_transmit_9bit(uint16_t data) {
    while (!(UCSRA & (1<<UDRE)));   // wait
    UCSRB = (UCSRB & ~(1<<TXB8)) | ((data >> 8) & 1) ? (1<<TXB8) : 0;
    UDR = (uint8_t)data;
}
6.3 Key Behaviour
UDRE flag is set when the transmit buffer (single‑level) is empty. Writing UDR clears it.

TXC flag is set when the shift register is empty and no data is pending in the buffer. It is cleared by writing a ‘1’ to bit 6 of UCSRA or by asserting tx_irq_ack.

If TXEN is cleared, the transmitter completes the current and any buffered frame before disabling the TxD pin.

When the buffer is loaded while a frame is in progress, the next frame starts immediately after the current stop bit(s) – back‑to‑back transmission.

7. Receiver Usage
7.1 Basic Reception (5–8 Data Bits)
c
uint8_t usart_receive(void) {
    while (!(UCSRA & (1<<RXC)));    // wait for data
    return UDR;                     // read data (and advance FIFO)
}
7.2 9‑Bit Reception with Error Checking
c
int16_t usart_receive_9bit(void) {
    uint8_t status = UCSRA;
    uint8_t rsrb   = UCSRB;
    uint8_t data   = UDR;

    if (status & ((1<<FE)|(1<<DOR)|(1<<PE))) return -1;   // error

    return ((rsrb & (1<<RXB8)) ? 0x100 : 0) | data;
}
Critical Rule: Always read UCSRA before UDR. Reading UDR pops the FIFO; error flags and RXB8 are then lost for that character.

7.3 Flushing the Receive Buffer
To discard unread data (e.g., after an error):

c
void usart_flush(void) {
    uint8_t dummy;
    while (UCSRA & (1<<RXC)) dummy = UDR;
}
7.4 Receiver Behaviour
The FIFO holds two characters. A third can be held in the shift register, making the core resistant to DOR.

If a new start bit is detected while the shift register already holds a completed frame and FIFO is full, the shift register frame is lost and DOR is set.

Disabling the receiver (RXEN=0) immediately flushes the FIFO and aborts any ongoing reception.

Error flags (FE, DOR, PE) are buffered inside the FIFO along with each character. They are only valid for the character at the head of the FIFO.

8. Synchronous Mode (UMSEL=1)
8.1 Master Mode (ddr_xck = 1)
The core generates the XCK clock on xck_out (with xck_oe = 1) at frequency fosc / [2·(UBRR+1)].

The clock starts only when the transmitter/receiver is active (or when a frame is pending).

Idle XCK level is chosen so that the first edge after idle is a data‑change edge according to UCPOL.

8.2 Slave Mode (ddr_xck = 0)
The external clock on xck_in is synchronised via a 3‑stage flip‑flop.

The maximum external clock frequency is fosc / 4 (not enforced in hardware – ensure in your design).

Data sampling and output change follow the same UCPOL edge relationship.

8.3 UCPOL Edge Selection
UCPOL	TxD Data Changed	RxD Data Sampled
0	Rising XCK	Falling XCK
1	Falling XCK	Rising XCK
Note: In asynchronous mode (UMSEL=0), UCPOL must be written to 0.

9. Multi‑Processor Communication Mode (MPCM)
When MPCM = 1, the receiver filters out data frames (non‑address) to reduce CPU load in multi‑drop networks.

For 5–8 data bits: the first stop bit indicates frame type. 1 = address, 0 = data.

For 9 data bits: the 9th bit (RXB8) indicates frame type. 1 = address, 0 = data.

Typical Master‑Slave Procedure
All slaves set MPCM = 1.

Master sends an address frame (frame type bit = 1). All slaves receive it (RXC set).

Each slave reads UDR, checks if it is addressed. The addressed slave clears MPCM=0; others keep MPCM=1.

Master sends data frames – only the addressed slave will receive them.

After the transaction, the addressed slave sets MPCM=1 again, waiting for the next address.

Do not use SBI/CBI (bit‑set/clear) instructions on MPCM, as it shares the same byte with TXC and could accidentally clear TXC. Write the whole UCSRA register.

10. Interrupt Handling
The core outputs three interrupt signals. They are level‑sensitive and remain asserted as long as the corresponding flag (RXC, TXC, UDRE) is set and the enable bit is 1.

Interrupt	Condition	Clear Mechanism
rx_irq	RXC=1 & RXCIE=1	Reading UDR until FIFO empty clears RXC
tx_irq	TXC=1 & TXCIE=1	Write‑one to TXC bit, or tx_irq_ack=1
udre_irq	UDRE=1 & UDRIE=1	Writing UDR (until buffer full) clears UDRE
If using UDRE interrupt, write new data inside the ISR. If no more data, disable UDRIE to prevent repeated interrupts.

11. Baud Rate Error & Operational Range
The receiver can tolerate certain mismatches between the incoming data rate and the internal baud clock. The core implements the majority‑vote sampling described in the datasheet.

Mode	Samples per bit	Recommended Max Error
Normal (U2X=0)	16	±3.0% (5‑bit) to ±1.5% (10‑bit)
Double Speed (U2X=1)	8	±2.5% (5‑bit) to ±1.0% (10‑bit)
Use the equation Error[%] = (fosc/(16·BAUD·(UBRR+1)) – 1) * 100 to check your UBRR selection. The core does not compute this – it is a design‑time consideration.

12. Register Access Rules (Shared UBRRH/UCSRC)
12.1 Write Access
Write to addr = 0x03 with bit 7 = 1 → updates UCSRC.

Write to addr = 0x03 with bit 7 = 0 → updates UBRRH (bits 11:8).

12.2 Read Access (Two‑Cycle Sequence)
Read addr = 0x03 → returns UBRRH (with bit 7 = 0).

Read addr = 0x03 again in the next clock cycle → returns UCSRC (with bit 7 = 1).

The core detects back‑to‑back reads automatically. If you need to read UBRRH, ensure the previous cycle was not a read to address 0x03.

13. Timing and Performance Considerations
Parameter	Value	Remarks
Max clock speed	Limited by your FPGA/ASIC technology	The core is fully synchronous.
Min XCK (slave) period	4 system clock cycles	External clock must be sampled.
Latency (TX)	Immediate – data output starts on next bit tick after UDR write.	–
Latency (RX)	One full frame delay (FIFO buffering).	Read data available after stop bit.
Resource count	Low – no multipliers, small FIFO.	Approximately 200–300 LUTs/FFs (FPGA‑dependent).
14. Integration Checklist
✅ Connect clk and reset.

✅ Connect rx_in, tx_out to external pads.

✅ Connect xck_in, xck_out, xck_oe and drive ddr_xck from software or GPIO.

✅ Connect addr, din, dout, we, re to your system bus decoder.

✅ Optionally connect interrupt outputs to your interrupt controller.

✅ Drive tx_irq_ack to '0' if not used, or connect it to your interrupt ACK logic.

15. Appendix A – Quick Reference Baud Rate Examples (Async Normal, fosc=16MHz)
BAUD	UBRR	Error
2400	416	-0.1%
9600	103	+0.2%
19200	51	+0.2%
38400	25	+0.2%
115200	8	-3.5% (use double‑speed for lower error)
For a comprehensive table, refer to the original datasheet (doc2466.pdf from ATMEL, Tables 68–71).

End of User Guide