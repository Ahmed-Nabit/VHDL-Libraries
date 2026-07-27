TWI (Two-Wire Interface) IP Core
User Guide Manual
Version 1.0
Compliant with ATmega16(L) TWI Specification
________________________________________
1. Introduction
The TWI IP Core is a production ready RTL implementation of the Two Wire Serial Interface (I²C compatible) found in the ATmega16(L) microcontroller. It handles all bus timing, arbitration, clock stretching, and protocol framing in hardware, offloading the host processor to simple register reads/writes.
This guide describes the external interface, internal registers, programming model, and typical use cases.
________________________________________
2. Features
	Only two bus lines – SCL (clock) and SDA (data).
	Master and Slave operation – Transmitter and Receiver modes.
	7 bit addressing – Up to 128 distinct slave addresses, plus General Call.
	Multi master arbitration – Built in detection and loss recovery.
	Bit rate up to 400 kHz – Programmable via TWBR and prescaler.
	Clock stretching – Slave can extend SCL low period; master waits.
	Bus error detection – Illegal START/STOP conditions flagged.
	Open drain outputs – External pull ups required (or internal if provided by SoC pads).
________________________________________
3. Module Interface (Ports)
Port name	Direction	Width	Description
clk	Input	1	System clock (all registers and FSM synchronous to this).
reset	Input	1	Active high synchronous reset.
addr	Input	3	Register select address (see Register Map).
din	Input	8	Data input for CPU writes.
dout	Output	8	Data output for CPU reads. Tristates when rd = 0.
wr	Input	1	Write strobe (active high).
rd	Input	1	Read strobe (active high).
irq	Output	1	Interrupt request (asserted when TWINT=1 and TWIE=1).
scl	Inout	1	Serial clock line (open drain, requires pull up).
sda	Inout	1	Serial data line (open drain, requires pull up).
Note: The core does not include analog spike filters or slew rate limiters – these are implemented in the pad ring.
________________________________________
4. Register Map
addr[2:0]	Register	Read/Write	Description
000	TWBR	R/W	Bit Rate Register. Sets SCL frequency.
001	TWCR	R/W	Control Register. Starts operations and clears flags.
010	TWSR	R (Status) / W (Prescaler)	Status Register (read) + Prescaler bits (write).
011	TWDR	R/W	Data Register. Transmit/Receive data byte.
100	TWAR	R/W	Slave Address Register (own address + General Call enable).
________________________________________
4.1 TWCR – TWI Control Register
Bit	7	6	5	4	3	2	1	0
Field	TWINT	TWEA	TWSTA	TWSTO	TWWC	TWEN	–	TWIE
R/W	R/W	R/W	R/W	R/W	R (read only)	R/W	–	R/W
	TWINT – TWI Interrupt Flag.
	Set by hardware when an operation completes or bus event occurs.
	Write 1 to clear it. The core holds SCL low while this bit is 1.
	TWEA – Enable Acknowledge.
	1 = return ACK when addressed as Slave, or when receiving data as Master.
	TWSTA – START Condition.
	Write 1 to transmit a START (or REPEATED START) when the bus is free.
	TWSTO – STOP Condition.
	In Master mode: write 1 to generate a STOP. Cleared automatically.
	In Slave mode: write 1 to recover from a bus error (releases SCL/SDA).
	TWWC – Write Collision Flag.
	Set if software writes TWDR while TWINT = 0. Cleared when TWINT becomes 1.
	TWEN – Enable.
	1 = activates the TWI interface and takes control of the SCL/SDA pins.
	TWIE – Interrupt Enable.
	1 allows the irq output to assert when TWINT = 1.
4.2 TWSR – TWI Status Register
Bit	7	6	5	4	3	2	1	0
Field	TWS7	TWS6	TWS5	TWS4	TWS3	–	TWPS1	TWPS0
R/W	R	R	R	R	R	–	R/W	R/W
	TWS[7:3] – Status code (5 bits). Valid only when TWINT = 1. Otherwise reads 0b11110 (0xF8).
	TWPS[1:0] – Prescaler bits for bit rate generator. Writeable.
4.3 TWDR – TWI Data Register
Bit	7	6	5	4	3	2	1	0
Field	TWD7	TWD6	TWD5	TWD4	TWD3	TWD2	TWD1	TWD0
	Holds the byte to be transmitted, or the byte just received.
	Must only be written when TWINT = 1; otherwise the write is discarded and TWWC is set.
4.4 TWAR – TWI (Slave) Address Register
Bit	7	6	5	4	3	2	1	0
Field	TWA6	TWA5	TWA4	TWA3	TWA2	TWA1	TWA0	TWGCE
R/W	R/W	R/W	R/W	R/W	R/W	R/W	R/W	R/W
	TWA[6:0] – 7 bit own slave address.
	TWGCE – General Call Enable. If 1, the core acknowledges the General Call address (0x00) when addressed.
________________________________________
5. SCL Frequency Calculation
The core generates the SCL clock according to the formula:
f_SCL=f_CLK/(16+2⋅TWBR⋅4^TWPS )

Where:
	TWBR = value in TWBR register (0–255).
	TWPS = prescaler value from TWSR[1:0]:
	00 → 1
	01 → 4
	10 → 16
	11 → 64
Example: For f_CLK = 16 MHz, TWBR = 72, TWPS = 00:
f_SCL = 16e6 / (16 + 2*72*1) = 16e6 / 160 = 100 kHz.
Important: The core uses a half period counter internally: half = 8 + TWBR * prescale. Ensure external pull up resistors are sized appropriately for your bus capacitance and chosen speed.
________________________________________
6. Operating Modes
The TWI core supports four primary modes:
Mode	Description
Master Transmitter (MT)	Master sends data to a Slave.
Master Receiver (MR)	Master reads data from a Slave.
Slave Transmitter (ST)	Slave sends data to a Master.
Slave Receiver (SR)	Slave receives data from a Master.
A transmission always begins with a START condition followed by a SLA+R/W byte.
	SLA+W (R/W = 0) → MT or SR mode.
	SLA+R (R/W = 1) → MR or ST mode.
________________________________________
6.1 Master Transmitter Flow
	Start – Write (1<<TWINT) | (1<<TWSTA) | (1<<TWEN) to TWCR.
	Wait for TWINT = 1. Status 0x08 (START sent).
	Load SLA+W into TWDR.
	Clear TWINT – Write (1<<TWINT) | (1<<TWEN) to TWCR.
	Wait for TWINT = 1. Status 0x18 (ACK) or 0x20 (NACK).
	Load data byte into TWDR.
	Clear TWINT – repeat step 4.
	Wait – status 0x28 (ACK) or 0x30 (NACK). Repeat data bytes if needed.
	Stop – Write (1<<TWINT) | (1<<TWSTO) | (1<<TWEN).
________________________________________
6.2 Master Receiver Flow
	Start – same as MT.
	Load SLA+R into TWDR.
	Clear TWINT – same as MT. Wait for status 0x40 (ACK) or 0x48 (NACK).
	Set ACK – To receive next byte, set TWEA = 1 before clearing TWINT.
Write (1<<TWINT) | (1<<TWEA) | (1<<TWEN).
	Wait – status 0x50 (data received with ACK) or 0x58 (data received with NACK).
	Read TWDR to get data.
	For last byte – clear TWEA before clearing TWINT (send NACK to slave).
Write (1<<TWINT) | (1<<TWEN) (i.e., TWEA=0).
	Stop – same as MT.
________________________________________
6.3 Slave Receiver & Transmitter
	Initialize – Set TWAR with own address. Write (1<<TWEA) | (1<<TWEN) to TWCR.
	Wait for TWINT. Status 0x60 (own SLA+W) or 0x70 (GCA) for SR; 0xA8 (own SLA+R) for ST.
	SR action – Clear TWINT with TWEA set to ACK next byte. Read TWDR on data interrupts (0x80, 0x90).
ST action – Load TWDR with the first byte to send. Clear TWINT. On 0xB8 (ACK received), load next byte.
	End – A STOP or REPEATED START condition triggers status 0xA0 (SR) or 0xC0/0xC8 (ST), releasing the slave.
________________________________________
7. Arbitration & Bus Errors
7.1 Multi Master Arbitration
	The core continuously monitors SDA during transmission.
	A Master loses arbitration if it outputs a 1 on SDA but reads a 0.
	On loss, status becomes 0x38 (MT lost) or similar.
	The core automatically switches to Slave mode and checks if the winning master is addressing it.
	If addressed, it responds as a Slave; otherwise, it releases the bus and waits for the next START.
7.2 Bus Error
	Detected when a START or STOP occurs at an illegal position (e.g., inside a data byte).
	Status 0x00 is set; TWINT is raised.
	Recovery: Write (1<<TWINT) | (1<<TWSTO) | (1<<TWEN) to TWCR.
This releases SCL/SDA and returns the core to the unaddressed Slave mode.
________________________________________
8. Programming Examples (Pseudo C)
8.1 Writing a single byte to a slave EEPROM (MT)
c
TWCR = (1<<TWINT)|(1<<TWSTA)|(1<<TWEN);   // Send START
while(!(TWCR & (1<<TWINT)));               // Wait
if (TWSR != 0x08) error();
TWDR = (SLAVE_ADDR << 1) | 0;              // SLA+W
TWCR = (1<<TWINT)|(1<<TWEN);               // Send address
while(!(TWCR & (1<<TWINT)));
if (TWSR != 0x18) error();                 // Check ACK
TWDR = MEMORY_ADDR;                        // e.g. EEPROM location
TWCR = (1<<TWINT)|(1<<TWEN);
while(!(TWCR & (1<<TWINT)));
if (TWSR != 0x28) error();
TWDR = DATA_BYTE;
TWCR = (1<<TWINT)|(1<<TWEN);
while(!(TWCR & (1<<TWINT)));
if (TWSR != 0x28) error();
TWCR = (1<<TWINT)|(1<<TWSTO)|(1<<TWEN);    // STOP
8.2 Reading a byte from a slave (MR with Repeated START)
c
// Steps 1-2: Write address as in MT (SLA+W) to set internal pointer
... (send START, SLA+W, MEMORY_ADDR) ...
// Now issue REPEATED START and read
TWCR = (1<<TWINT)|(1<<TWSTA)|(1<<TWEN);    // REPEATED START
while(!(TWCR & (1<<TWINT)));
if (TWSR != 0x10) error();                  // 0x10 = REPEATED START sent
TWDR = (SLAVE_ADDR << 1) | 1;              // SLA+R
TWCR = (1<<TWINT)|(1<<TWEN);               // Send address
while(!(TWCR & (1<<TWINT)));
if (TWSR != 0x40) error();                 // ACK received
// Prepare to receive, send ACK
TWCR = (1<<TWINT)|(1<<TWEA)|(1<<TWEN);
while(!(TWCR & (1<<TWINT)));
if (TWSR != 0x50) error();                 // Data + ACK
received = TWDR;
// NACK the last byte and STOP
TWCR = (1<<TWINT)|(1<<TWEN);               // TWEA=0
while(!(TWCR & (1<<TWINT)));
if (TWSR != 0x58) error();                 // Data + NACK
TWCR = (1<<TWINT)|(1<<TWSTO)|(1<<TWEN);    // STOP
________________________________________
9. Integration & Power Up
	Reset – Apply reset high for at least one clk cycle. All registers are initialised:
	TWBR = 0x00
	TWAR = 0xFE (own address 0x7F, GCE=0)
	TWCR = 0x00 (TWI disabled)
	Clock – No minimum frequency constraint, but slave mode requires f_CLK ≥ 16× f_SCL to sample correctly.
	Pull up – External resistors are mandatory unless the SoC pads provide internal pull ups. For 100 kHz, 4.7 kΩ is typical; for 400 kHz, use 1–2.2 kΩ (depending on bus capacitance).
________________________________________
10. Status Codes Reference (Masked, Prescaler bits = 0)
Code	Event
0x08	START transmitted
0x10	REPEATED START transmitted
0x18	MT: SLA+W transmitted, ACK received
0x20	MT: SLA+W transmitted, NACK received
0x28	MT: Data transmitted, ACK received
0x30	MT: Data transmitted, NACK received
0x38	MT: Arbitration lost
0x40	MR: SLA+R transmitted, ACK received
0x48	MR: SLA+R transmitted, NACK received
0x50	MR: Data received, ACK returned
0x58	MR: Data received, NACK returned
0x60	SR: Own SLA+W received, ACK returned
0x68	SR: Arbitration lost, own SLA+W received
0x70	SR: General call received, ACK returned
0x78	SR: Arbitration lost, general call received
0x80	SR: Data received, ACK returned
0x88	SR: Data received, NACK returned
0x90	SR: GCA data received, ACK returned
0x98	SR: GCA data received, NACK returned
0xA0	STOP / REPEATED START received as Slave
0xA8	ST: Own SLA+R received, ACK returned
0xB0	ST: Arbitration lost, own SLA+R received
0xB8	ST: Data transmitted, ACK received
0xC0	ST: Data transmitted, NACK received
0xC8	ST: Last data transmitted, ACK received
0xF8	No relevant status (TWINT=0)
0x00	Bus error
________________________________________
12. Support & Additional Notes
	Spike filtering – Add external RC filters or enable internal pad filters if available.
	Clock synchronization – The core automatically synchronises SCL when multiple masters are present.
	Wake up from sleep – Not modelled in pure RTL; implement at the SoC power management level if required.
For detailed protocol behavior, refer to the original ATmega16(L) datasheet, sections 22–24 (Two wire Serial Interface). This IP core is a direct derivative and fully compatible.

