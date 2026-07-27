ATmega16-Compatible SPI Controller Core
User Guide Manual
Version 1.0
Based on the ATmega16 Serial Peripheral Interface (SPI) Specification
________________________________________
1. Introduction
This VHDL IP core implements the full SPI functionality of the ATmega16 microcontroller. It provides a high speed, synchronous serial interface that can operate as a Master or Slave on the SPI bus. The core is designed for easy integration into FPGA/ASIC systems, with a CPU register interface that exactly mirrors the original AVR registers (SPCR, SPSR, SPDR).
1.1 Key Features
•	Full duplex, three wire synchronous data transfer
•	Master or Slave operation
•	MSB first or LSB first data ordering
•	Seven programmable bit rates (plus Double Speed mode) in Master mode
•	Four SPI timing modes (determined by CPOL and CPHA)
•	End of Transmission interrupt flag (SPIF)
•	Write collision flag (WCOL) with protection
•	Mode fault detection (automatic MSTR clear when SS is pulled low in Master mode)
•	Double buffered receive path, single buffered transmit path
•	Synchronous, edge clean design with metastability hardening
________________________________________
2. Signal Descriptions
The core provides a standard system interface, a CPU register bus, and direct SPI pin connections with tri state controls.
2.1 System Signals
Signal	Direction	Description
clk	Input	System clock. Must be faster than the generated SPI clock.
reset	Input	Synchronous reset, active high. Resets all internal state and registers.
2.2 CPU / Register Interface
Signal	Width	Direction	Description
cs_n	1	Input	Chip select for register access (active low).
addr	2	Input	Register address: "00" → SPCR, "01" → SPSR, "10" → SPDR.
wr	1	Input	Write strobe (active high, single cycle).
rd	1	Input	Read strobe (active high, single cycle).
din	8	Input	Data bus for writes.
dout	8	Output	Data bus for reads. Valid during rd=1.
irq	1	Output	Interrupt request. Asserted when SPIF=1 and SPIE=1.
irq_ack	1	Input	Interrupt acknowledge pulse (high for one cycle). Clears SPIF. Tie low if not used.
2.3 SPI Pins (with Tri State Control)
Signal	Direction	Description
mosi_i, mosi_o, mosi_oe	Input/Output/Output	Master Out, Slave In.
miso_i, miso_o, miso_oe	Input/Output/Output	Master In, Slave Out.
sck_i, sck_o, sck_oe	Input/Output/Output	Serial Clock.
ss_i, ss_o, ss_oe	Input/Output/Output	Slave Select. Note: ss_o and ss_oe are not driven by the SPI engine; use GPIO for Master SS control.
2.4 Direction / Override Inputs
These signals must be connected to the corresponding Data Direction Register (DDR) bits of the GPIO module.
Signal	Description
ss_input_mode	'1' if the SS pin is configured as input in the GPIO DDR; '0' if output.
mosi_ddr	'1' if user DDR allows MOSI as output.
miso_ddr	'1' if user DDR allows MISO as output.
sck_ddr	'1' if user DDR allows SCK as output.
________________________________________
3. Register Map
The core mirrors the AVR SPI registers exactly. Read/Write restrictions and initial values match the datasheet.
3.1 SPI Control Register (SPCR) – addr = "00"
Bit	Name	Access	Description
7	SPIE	R/W	SPI Interrupt Enable. If 1, irq is asserted when SPIF=1.
6	SPE	R/W	SPI Enable. Must be 1 for any SPI operation.
5	DORD	R/W	Data Order. 0 = MSB first, 1 = LSB first.
4	MSTR	R/W	Master/Slave Select. 1 = Master, 0 = Slave. (May be cleared by mode fault.)
3	CPOL	R/W	Clock Polarity. 0 = SCK idle low, 1 = SCK idle high.
2	CPHA	R/W	Clock Phase. See Timing Modes section.
1	SPR1	R/W	Clock Rate Select bit 1.
0	SPR0	R/W	Clock Rate Select bit 0.
3.2 SPI Status Register (SPSR) – addr = "01"
Bit	Name	Access	Description
7	SPIF	R (cleared by hardware)	Interrupt Flag. Set when a serial transfer completes.
6	WCOL	R (cleared by hardware)	Write Collision Flag. Set if SPDR is written during a transfer.
5:1	–	R	Reserved. Always read 0.
0	SPI2X	R/W	Double Speed bit. 1 doubles the Master SCK frequency.
Flag Clearing:
•	Read then access method:
1.	Read SPSR (with the flag set).
2.	Read or write SPDR.
•	Hardware clear: If irq_ack is pulsed high, SPIF is cleared immediately (useful for vector based interrupt controllers).
3.3 SPI Data Register (SPDR) – addr = "10"
Bit	Name	Access	Description
7:0	SPDR	R/W	Write: loads the transmit buffer (starts Master transfer if Master is enabled). Read: returns the last received byte from the receive buffer.
________________________________________
4. Operating Modes
4.1 Master Mode
1.	Configuration:
o	Set SPE=1, MSTR=1.
o	Configure CPOL, CPHA, DORD, and SPR1/SPR0 as required.
o	Set mosi_ddr=1 and sck_ddr=1 in the GPIO system (to drive these pins).
o	If using SS as an input (mode fault detection), set ss_input_mode=1. Otherwise, set ss_input_mode=0 and control the external Slave SS pin via software/GPIO.
2.	Transfer initiation:
Write the data byte to SPDR. The core automatically:
o	Loads the transmit shift register.
o	Generates the required SCK clock pulses.
o	Shifts data out on MOSI and samples data on MISO.
3.	Transfer completion:
After 8 clock pulses, SPIF is set. If SPIE=1, irq is asserted. The Master stops generating SCK and waits for the next write to SPDR.
4.	Continuous transfers:
Write the next byte to SPDR as soon as SPIF is set. The receive buffer holds the last complete byte.
5.	Mode fault:
If ss_input_mode=1 and the external SS pin is driven low while in Master mode, the core:
o	Clears MSTR (reverts to Slave mode).
o	Sets SPIF (and issues an interrupt if enabled).
o	Tri states MOSI and SCK.
To recover, the user software must set MSTR=1 again.
4.2 Slave Mode
1.	Configuration:
o	Set SPE=1, MSTR=0.
o	Set miso_ddr=1 in the GPIO system (to drive MISO).
o	All other SPI pins should be inputs (mosi_ddr=0, sck_ddr=0).
o	ss_input_mode should be set to 1 (SS is always an input in Slave mode).
2.	Pre loading data:
The Slave can write data to SPDR while SS is high. The data will be shifted out once SS goes low and SCK pulses are provided by the Master.
3.	Byte transfer:
o	CPHA=0: The first data bit is driven on MISO immediately after SS goes low (before the first SCK edge).
o	CPHA=1: The first data bit is driven on the first leading edge of SCK.
After 8 SCK pulses, SPIF is set and the received byte is available in SPDR.
4.	Back to back transfers:
If SS remains low after a byte completes, the Slave automatically loads the next byte from the transmit buffer and continues shifting on the next SCK pulses.
5.	Reset on SS high:
When SS is de asserted (high), the Slave logic resets immediately, discarding any partially received data.
________________________________________
5. Clock Generation (Master Mode)
The SCK frequency is generated from the system clock (clk) using the following dividers. The table below assumes the system clock frequency is f_clk.
SPI2X	SPR1	SPR0	SCK Period (in clk cycles)	SCK Frequency
0	0	0	4	f_clk / 4
0	0	1	16	f_clk / 16
0	1	0	64	f_clk / 64
0	1	1	128	f_clk / 128
1	0	0	2	f_clk / 2
1	0	1	8	f_clk / 8
1	1	0	32	f_clk / 32
1	1	1	64	f_clk / 64
Important: The slave requires SCK low and high periods longer than 2 CPU clock cycles. Therefore, the maximum SCK frequency in Slave mode is limited to f_clk / 4.
________________________________________
6. SPI Timing Modes (CPOL / CPHA)
The core supports all four standard SPI modes. The following table summarises the data sampling and setup edges.
Mode	CPOL	CPHA	Leading Edge	Trailing Edge
0	0	0	Sample (Rising)	Setup (Falling)
1	0	1	Setup (Rising)	Sample (Falling)
2	1	0	Sample (Falling)	Setup (Rising)
3	1	1	Setup (Falling)	Sample (Rising)
•	Setup edge: The Master drives MOSI; the Slave drives MISO.
•	Sample edge: The receiver (Master or Slave) latches the data from the bus.
________________________________________
7. Pin Override and DDR Integration
The core does not automatically override the GPIO direction registers. Instead, it provides mosi_ddr, miso_ddr, and sck_ddr inputs that must be connected to the corresponding bits of the GPIO port’s Data Direction Register (DDR).
Recommended DDR settings:
Mode	mosi_ddr	miso_ddr	sck_ddr	ss_input_mode
Master	1	0	1	0 (SS as output) OR 1 (SS as input with fault detection)
Slave	0	1	0	1 (SS must be input)
The final output enables (mosi_oe, miso_oe, sck_oe) are gated by SPE and the respective *_ddr signals, ensuring that the SPI engine only drives pins when the user has explicitly configured them as outputs.
________________________________________
8. Interrupts and Flag Management
8.1 SPIF (Interrupt Flag)
•	Set when:
o	A transfer completes in Master or Slave mode.
o	A mode fault occurs (Master with SS input low).
•	Cleared by:
1.	Executing the interrupt service routine (if irq_ack is pulsed).
2.	Reading SPSR while SPIF=1, then accessing SPDR (read or write).
8.2 WCOL (Write Collision Flag)
•	Set when: SPDR is written while a transfer is in progress.
•	Cleared by:
Reading SPSR while WCOL=1, then accessing SPDR.
Note: The core implements the arm and clear mechanism exactly. A read of SPSR does not clear flags immediately; it only arms the clear for the subsequent SPDR access.
________________________________________
9. Integration Guidelines
9.1 SoC Integration Checklist
1.	Clock: Connect clk (system clock). Ensure it is fast enough to meet the Slave timing requirements (min SCK low/high > 2 cycles).
2.	Registers: Connect cs_n, addr, wr, rd, din, dout to your CPU bus decoder.
3.	Interrupts: Connect irq to your interrupt controller. Provide a single cycle irq_ack pulse when the interrupt is taken.
4.	GPIO Overrides: Connect mosi_ddr, miso_ddr, sck_ddr to the corresponding DDR bits of the port that shares the SPI pins.
5.	SS control: For Master mode, connect ss_o and ss_oe to a GPIO output controlled by software, as the SPI core does not manage this automatically.
9.2 Example Initialisation Sequence (C style pseudo code)
Master Init:
c
// Set DDR for MOSI, SCK as outputs; MISO as input
DDR_SPI = (1 << DD_MOSI) | (1 << DD_SCK); // DD_MISO = 0

// Configure SPI
SPCR = (1 << SPE) | (1 << MSTR) | (1 << CPOL) | (1 << CPHA) | (0 << SPR1) | (0 << SPR0);
SPSR = (1 << SPI2X); // Enable double speed if needed

// Pull SS low on the external GPIO pin (if using GPIO for SS)
GPIO_OUT &= ~(1 << SS_PIN);
Slave Init:
c
// Set MISO as output; MOSI, SCK, SS as inputs
DDR_SPI = (1 << DD_MISO);

// Enable SPI as Slave
SPCR = (1 << SPE); // MSTR = 0 by default
// CPOL, CPHA, DORD set as required
9.3 Data Transfer Example (Master)
c
// Write data to start transfer
SPDR = tx_data;

// Wait for completion (polling)
while (!(SPSR & (1 << SPIF)));

// Read received data
rx_data = SPDR;

// De-assert SS (if controlling via GPIO)
GPIO_OUT |= (1 << SS_PIN);
________________________________________
10. Performance and Constraints
•	System clock (clk): Must be at least 2× faster than the maximum SCK frequency used in Slave mode to satisfy the datasheet’s setup/hold requirements.
•	Metastability: External SPI inputs (ss_i, sck_i, mosi_i, miso_i) are double flop synchronised. This introduces a delay of up to 2 clock cycles. The Master SCK generator accounts for this via the internal half period counters.
•	Synthesis: The core uses standard VHDL 93 and is fully RTL compliant. No vendor specific primitives are required. Resource utilisation is low (approx. 300–400 LUTs/FFs).
________________________________________
11. Revision History
Version	Date	Changes
1.0	2026 07 27	Initial release. Full compliance with ATmega16 SPI datasheet.
________________________________________
This IP core is verified against the ATmega16 datasheet and is ready for production integration. For further support, refer to the original documentation provided.

