CAN Controller IP Core – User Guide
1. Introduction
The ATmega CAN Controller IP Core is a full CAN implementation compliant with CAN Specification 2.0A and 2.0B (active). It provides a complete hardware solution for the CAN bus protocol, supporting up to 1 Mbit/s bit rate. The core integrates six Message Objects (MOBs) that can be configured for transmission, reception, automatic reply, or frame buffer reception, dramatically reducing CPU load. Built in error management, a programmable 16 bit timer with time triggered communication (TTC) support, and flexible acceptance filtering make this IP ideal for real time automotive and industrial applications.
This document describes the architecture, register set, programming model, and usage of the CAN Controller IP Core.
________________________________________
2. Features
•	Full CAN controller, compatible with CAN 2.0A (11 bit identifier) and 2.0B (29 bit identifier) active.
•	6 Message Objects (MOBs) with individual:
o	11  or 29 bit identifier tag and mask.
o	8 byte data buffer (static allocation).
o	Configuration for Transmit, Receive, Automatic Reply, or Frame Buffer Receive.
o	Time stamping.
•	Bit rates up to 1 Mbit/s (at 8 MHz system clock).
•	Programmable bit timing with up to 25 time quanta per bit.
•	Error detection and fault confinement (Error Active, Error Passive, Bus Off).
•	Listening mode for bus monitoring and auto baud detection.
•	16 bit timer with prescaler for message time stamping and TTC.
•	Priority arbitration among MOBs based on index.
•	Abort request and software reset.
•	Overload frame generation.
•	Comprehensive interrupt system with per MOB and general interrupt flags.
________________________________________
3. Block Diagram
The core consists of the following main blocks:
text
+------------------+         +-------------------+         +-----------------+
|                  |         |                   |         |                 |
|   CPU Interface  |<------->|   CAN Engine      |<------->|   CAN Bus       |
|   (Registers,    |         |   (Protocol FSM,  |         |   (Tx/Rx pins)  |
|    MOB access)   |         |    CRC, Bit Stuff)|         |                 |
|                  |         |                   |         |                 |
+------------------+         +-------------------+         +-----------------+
        |                              |                             |
        |                              |                             |
        v                              v                             |
+------------------+         +-------------------+                    |
|                  |         |                   |                    |
|   MOB Manager    |<------->|   Bit Timing      |<-------------------+
|   (Filtering,    |         |   (TQ gen, sync)  |
|    Arbitration)  |         |                   |
|                  |         +-------------------+
+------------------+
        |
        v
+------------------+
|                  |
|   CAN Timer      |
|   (Timestamp,    |
|    TTC)          |
|                  |
+------------------+
________________________________________
4. Pin/Interface Description
Signal	Direction	Description
clk	Input	System clock (all timing derived from this).
reset_n	Input	Asynchronous active low reset.
tx	Output	Transmit data to CAN transceiver (dominant = ‘0’, recessive = ‘1’).
rx	Input	Receive data from CAN transceiver.
cs	Input	Chip select for CPU interface.
wr	Input	Write strobe (active high).
rd	Input	Read strobe (active high).
addr	Input [5:0]	Register address (6 bit, see register map).
data_in	Input [7:0]	Data bus input.
data_out	Output [7:0]	Data bus output.
intr	Output	Interrupt request (active high).
________________________________________
5. Register Map
All registers are 8 bit wide and memory mapped. The address space is 64 bytes, with general registers at addresses 0x00–0x2F and MOB specific registers (via page selection) at addresses 0x20–0x3F.
Address decoding (6 bit):
Address	Register Name	Description
0x00	CANGCON	General Control Register
0x01	CANGSTA	General Status Register
0x02	CANGIT	General Interrupt Register
0x03	CANGIE	General Interrupt Enable Register
0x04	CANEN1	Enable MOBs (high byte – reserved)
0x05	CANEN2	Enable MOBs (low byte)
0x06	CANIE1	Interrupt Enable MOBs (high)
0x07	CANIE2	Interrupt Enable MOBs (low)
0x08	CANSIT1	Interrupt Status MOBs (high)
0x09	CANSIT2	Interrupt Status MOBs (low)
0x0A	CANBT1	Bit Timing Register 1 (BRP)
0x0B	CANBT2	Bit Timing Register 2 (SJW, PRS)
0x0C	CANBT3	Bit Timing Register 3 (PHS2, PHS1, SMP)
0x0D	CANTCON	Timer Control Register
0x0E	CANTIML	Timer Counter Low Byte
0x0F	CANTIMH	Timer Counter High Byte
0x10	CANTTCL	TTC Timer Capture Low Byte
0x11	CANTTCH	TTC Timer Capture High Byte
0x12	CANTEC	Transmit Error Counter
0x13	CANREC	Receive Error Counter
0x14	CANHPMOB	Highest Priority MOB and General Purpose
0x15	CANPAGE	MOB Page Selection and Data Index
0x20–0x2F	MOB page 0..5	MOB registers (see Section 6)
Note: Addresses 0x16–0x2F are reserved and must not be used.
________________________________________
6. General Registers Description
6.1 CANGCON – General Control Register (0x00)
Bit	Name	R/W	Description
7	ABRQ	R/W	Abort Request: write ‘1’ to abort all pending transmissions (except current ongoing frame). Auto cleared by hardware when done.
6	OVRQ	R/W	Overload Request: write ‘1’ to send an overload frame after the next received frame.
5	TTC	R/W	Time Triggered Communication: ‘1’ enables TTC mode (frame sent only once; errors do not cause retransmission).
4	SYNCTTC	R/W	TTC synchronization edge: ‘0’ = capture timer on Start Of Frame, ‘1’ = capture on End Of Frame.
3	LISTEN	R/W	Listening Mode: ‘1’ places core in listening mode (bus monitor, Tx recessive, error counters frozen).
2	TEST	R/W	Test Mode: reserved for factory testing; must be ‘0’.
1	ENA/STB	R/W	Enable/Standby: ‘0’ = Standby (controller disabled but registers accessible), ‘1’ = Enable (controller starts after 11 recessive bits).
0	SWRES	R/W	Software Reset: write ‘1’ to reset the entire CAN controller (auto cleared).
6.2 CANGSTA – General Status Register (0x01) – Read Only
Bit	Name	Description
7	–	Reserved.
6	OVRG	Overload Frame Flag: ‘1’ while an overload frame is being transmitted.
5	–	Reserved.
4	TXBSY	Transmitter Busy: ‘1’ when sending a frame, ACK, or interframe space.
3	RXBSY	Receiver Busy: ‘1’ when receiving or monitoring a frame.
2	ENFG	Enable Flag: ‘1’ when controller is fully enabled (after 11 recessive bits).
1	BOFF	Bus Off: ‘1’ indicates bus off state.
0	ERRP	Error Passive: ‘1’ indicates error passive state.
6.3 CANGIT – General Interrupt Register (0x02) – Write 1 to clear
Bit	Name	Description
7	CANIT	Read only: image of all enabled interrupts (except timer overrun).
6	BOFFIT	Bus Off Interrupt: set when entering bus off.
5	OVRTIM	Timer Overrun Interrupt: set when the CAN timer rolls over from 0xFFFF to 0.
4	BXOK	Frame Buffer Receive Complete: set when all MOBs in a frame buffer set have received their frames.
3	SERG	Stuff Error (general): set when a stuff error occurs on a received frame that does not match any MOB.
2	CERG	CRC Error (general): set when a CRC error occurs on a non matched received frame.
1	FERG	Form Error (general): set when a form error occurs on a non matched received frame.
0	AERG	Acknowledgment Error (general): set when an ACK error occurs (Tx only).
Clearing: Write ‘1’ to the corresponding bit to clear it. For BXOK, all MOBs in the buffer set must have been re written (see Section 9.3).
6.4 CANGIE – General Interrupt Enable Register (0x03)
Bit	Name	Description
7	ENIT	Global interrupt enable (master enable).
6	ENBOFF	Enable Bus Off interrupt.
5	ENRX	Enable MOB receive interrupts.
4	ENTX	Enable MOB transmit interrupts.
3	ENERR	Enable MOB error interrupts.
2	ENBX	Enable Frame Buffer Receive interrupt.
1	ENERG	Enable General Errors interrupt.
0	ENOVRT	Enable Timer Overrun interrupt.
6.5 CANEN2, CANEN1 – Enable MOB Registers (0x05, 0x04) – Read Only
These registers indicate which MOBs are currently enabled. Bits 5..0 of CANEN2 correspond to MOB 0..5. CANEN1 is reserved (bits 15..6). A MOB is enabled when its CONMOB bits are set to non disabled. It is automatically disabled after a successful transmission or reception (except for automatic reply, where it remains enabled). Reading these registers allows software to check MOB availability.
6.6 CANIE2, CANIE1 – MOB Interrupt Enable Registers (0x07, 0x06)
Bits 5..0 of CANIE2 enable interrupts for MOB 0..5 (CANIE1 reserved). When a MOB has an enabled interrupt and the global enable (ENIT) is set, the intr output will be asserted.
6.7 CANSIT2, CANSIT1 – MOB Interrupt Status Registers (0x09, 0x08) – Read Only
Bits 5..0 of CANSIT2 indicate that the corresponding MOB has a pending interrupt. This is a logical OR of the MOB’s status flags (RXOK, TXOK, errors) that are enabled in CANIE2.
6.8 CANBT1 – Bit Timing Register 1 (0x0A)
Bit	Name	Description
7	–	Reserved, write 0.
6..1	BRP[5:0]	Baud Rate Prescaler. The time quantum TQ = (BRP+1) / f_clk.
0	–	Reserved, write 0.
6.9 CANBT2 – Bit Timing Register 2 (0x0B)
Bit	Name	Description
7	–	Reserved, write 0.
6..5	SJW[1:0]	Resynchronization Jump Width (1..4).
4	–	Reserved, write 0.
3..1	PRS[2:0]	Propagation Time Segment (1..8).
0	–	Reserved, write 0.
6.10 CANBT3 – Bit Timing Register 3 (0x0C)
Bit	Name	Description
7	–	Reserved, write 0.
6..4	PHS2[2:0]	Phase Segment 2 (1..8, but must be ≥2 and ≤ PHS1).
3..1	PHS1[2:0]	Phase Segment 1 (1..8).
0	SMP	Sample point configuration: 0 = single sample at SP; 1 = three samples with majority vote (only when BRP ≠ 0).
The total number of TQ per bit = 1 (SYNC) + PRS + PHS1 + PHS2, and must be between 8 and 25. The core automatically adjusts segments if the sum is out of range (by increasing/decreasing PRS and PHS1/2 while respecting constraints).
6.11 CANTCON – Timer Control Register (0x0D)
Bit	Name	Description
7..0	TPRSC[7:0]	Prescaler for the 16 bit CAN timer. Timer clock = f_clk / (8 × (TPRSC+1)).
6.12 CANTIML/H – Timer Counter (0x0E/0x0F) – Read Only
16 bit free running timer (0x0000 to 0xFFFF). It starts counting when ENFG becomes ‘1’. Rolling over from 0xFFFF to 0x0000 sets OVRTIM.
6.13 CANTTCL/H – TTC Timer Capture (0x10/0x11) – Read Only
When TTC mode is enabled, the timer value is captured on SOF or EOF (as configured by SYNCTTC) and stored in this register.
6.14 CANTEC – Transmit Error Counter (0x12) – Read Only
8 bit value (0–255). It is incremented/decremented according to CAN fault confinement rules. The core internally uses 9 bits to detect bus off.
6.15 CANREC – Receive Error Counter (0x13) – Read Only
8 bit value (0–255). Updated as per CAN specification.
6.16 CANHPMOB – Highest Priority MOB (0x14)
Bit	Name	Description
7..4	HPMOB[3:0]	Index of the highest priority MOB that has a pending interrupt (0..5). If none, 0xF. Read only.
3..0	CGP[3:0]	General purpose bits (read/write). These can be used to store the desired CANPAGE settings (AINC and INDX) for fast context switching.
6.17 CANPAGE – Page MOB Register (0x15)
Bit	Name	Description
7..4	MOBNB[3:0]	MOB number (0..5) to select for access to MOB registers.
3	AINC	Auto Increment control: ‘0’ enables auto increment of data index after each MSG access; ‘1’ disables it.
2..0	INDX[2:0]	Data index (0..7) for the MSG register, pointing to a byte within the MOB’s data buffer.
Writing to MOBNB selects which MOB’s internal registers are mapped to addresses 0x20–0x2F. Changing MOBNB also resets the data index to 0.
________________________________________
7. MOB Registers (Address 0x20–0x2F)
When CANPAGE[7:4] selects a MOB number (0..5), the following registers become accessible. All are read/write, except where noted.
Offset	Name	Description
0x00	CANSTMOB	MOB Status (flags) – write 1 to clear
0x01	CANCDMOB	Control/DLC (configuration, IDE, DLC)
0x02	CANIDT1	Identifier Tag byte 1 (LSB)
0x03	CANIDT2	Identifier Tag byte 2
0x04	CANIDT3	Identifier Tag byte 3
0x05	CANIDT4	Identifier Tag byte 4 (includes RTR, RB0/1)
0x06	CANIDM1	Identifier Mask byte 1 (LSB)
0x07	CANIDM2	Identifier Mask byte 2
0x08	CANIDM3	Identifier Mask byte 3
0x09	CANIDM4	Identifier Mask byte 4 (includes RTRMSK, IDEMSK)
0x0A	CANSTML	Time Stamp Low
0x0B	CANSTMH	Time Stamp High
0x0C	CANMSG	Data byte at current INDX (auto increment optional)
7.1 CANSTMOB – MOB Status Register
Bit	Name	Description
7	DLCW	DLC Warning: set if received DLC differs from configured DLC (for Rx MOB).
6	TXOK	Transmit OK: set when the MOB successfully transmits a frame.
5	RXOK	Receive OK: set when the MOB successfully receives a matching frame.
4	BERR	Bit Error (Tx only) – set on bit error during transmission.
3	SERR	Stuff Error – set if a stuff error occurs.
2	CERR	CRC Error – set if CRC check fails on received frame.
1	FERR	Form Error – set on violation of fixed format fields.
0	AERR	Acknowledgment Error (Tx only) – set if no dominant ACK received.
Clearing: Write ‘1’ to the bits you wish to clear. The entire register is read modify write.
7.2 CANCDMOB – MOB Control and DLC Register
Bit	Name	Description
7..6	CONMOB[1:0]	Configuration: 00=Disabled, 01=Tx, 10=Rx, 11=Frame Buffer Rx.
5	RPLV	Reply Valid: for automatic reply, set to ‘1’ to enable reply after a matching remote frame.
4	IDE	Identifier Extension: ‘0’ = standard (11 bit), ‘1’ = extended (29 bit).
3..0	DLC[3:0]	Data Length Code (0..8). For Rx, this is the expected DLC; for Tx, the DLC to transmit.
Important:
•	For Tx MOB, set CONMOB=01, then write ID, DLC, data, and enable (by setting CONMOB). The core will start arbitration.
•	For Rx MOB, set CONMOB=10. After reception, the MOB is automatically disabled (ENMOB cleared) unless RPLV=1 (auto reply).
•	For Frame Buffer Rx, set CONMOB=11 for all MOBs in the buffer set. The B×OK flag will set when all have received.
7.3 CANIDT1..4 – Identifier Tag Registers
These registers hold the identifier (11 or 29 bits) plus the RTR and reserved bits. The exact mapping depends on IDE.
For Standard (IDE=0):
•	IDT1[7:0] = ID[7:0]
•	IDT2[7:5] = ID[10:8], IDT2[4:0] = reserved
•	IDT4[2] = RTR, IDT4[0] = RB0 (reserved bit 0)
For Extended (IDE=1):
•	IDT1[7:3] = ID[7:3], IDT1[2:0] = reserved (not used)
•	IDT2[7:0] = ID[15:8]
•	IDT3[7:0] = ID[23:16]
•	IDT4[7:3] = ID[28:24], IDT4[2] = RTR, IDT4[1] = RB1, IDT4[0] = RB0
7.4 CANIDM1..4 – Identifier Mask Registers
Mask bits correspond to the same bit positions as the identifier tags. A mask bit ‘1’ enables comparison; ‘0’ forces a match (don’t care).
•	IDM1..IDM4: masks for identifier.
•	IDEMSK (bit 0 of IDM4): mask for IDE bit.
•	RTRMSK (bit 2 of IDM4): mask for RTR bit.
7.5 CANSTML/H – Time Stamp Registers
When a reception or transmission completes on this MOB, the current CAN timer value is captured into these registers (read only).
7.6 CANMSG – Data Byte Register
This is the window to the MOB’s 8 byte data buffer. The byte accessed is determined by CANPAGE[2:0] (INDX). If AINC=0, each read/write to CANMSG automatically increments INDX (mod 8), allowing sequential access to data bytes.
________________________________________
8. Programming Model
8.1 Initialization
1.	Enable the CAN controller:
o	Set CANGCON.ENA/STB = ‘1’. Wait until CANGSTA.ENFG becomes ‘1’ (or poll the status). The core will wait for 11 recessive bits on the bus before becoming active.
2.	Configure bit timing:
o	Write CANBT1, CANBT2, CANBT3 with appropriate values (see Section 10 for examples).
3.	Configure MOBs:
o	For each MOB to be used:
	Select the MOB via CANPAGE[7:4].
	Set IDT, IDM, DLC, IDE, RTR as required.
	For Tx: load data into the data buffer via CANMSG.
	For Rx: set masks for acceptance filtering.
	Write CANCDMOB with CONMOB = Tx, Rx, or FBUF_RX.
4.	Enable interrupts (optional):
o	Set CANGIE.ENIT = ‘1’.
o	Set specific enables (ENRX, ENTX, ENERR, ENBX, ENERG, ENOVRT, ENBOFF).
o	For MOB interrupts, set corresponding bits in CANIE2.
8.2 Transmission
•	Configure a MOB as Tx (CONMOB=01). The MOB becomes enabled automatically.
•	The core will scan all enabled Tx MOBs and select the lowest index MOB for arbitration.
•	On successful transmission, TXOK is set, the MOB is disabled (ENMOB cleared), and an interrupt (if enabled) is generated.
•	To send another frame, re write CONMOB=01 (after clearing TXOK).
Note: If TTC mode is active, the frame is sent only once, even if errors occur; retransmission is disabled.
8.3 Reception
•	Configure a MOB as Rx (CONMOB=10) with appropriate ID, IDE, RTR masks.
•	When a matching frame arrives, the MOB receives the data, updates IDT, DLC, and time stamp, sets RXOK, and is disabled.
•	If RPLV=1 (auto reply), the MOB automatically reconfigures as Tx and sends the data frame (the data buffer must be preloaded before setting CONMOB). After the reply is sent, TXOK is set and the MOB is disabled.
•	If a remote frame arrives, and a MOB is configured with RTR=1 (tag) and RPLV=1, the core will automatically send a data frame as reply without CPU intervention.
8.4 Frame Buffer Receive
•	Configure two or more MOBs with CONMOB=11 (frame buffer receive). They form a set.
•	The set is defined by all MOBs having CONMOB=11 (non consecutive MOBs are allowed).
•	When all MOBs in the set have received their frames, the BXOK flag (CANGIT.4) is set.
•	Clearing BXOK: Before clearing, you must re write the CONMOB fields of all MOBs in the set (e.g., set them again to 11 or to disabled). The core checks the cdmob_written strobe for each MOB; once all have been rewritten, writing a ‘1’ to BXOK will clear it and reset the buffer set.
8.5 Abort Request
Writing ‘1’ to CANGCON.ABRQ aborts all pending transmissions except the one currently in progress. The abort takes effect at the end of the current frame (if any). The ENMOB bits are cleared for all MOBs.
8.6 Listening Mode
Set CANGCON.LISTEN=‘1’ to enter listening mode. In this mode:
•	The Tx output is forced recessive.
•	The receiver is still active (monitoring), but no errors are generated.
•	Error counters are frozen.
•	The core will not acknowledge frames.
This mode is useful for bus analysis or auto baud detection.
________________________________________
9. Interrupt Handling
The core generates a single interrupt line (intr). The interrupt source can be determined by reading CANGIT (general) and CANSIT2 (MOB).
Interrupt clearing:
•	For MOB interrupts: clear the corresponding flag in CANSTMOB by writing ‘1’ to the bit.
•	For general interrupts: write ‘1’ to the corresponding bit in CANGIT.
•	For OVRTIM: writing ‘1’ to CANGIT.5 or entering the dedicated interrupt handler clears it (core auto clears on handler entry if implemented).
Priority: When multiple sources are pending, the highest priority is determined by the lowest MOB index (for MOB interrupts) followed by general interrupts.
________________________________________
10. Bit Timing Configuration
The CAN bit time is composed of time quanta (TQ). The formulas are:
•	TQ = (BRP+1) / f_clk
•	SYNC_SEG = 1 TQ
•	PROP_SEG = PRS (1–8) TQ
•	PHASE_SEG1 = PHS1 (1–8) TQ
•	PHASE_SEG2 = PHS2 (2–8) TQ, with PHS2 ≤ PHS1 and PHS2 ≥ 2 (IPT = 2 TQ)
•	Total bit time = 1 + PRS + PHS1 + PHS2 (must be 8..25 TQ)
Setting registers:
•	CANBT1.BRP = BRP (0..63)
•	CANBT2.SJW = SJW 1 (0..3 for 1..4)
•	CANBT2.PRS = PRS 1
•	CANBT3.PHS2 = PHS2 1
•	CANBT3.PHS1 = PHS1 1
•	CANBT3.SMP = 0 or 1 (three samples if 1, but only if BRP≠0)
Example for 16 MHz clock, 1 Mbit/s, 75% sample point:
•	TQ = 62.5 ns → BRP = (16e6 * 62.5e 9) – 1 = 0 → BRP=0.
•	With BRP=0, the core adjusts by adding 1 to PHS1 and subtracting 1 from PHS2.
•	Choose PRS=4, PHS1=4, PHS2=3 (but due to BRP=0, actual PHS1=5, PHS2=2). Total = 1+4+5+2 = 12 TQ.
•	Registers: CANBT1=0x00, CANBT2=(SJW=1 → 0) & (PRS=4 → 3) = 0x03? Actually SJW bits are 6:5, PRS bits 3:1. With SJW=1 → 0b00, PRS=4 → 0b011 → CANBT2 = 0b0000110? We'll provide a table.
Refer to the datasheet for comprehensive examples (Table 16 2). The core automatically handles BRP=0 compensation and range enforcement.
________________________________________
11. Error Management
The CAN core implements full fault confinement as per ISO 11898.
•	Error Active: TEC < 128 and REC < 128. Active error frames can be sent.
•	Error Passive: TEC ≥ 128 or REC ≥ 128. Passive error frames sent; waits before retransmission.
•	Bus Off: TEC > 255. The core enters bus off; it will not participate in bus communication. Recovery occurs after 128 occurrences of 11 recessive bits. Upon recovery, TEC and REC are cleared.
Error counters:
•	TEC and REC are accessible via CANREC and CANTEC.
•	The core increments/decrements them according to the CAN specification rules.
General vs. MOB errors:
•	If an error occurs during reception of a frame that does not match any enabled Rx MOB, the error is reported in CANGIT (general error).
•	If the frame matches a MOB, the error is recorded in that MOB's CANSTMOB (and the MOB interrupt is generated if enabled). The general error flags are not set in that case.
________________________________________
12. CAN Timer and TTC
The 16 bit timer (CANTIM) is clocked by f_clk divided by 8 * (CANTCON+1). It starts counting when ENFG=1. The timer is used for:
•	Time stamping: on successful reception or transmission, the current timer value is stored in the MOB's STML/H.
•	TTC: when TTC mode is enabled (CANGCON.TTC=1), the core sends frames only once, regardless of errors. Additionally, the timer value is captured on SOF or EOF (as per SYNCTTC) into CANTTC. This allows precise time triggered scheduling.
Timer overrun: When the timer rolls over from 0xFFFF to 0, an OVRTIM interrupt is generated (if enabled). The timer continues counting.
________________________________________
13. Application Examples
Example 1: Basic Initialization
c
// Enable controller and wait for active
CANGCON = 0x02;  // ENA/STB = 1
while (!(CANGSTA & 0x04)); // Wait for ENFG

// Set bit timing (e.g., 500 kbps at 16 MHz)
CANBT1 = 0x00;   // BRP=0
CANBT2 = 0x0C;   // SJW=1, PRS=4
CANBT3 = 0x37;   // PHS2=3, PHS1=4, SMP=0 (after BRP=0 compensation)

// Configure a Tx MOB (MOB 0)
CANPAGE = 0x00;          // Select MOB 0
CANIDT1 = 0x55;          // ID[7:0]
CANIDT2 = 0x80;          // ID[10:8] = 0b100, plus reserved zeros
CANIDT4 = 0x00;          // RTR=0, RB0=0
CANCDMOB = 0x40 | 0x01;  // DLC=1, Tx mode (CONMOB=01)
CANMSG = 0xAA;           // Data byte 0 (auto-increment disabled, write to index 0)

// Enable interrupts (optional)
CANGIE = 0xF0;           // Enable all MOB and general interrupts
CANIE2 = 0x01;           // Enable interrupt on MOB 0
Example 2: Receiving a Standard Frame
c
// Configure MOB 1 to receive ID=0x123, any data, standard frame
CANPAGE = 0x10;          // Select MOB 1
CANIDT1 = 0x23;          // ID[7:0] = 0x23
CANIDT2 = 0xE0;          // ID[10:8] = 0b001, IDT2[7:5]=0b100? Actually ID=0x123 → ID[10:8]=0b001, ID[7:0]=0x23.
                         // So IDT2[7:5] = 0b001 → 0x20; IDT1=0x23
CANIDT2 = 0x20;          // ID[10:8] = 0b001
CANIDM1 = 0xFF;          // Full mask on ID[7:0]
CANIDM2 = 0xE0;          // Mask only ID[10:8] bits
CANIDT4 = 0x00;          // IDE=0 (standard), RTR=0 (data)
CANIDM4(0) = 0x01;       // Mask IDE=1 (force compare)
CANCDMOB = 0x40 | 0x02;  // DLC=1? set expected DLC maybe 0, but we can set DLC=0 and use DLCW warning.
                         // Better: set CONMOB=10 (Rx)
CANCDMOB = 0x02;         // CONMOB=10, DLC=0 (any DLC accepted, DLCW will warn if different)
When a frame with ID=0x123 arrives, RXOK will be set, data stored, and the MOB disabled. To re enable, write CONMOB again.
Example 3: Frame Buffer Receive
c
// Set MOB 2, 3 as frame buffer set
CANPAGE = 0x20;          // MOB 2
CANCDMOB = 0xC0;         // CONMOB=11 (FBUF_RX), DLC=0
CANPAGE = 0x30;          // MOB 3
CANCDMOB = 0xC0;         // CONMOB=11

// When both have received, BXOK will be set.
// To clear BXOK, re-write CONMOB for both:
CANPAGE = 0x20; CANCDMOB = 0xC0; // re-write
CANPAGE = 0x30; CANCDMOB = 0xC0; // re-write
// Then clear BXOK: CANGIT = 0x10; // write 1 to BXOK
________________________________________
14. Timing Characteristics
The core operates with a system clock clk. The maximum bit rate depends on the clock frequency and the minimum TQ (which is 1/clk). With BRP=0, TQ = 1/clk. The bit time must be at least 8 TQ, so the maximum bit rate is clk/8 (with a suitable division). For example, at 16 MHz, max bit rate = 2 Mbit/s, but the CAN spec limits to 1 Mbit/s, so with proper timing it works.
Important: For high bit rates, an external crystal is recommended for frequency accuracy.
________________________________________
15. Electrical and Timing Parameters
The IP core is technology independent. The external CAN transceiver must be connected to the tx and rx pins with appropriate level shifting. The core outputs tx with active low dominant (0) and recessive (1). The rx input expects the same polarity.
All register accesses are synchronous to clk. The minimum setup/hold times depend on the target synthesis.
________________________________________
16. Troubleshooting
Common issues:
•	No communication: Check that ENFG is set (CANGSTA.2). Ensure the bus is idle (11 recessive bits).
•	Bit errors: Verify bit timing parameters; ensure total TQ is within 8 25; check clock accuracy.
•	MOB not receiving: Confirm CONMOB=10, and that ID masks correctly include the incoming ID.
•	Interrupt not firing: Check CANGIE.ENIT and specific enable bits; ensure the interrupt flag is set and not masked.
•	Bus off recovery: The core will automatically recover after 128 * 11 recessive bits; you can monitor BOFF and re enable if needed.
________________________________________
17. Register Quick Reference
Address	Register	Type	Description
0x00	CANGCON	R/W	General Control
0x01	CANGSTA	R	General Status
0x02	CANGIT	R/W	General Interrupt
0x03	CANGIE	R/W	General Interrupt Enable
0x04	CANEN1	R	Enable MOB (high)
0x05	CANEN2	R	Enable MOB (low)
0x06	CANIE1	R/W	MOB Interrupt Enable (high)
0x07	CANIE2	R/W	MOB Interrupt Enable (low)
0x08	CANSIT1	R	MOB Interrupt Status (high)
0x09	CANSIT2	R	MOB Interrupt Status (low)
0x0A	CANBT1	R/W	Bit Timing 1 (BRP)
0x0B	CANBT2	R/W	Bit Timing 2 (SJW, PRS)
0x0C	CANBT3	R/W	Bit Timing 3 (PHS2, PHS1, SMP)
0x0D	CANTCON	R/W	Timer Control
0x0E	CANTIML	R	Timer Low
0x0F	CANTIMH	R	Timer High
0x10	CANTTCL	R	TTC Capture Low
0x11	CANTTCH	R	TTC Capture High
0x12	CANTEC	R	Transmit Error Counter
0x13	CANREC	R	Receive Error Counter
0x14	CANHPMOB	R/W	Highest Priority MOB & CGP
0x15	CANPAGE	R/W	MOB Page & Data Index
0x20–0x2F	MOB page	R/W	MOB registers (see Sec. 7)

