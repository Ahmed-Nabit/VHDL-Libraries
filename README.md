VHDL Libraries:

1- Math Engine
  * cordic_atan - 18-stage CORDIC vectoring mode pipeline computing Q32.32 arctangent
  * cordic_hyperbolic - Single‑shot CORDIC hyperbolic core for ln(x) and exp(x) (Q32.32)
  * cordic_sincos - 18-stage CORDIC rotation mode pipeline computing Q32.32 sine and cosine
  * fixed_div - 97-stage pipelined Q32.32 signed divider with rounding, saturation and abort
  * fixed_mod_pipe - Pipelined Q32.32 modulo reduction (64-bit input, 32-bit output)
  * fixed_mul_32x32 - 3-stage pipelined Q16.16 x Q16.16 multiplier with overflow saturation
  * fixed_mul_64x64 - 4-stage pipelined Q32.32 x Q32.32 multiplier with overflow detection and saturation
  * fixed_sqrt - 49-stage pipelined radix-2 restoring square root for Q32.32 fixed-point
  * fixd_util - Fixed-point utility functions: unit conversions, angle limiters and inverse operations
    
2- Interfaces:
  * SPI
  * USART
  * TWI or I2C
  * CAN
  * TSN Endpoint
