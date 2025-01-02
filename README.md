# MPPT BDZ

1. DataRate = 2400bps, data scale: 0.1

2. Message Protocol  
   a. Read Example:  `[00 03 00 0C 00 01] + [16bit ModbusCRC]`  
      `Device(0x00) + Read (0x03) + Function (0x00, 0x0C) + header(0x00, 0x01) + CRC[2bytes]`

   b. Write Example: `[00 06 00 03 00 64] + [16bit ModbusCRC]`  
      `Device(0x00) + Read (0x06) + Function (0x00, 0x03) + data(0x00, 0x64) + CRC[2bytes]`

3. Functions  
   a. READ BAT CHARGING CURRENT: `0x0A`  
   b. READ BAT CHARGING VOLT: `0x07`  

   c. READ BAT/LOAD VOLTAGE: `0x0C`  
   d. READ BAT/LOAD CURRENT: `0x0D`  

   e. READ INPUT VOLT: `0x0B`  

   f. SET BAT CHARGING VOLT: `0x04`  
   g. SET BAT CHARGING CURRENT: `0x03`  

4. Reply example: `01 03 02 [data] [data] + [CRC]`

---

# EPEVER (Read only)

1. DataRate = 115200, data scale: 0.01

2. Message Protocol  
   a. Read Example: `[01 04 31 04 00 01] + [16bit ModbusCRC]`  
      `Device(0x01) + Read (0x04) + Function (0x31, 0x04) + header(0x00, 0x01) + CRC[2bytes]`

3. Functions  
   a. BAT V: `01 04 31 04 00 01 + [CRC]`  
   b. BAT A: `01 04 33 1B 00 02 + [CRC]`  
   c. BAT SOC: `01 04 31 1A 00 01 + [CRC]`

   d. INPUT V: `01 04 31 00 00 01 + [CRC]`  
   e. INPUT A: `01 04 31 01 00 01 + [CRC]`

   f. LOAD V: `01 04 31 0C 00 01 + [CRC]`  
   g. LOAD A: `01 04 31 0D 00 01 + [CRC]`

4. Reply example: `01 04 31 [data] [data] + [CRC]`
