# XG Mobile Dock MCU firmware: quad redriver board

Firmware for `XG_Mobile_Dock_Quad_Redriver`, which carries four DS160PR410 redrivers and the same
STM32F030C8 as the other redriver boards. It is the dual-redriver firmware with the redriver
driver ([redriver.c](Core/Src/redriver.c)) rewritten for the DS160PR410. The connector handshake,
fan control and logging are unchanged.

## Redrivers

Each DS160PR410 carries four lanes in one direction. Addresses are set with the slide switches on
the `EQ1_ADDR1` / `EQ0_ADDR0` pins (data sheet Table 5), as (`EQ1_ADDR1`, `EQ0_ADDR0`):

| Direction | Lanes | Strap | 7-bit address | Board reference |
|-----------|-------|-------|---------------|-----------------|
| CPU to GPU | 0-3 | L0,L0 | `0x18` | U6 |
| CPU to GPU | 4-7 | L0,L1 | `0x19` | U2 |
| GPU to CPU | 0-3 | L1,L0 | `0x1C` | U1 |
| GPU to CPU | 4-7 | L1,L1 | `0x1D` | U9 |

If you strap them differently, change the four `*_ADDR_I2C` defines at the top of `redriver.c`.

## Settings applied

| Direction | CTLE index | CTLE DC gain | TX VOD |
|-----------|------------|--------------|--------|
| CPU to GPU | 8 (6.7 dB at 4 GHz, 11.3 dB at 8 GHz) | 0 dB | 0 dB |
| GPU to CPU | 0 (-0.3 dB at 4 GHz, -0.8 dB at 8 GHz) | 0 dB | -6 dB |

The DS160PR410 has no "flat gain" setting like the 8-channel redrivers. Its DC level comes from the
CTLE DC gain (0 or +3.5 dB) and the TX VOD (0, -1.6, -3.5 or -6 dB), so the GPU to CPU "-5 dB" is
set with the nearest VOD step, -6 dB. Edit the `redriver_banks[]` table to change any of this.

RX detect is forced to always report a valid receiver, as on the other redriver boards; set
`REDRIVER_FORCE_RX_DETECT` to 0 to use the redrivers' own detection.

## Board switches

- The redrivers are held in standby while the host is in reset and powered up when it releases
  reset, through `PWDN`. Set each redriver's `PWDN` slide switch to the MCU position.
- Set the `SCL`/`SDA` slide switches to the MCU's I2C2 (`/SCL2`, `/SDA2`).
- Set `EN_SMB` to SMBus slave mode (L3).

## How it works

The DS160PR410 selects channels through the share registers instead of per-channel address
offsets: `0xFF = 0x03` selects the channel page with broadcast writes, and `0xFC` picks the channel
used for read-back (programming guide SNLU255). The driver:

1. checks the device answers and reads device ID `0x26`;
2. writes the CTLE, DC gain and VOD settings to all four channels at once;
3. reads each channel back and reports any mismatch.

The log shows `Redrivers configured: N of 4` after each host reset release, and the redrivers'
latched RX detect flags every 10 seconds while PCIe is out of reset.

## Sources

DS160PR410 data sheet (SNLS645) and DS160PR410 Programming Guide (SNLU255), both public.
