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
set with the nearest VOD step, -6 dB.

## Changing the settings per lane

The `redriver_channels[]` table in `redriver.c` has one row per lane (16 rows), in the same layout as
the original redriver board: a **boot** setting written when the host releases reset (while PCIe
trains) and a **run** setting written afterwards. Each setting has:

- `eq`: `EQ_LEVEL_0` to `EQ_LEVEL_15`. The 4 GHz / 8 GHz boost of each level is listed next to the
  `EQ_LEVEL_<n>` defines.
- `dc_gain`: `DC_GAIN_0_DB` or `DC_GAIN_3_5_DB`.
- `vod`: `VOD_MINUS_6_DB`, `VOD_MINUS_3_5_DB`, `VOD_MINUS_1_6_DB` or `VOD_0_DB`.

Every lane gets its boot setting at the same time. After `REDRIVER_BOOT_HOLD_MS` (5 s, in
`xg_mobile_dock.c`) every lane gets its run setting at the same time, with no ramp or per-lane delay.
Boot equals run on every row at the moment. The log prints every lane's boot and running values, for
example
`run  GPU->CPU lane 2 (0x1C ch2): EQ level 0 (-0.3 / -0.8 dB @ 4/8 GHz), DC gain 0 dB, VOD -6 dB`.

Set a row's `enabled` column to `CHANNEL_DISABLED` to turn that lane off. The driver then uses the
programming guide's enable override (reg `0x06[5]` with the driver and CTLE enables cleared) to power
the channel's driver and equalizer down, and prints `disabled (driver and equalizer powered down)`.
`CHANNEL_ENABLED` hands the enables back to the part. The effect on the lane's receive termination
is not described in the guide, so check it on the board before relying on it.

RX detect is forced to always report a valid receiver, as on the other redriver boards; set
`REDRIVER_FORCE_RX_DETECT` to 0 to use the redrivers' own detection.

## Board switches

- The redrivers are held in standby while the host is in reset and powered up when it releases
  reset, through `PWDN`. Set each redriver's `PWDN` slide switch to the MCU position.
- Set the `SCL`/`SDA` slide switches to the MCU's I2C2 (`/SCL2`, `/SDA2`).
- Set `EN_SMB` to SMBus slave mode (L3).

## How it works

The DS160PR410 selects channels through the share registers instead of per-channel address
offsets: `0xFF = 0x01` selects the channel page and `0xFC` picks the channel(s) that reads and writes
go to (programming guide SNLU255). For every table row the driver:

1. checks the redriver answers and reads device ID `0x26`;
2. selects the channel and writes the CTLE, DC gain and VOD of the boot setting, then reads it back;
3. after the boot hold, writes the run setting to every lane and reads it back.

The log shows `Redriver boot settings:` and `Redriver running settings:` with one line per lane after
each host reset release, and the redrivers' latched RX detect flags every 10 seconds while PCIe is out
of reset.

For debugging, every host reset release also prints:

- `Redriver probe on I2C2:` with one line per redriver: `found, device ID 0x26`, `NOT FOUND` (with the
  HAL status and I2C error code), `ACKs its address but register access failed`, or an unexpected
  device ID; then `Redriver probe: N of 4 redrivers found`.
- If any redriver is missing, a scan of I2C2 that lists every address that answers, which shows a
  wrong address strap or a bus not connected to the MCU (`PWDN`, `SCL`/`SDA` or `EN_SMB` switches).
- For a lane that cannot be written, the step that failed (which register, read or write, HAL status
  and I2C error code, or the read-back values against the expected ones) after all retries.
- `Redriver boot: M of K enabled lanes written and verified` and the same for running.

## Sources

DS160PR410 data sheet (SNLS645) and DS160PR410 Programming Guide (SNLU255), both public.
