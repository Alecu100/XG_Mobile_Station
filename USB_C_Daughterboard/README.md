# USB-C Daughterboard: Engineering Draft A

Open `XG_Mobile_USB_Hub.kicad_pro` in KiCad 9 or newer. The root schematic contains three child sheets. PDF and SVG exports provide previews; the editable schematics use named-net connections. Existing dock files are unchanged.

## Schematic Hierarchy

- `Power.kicad_sch`: GPU/EPS inputs, 3.3 V, 5 V, 1.15 V core buck and LM51772 PD buck-boost.
- `MCU.kicad_sch`: STM32 PD controller, upstream USB-C connector, protection, power-path FETs and orientation mux.
- `USB_Hub.kicad_sch`: USB7206C, decoupling, two downstream Type-C ports and four Type-A ports.

Each child uses a large custom page with labeled circuit blocks. The PDF contains the root plus these three pages; zoom in for component-level review. Original component references and nets are retained. PCB schematic paths follow the consolidated hierarchy without changing placement. The manifest retains each original circuit block in `block` for placement regeneration.

**Not released for fabrication, assembly, or connection to a laptop.** This is a pin-connected schematic draft, not a validated power supply or USB-certified product. Zero ERC errors does not establish electrical performance or protection adequacy. Connector selection, several footprints and passive MPNs remain unresolved and are explicitly marked in the BOM.

## Agreed Topology

- Upstream: additional USB-C receptacle, up to 10 Gbps, 100 W maximum PD source at 20 V / 5 A.
- STM32G071KBT6N runs the PD policy engine using its UCPD peripheral and controls LM51772 over I2C. TCPP03-M20 is its protection/VCONN companion, not the PD policy engine.
- Downstream: two HD3SS3220 Type-C ports at 5 V / 3 A; three SuperSpeed Type-A ports; one USB 2.0-only Type-A port. USB-A BC1.2 charging is disabled.
- Hub: USB7206CT/KDX. Core rail is 1.15 V, not 1.2 V nominal; allowable range is 1.09-1.21 V.
- Inputs: two GPU PCIe 8-pin cables from the SAME PSU, OR one EPS 8-pin cable. A mutually exclusive, break-before-make power selector is shown. No 12 V pass-through to another board is included.
- Intended supply: regulated nominal 12 V. The 5 V WEBENCH design was supplied for 11-13 V; do not assume the complete assembly has been qualified for 10-14 V.
- An unrouted PCB placement draft is included; no MCU firmware is included. The schematic does not establish USB-PD 3.2 compliance; that requires a suitable qualified policy stack and compliance testing. No EPR or PPS capability is claimed.

## Files

- `XG_Mobile_USB_Hub.kicad_sch`: hierarchy root.
- `Daughterboard.kicad_sym` and `sym-lib-table`: project-local symbols, embedded in the schematics as well.
- `BOM.csv`: individual component BOM, not a procurement-ready or JLC assembly upload BOM.
- `connectivity.json`: generator's intended pin-to-net assignments.
- `ERC.json`: KiCad electrical-rule report.
- Two expected ERC warnings retain the GPIO-capable types of grounded MCU pins 19/21. Confirm the exact N-variant dead-battery termination and GPIO mapping before release; these warnings have not been suppressed.
- `USB_C_Daughterboard.pdf` and `SVG/`: rendered schematics.
- `Reference/WEBENCH/`: the three unmodified supplied SVGs and BOMs.
- `Reference/`: downloaded manufacturer datasheets used in the design.
- `generate_schematic.py`: reproducible source. Running it overwrites the generated daughterboard sheets/library/BOM, so preserve manual schematic edits before regenerating.
- `validate_schematic.py`: parses each sheet, exports a real KiCad XML netlist, and checks every connected physical pin plus selected independent design invariants.

## PCB Placement Draft

Open `XG_Mobile_USB_Hub.kicad_pcb` from the project. The provisional outline is 120 x 80 mm. Downstream USB connector positions are reserved on the front edge; upstream USB-C and GPU/EPS power inputs are reserved on the rear. No mounting holes or enclosure constraints have been specified. The TPS62130 core buck is retained.

- 260 physical footprints are placed inside the outline with schematic UUID paths and assigned nets. Grouped placement is a floorplan, not routing-optimized placement; decoupling proximity, switching loops and high-speed escape still need layout work.
- 37 components without resolved packages are represented by padless markers outside the outline. These are NOT usable footprints and carry no copper. They include connectors, inductors, some power ICs, bulk capacitors, fuses and the power selector. Connector rectangles inside the outline are reservations, not selected land patterns.
- Every loaded footprint's numbered pad set is checked against the schematic, and 825 connected pads are checked against a fresh KiCad XML netlist. This does not verify the 37 missing packages or complete physical connectivity for the design.
- Four copper layers and 1.6 mm thickness are provisional. No dielectric stack-up, controlled-impedance geometry, power planes, tracks, or routing are supplied. The inner layers are empty. No Gerbers or fabrication release is provided.
- DRC: 499 unconnected items, 37 unresolved-footprint library findings, 12 within-package clearance findings against the default 0.2 mm rule, and four 0.2 mm thermal drills below the default 0.3 mm minimum. No shorts, overlapping courtyards or solder-mask bridges were reported after placement correction. These results are not a DRC pass; set fabrication rules only after selecting a process.
- The hub footprint is KiCad's USB7206C-referenced VQFN-100 land pattern. HD3SS3220 footprints match TI RNH0030A. C403/C404 were corrected to 0805 to match their specified GRM21 parts. All other assigned packages still require final manufacturing review.

`PCB_Top.svg` and `PCB_Bottom.svg` show the placement and off-board staging. The bottom preview is viewed through the board, not mirrored. `PCB_Placement_Report.json` lists unresolved items and imported references; `PCB_DRC.json` contains the complete DRC findings.

Regenerate only before making manual PCB edits: `generate_pcb.py` OVERWRITES the board and its placement report. It uses KiCad's bundled Python, not ordinary system Python:

```powershell
& 'C:\Program Files\KiCad\9.0\bin\python.exe' USB_C_Daughterboard/generate_pcb.py
```

After a hierarchy-only change, use `generate_pcb.py --relink-sheets` to update PCB sheet paths without regenerating placement.

## Power-Design Changes

| Block | Implementation / Deviation |
| --- | --- |
| 3.3 V | TPS56A37 and Design104 values; EN left floating as allowed by datasheet. PG pull-up moved to 3.3 V for MCU logic. |
| 5 V | TPS40305 and Design105 compensation, inductance, MOSFETs and capacitance preserved. |
| Hub core | Added TPS62130, 43.7k/100k divider for 1.1496 V nominal, 3 A regulator capability. |
| PD inductor | APS1040M2R2A, 2.2 uH, JLC C47327242, as requested. Replaces Design103's 1.8 uH. |
| PD bridge | Four CSD17577Q3A 30 V MOSFETs replace Design103's mixed MOSFET set, including its 25 V output-side device. This substitution requires loss/gate-drive validation. |
| PD control | ADDR tied to ground enables I2C at 7-bit address 0x6A. Original 6.49k CFG1 resistor DISABLES I2C and is not retained. CFG3/4 resistors removed. |
| PD feedback | Internal feedback selected by FB tied to VCC2; VIN-FB grounded. Original fixed-20 V feedback dividers removed. |
| PD defaults | EN and NRST pulled low. Firmware must explicitly initialize the controller and establish 5 V before port power is enabled. Do not rely on an assumed reset output voltage. |
| Current sensing | Original 2 mOhm inductor shunt retained provisionally. Average-current block disabled at ISET; TCPP03 provides separate port OCP using 7 mOhm. These protect different fault conditions. |
| Port protection | TPS259470L per downstream port; Type-C enable is attach-gated. TCPP03 and common-source back-to-back FETs protect upstream VBUS. TVS parts are preliminary; transient/clamping coordination remains a release gate. |

At 100 W and assumed 90% conversion efficiency, the PD stage alone draws approximately 10.1 A from 11 V, or 11.1 A from 10 V. The inductor listing gives 12.12 A rated and 18.18 A saturation current. This is not generous thermal margin and is not a qualification result. Include ripple, temperature derating and fault current, not just average current.

The 5 V operating allocation is 6 A for the two C ports, 2.7 A for three SuperSpeed A ports, and 0.5 A for the USB 2.0 A port, leaving about 2.8 A for VCONN, control overhead and margin. Fault current limits sum to more than normal operating allocations; coordinate upstream protection and regulator current limits accordingly.

## Firmware Contract

1. Keep hub reset asserted, VBUS_DET low, upstream mux disabled, LM51772 EN/NRST low and TCPP03 disabled during reset/startup. Configure GPIOs before releasing those controls.
2. Initialize I2C1 on PB6/PB7, UCPD1 on PA8/PB15, ADCs and fault interrupts. Retain NRST on PF2; configure flash-boot option bytes and preserve SWD on PA13/PA14. Pins 19/21 are grounded for the unused UCPD dead-battery function; do not configure them as driven GPIO outputs.
3. Raise LM51772 NRST with EN still low. Read back configuration, set internal-feedback mode, a verified current/slope setting and a 5 V target before enabling switching. Verify PD_RAW by ADC. Apply the input-voltage limits established by bench qualification.
4. Enable TCPP03 (7-bit address 0x34 with I2C_ADD grounded), enter normal mode, keep the unused consumer path disabled, and keep provider MOSFETs off until a valid source attachment and safe VBUS state are established.
5. Advertise only qualified fixed PDOs. Intended starting set is 5 V / 3 A, 9 V / 3 A, 15 V / 3 A, and 20 V / up to 5 A. Offer more than 3 A only after successful cable discovery identifies a 5 A e-marked cable. Otherwise limit to 3 A. Do not advertise 20 V / 5.5 A.
6. On a voltage request, follow PD transition timing, program LM51772, check measured voltage/fault state, then send PS_RDY. Implement detach, hard reset, watchdog recovery, overcurrent and overvoltage handling, and VBUS/VCONN discharge. Never discharge against an enabled provider path.
7. Power source and USB data UFP are independent roles. Request/accept the appropriate PD data-role swap so the laptop is data DFP while this board stays power source. Release HUB_RESET_N only after both hub rails are stable; assert HUB_VBUS_DET only in the valid upstream attachment state. Drive orientation and mux enable from the negotiated attachment.
8. Handle downstream fault latches through hub port control. The two HD3SS3220s use GPIO mode, so their I2C addresses do not appear on the shared MCU bus. Their ID outputs are available to the MCU.

The GPU ADC dividers measure rail voltage, NOT independent cable presence: the fused GPU branches are electrically combined and can backfeed an unused input. Do not use those ADC readings to infer that both GPU cables are plugged in. Full-load operation requires an external assembly/operating check or a separately designed cable-presence interlock.

## Release Gates

- Recalculate/simulate LM51772 compensation, slope compensation, switching loss and inductor peak limit for the new 2.2 uH inductor and every advertised PDO. The retained 2 mOhm shunt and WEBENCH compensation are explicitly provisional; port OCP does not protect the inductor from every internal fault.
- Validate all power rails, startup/sequencing, MOSFET SOA, fault energy, input fuses, cable/connector current sharing and thermal rise at load. Verify loop stability and voltage transitions with representative cable/load capacitance.
- Each Type-A port includes a provisional 220 uF +/-20% bulk capacitor in addition to 10 uF ceramic. Select its ESR/ripple rating and qualify port droop, eFuse ramp/inrush and fault timing with that load. Verify upstream and downstream Type-C source capacitance/discharge requirements independently; the drawn capacitor values are not compliance results.
- Validate dynamic negotiated-voltage OVP. The TCPP divider supplies a fixed approximately 22 V ceiling, not hardware tracking OVP for every 5/9/15 V contract. Firmware fault detection alone is not a certified substitute for appropriate protection.
- Qualify TVS clamping and layout parasitics against every device's transient/absolute ratings. Add input reverse-polarity/hot-plug protection as required by the final connector and power-source specification. Present input protection is fuses, selector and a preliminary TVS, not an electronic hot-swap controller.
- Finalize physical connectors, the DC-rated selector, fuses/holders, all missing footprints and `SELECTION_REQUIRED` MPNs. Verify manufacturer pin numbering, exposed pads, connector keying and mating-face orientation. Generic connector symbols are electrical assignments, not verified land patterns.
- Verify STM32 option bytes, clock accuracy, firmware capacity/stack compatibility and PD source/data-role behavior on target laptops. Qualify source-only use of TCPP03 with its unused sink path terminated as drawn. No firmware or PD 3.2 certification has been delivered.
- Validate USB 10 Gbps routing, stack-up, insertion loss, return paths, ESD placement and Type-C orientation with both cable flips. Crystal load values are initial estimates; tune against actual pad/input capacitance.

## Reproduction

Latest automated checks: 297 physical components and 1,098 connected pins match the KiCad XML export; ERC has zero errors and the two documented MCU warnings. The PDF has four pages. There are 185 BOM rows with explicitly unresolved part selections and 37 rows without footprints. Counts include generic passives and connectors and must not be mistaken for a completed procurement BOM.

Dependencies: Python with `sexpdata`, installed KiCad 9 symbol libraries and `kicad-cli`. Generator and validator currently use this machine's KiCad 9 install path.

```powershell
python USB_C_Daughterboard/generate_schematic.py
python USB_C_Daughterboard/validate_schematic.py
& 'C:\Program Files\KiCad\9.0\bin\kicad-cli.exe' sch erc --format json --output USB_C_Daughterboard/ERC.json USB_C_Daughterboard/XG_Mobile_USB_Hub.kicad_sch
```

Sources: TI LM51772 SNVSC22D, TPS4030x SLUS964D, TPS56A37 SLVSHC9, TPS25947 SLVSFC9C, HD3SS3220 SLLSES1E, HD3SS3212 SLASE74F; Microchip USB7206C DS00003850F; ST TCPP03 DS13618 Rev 2 and ST's `STM32_open_pin_data` STM32G071K(8-B)TxN pin database. Standard supporting symbol pin maps were resolved from the installed KiCad 9 libraries. Stock, orderability and package compatibility of the complete BOM have not been qualified.