#include "main.h"

#include <stdint.h>
#include <stdio.h>

extern I2C_HandleTypeDef hi2c2;

/* DS160PT801 source map (all links are publicly accessible):
 *
 * [TI-IMAGE-THREAD] Base thread containing the public EEPROM images, SigCon
 * screenshots, configuration discussion, and test results:
 * https://e2e.ti.com/support/interface-group/interface/f/interface-forum/1592822/ds160pt801-can-t-link-device
 *
 * [TI-4X4-IMAGE] Public 4x4 EEPROM image attachment:
 * https://e2e.ti.com/cfs-file/__key/communityserver-discussions-components-files/138/DS160PT801_5F00_4x4.zip
 *
 * [TI-4X4X4X4-IMAGE] TI reply with the working 4x4x4x4 image and SigCon
 * configuration screenshot:
 * https://e2e.ti.com/support/interface-group/interface/f/interface-forum/1592822/ds160pt801-can-t-link-device/6166320
 *
 * [TI-4X4-CLKREQ-IMAGE] TI reply with the 4x4 CLKREQ EEPROM image:
 * https://e2e.ti.com/support/interface-group/interface/f/interface-forum/1592822/ds160pt801-can-t-link-device/6180872
 *
 * [TI-X8-IMAGE] TI employee posts the original x8/REFCLK_OUT EEPROM image:
 * https://e2e.ti.com/support/interface-group/interface/f/interface-forum/1592822/ds160pt801-can-t-link-device/6188921
 *
 * [TI-X8-RESULTS] User posts image.zip, F2=81/F3=A0, 0D=91, and explicitly
 * cautions that the initial "Pass" might be inaccurate without a power cycle:
 * https://e2e.ti.com/support/interface-group/interface/f/interface-forum/1592822/ds160pt801-can-t-link-device/6213179
 * https://e2e.ti.com/cfs-file/__key/communityserver-discussions-components-files/138/image.zip
 *
 * [TI-REGISTER-DUMP] Public register-dump attachment, renamed from .hex to
 * .txt for upload. It contains one active EEPROM command:
 * https://e2e.ti.com/cfs-file/__key/communityserver-discussions-components-files/138/EFOCU8B_5F00_20251128_5F00_1426.txt
 *
 * [TI-CLKREQ] TI confirms that 0xFA contains the intended CLKREQ# control bits
 * and is a global register that may need manual insertion into an EEPROM image:
 * https://e2e.ti.com/support/interface-group/interface/f/interface-forum/1592822/ds160pt801-can-t-link-device/6180846
 *
 * [TI-DFE-THREAD] Public DS160PT801 discussion associates register 0x58 with
 * DFE control/masking attempts and identifies C3-C6 plus B6[3:2] as override
 * controls. Exact fields and a working sequence were not published:
 * https://e2e.ti.com/support/interface-group/interface/f/interface-forum/1584903/ds160pt801-is-there-an-algorithm-to-override-dfe-and-ref0-ref1
 *
 * [TI-SMBUS] TI confirms that the device powers up in 16-bit offset mode and
 * supports standard SMBus reads and writes with a 16-bit register offset. It
 * does not implement the Intel command-code format. SigCon and TI's Python API
 * switch the device to the 8-bit mode assumed by most of the programming guide:
 * https://e2e.ti.com/support/interface-group/interface/f/interface-forum/1188590/ds160pt801-smbus-interface/4481703
 *
 * [TI-NON-COMMON-CLOCK] Public discussion identifying AF/SRIS_EN and related
 * field names; TI's configuration answer was moved to private messages:
 * https://e2e.ti.com/support/interface-group/interface/f/interface-forum/1421060/ds160pt801-eeprom-config-for-non-common-clock
 *
 * [TI-SRIS-IMAGE] A later public TI reply supplies an EEPROM image specifically
 * for PCIe x4 with REFCLK_OUT and SRIS enabled. It writes AF=DC. This board
 * uses a common reference clock, so that topology-specific setting is not used:
 * https://e2e.ti.com/support/interface-group/interface/f/interface-forum/1631316/ds160pt801-ds160pt801-sincon-eeprom-programming-sris-mode-configuration-pcie-x4-refclkout/6295540
 * https://e2e.ti.com/cfs-file/__key/communityserver-discussions-components-files/138/DS160PT801_5F00_4x4_5F00_REFCLKen_5F00_SRIS.hex
 *
 * [OPENBIC] Public Apache-2.0 DS160PT801 driver. It switches power-up 16-bit
 * mode to 8-bit mode by transmitting 68 01 00, then uses the 8-bit register
 * map. It identifies vendor 0x4172 at F6 and device 0x24 at F1:
 * https://github.com/facebook/OpenBIC/blob/main/common/dev/ds160pt801.c
 *
 * [TI-WIDTH] 0xF2/0xF3 are global link-width registers and must be written
 * before PCIe link training when WIDTH is floating:
 * https://e2e.ti.com/support/interface-group/interface/f/interface-forum/1185828/ds160pt801-asking-for-ds160pt801-updated-design-review-and-suggestions/4552175
 *
 * [TI-WIDTH-EDIT] Public configuration-set-14 screenshot showing the metadata
 * used for the F3 EEPROM command:
 * https://e2e.ti.com/support/interface-group/interface/f/interface-forum/1592822/ds160pt801-can-t-link-device/6195109
 *
 * [TI-ADDR20] Public TI review identifying strap address 0x20:
 * https://e2e.ti.com/support/interface-group/interface/f/interface-forum/1185828/ds160pt801-asking-for-ds160pt801-updated-design-review-and-suggestions/4472257
 *
 * [TI-ADDR28] Public TI support discussion using single-chip address 0x28:
 * https://e2e.ti.com/support/interface-group/interface/f/interface-forum/1592822/ds160pt801-can-t-link-device/6248261
 *
 * [TI-PR410-ADDR] Public DS160PR410 datasheet, section 7.5.1.2, documents
 * SMBus/I2C register control and strap-selected addresses. This related
 * redriver is only the heuristic origin for the broad fallback scan; it does
 * not prove that every scanned address is valid for DS160PT801:
 * https://www.ti.com/document-viewer/DS160PR410/datasheet#smbus-i2c-register-control-interface-t5706319-18/t5706319-18
 *
 * [TI-PR810-ADDR] Public DS160PR810 Programming Guide, section 1.1,
 * explicitly labels its complete 0x18..0x37 map as 7-bit addresses. This
 * related redriver supports the broad probe heuristic, not PT801 equivalence:
 * https://www.ti.com/lit/pdf/SNLU268
 *
 * Hardware design provenance for the PCB and schematics:
 *
 * [TI-PUBLIC-PINOUT] TI's public Ultra Librarian integration supplies the
 * DS160PT801ACBR ACB-332 CAD symbol and complete ball-to-signal mapping:
 * https://vendor.ultralibrarian.com/TI/embedded/?gpn=DS160PT801&package=ACB&pin=332&sid=019aa0ca92500020f4d7bdfbcb200507d002a07500a83&c=1
 *
 * [TI-ASUS-SCHEMATIC-REVIEW] The retimer schematic was based in part on this
 * public TI E2E review of an ASUS design. TI reviews the PCIe coupling,
 * REFCLK/REFCLK_OUT, JTAG, reset, CLKREQ#, WIDTH, SMBus straps and pull-ups,
 * EEPROM interface, and power rails. The follow-up explicitly records comments
 * from an ASUS email for public reference:
 * https://e2e.ti.com/support/interface-group/interface/f/interface-forum/1185828/ds160pt801-asking-for-ds160pt801-updated-design-review-and-suggestions/4472257
 * https://e2e.ti.com/support/interface-group/interface/f/interface-forum/1185828/ds160pt801-asking-for-ds160pt801-updated-design-review-and-suggestions/4473059
 *
 * [TI-ASUS-LAYOUT-REVIEW] TI's public PCB review calls for back-drilling
 * high-speed via stubs, nearby ground vias, less than 5 mil intra-pair skew,
 * and ground voids under series coupling capacitors. TI also confirms the
 * shared 100 MHz clock topology shown by the ASUS discussion:
 * https://e2e.ti.com/support/interface-group/interface/f/interface-forum/1185828/ds160pt801-asking-for-ds160pt801-updated-design-review-and-suggestions/4475450
 * https://e2e.ti.com/support/interface-group/interface/f/interface-forum/1185828/ds160pt801-asking-for-ds160pt801-updated-design-review-and-suggestions/4488485
 *
 * [TI-EVM-HARDWARE] Public DS160PT801X16EVM schematic and TI high-speed layout
 * guidance used as additional schematic and PCB references:
 * https://www.ti.com/lit/ug/snlu254a/snlu254a.pdf
 * https://www.ti.com/lit/an/slla414/slla414.pdf
 *
 * [TI-PUBLIC-X8-SCHEMATIC] A separate public E2E design-verification thread
 * includes a user-posted Gen4 x8 DS160PT801 schematic PDF and TI review. TI
 * confirms the REFCLK-to-retimer/REFCLK_OUT-to-endpoint topology, 33-ohm
 * REFCLK_OUT resistor, RX_DET_BYP, EEPROM mode, and shared SMBus usage:
 * https://e2e.ti.com/support/interface-group/interface/f/interface-forum/1669549/ds160pt801-ds160pt801acbr-schematic-design-verification
 * https://e2e.ti.com/cfs-file/__key/communityserver-discussions-components-files/138/Gen4_2D00_x8_2D00_PCIe_2D00_Retimer_2D00_DS160PT801ACBR.pdf
 * https://e2e.ti.com/support/interface-group/interface/f/interface-forum/1669549/ds160pt801-ds160pt801acbr-schematic-design-verification/6441595
 *
 * [TI-PUBLIC-EFOCU8B-SCHEMATIC] Another public E2E thread includes a posted
 * EFOCU8B schematic archive used as a comparison design. The archive is
 * password protected, so only the publicly visible discussion was relied on:
 * https://e2e.ti.com/cfs-file/__key/communityserver-discussions-components-files/138/SPK_5F00_EFOCU8B_5F00_V01_5F00_1023_5F00_SCH_5F00_lock.zip
 *
 * [BOARD-ADDR] This board's floating SMB_ADDR_0/1 straps and U1 wiring:
 * ../../../XG_Mobile_Dock_Retimer.kicad_sch
 */

/* Software-only sentinel and timeout; these are implementation choices, not
 * DS160PT801 register or strap values. */
#define RETIMER_ADDRESS_INVALID         0xFFU
#define RETIMER_I2C_TIMEOUT_MS          50U

#define RETIMER_REG_REFCLK_OUT          0x0DU /* [TI-X8-RESULTS] */
#define RETIMER_REG_DEVICE_ID           0xF1U /* [OPENBIC] */
#define RETIMER_REG_LINK_WIDTH_0        0xF2U /* [TI-X8-RESULTS], [TI-WIDTH] */
#define RETIMER_REG_LINK_WIDTH_1        0xF3U /* [TI-X8-RESULTS], [TI-WIDTH] */
#define RETIMER_REG_VENDOR_ID           0xF6U /* [OPENBIC] */

#define RETIMER_REFCLK_OUT_ENABLED      0x91U /* [TI-X8-RESULTS] */
#define RETIMER_LINK_WIDTH_X8_0         0x81U /* [TI-X8-RESULTS] */
#define RETIMER_LINK_WIDTH_X8_1         0xA0U /* [TI-X8-RESULTS] */
#define RETIMER_DEVICE_ID               0x24U /* [OPENBIC] */
#define RETIMER_VENDOR_ID               0x4172U /* [OPENBIC] */

typedef struct {
    uint8_t source_prefix0;
    uint8_t source_prefix1;
    uint8_t reg;
    uint8_t value;
    uint8_t verify;
} retimer_register_t;

/* Probe the board's expected 7-bit address first. The related DS160PR810
 * programming guide explicitly defines 7-bit address pairs from 0x18 through
 * 0x37. Retain that broad range as a heuristic; only 0x20 and 0x28 have direct
 * DS160PT801 public examples cited above. */
static const uint8_t retimer_address_candidates[] = {
    0x1A, /* [BOARD-ADDR]: expected address for this assembly */
    0x18, 0x19, 0x1B, 0x1C, 0x1D, 0x1E, 0x1F,
    0x20, /* [TI-ADDR20] */
    0x21, 0x22, 0x23, 0x24, 0x25, 0x26, 0x27,
    0x28, /* [TI-ADDR28] */
    0x29, 0x2A, 0x2B, 0x2C, 0x2D, 0x2E, 0x2F,
    0x30, 0x31, 0x32, 0x33, 0x34, 0x35, 0x36, 0x37,
};

/* [TI-X8-RESULTS]: exact command order from EFOCU8x_clk_x8_Pass.hex.
 * A public SigCon screenshot identifies 1C 07 configurations as device 0,
 * shared page, both dies, mask 1, one-byte payload, active Manager. It does not
 * establish direct-SMBus operations for 1D 07 or 1C 17, so all prefix bytes are
 * retained as provenance but not sent. Unknown registers are written without readback gating;
 * the three publicly identified settings are verified before reset release.
 * TI confirms FA=30 as the intended global CLKREQ# control setting, but the
 * comparison image containing it failed after a full power cycle, so it is
 * omitted. The forum author cautions that the initial "Pass" may not have
 * included a power cycle, so this sequence still requires cold-boot validation
 * on this board. */
static const retimer_register_t retimer_public_x8_sequence[] = {
    { 0x1C, 0x07, 0xDA, 0x01, 0 },
    { 0x1C, 0x07, 0xDE, 0x10, 0 },
    { 0x1C, 0x07, 0xDF, 0x10, 0 },
    { 0x1C, 0x17, 0xE0, 0x01, 0 },
    { 0x1D, 0x07, 0x9E, 0x44, 0 },
    { 0x1C, 0x07, 0xD6, 0x07, 0 },
    { 0x1D, 0x07, 0x58, 0xC0, 0 }, /* DFE-related; fields unknown [TI-DFE-THREAD] */
    { 0x1D, 0x07, 0xC0, 0x00, 0 },
    { 0x1C, 0x07, 0xB7, 0x00, 0 },
    { 0x1D, 0x07, 0x91, 0x0A, 0 },
    { 0x1C, 0x07, 0x93, 0x03, 0 },
    { 0x1D, 0x07, 0x07, 0x03, 0 },
    { 0x1C, 0x07, 0xDA, 0x05, 0 },
    { 0x1C, 0x07, RETIMER_REG_LINK_WIDTH_0, RETIMER_LINK_WIDTH_X8_0, 1 },
    { 0x1C, 0x07, RETIMER_REG_LINK_WIDTH_1, RETIMER_LINK_WIDTH_X8_1, 1 },
    { 0x1C, 0x07, RETIMER_REG_REFCLK_OUT, RETIMER_REFCLK_OUT_ENABLED, 1 },
};

static uint8_t retimer_address = RETIMER_ADDRESS_INVALID;
static uint8_t retimer_8bit_mode;

static HAL_StatusTypeDef retimer_read(uint8_t reg, uint8_t *value)
{
    return HAL_I2C_Mem_Read(&hi2c2, (uint16_t)(retimer_address << 1), reg,
                            I2C_MEMADD_SIZE_8BIT, value, 1,
                            RETIMER_I2C_TIMEOUT_MS);
}

static HAL_StatusTypeDef retimer_write(uint8_t reg, uint8_t value)
{
    return HAL_I2C_Mem_Write(
        &hi2c2, (uint16_t)(retimer_address << 1), reg,
        I2C_MEMADD_SIZE_8BIT, &value, 1, RETIMER_I2C_TIMEOUT_MS);
}

static HAL_StatusTypeDef retimer_write_verified(uint8_t reg, uint8_t value)
{
    uint8_t readback = 0;
    HAL_StatusTypeDef status = retimer_write(reg, value);

    if (status != HAL_OK) {
        return status;
    }

    status = retimer_read(reg, &readback);
    if (status != HAL_OK) {
        return status;
    }

    return readback == value ? HAL_OK : HAL_ERROR;
}

static HAL_StatusTypeDef retimer_enter_8bit_mode(void)
{
    static uint8_t mode_switch[] = { 0x68U, 0x01U, 0x00U }; /* [OPENBIC] */
    uint8_t vendor[2] = { 0 };
    uint8_t device = 0;

    if (!retimer_8bit_mode) {
        if (HAL_I2C_Master_Transmit(&hi2c2, (uint16_t)(retimer_address << 1),
                                    mode_switch, sizeof(mode_switch),
                                    RETIMER_I2C_TIMEOUT_MS) != HAL_OK) {
            return HAL_ERROR;
        }
        retimer_8bit_mode = 1;
    }

    if (HAL_I2C_Mem_Read(&hi2c2, (uint16_t)(retimer_address << 1),
                         RETIMER_REG_VENDOR_ID, I2C_MEMADD_SIZE_8BIT,
                         vendor, sizeof(vendor), RETIMER_I2C_TIMEOUT_MS) != HAL_OK ||
        retimer_read(RETIMER_REG_DEVICE_ID, &device) != HAL_OK) {
        return HAL_ERROR;
    }

    if (((uint16_t)vendor[0] | ((uint16_t)vendor[1] << 8)) != RETIMER_VENDOR_ID ||
        device != RETIMER_DEVICE_ID) {
        printf("DS160PT801 identity mismatch: vendor=%02X%02X device=%02X\n",
               vendor[1], vendor[0], device);
        return HAL_ERROR;
    }

    printf("DS160PT801 switched to 8-bit mode; vendor=0x%04X device=0x%02X\n",
           RETIMER_VENDOR_ID, RETIMER_DEVICE_ID);
    return HAL_OK;
}

void retimer_reset_state(void)
{
    retimer_address = RETIMER_ADDRESS_INVALID;
    retimer_8bit_mode = 0;
}

static HAL_StatusTypeDef retimer_verify_public_x8_sequence(void)
{
    for (size_t index = 0;
         index < sizeof(retimer_public_x8_sequence) /
                     sizeof(retimer_public_x8_sequence[0]);
         ++index) {
        const retimer_register_t *setting = &retimer_public_x8_sequence[index];
        uint8_t value = 0;

        if (!setting->verify) {
            continue;
        }

        if (retimer_read(setting->reg, &value) != HAL_OK ||
            value != setting->value) {
            printf("DS160PT801 final verify failed: reg 0x%02X expected 0x%02X got 0x%02X\n",
                   setting->reg, setting->value, value);
            return HAL_ERROR;
        }
    }

    return HAL_OK;
}

HAL_StatusTypeDef retimer_probe(void)
{
    retimer_address = RETIMER_ADDRESS_INVALID;

    for (size_t index = 0;
         index < sizeof(retimer_address_candidates) / sizeof(retimer_address_candidates[0]);
         ++index) {
        uint8_t candidate = retimer_address_candidates[index];
        if (HAL_I2C_IsDeviceReady(&hi2c2, (uint16_t)(candidate << 1), 2,
                                  RETIMER_I2C_TIMEOUT_MS) == HAL_OK) {
            retimer_address = candidate;
            printf("DS160PT801 found at 7-bit address 0x%02X\n", candidate);
            return HAL_OK;
        }
    }

    printf("DS160PT801 not found on I2C2\n");
    return HAL_ERROR;
}

HAL_StatusTypeDef retimer_configure_x8(void)
{
    if (retimer_address == RETIMER_ADDRESS_INVALID && retimer_probe() != HAL_OK) {
        return HAL_ERROR;
    }

    if (retimer_enter_8bit_mode() != HAL_OK) {
        printf("DS160PT801 failed to enter verified 8-bit register mode\n");
        return HAL_ERROR;
    }

    printf("DS160PT801 init 1/4: replay public Pass-image register values\n");
    for (size_t index = 0;
         index < sizeof(retimer_public_x8_sequence) /
                     sizeof(retimer_public_x8_sequence[0]);
         ++index) {
        const retimer_register_t *setting = &retimer_public_x8_sequence[index];

        if (setting->reg == RETIMER_REG_LINK_WIDTH_0) {
            printf("DS160PT801 init 2/4: program x8 width before PCIe training\n");
        }
        if (setting->reg == RETIMER_REG_REFCLK_OUT) {
            printf("DS160PT801 init 3/4: enable REFCLK_OUT (0x0D=0x91)\n");
        }
        HAL_StatusTypeDef status = setting->verify
            ? retimer_write_verified(setting->reg, setting->value)
            : retimer_write(setting->reg, setting->value);
        if (status != HAL_OK) {
            printf("DS160PT801 command %02X %02X %02X %02X failed%s\n",
                   setting->source_prefix0, setting->source_prefix1,
                   setting->reg, setting->value,
                   setting->verify ? " write/readback" : " write");
            return HAL_ERROR;
        }
    }

    printf("DS160PT801 init 4/4: verify documented settings\n");
    if (retimer_verify_public_x8_sequence() != HAL_OK) {
        return HAL_ERROR;
    }

    printf("DS160PT801 x8 and REFCLK_OUT configuration verified at 0x%02X\n",
           retimer_address);
    return HAL_OK;
}

void retimer_print_status(void)
{
    uint8_t width0 = 0;
    uint8_t width1 = 0;
    uint8_t refclk = 0;

    if (retimer_address == RETIMER_ADDRESS_INVALID && retimer_probe() != HAL_OK) {
        return;
    }

    if (retimer_read(RETIMER_REG_LINK_WIDTH_0, &width0) == HAL_OK &&
        retimer_read(RETIMER_REG_LINK_WIDTH_1, &width1) == HAL_OK &&
        retimer_read(RETIMER_REG_REFCLK_OUT, &refclk) == HAL_OK) {
        printf("DS160PT801 0x%02X: F2=%02X F3=%02X REFCLK=%02X\n",
               retimer_address, width0, width1, refclk);
    }
}