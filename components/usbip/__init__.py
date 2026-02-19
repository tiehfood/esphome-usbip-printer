import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.const import CONF_ID, CONF_PORT
from pathlib import Path

CODEOWNERS = ["@custom"]
DEPENDENCIES = []
AUTO_LOAD = []

CONF_PRINTER_FIRMWARE = "printer_firmware"

usbip_ns = cg.esphome_ns.namespace("usbip")
USBIPComponent = usbip_ns.class_("USBIPComponent", cg.Component)

CONFIG_SCHEMA = cv.Schema(
    {
        cv.GenerateID(): cv.declare_id(USBIPComponent),
        cv.Optional(CONF_PORT, default=3240): cv.port,
        cv.Optional(CONF_PRINTER_FIRMWARE): cv.file_,
    }
).extend(cv.COMPONENT_SCHEMA)


async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)

    cg.add(var.set_port(config[CONF_PORT]))

    # Add ESP-IDF USB Host component
    cg.add_build_flag("-DCONFIG_USB_OTG_SUPPORTED=1")

    if CONF_PRINTER_FIRMWARE in config:
        firmware_path = Path(config[CONF_PRINTER_FIRMWARE])
        if not firmware_path.is_absolute():
            # Resolve relative to the config file directory
            firmware_path = Path(cg.relative_config_path(".")) / firmware_path
        firmware_data = firmware_path.read_bytes()
        size = len(firmware_data)

        # Convert to hex string
        hex_values = ", ".join(f"0x{b:02X}" for b in firmware_data)

        cg.add_build_flag("-DHAS_PRINTER_FIRMWARE")
        cg.add_global(cg.RawStatement(""))
        cg.add_global(
            cg.RawStatement(
                f"namespace esphome {{\n"
                f"namespace usbip {{\n"
                f"extern const uint8_t printer_fw_data[] = {{{hex_values}}};\n"
                f"extern const size_t printer_fw_data_size = {size};\n"
                f"}}  // namespace usbip\n"
                f"}}  // namespace esphome"
            )
        )
