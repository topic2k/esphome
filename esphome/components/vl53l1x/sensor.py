import esphome.codegen as cg
import esphome.config_validation as cv
from esphome import pins
from esphome.components import i2c, sensor
from esphome.const import (
    CONF_ADDRESS,
    CONF_ENABLE_PIN,
    CONF_HIGH,
    CONF_ID,
    CONF_MODE,
    CONF_OFFSET,
    CONF_DISTANCE,
    CONF_LOW,
    CONF_THRESHOLD,
    ICON_ARROW_EXPAND_VERTICAL,
    STATE_CLASS_MEASUREMENT,
    STATE_CLASS_NONE,
)

DEPENDENCIES = ["i2c"]

VL53L1X_I2C_ADDR = 0x29
DEFAULT_CONF_IO_2V8 = False
DEFAULT_POLLING_TIME = "60s"
DEFAULT_CONF_LONG_RANGE = True
DEFAULT_CONF_TIMING_BUDGET = 100
DEFAULT_CONF_OFFSET = 0

CONF_IO_2V8 = "io_2v8"
CONF_IRQ_PIN = "irq_pin"
CONF_LONG_RANGE = "long_range"
CONF_TIMING_BUDGET = "timing_budget"

UNIT_MILLIMETER = "mm"


def check_keys(obj):
    if obj[CONF_ADDRESS] != VL53L1X_I2C_ADDR and CONF_ENABLE_PIN not in obj:
        msg = f"Address other then {hex(VL53L1X_I2C_ADDR)} requires enable_pin definition to allow sensor\r"
        msg += "re-addressing. Also if you have more then one VL53 device on the same\r"
        msg += "i2c bus, then all VL53 devices must have enable_pin defined."
        raise cv.Invalid(msg)

    if obj[CONF_TIMING_BUDGET] not in (15, 20, 33, 50, 100, 200, 500):
        msg = f"'{CONF_TIMING_BUDGET}' must be one of:\r"
        msg += "  15, 20, 33, 50, 100, 200 or 500 (ms)!\r"
        raise cv.Invalid(msg)

    if CONF_THRESHOLD in obj:
        if obj[CONF_THRESHOLD][CONF_MODE] not in (0, 1, 2, 3):
            msg = f"'{CONF_MODE}' must be one of 0, 1, 2 or 3.\r"
            msg += f"  0 = below the '{CONF_LOW}' value.\r"
            msg += f"  1 = above the '{CONF_HIGH}' value.\r"
            msg += f"  2 = outside (below the '{CONF_LOW}' value OR above the '{CONF_HIGH}' value).\r"
            msg += f"  3 = inside (above the '{CONF_LOW}' value AND below the '{CONF_HIGH}' value).\r"
            raise cv.Invalid(msg)
    return obj


vl53l1x_ns = cg.esphome_ns.namespace("vl53l1x")
VL53L1XComponent = vl53l1x_ns.class_(
    "VL53L1XComponent", cg.PollingComponent, i2c.I2CDevice
)

CONFIG_SCHEMA = cv.All(
    cv.Schema(
        {
            cv.GenerateID(): cv.declare_id(VL53L1XComponent),
            cv.Optional(CONF_DISTANCE): sensor.sensor_schema(
                unit_of_measurement=UNIT_MILLIMETER,
                icon=ICON_ARROW_EXPAND_VERTICAL,
                accuracy_decimals=0,
                state_class=STATE_CLASS_MEASUREMENT,
            ),
            cv.Optional(CONF_THRESHOLD): sensor.sensor_schema(
                icon="mdi:selection-ellipse-arrow-inside",
                accuracy_decimals=0,
                state_class=STATE_CLASS_NONE,
            ).extend(
                {
                    cv.Required(CONF_LOW): cv.int_range(0, 4000),
                    cv.Required(CONF_HIGH): cv.int_range(0, 4000),
                    cv.Required(CONF_MODE): cv.int_,
                }
            ),
        }
    )
    .extend(
        {
            cv.Optional(CONF_ENABLE_PIN): pins.gpio_output_pin_schema,
            cv.Optional(CONF_IRQ_PIN): pins.gpio_output_pin_schema,
            cv.Optional(CONF_IO_2V8, default=DEFAULT_CONF_IO_2V8): cv.boolean,
            cv.Optional(CONF_LONG_RANGE, default=DEFAULT_CONF_LONG_RANGE): cv.boolean,
            cv.Optional(
                CONF_TIMING_BUDGET, default=DEFAULT_CONF_TIMING_BUDGET
            ): cv.uint16_t,
            cv.Optional(CONF_OFFSET, default=DEFAULT_CONF_OFFSET): cv.int_,
        }
    )
    .extend(cv.polling_component_schema(DEFAULT_POLLING_TIME))
    .extend(i2c.i2c_device_schema(VL53L1X_I2C_ADDR)),
    check_keys,
)


async def to_code(config):
    component = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(component, config)
    await i2c.register_i2c_device(component, config)

    if CONF_ENABLE_PIN in config:
        enable = await cg.gpio_pin_expression(config[CONF_ENABLE_PIN])
        cg.add(component.set_enable_pin(enable))

    if CONF_IRQ_PIN in config:
        irq = await cg.gpio_pin_expression(config[CONF_IRQ_PIN])
        cg.add(component.set_irg_pin(irq))

    cg.add(component.set_io_2v8(config[CONF_IO_2V8]))
    cg.add(component.set_long_range(config[CONF_LONG_RANGE]))
    cg.add(component.set_timing_budget(config[CONF_TIMING_BUDGET]))
    cg.add(component.set_offset(config[CONF_OFFSET]))

    if CONF_DISTANCE in config:
        sens_dist = await sensor.new_sensor(config[CONF_DISTANCE])
        cg.add(component.set_distance_sensor(sens_dist))

    if CONF_THRESHOLD in config:
        sens_thresh = await sensor.new_sensor(config[CONF_THRESHOLD])
        cg.add(component.set_threshold_sensor(sens_thresh))
        cg.add(
            component.set_threshold(
                config[CONF_THRESHOLD][CONF_LOW],
                config[CONF_THRESHOLD][CONF_HIGH],
                config[CONF_THRESHOLD][CONF_MODE],
            )
        )
