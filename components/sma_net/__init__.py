import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import uart
from esphome.const import CONF_ID

CODEOWNERS = ["@robert-budde"]
DEPENDENCIES = ["uart", "sensor", "text_sensor"]
AUTO_LOAD = ["sensor", "text_sensor"]

CONF_UART_ID = "uart_id"
CONF_DEBUG = "debug"
CONF_SLOW_FACTOR = "slow_factor"

sma_net_ns = cg.esphome_ns.namespace("sma_net")
SmaNetComponent = sma_net_ns.class_(
    "SmaNetComponent", cg.PollingComponent, uart.UARTDevice
)

CONFIG_SCHEMA = (
    cv.Schema(
        {
            cv.GenerateID(): cv.declare_id(SmaNetComponent),
            cv.GenerateID(CONF_UART_ID): cv.use_id(uart.UARTComponent),
            cv.Optional(CONF_DEBUG, default=False): cv.boolean,
            cv.Optional(CONF_SLOW_FACTOR, default=2): cv.int_range(min=2),
        }
    )
    .extend(cv.polling_component_schema("10s"))
    .extend(uart.UART_DEVICE_SCHEMA)
)


async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)
    await uart.register_uart_device(var, config)
    cg.add(var.set_debug(config[CONF_DEBUG]))
    cg.add(var.set_slow_factor(config[CONF_SLOW_FACTOR]))
