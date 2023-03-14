#include "drv_spi.h"
#include "drv_gpio.h"

static int w5500_init(void)
{
	return rt_hw_spi_device_attach("spi0", "spi00", GET_PIN(B, 3));	//W5500
}
INIT_COMPONENT_EXPORT(w5500_init);
