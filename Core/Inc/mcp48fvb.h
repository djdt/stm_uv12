#ifndef _MCP48FVB_H_
#define _MCP48FVB_H_

#include "stm32f0xx_hal.h"

#define MCP_CMD_WRITE 0x00
#define MCP_CMD_READ 0x06

#define MCP_ADR_DAC0 0x00 << 2
#define MCP_ADR_DAC1 0x01 << 2
#define MCP_ADR_VREF 0x08 << 2
#define MCP_ADR_PWRD 0x09 << 2
#define MCP_ADR_GAIN 0x0a << 2

#define MCP_VREF_VDD 0x00
#define MCP_VREF_GAP 0x01
#define MCP_VREF_REF 0x02
#define MCP_VREF_BUF 0x03

#define MCP_PWRD_NORM 0x00
#define MCP_PWRD_1K 0x01
#define MCP_PWRD_100K 0x02
#define MCP_PWRD_OPEN 0x03

#define MCP_GAIN_1X 0x00
#define MCP_GAIN_2X 0x80
#define MCP_GAIN_POR 0x40

typedef struct {
    SPI_HandleTypeDef* spi;
    GPIO_TypeDef* cs_port;
    uint16_t cs_pin;
    GPIO_TypeDef* lat_port;
    uint16_t lat_pin;
} mcp48fvb_t;

void mcp_init(mcp48fvb_t* mcp, uint8_t vref, uint8_t power_down, uint8_t gain);

void mcp_set_dac(mcp48fvb_t* mcp, uint8_t value);

#endif /* ifndef _MCP48FVB_H_ */
