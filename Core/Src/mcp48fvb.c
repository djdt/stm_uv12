#include "mcp48fvb.h"

void mcp_send(mcp48fvb_t* mcp, uint8_t adr, uint8_t data)
{
    HAL_GPIO_WritePin(mcp->cs_port, mcp->cs_pin, GPIO_PIN_RESET);
    uint8_t cmd[3] = { adr | MCP_CMD_WRITE, 0x00, data };
    HAL_SPI_Transmit(mcp->spi, cmd, 3, 100);
    while (HAL_SPI_GetState(mcp->spi) != HAL_SPI_STATE_READY) {
    };
    HAL_GPIO_WritePin(mcp->cs_port, mcp->cs_pin, GPIO_PIN_SET);
    // Toggle latch
    HAL_GPIO_WritePin(mcp->lat_port, mcp->lat_pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(mcp->lat_port, mcp->lat_pin, GPIO_PIN_SET);
}

uint8_t mcp_read(mcp48fvb_t* mcp, uint8_t adr)
{
    uint8_t cmd[3] = { adr | MCP_CMD_READ, 0x00, 0x00 };
    uint8_t data[3] = { 0x00 };
    // Ensure latch is high
    HAL_GPIO_WritePin(mcp->lat_port, mcp->lat_pin, GPIO_PIN_RESET);

    HAL_GPIO_WritePin(mcp->cs_port, mcp->cs_pin, GPIO_PIN_RESET);
    HAL_SPI_TransmitReceive(mcp->spi, cmd, data, 3, 100);
    while (HAL_SPI_GetState(mcp->spi) != HAL_SPI_STATE_READY) {
    };
    HAL_GPIO_WritePin(mcp->cs_port, mcp->cs_pin, GPIO_PIN_SET);
    return data[2];
}

void mcp_init(mcp48fvb_t* mcp, uint8_t vref, uint8_t power_down, uint8_t gain)
{
    HAL_GPIO_WritePin(mcp->cs_port, mcp->cs_pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(mcp->lat_port, mcp->lat_pin, GPIO_PIN_RESET);
    // Avoid update if default values
    if (vref != 0x00) {
        mcp_send(mcp, MCP_ADR_VREF, vref);
    }
    if (power_down != 0x00) {
        mcp_send(mcp, MCP_ADR_PWRD, power_down);
    }
    if (gain != 0x80) {
        mcp_send(mcp, MCP_ADR_GAIN, gain);
    }
}

void mcp_set_dac(mcp48fvb_t* mcp, uint8_t value)
{
    mcp_send(mcp, MCP_ADR_DAC0, value);
}
