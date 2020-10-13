#include "mcp48fvb.h"

uint8_t mcp_send(mcp48fvb_t* mcp, uint8_t adr, uint8_t hi, uint8_t low)
{
    // LAT low for immediate update
    HAL_GPIO_WritePin(mcp->lat_port, mcp->lat_pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(mcp->cs_port, mcp->cs_pin, GPIO_PIN_RESET);

    uint8_t cmd[3] = { adr | MCP_CMD_WRITE, hi, low };
    uint8_t data[3];
    HAL_SPI_TransmitReceive(mcp->spi, cmd, data, 3, 1000);
    while (HAL_SPI_GetState(mcp->spi) != HAL_SPI_STATE_READY) {
    };

    HAL_GPIO_WritePin(mcp->cs_port, mcp->cs_pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(mcp->lat_port, mcp->lat_pin, GPIO_PIN_SET);
    return data[0] & 1;
}

uint8_t mcp_read(mcp48fvb_t* mcp, uint8_t adr, uint8_t* data)
{
    // Ensure latch is high
    HAL_GPIO_WritePin(mcp->lat_port, mcp->lat_pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(mcp->cs_port, mcp->cs_pin, GPIO_PIN_RESET);

    uint8_t cmd[3] = { adr | MCP_CMD_READ, 0x00, 0x00 };

    HAL_GPIO_WritePin(mcp->cs_port, mcp->cs_pin, GPIO_PIN_RESET);
    HAL_SPI_TransmitReceive(mcp->spi, cmd, data, 3, 1000);
    while (HAL_SPI_GetState(mcp->spi) != HAL_SPI_STATE_READY) {
    };

    HAL_GPIO_WritePin(mcp->cs_port, mcp->cs_pin, GPIO_PIN_SET);
    return data[0] & 1;
}

void mcp_init(mcp48fvb_t* mcp, uint8_t vref, uint8_t power_down, uint8_t gain)
{
    HAL_GPIO_WritePin(mcp->cs_port, mcp->cs_pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(mcp->lat_port, mcp->lat_pin, GPIO_PIN_SET);
    // Clear the POR bit
    uint8_t data[3];
    mcp_read(mcp, MCP_ADR_GAIN, data);
    // Avoid update if default values
    if (vref != 0x00) {
        mcp_send(mcp, MCP_ADR_VREF, 0x00, vref);
    }
    if (power_down != 0x00) {
        mcp_send(mcp, MCP_ADR_PWRD, 0x00, power_down);
    }
    if (gain != 0x00) {
        mcp_send(mcp, MCP_ADR_GAIN, gain, 0x00);
    }
}

void mcp_set_dac(mcp48fvb_t* mcp, uint8_t value)
{
    mcp_send(mcp, MCP_ADR_DAC0, 0x00, value);
}
