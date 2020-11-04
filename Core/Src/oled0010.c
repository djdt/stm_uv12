#include "oled0010.h"

#include "stm32f0xx_hal.h"

static inline void oled_clock(oled0010_t* oled)
{
    // Clock time is 250 ns ? so this should be fine
    HAL_GPIO_WritePin(oled->e_port, oled->e_pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(oled->e_port, oled->e_pin, GPIO_PIN_RESET);
}

/* uint8_t oled_read_4bit(oled0010_t* oled) */
/* { */
/*     oled_clock(oled); */
/*     uint8_t result = 0x00; */
/*     result |= HAL_GPIO_ReadPin(oled->data_port, oled->data_pins[4]) << 4; */
/*     result |= HAL_GPIO_ReadPin(oled->data_port, oled->data_pins[5]) << 5; */
/*     result |= HAL_GPIO_ReadPin(oled->data_port, oled->data_pins[6]) << 6; */
/*     result |= HAL_GPIO_ReadPin(oled->data_port, oled->data_pins[7]) << 7; */
/*     return result; */
/* } */

uint8_t oled_read_bits(oled0010_t* oled, uint8_t bits)
{
    uint8_t result = 0x00;
    oled_clock(oled);
    for (uint8_t i = 8 - bits; i < 8; ++i) {
        result |= HAL_GPIO_ReadPin(oled->data_port, oled->data_pins[i]) << i;
    }
    return result;
}

uint8_t oled_read(oled0010_t* oled, GPIO_PinState rs_state)
{
    GPIO_InitTypeDef init = { 0 };
    init.Mode = GPIO_MODE_INPUT;
    // Set gpio as input
    if (oled->bits == OLED_FS_4BIT) {
        init.Pin = oled->data_pins[4] | oled->data_pins[5] | oled->data_pins[6] | oled->data_pins[7];
    } else {
        init.Pin = oled->data_pins[0] | oled->data_pins[1] | oled->data_pins[2] | oled->data_pins[3]
            | oled->data_pins[4] | oled->data_pins[5] | oled->data_pins[6] | oled->data_pins[7];
    }
    HAL_GPIO_Init(oled->data_port, &init);
    // Set to read desired register
    HAL_GPIO_WritePin(oled->rw_port, oled->rw_pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(oled->rs_port, oled->rs_pin, rs_state);

    uint8_t result;
    if (oled->bits == OLED_FS_4BIT) {
        result = oled_read_bits(oled, 4);
        result |= oled_read_bits(oled, 4) >> 4;
    } else {
        result = oled_read_bits(oled, 8);
    }
    // Return to ouput
    init.Mode = GPIO_MODE_OUTPUT_PP;
    HAL_GPIO_Init(oled->data_port, &init);

    return result;
}

/* void oled_send_4bit(oled0010_t* oled, uint8_t data) */
/* { */
/*     HAL_GPIO_WritePin(oled->data_port, oled->data_pins[4], data & 0x10); */
/*     HAL_GPIO_WritePin(oled->data_port, oled->data_pins[5], data & 0x20); */
/*     HAL_GPIO_WritePin(oled->data_port, oled->data_pins[6], data & 0x40); */
/*     HAL_GPIO_WritePin(oled->data_port, oled->data_pins[7], data & 0x80); */
/*     oled_clock(oled); */
/* } */

void oled_send_bits(oled0010_t* oled, uint8_t data, uint8_t bits)
{
    for (uint8_t i = 8 - bits; i < 8; ++i) {
        HAL_GPIO_WritePin(oled->data_port, oled->data_pins[i], data & (0x01 << i));
    }
    oled_clock(oled);
}

/* inline uint8_t oled_is_busy(oled0010_t* oled) */
/* { */
/*     return (oled_read(oled, GPIO_PIN_RESET) & 0x80) > 0; */
/* } */

void oled_send(oled0010_t* oled, uint8_t data, GPIO_PinState rs_state)
{
    // Bring RW to low to write and enable up
    HAL_GPIO_WritePin(oled->rw_port, oled->rw_pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(oled->rs_port, oled->rs_pin, rs_state);
    if (oled->bits == OLED_FS_4BIT) {
        // Send upper
        oled_send_bits(oled, data, 4);
        // Send lower
        oled_send_bits(oled, data << 4, 4);
    } else {
        oled_send_bits(oled, data, 8);
    }
    // Wait until instruction complete
    while ((oled_read(oled, GPIO_PIN_RESET) & 0x80) > 0) {
    }
}

void oled_init(oled0010_t* oled, uint8_t function_set, uint8_t display_control, uint8_t entry_mode)
{
    // Function set
    HAL_GPIO_WritePin(oled->rw_port, oled->rw_pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(oled->rs_port, oled->rs_pin, GPIO_PIN_RESET);

    if (!(function_set & OLED_FS_4BIT)) {
        oled->bits = OLED_FS_4BIT;
        oled_send_4bit(oled, OLED_FUNCTION_SET);
    } else {
        oled->bits = OLED_FS_8BIT;
    }
    oled_send(oled, OLED_FUNCTION_SET | function_set, GPIO_PIN_RESET);
    // Display ON
    oled_send(oled, OLED_DISPLAY_CONTROL | OLED_DC_DISPLAY_ON | display_control,
        GPIO_PIN_RESET);
    // Display Clear
    oled_send(oled, OLED_CLEAR_DISPLAY, GPIO_PIN_RESET);
    HAL_Delay(6);
    // Entry mode set
    oled_send(oled, OLED_ENTRY_MODE_SET | entry_mode, GPIO_PIN_RESET);
    // Return home
    oled_send(oled, OLED_RETURN_HOME, GPIO_PIN_RESET);
}

void oled_clear_display(oled0010_t* oled)
{
    oled_send(oled, OLED_CLEAR_DISPLAY, GPIO_PIN_RESET);
    HAL_Delay(6);
}

void oled_return_home(oled0010_t* oled)
{
    oled_send(oled, OLED_RETURN_HOME, GPIO_PIN_RESET);
}

void oled_move_cursor(oled0010_t* oled, uint8_t x, uint8_t y)
{
    uint8_t addr = x + 0x40 * y;
    oled_send(oled, OLED_SET_DDRAM | addr, GPIO_PIN_RESET);
}

void oled_shift_display(oled0010_t* oled, int8_t dir)
{
    uint8_t RL = OLED_SM_RIGHT;
    if (dir < 0) {
        dir *= -1;
        RL = OLED_SM_LEFT;
    }
    for (uint8_t i = 0; i < dir; ++i) {
        oled_send(oled, OLED_SHIFT_MODE_SET | RL, GPIO_PIN_RESET);
    }
}

void oled_add_character(oled0010_t* oled,
    uint8_t addr, uint8_t* char_data, uint8_t char_height)
{
    uint8_t addr = addr << (char_height == 8 ? 3 : 4);
    oled_send(oled, OLED_SET_CGRAM | addr, GPIO_PIN_RESET);

    for (uint8_t i = 0; i < height; ++i) {
        oled_send(oled, char_data[i], GPIO_PIN_SET);
    }
}

void oled_print_char(oled0010_t* oled, char c)
{
    oled_send(oled, c, GPIO_PIN_SET);
}

void oled_print(oled0010_t* oled, char* str)
{
    char* c = str;
    while (*c != '\0') {
        oled_send(oled, *c, GPIO_PIN_SET);
        c++;
    }
}
