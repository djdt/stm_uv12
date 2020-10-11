#ifndef _OLED_0010_H_
#define _OLED_0010_H_

#include "stm32f0xx_hal.h"

/**********************************************************************
*                              COMMANDS                              *
**********************************************************************/

#define OLED_CLEAR_DISPLAY 0x01

#define OLED_RETURN_HOME 0x02

#define OLED_ENTRY_MODE_SET 0x04
#define OLED_EM_INC 0x02
#define OLED_EM_DEC 0x00
#define OLED_EM_DISP_SHIFT_ON 0x01
#define OLED_EM_DISP_SHIFT_OFF 0x00

#define OLED_DISPLAY_CONTROL 0x08
#define OLED_DC_BLINK_ON 0x01
#define OLED_DC_BLINK_OFF 0x00
#define OLED_DC_CURSOR_ON 0x02
#define OLED_DC_CURSOR_OFF 0x00
#define OLED_DC_DISPLAY_ON 0x04
#define OLED_DC_DISPLAY_OFF 0x00

#define OLED_SHIFT_MODE_SET 0x10
#define OLED_SM_DISP 0x08
#define OLED_SM_CURS 0x00
#define OLED_SM_RIGHT 0x04
#define OLED_SM_LEFT 0x00

#define OLED_GRAPHIC_MODE_SET 0x13
#define OLED_GM_GRAPHIC 0x08
#define OLED_GM_CHARACTER 0x00
#define OLED_GM_INT_PWR_ON 0x04
#define OLED_GM_INT_PWR_OFF 0x00

#define OLED_FUNCTION_SET 0x20
#define OLED_FS_8BIT 0x10
#define OLED_FS_4BIT 0x00
#define OLED_FS_2LINES 0x08
#define OLED_FS_1LINE 0x00
#define OLED_FS_FONT_LARGE 0x04
#define OLED_FS_FONT_SMALL 0x00
#define OLED_FS_FONT_ENG_JAP 0x00
#define OLED_FS_FONT_WEST_EUR1 0x01
#define OLED_FS_FONT_ENG_RUS 0x02
#define OLED_FS_FONT_WEST_EUR2 0x03

#define OLED_SET_CGRAM 0x40
#define OLED_SET_DDRAM 0x80

typedef struct {
    GPIO_TypeDef* rs_port;
    uint16_t rs_pin;
    GPIO_TypeDef* rw_port;
    uint16_t rw_pin;
    GPIO_TypeDef* e_port;
    uint16_t e_pin;
    GPIO_TypeDef* data_port;
    uint16_t* data_pins;
    uint8_t bits;
} oled0010_t;

void oled_init(oled0010_t* oled, uint8_t function_set, uint8_t display_control, uint8_t entry_mode);

void oled_clear_display(oled0010_t* oled);
void oled_return_home(oled0010_t* oled);
void oled_move_cursor(oled0010_t* oled, uint8_t x, uint8_t y);
void oled_shift_display(oled0010_t* oled, int8_t dir);

void oled_print(oled0010_t* oled, char* str);

#endif /* ifndef _OLED_0010_H_ */
