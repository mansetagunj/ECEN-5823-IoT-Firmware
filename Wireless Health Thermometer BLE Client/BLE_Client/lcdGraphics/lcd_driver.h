#ifndef LCD_DRIVER_H_
#define LCD_DRIVER_H_

#include "hal-config.h"

#if (HAL_SPIDISPLAY_ENABLE == 1)

#include "bg_types.h"

/**
 *  LCD content can be updated one row at a time using function LCD_write().
 *  Row number is passed as parameter,the possible values are defined below.
 */
#define LCD_ROW_NAME         1    /* 1st row, device name */
#define LCD_ROW_BTADDR1      2    /* 2nd row, bt addr 0-2 bytes */
#define LCD_ROW_BTADDR2      3    /* 3nd row, bt addr 3-5 bytes */
#define LCD_ROW_CONNECTION   4    /* 4rd row, connection status */
#define LCD_ROW_PASSKEY   	 5    /* 5th row, passkey */
#define LCD_ROW_ACTION   	 6    /* 6th row, passkey confirm action */
#define LCD_ROW_TEMPVALUE    7    /* 7th row, temp in C */
#define LCD_ROW_MAX          7    /* total number of rows used */

#define LCD_ROW_LEN        32   /* up to 32 characters per each row */

void LCD_init(char *header);
void LCD_write(char *str, uint8 row);

#endif /* HAL_SPIDISPLAY_ENABLE */

#endif /* LCD_DRIVER_H_ */
