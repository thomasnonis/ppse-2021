#include "ssd1306.h"
#include "pico/stdlib.h"
#include "pico/binary_info.h"
#include "hardware/i2c.h"

#include "raspberry26x32.h"
#include "ssd1306_fonts.h"

//random gpio
#define I2C_SDA_PIN 6
#define I2C_SCL_PIN 7
#define I2C_PERIPHERAL (i2c1)

int main() {
    stdio_init_all();

    // useful information for picotool
    bi_decl(bi_2pins_with_func(I2C_SDA_PIN, I2C_SCL_PIN, GPIO_FUNC_I2C));
    bi_decl(bi_program_description("OLED I2C example for the Raspberry Pi Pico"));

    printf("Hello, OLED display! Look at my raspberries..\n");

    // I2C is "open drain", pull ups to keep signal high when no data is being
    // sent
    i2c_init(I2C_PERIPHERAL, 400 * 1000);
    gpio_set_function(I2C_SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(I2C_SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(I2C_SDA_PIN);
    gpio_pull_up(I2C_SCL_PIN);

    // run through the complete initialization process
    oled_init();

    // initialize render area for entire frame (128 pixels by 4 pages)
    struct render_area frame_area = {start_col: 0, end_col : OLED_WIDTH - 1, start_page : 0, end_page : OLED_NUM_PAGES -
                                                                                                        1};
    calc_render_area_buflen(&frame_area);

    // zero the entire display
    uint8_t buf[OLED_BUF_LEN];
    fill(buf, 0x00);
    render(buf, &frame_area);

    // intro sequence: flash the screen 3 times
    for (int i = 0; i < 3; i++) {
        oled_send_cmd(0xA5); // ignore RAM, all pixels on
        sleep_ms(500);
        oled_send_cmd(0xA4); // go back to following RAM
        sleep_ms(500);
    }

    // render 3 cute little raspberries
    struct render_area area = {start_col: 0, end_col : IMG_WIDTH - 1, start_page : 0, end_page : OLED_NUM_PAGES - 1};
    calc_render_area_buflen(&area);
    render(raspberry26x32, &area);
    for (int i = 1; i < 3; i++) {
        uint8_t offset = 5 + IMG_WIDTH; // 5px padding
        area.start_col += offset;
        area.end_col += offset;
        render(raspberry26x32, &area);
    }

    // configure horizontal scrolling
    oled_send_cmd(OLED_SET_HORIZ_SCROLL | 0x00);
    oled_send_cmd(0x00); // dummy byte
    oled_send_cmd(0x00); // start page 0
    oled_send_cmd(0x00); // time interval
    oled_send_cmd(0x03); // end page 3
    oled_send_cmd(0x00); // dummy byte
    oled_send_cmd(0xFF); // dummy byte

    // let's goooo!
    oled_send_cmd(OLED_SET_SCROLL | 0x01);

    //TODO:Test with ssd1306_fonts.h
    //Note: fonts are uint16_t, ssd driver need some japanese tuning!

    return 0;
}