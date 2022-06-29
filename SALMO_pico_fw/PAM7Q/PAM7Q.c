/* Includes */
#include "PAM7Q.h"

/** Init PAM7Q peripheral
 * @param uart_TX_pin
 * @param UART_RX_pin
 */
void PAM7Q_init(uint8_t txpin, uint8_t rxpin){
    // GPS UART initialization
    uart_init(GPS_UART_ID, BAUD_RATE);
    gpio_set_function(txpin, GPIO_FUNC_UART);
    gpio_set_function(rxpin, GPIO_FUNC_UART);
    uart_set_hw_flow(GPS_UART_ID, false, false);
    uart_set_format(GPS_UART_ID, DATA_BITS, STOP_BITS, PARITY);
    uart_set_fifo_enabled(GPS_UART_ID, false);
}
