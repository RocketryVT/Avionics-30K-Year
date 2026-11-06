#include <cstdio>

#include "hardware/uart.h"
#include "pico/stdio_usb.h"
#include "pico/stdlib.h"

#include "gps_parser.hpp"
#include "sentence_collector.hpp"

namespace
{
    constexpr unsigned GPS_BAUD = 115200;
    constexpr unsigned GPS_RX_PIN = 1;
}

int main()
{
    // Wait for the USB serial monitor before starting the test.
    stdio_init_all();
    while (!stdio_usb_connected())
        sleep_ms(10);

    // Receive GPS TX on GP1 (physical pin 2). No GPS RX connection is needed.
    uart_init(uart0, GPS_BAUD);
    uart_set_format(uart0, 8, 1, UART_PARITY_NONE);
    uart_set_hw_flow(uart0, false, false);
    gpio_set_function(GPS_RX_PIN, GPIO_FUNC_UART);

    SentenceCollector collector;

    std::printf("GPS parser test: UART0 RX on GP1, %u baud\n", GPS_BAUD);
    std::printf("Waiting for valid GGA fixes...\n");

    while (true)
    {
        while (uart_is_readable(uart0))
        {
            const char byte = uart_getc(uart0);
            const auto sentence = collector.feed(byte);

            if (!sentence)
                continue;

            // Consume the view before feeding another byte into its buffer.
            const auto position = parseGGA(*sentence);

            if (position)
            {
                std::printf("lat=%.8f lon=%.8f ellipsoid_alt=%.2f m\n",
                            position->latitude,
                            position->longitude,
                            position->altitude);
            }
        }

        tight_loop_contents();
    }
}
