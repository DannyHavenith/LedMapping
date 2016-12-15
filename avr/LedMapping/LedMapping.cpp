//
//  Copyright (C) 2016 Danny Havenith
//
//  Distributed under the Boost Software License, Version 1.0. (See
//  accompanying file LICENSE_1_0.txt or copy at
//  http://www.boost.org/LICENSE_1_0.txt)
//
#include <avr/io.h>
#include <util/delay.h>
#include <stdlib.h>

// Define the port at which the signal will be sent. The port needs to
// be known at compilation time, the pin (0-7) can be chosen at run time.
#define WS2811_PORT PORTC

// send RGB in R,G,B order instead of the standard WS2812 G,R,B order.
// YOU TYPICALLY DO NOT WANT TO DEFINE THIS SYMBOL!
// It's just that one led string I encountered had R,G,B wired in the "correct" RGB order.
#define STRAIGHT_RGB

#include "ws2811/ws2811.h"

namespace {
    const uint8_t led_count = 50;
    const uint8_t channel = 4;

    /**
     * Not the fastest way to calculate this, but for about 8 iterations at compile time, this will do.
     *
     * This is used to find the lowest power of two that is equal or larger than the number of LEDs at
     * compilation time.
     */
    template<uint8_t number, uint8_t guess = 1, bool fits = guess >= number>
    struct lowest_power_of_2
    {
        static const uint8_t value = lowest_power_of_2<number, 2 * guess>::value;
    };

    template< uint8_t number, uint8_t guess>
    struct lowest_power_of_2<number, guess, true>
    {
        static const uint8_t value = guess;
    };

    template< typename buffer_type>
    void write_block( buffer_type &leds, uint8_t &offset, uint8_t end_offset, uint8_t size, const ws2811::rgb &color)
    {
        while (size-- && offset != end_offset)
        {
            get( leds, offset++) = color;
        }
    }

    template< typename buffer_type>
    void fill( buffer_type &leds, const ws2811::rgb &color)
    {
        for (uint16_t count = 0;
                count < ws2811::led_buffer_traits<buffer_type>::count;
                ++count)
        {
            get( leds, count) = color;
        }
    }

    /**
     * Send a sequence to an LED string that can be used to identify individual LEDs.
     * This can be used to identify 2^n LEDs in n steps.
     *
     * The sequence consists of steps where first all leds are switched off, followed by a period in
     * which every LED in the string will be lit and be either blue or red.
     * In the first step, the 1st half of the LEDs will be red;
     * In the second step the 1st and 3rd quarter of LEDs will be red;
     * In the third step the 1st, 3rd, 5th and 7th eight will be red, etc., etc.
     *
     *
     */
    template< typename buffer_type>
    void binary_pattern( buffer_type &leds, uint8_t channel)
    {
        static const uint8_t frame_delay = 100; // in ms;
        static const uint8_t number_of_leds = ws2811::led_buffer_traits<buffer_type>::count;
        uint8_t block_size = lowest_power_of_2<number_of_leds>::value/2;
        using ws2811::rgb;

        while (block_size)
        {
            // write binary pattern.
            uint8_t current_led = 0;

            while (current_led < number_of_leds)
            {
                write_block( leds, current_led, number_of_leds, block_size, rgb(16, 0, 0));
                write_block( leds, current_led, number_of_leds, block_size, rgb(0, 0, 16));
            }
            send( leds, channel);
            _delay_ms( frame_delay);

            current_led = 0;
            write_block( leds, current_led, number_of_leds, number_of_leds, rgb(0,0,0));
            send( leds, channel);
            _delay_ms( frame_delay);

            block_size /= 2;
        }
    }

    template< typename buffer_type>
    void registration_pattern( buffer_type &leds, uint8_t channel)
    {
        static const uint8_t frame_delay_ms = 2000; // in ms;
        static const uint8_t number_of_leds = ws2811::led_buffer_traits<buffer_type>::count;
        using ws2811::rgb;

        uint8_t current_led = 0;
        for (uint8_t count = 4; count; --count)
        {
            current_led = 0;
            write_block( leds, current_led, number_of_leds, number_of_leds, rgb( 16, 0, 0));
            send( leds, channel);
            _delay_ms( frame_delay_ms);

            current_led = 0;
            write_block( leds, current_led, number_of_leds, number_of_leds, rgb( 0, 0, 16));
            send( leds, channel);
            _delay_ms( frame_delay_ms);
        }
        current_led = 0;
        clear( leds);
        send( leds, channel);
    }

    /**
     * Simply flash the LEDs one by one.
     */
    template< typename buffer_type>
    void simple_registration( buffer_type &leds, uint8_t channel, const ws2811::rgb &color)
    {

        static const uint8_t number_of_leds = ws2811::led_buffer_traits<buffer_type>::count;
        static const uint8_t frame_delay_ms = 100; // in ms;

        fill( leds, color);
        send( leds, channel);
        _delay_ms( 2*frame_delay_ms);
        clear( leds);
        send( leds, channel);
        _delay_ms( 2* frame_delay_ms);

        for (uint8_t count = 0; count < number_of_leds; ++count)
        {

            clear( leds);
            get( leds, count) = color;
            send( leds, channel);
            _delay_ms( frame_delay_ms);

            clear( leds);
            send( leds, channel);
            _delay_ms( frame_delay_ms);
        }
        clear( leds);
        send( leds, channel);
    }
}

ws2811::rgb leds[led_count];
int main()
{

    DDRC = 255;
    clear( leds);
    for(;;)
    {
        simple_registration( leds, channel, ws2811::rgb( 16, 0, 0));
        _delay_ms( 2000);
    }

}

