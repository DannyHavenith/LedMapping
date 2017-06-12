//
//  Copyright (C) 2016 Danny Havenith
//
//  Distributed under the Boost Software License, Version 1.0. (See
//  accompanying file LICENSE_1_0.txt or copy at
//  http://www.boost.org/LICENSE_1_0.txt)
//
#include <avr_utilities/devices/uart.h>
#include <avr_utilities/pin_definitions.hpp>
#include <avr/io.h>
#include <util/delay.h>
#include <stdlib.h>
#include <avr/pgmspace.h>
#include <avr_utilities/esp-link/client.hpp>

// Define the port at which the signal will be sent. The port needs to
// be known at compilation time, the pin (0-7) can be chosen at run time.
#define WS2811_PORT PORTC

// send RGB in R,G,B order instead of the standard WS2812 G,R,B order.
// YOU TYPICALLY DO NOT WANT TO DEFINE THIS SYMBOL!
// It's just that one led string I encountered had R,G,B wired in the "correct" RGB order.
#define STRAIGHT_RGB

#include <ws2811/ws2811.h>

serial::uart<> uart( 19200);

IMPLEMENT_UART_INTERRUPT(uart);
PIN_TYPE( B, 0) movement_detector;

namespace {
    const uint8_t channel = 4;

    template<typename CoordinateType>
    struct Position {
//        Position( CoordinateType x, CoordinateType y)
//        :x(x), y(y)
//        {}

        CoordinateType x;
        CoordinateType y;
    };

    template<typename CoordinateType>
    struct Size
    {
//        Size( CoordinateType width, CoordinateType height)
//        : width( width), height( height)
//        {}

        CoordinateType width;
        CoordinateType height;
    };


    typedef Position<uint8_t>   Position8;
    typedef Position<uint16_t>  Position16; // position in 8.8 fixed point
    typedef Size<uint8_t>       Size8;
    typedef Size<uint16_t>      Size16;

    template<typename buffer, uint8_t shade_count = 4>
    class ball
    {
    public:
        ball( Position8 position, Size8 size)
        : m_position( position), m_size(size)
        {

        }

        void draw(
                buffer &leds,
                const Position8 (&pos)[ws2811::led_buffer_traits<buffer>::count],
                const ws2811::rgb (&shades)[shade_count])
        {
            // for each LED we know
            for (uint16_t count = 0; count < m_count; ++count)
            {
                // if the LED is within the bounding box of our shape.
                if ( absolute_difference( pos[count].x, m_position.x) < m_size.width
                     and absolute_difference( pos[count].y, m_position.y) < m_size.height)
                {
                    // calculate if it is within the ellipse.
                    uint16_t dist = square_distance( pos[count]);
                    if (dist < 256)
                    {
                        uint8_t index = (dist * shade_count) >> 8;
                        leds[count] = shades[index];
                    }
                }
            }
        }

    private:
        static const uint16_t m_count = ws2811::led_buffer_traits<buffer>::count;
        Position8   m_position;
        Size8       m_size;

        static uint16_t absolute_difference( uint8_t left, uint8_t right)
        {
            if (left > right) return left - right;
            else return right - left;
        }

        /**
         * Return a number >= 256 if the given point is outside the ellipse, but if the given point is inside the ellipse
         * return a number between 0 and 255 that indicates how close the point is to the center (0) or the edge (255) of the ellipse
         *
         */
        uint16_t square_distance( const Position8 &pos)
        {
            Size16 distance = {
                    absolute_difference( pos.x, m_position.x) << 8,
                    absolute_difference( pos.y, m_position.y) << 8
            };

            distance.width /= m_size.width;
            distance.height /= m_size.height;

            if (distance.width < 256 && distance.height < 256)
            {
                return ((distance.width * distance.width) >> 8) + ((distance.height * distance.height) >> 8);
            }
            else
            {
                return 256;
            }
        }

    };

    /**
     * These are hard-coded LED positions that were obtained by running the LedMapping OpenCV registry algorithm.
     */
    const Position8 pos[] = {
            { 2, 102},
            { 55, 95},
            { 73, 80},
            { 121, 73},
            { 94, 56},
            { 40, 56},
            { 0, 45},
            { 41, 34},
            { 19, 17},
            { 50, 3},
            { 109, 2},
            { 171, 0},
            { 205, 14},
            { 174, 30},
            { 223, 39},
            { 239, 56},
            { 212, 69},
            { 178, 82},
            { 211, 93},
            { 186, 107},
            { 239, 114},
            { 246, 132},
            { 197, 144},
            { 145, 137},
            { 150, 119},
            { 114, 107},
            { 63, 118},
            { 36, 134},
            { 95, 141},
            { 103, 158},
            { 41, 156},
            { 18, 172},
            { 75, 175},
            { 129, 182},
            { 171, 168},
            { 224, 159},
            { 246, 176},
            { 255, 196},
            { 212, 209},
            { 151, 204},
            { 89, 202},
            { 32, 211},
            { 86, 222},
            { 149, 226},
            { 214, 227},
            { 230, 240},
            { 171, 247},
            { 110, 255},
            { 77, 241},
            { 18, 241}
    };


    const uint8_t led_count = sizeof pos/ sizeof pos[0];



using ws2811::rgb;

void animate(Position8& p1, Size8 s, Position8& v1)
{
    p1.x += v1.x;
    p1.y += v1.y;
    if (p1.y < s.height / 2 || p1.y > 255 - s.height / 2)
    {
        v1.y = -v1.y;
    }
    if (p1.x < s.width / 2 || p1.x > 255 - s.width / 2)
    {
        v1.x = -v1.x;
    }
}

template< typename buffer_type, int shade_count>
void bouncing_ball( buffer_type &buffer, const rgb (&fades)[shade_count])
{
    Position8 p1 = {128,128};
    Position8 v1 = { 3, 2};
    Size8 s = {120, 36};
    for(;;)
    {
        ball<buffer_type, shade_count> b1( p1, s);


        fill( buffer, rgb(10, 10, 10));
        b1.draw( buffer, pos, fades);

        send( buffer, channel);
        _delay_ms( 5);

        animate( p1, s, v1);
    }
}

const uint8_t distances2[] =
{
        46,
        66,
        96,
        120,
        141,
        133,
        152,
        175,
        205,
        233,
        239,
        255,
        237,
        205,
        205,
        185,
        158,
        128,
        131,
        108,
        133,
        134,
        112,
        81,
        83,
        72,
        38,
        24,
        58,
        81,
        58,
        85,
        97,
        124,
        121,
        136,
        162,
        189,
        192,
        166,
        148,
        157,
        182,
        201,
        220,
        245,
        242,
        245,
        217,
        213
};

// distances from the center point.
const uint8_t distances[] = {
        88,
        76,
        99,
        107,
        141,
        149,
        179,
        192,
        228,
        251,
        248,
        255,
        228,
        196,
        185,
        155,
        124,
        94,
        83,
        52,
        70,
        69,
        52,
        22,
        20,
        39,
        41,
        55,
        33,
        63,
        76,
        110,
        99,
        109,
        86,
        85,
        120,
        156,
        171,
        155,
        152,
        176,
        190,
        197,
        206,
        233,
        240,
        254,
        230,
        236
};

const uint8_t PROGMEM gamma8[] = {
    0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,
    0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  1,  1,  1,  1,
    1,  1,  1,  1,  1,  1,  1,  1,  1,  2,  2,  2,  2,  2,  2,  2,
    2,  3,  3,  3,  3,  3,  3,  3,  4,  4,  4,  4,  4,  5,  5,  5,
    5,  6,  6,  6,  6,  7,  7,  7,  7,  8,  8,  8,  9,  9,  9, 10,
   10, 10, 11, 11, 11, 12, 12, 13, 13, 13, 14, 14, 15, 15, 16, 16,
   17, 17, 18, 18, 19, 19, 20, 20, 21, 21, 22, 22, 23, 24, 24, 25,
   25, 26, 27, 27, 28, 29, 29, 30, 31, 32, 32, 33, 34, 35, 35, 36,
   37, 38, 39, 39, 40, 41, 42, 43, 44, 45, 46, 47, 48, 49, 50, 50,
   51, 52, 54, 55, 56, 57, 58, 59, 60, 61, 62, 63, 64, 66, 67, 68,
   69, 70, 72, 73, 74, 75, 77, 78, 79, 81, 82, 83, 85, 86, 87, 89,
   90, 92, 93, 95, 96, 98, 99,101,102,104,105,107,109,110,112,114,
  115,117,119,120,122,124,126,127,129,131,133,135,137,138,140,142,
  144,146,148,150,152,154,156,158,160,162,164,167,169,171,173,175,
  177,180,182,184,186,189,191,193,196,198,200,203,205,208,210,213,
  215,218,220,223,225,228,231,233,236,239,241,244,247,249,252,255 };

const uint8_t PROGMEM sin8[] = {
 127, 133, 140, 146, 152, 158, 164, 170, 176, 182, 187, 193, 198, 203, 208, 213,
 218, 222, 226, 230, 233, 237, 240, 243, 245, 248, 249, 251, 253, 254, 254, 255,
 255, 255, 254, 254, 253, 251, 249, 248, 245, 243, 240, 237, 233, 230, 226, 222,
 218, 213, 208, 203, 198, 193, 187, 182, 176, 170, 164, 158, 152, 146, 140, 133,
 127, 121, 114, 108, 102,  96,  90,  84,  78,  72,  67,  61,  56,  51,  46,  41,
  36,  32,  28,  24,  21,  17,  14,  11,   9,   6,   5,   3,   1,   0,   0,   0,
   0,   0,   0,   0,   1,   3,   5,   6,   9,  11,  14,  17,  21,  24,  28,  32,
  36,  41,  46,  51,  56,  61,  67,  72,  78,  84,  90,  96, 102, 108, 114, 121
};


uint8_t scale( int8_t lhs, uint8_t rhs)
{
    return (static_cast<uint16_t>(lhs + 128) * rhs) >> 8;
}

/**
 * Send LED data to an LED string while switching off interrupts.
 */
template< typename buffer>
void send_protected( const buffer &b, uint8_t channel)
{
    cli();
    send( b, channel);
    sei();
}

template< typename buffer_type, uint16_t shade_count>
void ripples( buffer_type &buffer, const rgb (&fades)[shade_count])
{
    PIN_TYPE( B, 0) detector;
    set( detector);
    make_input( detector);

    constexpr auto led_count = ws2811::led_buffer_traits<buffer_type>::count;
    uint16_t offset = 0;
    const uint8_t b =  pgm_read_byte(&gamma8[128]);
    const auto ambient_color = rgb{b,b,b};
//    const auto ambient_color = rgb{0,0,0};
    for(;;)
    {
        fill( buffer, ambient_color);
        send(buffer, channel);
        while (not is_set( detector))
        {
        }

        for (uint16_t time = 2000; time; --time)
        {
            for (uint16_t count = 0; count < led_count; ++count)
            {
                get( buffer, count) = fades[ (static_cast<uint16_t>(distances2[count]) - offset) % shade_count];
            }
            send( buffer, channel);
            ++offset;
            if (offset == shade_count) offset = 0;
            _delay_ms( 4);
        }
    }
}

template< typename buffer_type>
void fade( buffer_type &leds, bool in = true)
{
    constexpr auto led_count = ws2811::led_buffer_traits<buffer_type>::count;
    const auto base_color = in?rgb( 0,0,0):rgb( 255, 255, 255);

    for (uint16_t count = 0; count < 512; ++count)
    {
        fill( leds, base_color);
        for ( uint8_t led = 0; led < led_count; ++led)
        {
            const uint8_t distance = distances[led];
            if (distance <= count)
            {
                uint16_t offset = count-distance;
                if (offset > 255) offset = 255;
                if (not in)
                {
                    offset = 255 - offset;
                }
                const uint8_t br = pgm_read_byte( &gamma8[offset]);
                get( leds, led) = rgb( br, br, br);
            }
        }
        send_protected( leds, channel);
        _delay_ms( 2);
    }
}

rgb leds[led_count];
void wait_for_non_movement()
{
    constexpr uint16_t timeout = 3000;
    uint16_t count_down = timeout;
    while ( count_down--)
    {
        if (is_set(movement_detector))
        {
            count_down = timeout;
        }
        _delay_ms( 10);
    }
}

void wait_for_movement()
{
    while (!is_set( movement_detector))
    {
    }
}

void watch()
{
    set( movement_detector);
    make_input( movement_detector);
    esp_link::client esp{uart};

    using esp_link::mqtt::setup;
    using esp_link::mqtt::publish;
    const char topic[] = "spider/switch/0";

    fill( leds, rgb( 0, 5, 5));
    send( leds, channel);

    // get startup logging of the uart out of the way.
    _delay_ms( 2000);     // wait for an eternity.
    while (not esp.sync()) /*repeat*/;

    esp.execute( setup, nullptr, nullptr, nullptr, nullptr);


    for (;;)
    {
        clear( leds);
        send_protected( leds, channel);
        esp.execute( publish, topic, "0", 0, 0);
        wait_for_movement();
        fade( leds, true); // fade in
        esp.execute( publish, topic, "1", 0, 0);

        wait_for_non_movement();
        fade( leds, false); // fade out
    }
}

}

int main()
{
//    rgb fades[128];
//    for (uint8_t count = 0; count < 128; ++count)
//    {
//        uint8_t b =  pgm_read_byte(&sin8[count])/4;
//        fades[count] = rgb(b,b,b);
//    }

    DDRC = 255;
    clear( leds);
    watch();
    //ripples( leds, fades);
}
