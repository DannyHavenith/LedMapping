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

}

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


rgb leds[led_count];
int main()
{
    const rgb fades[] = {
            rgb(0,0,0),
            rgb(0,0,0),
            rgb(0,0,0),
            rgb(0,0,0),
            rgb(1,1,1),
            rgb(2,2,2),
            rgb(4,4,4),
            rgb(8,8,8),
    };

    DDRC = 255;
    clear( leds);
    bouncing_ball( leds, fades);
}
