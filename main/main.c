#include <stdio.h>
#include <unistd.h> // needed for sleep
#include "mRN2483.h"

#define TAG "Main"

typedef enum
{
    wakeup,
    idle,
    led_on,
    led_blink,
    low_bat,
    txrx,
    stolen,
    light_auto,
    dsleep,
    lsleep,
    UNKNOWN
} state_t;

/* Vars that should be stored persistenly over deep sleep should be stored in RTC memory: */
// Last TXRX
// Last Wakeuptime
// flags (stolen,...)
// initial boot flag

/* Configuration data that is rarely accessed should be stored in NVS*/
// dont know yet, but could be user (at runtime) configured WIFI credentials (FOR EXAMPLE)

/* Some Considerations:
Instead of continously running through the while loop, and repeatedly checking conditions in a state us the FreeRTOS tools like event bits to block the task.
This allows the FreeRTOS to allocate ticks to other tasks since the main task is not busy.
*/

void app_main(void)
{
    state_t state = init;
    bool stop = 0;

    while (!stop)
    {
        switch (state)
        {
        case wakeup:
            // get wake up reason
            // esp_sleep_get_wakeup_cause()
            // esp_sleep_get_ext1_wakeup_status()
            // if move,                                 --> goto: light_auto
            // if button,                               --> goto: light_on
            // if timer,                                --> goto: idle
            break;

        case light_auto:
            // measure light outside
            // if dark,                                 --> goto: led_on
            // else,                                    --> goto: idle
            break;

        case idle:
            // check when last transmission was made
            // if led is on, turn off
            // if last transm. older than T hours,      --> goto: TXRX
            // else,                                    --> goto: dsleep
            break;

        case led_on:
            // turn led_on
            // configure wakeup sources: button,
            // and timeout (and inactivity?)            --> goto: lsleep
            break;

        case lsleep:
            // measure battery level
            // if low_batt,                             --> goto: led_blink
            // enter light sleep (or potentially deep sleep)
            // after wakeup execution continues here (in l. sleep only)
            // get wake reason
            // if wake reason button or inactivity,     --> goto: idle
            // else,                                    --> goto: lsleep
            break;

        case led_blink:
            // start led flash task
            // block and wait for button_press or inactivity
            // then,                                    --> goto: idle
            break;
        case: txrx:
            // if !stolen, send battery
            // else, send battery + location
            // process RX
            // if RX contains new stolen,
            // send battery + location
            // reconfigure wakeups

            break;

        case dsleep:
            stop = 1;
            break;
        default:
            break;
        }
        usleep(10);
    }

    // enter deep sleep
}