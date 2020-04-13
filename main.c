/*
 * Project: h9ps (power supply)
 *
 * Created: 2020-03-28
 * Author : SQ8KFH
 */ 
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>

#include "can/can.h"

#define STATUS_LED PD7
#define STATUS_LED_DDR DDRD
#define STATUS_LED_PORT PORTD

#define PWR_LED PB6
#define PWR_LED_DDR DDRB
#define PWR_LED_PORT PORTB

#define PS_ON PB1
#define PS_ON_DDR DDRB
#define PS_ON_PORT PORTB

#define PWR_OUT PB5
#define PWR_OUT_DDR DDRB
#define PWR_OUT_PORT PORTB

#define BUS_PWR_ON PD6
#define BUS_PWR_ON_DDR DDRD
#define BUS_PWR_ON_PORT PORTD

#define BUTTON PC0
#define BUTTON_DDR DDRC
#define BUTTON_PORT PORTC
#define BUTTON_PIN PINC

#define PG PD5
#define PG_DDR DDRD
#define PG_PORT PORTD
#define PG_PIN PIND

#define BUS_I PC4
#define BUS_I_DDR DDRC
#define BUS_I_ADMUX ((1 << MUX3))

#define PWR_I PC5
#define PWR_I_DDR DDRC
#define PWR_I_ADMUX ((1 << MUX3)|(1 << MUX0))

//reg
volatile uint8_t out_power = 0;
volatile uint8_t bus_power = 0;


int main(void) {
    DDRB = 0xff;
    DDRC = 0xff;
    DDRD = 0xff;
    DDRE = 0xff;

    //STATUS_LED_DDR |= (1<<STATUS_LED);

    PS_ON_PORT |= (1 << PS_ON);

    PWR_OUT_PORT &= ~(1 << PWR_OUT);
    BUS_PWR_ON_PORT &= ~(1 << BUS_PWR_ON);

    CAN_init();
    sei();

    _delay_ms(100);
    CAN_send_turned_on_broadcast();

    BUTTON_DDR &= ~(1 << BUTTON);
    BUTTON_PORT &= ~(1 << BUTTON);

    PWR_I_DDR &= ~(1 << PWR_I);
    BUS_I_DDR &= ~(1 << BUS_I);

    PG_DDR &= ~(1 << PG);
    PG_PORT |= (1 << PG);

    uint32_t led_counter = 0x1000;
    uint32_t button_counter = 0;

    ADCSRA = (1<<ADEN) | (1<<ADPS0) /*| (1<<ADPS1)*/ | (1<<ADPS2);
    ADCSRB = (1 << AREFEN);

    while (1) {
        if (led_counter == 0) {
            STATUS_LED_PORT ^= (1<<STATUS_LED);
            if (STATUS_LED_PORT & (1<<STATUS_LED)) {
                led_counter = 0x1000;
            }
            else {
                led_counter = 0x80000;
            }
        }
        --led_counter;

        if (!(BUTTON_PIN & (1 << BUTTON))) {
            ++button_counter;
            if (button_counter > 0xfff0) button_counter = 0xfff0;
        }
        else {
            if (button_counter > 1000) {
                if (out_power) {
                    out_power = 0;
                    h9msg_t cm;
                    CAN_init_new_msg(&cm);

                    cm.type = H9MSG_TYPE_REG_INTERNALLY_CHANGED;
                    cm.destination_id = H9MSG_BROADCAST_ID;
                    cm.dlc = 2;
                    cm.data[0]= 10;
                    cm.data[1] = out_power;
                    CAN_put_msg(&cm);

                    if (bus_power) {
                        bus_power = 0;
                        h9msg_t cm;
                        CAN_init_new_msg(&cm);

                        cm.type = H9MSG_TYPE_REG_INTERNALLY_CHANGED;
                        cm.destination_id = H9MSG_BROADCAST_ID;
                        cm.dlc = 2;
                        cm.data[0] = 11;
                        cm.data[1] = bus_power;
                        CAN_put_msg(&cm);
                    }
                }
                else {
                    if (out_power == 0) {
                        out_power = 1;
                        h9msg_t cm;
                        CAN_init_new_msg(&cm);

                        cm.type = H9MSG_TYPE_REG_INTERNALLY_CHANGED;
                        cm.destination_id = H9MSG_BROADCAST_ID;
                        cm.dlc = 2;
                        cm.data[0] = 10;
                        cm.data[1] = out_power;
                        CAN_put_msg(&cm);
                    }
                    if (bus_power == 0) {
                        bus_power = 1;
                        h9msg_t cm;
                        CAN_init_new_msg(&cm);

                        cm.type = H9MSG_TYPE_REG_INTERNALLY_CHANGED;
                        cm.destination_id = H9MSG_BROADCAST_ID;
                        cm.dlc = 2;
                        cm.data[0] = 11;
                        cm.data[1] = bus_power;
                        CAN_put_msg(&cm);
                    }
                }
            }
            button_counter = 0;
        }

        if (out_power || bus_power) {
            PS_ON_PORT &= ~(1 << PS_ON);
        }
        else {
            PS_ON_PORT |= (1 << PS_ON);
        }

        if (out_power || bus_power) {
        //if (PG_PIN & (1 << PG)) { //power good
            PWR_LED_PORT |= (1 << PWR_LED);

            if (out_power) PWR_OUT_PORT |= (1 << PWR_OUT);
            else PWR_OUT_PORT &= ~(1 << PWR_OUT);

            if (bus_power) BUS_PWR_ON_PORT |= (1 << BUS_PWR_ON);
            else BUS_PWR_ON_PORT &= ~(1 << BUS_PWR_ON);
        }
        else {
            PWR_OUT_PORT &= ~(1 << PWR_OUT);
            BUS_PWR_ON_PORT &= ~(1 << BUS_PWR_ON);
            PWR_LED_PORT &= ~(1 << PWR_LED);
        }

        h9msg_t cm;
        if (CAN_get_msg(&cm)) {
            STATUS_LED_PORT |= (1<<STATUS_LED);
            led_counter = 0x10000;
            if (cm.type == H9MSG_TYPE_GET_REG &&
                     (cm.destination_id == can_node_id || cm.destination_id == H9MSG_BROADCAST_ID)) {
                h9msg_t cm_res;
                CAN_init_response_msg(&cm, &cm_res);
                cm_res.data[0] = cm.data[0];

                int32_t tmp;

                switch (cm.data[0]) {
                    case 10:
                        cm_res.dlc = 2;
                        cm_res.data[1] = out_power;
                        CAN_put_msg(&cm_res);
                        break;
                    case 11:
                        cm_res.dlc = 2;
                        cm_res.data[1] = bus_power;
                        CAN_put_msg(&cm_res);
                        break;
                    case 12:
                        ADMUX = (1 << ADMUX) | PWR_I_ADMUX;
                        cm_res.dlc = 3;
                        ADCSRA |= (1<<ADSC);
                        while(ADCSRA & (1<<ADSC));
                        tmp = ADC;
                        tmp = (tmp - 157) * 1000000 / 39498;
                        cm_res.data[2] = tmp & 0xff;
                        cm_res.data[1] = (tmp >> 8) & 0xff;
                        CAN_put_msg(&cm_res);
                        break;
                    case 13:
                        ADMUX = (1 << ADMUX) | BUS_I_ADMUX;
                        cm_res.dlc = 3;
                        ADCSRA |= (1<<ADSC);
                        while(ADCSRA & (1<<ADSC));
                        tmp = ADC;
                        tmp = (tmp - 136) * 100 / 149;
                        cm_res.data[2] = tmp & 0xff;
                        cm_res.data[1] = (tmp >> 8) & 0xff;
                        CAN_put_msg(&cm_res);
                        break;
                    default:
                        cm_res.type = H9MSG_TYPE_ERROR;
                        cm_res.data[0] = H9MSG_ERROR_INVALID_REGISTER;
                        cm_res.dlc = 1;
                        CAN_put_msg(&cm_res);
                }
            }
            else if (cm.type == H9MSG_TYPE_SET_REG && cm.destination_id == can_node_id) {
                h9msg_t cm_res;
                CAN_init_response_msg(&cm, &cm_res);
                cm_res.data[0] = cm.data[0];
                switch (cm.data[0]) {
                    case 10:
                        out_power = cm.data[1] & 0x01;
                        cm_res.dlc = 2;
                        cm_res.data[1] = out_power;
                        CAN_put_msg(&cm_res);
                        break;
                    case 11:
                        bus_power = cm.data[1] & 0x01;
                        cm_res.dlc = 2;
                        cm_res.data[1] = bus_power;
                        CAN_put_msg(&cm_res);
                        break;
                    default:
                        cm_res.type = H9MSG_TYPE_ERROR;
                        cm_res.data[0] = H9MSG_ERROR_INVALID_REGISTER;
                        cm_res.dlc = 1;
                        CAN_put_msg(&cm_res);
                }
            }
            else {
                h9msg_t cm_res;
                CAN_init_response_msg(&cm, &cm_res);
                cm_res.type = H9MSG_TYPE_ERROR;
                cm_res.data[0] = H9MSG_ERROR_INVALID_MSG;
                cm_res.dlc = 1;
                CAN_put_msg(&cm_res);
            }
        }
    }
}
