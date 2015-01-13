//
//  joystick.cpp
//  tvs
//
//  Created by Federico Ferri on 13/01/2015.
//  Copyright (c) 2014 Federico Ferri. All rights reserved.
//

#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <linux/input.h>

static int joy = -1;
dReal joy_l = 0.0, joy_r = 0.0;

void joy_open() {
    joy = open("/dev/input/by-id/usb-Logitech_Logitech_Cordless_RumblePad_2-event-joystick", O_RDONLY | O_NONBLOCK);
    if(joy == -1) {perror("joy_open"); exit(1);}
    std::cout << "opened joystick device" << std::endl;
}

void joy_poll() {
    if(joy == -1) return;
    input_event ev;
    while(1) {
        ssize_t n = read(joy, &ev, sizeof(input_event));
        if(n == -1) return;
        if(n < sizeof(input_event)) {perror("joy_poll: short read()"); exit(1);}
        if(ev.type == EV_ABS) {
            if(ev.code == ABS_Y) {
                joy_l = (ev.value - 127) / 127.0 * environment->config.world.max_track_speed;
                std::cout << "L = " << joy_l << std::endl;
            } else if(ev.code == ABS_RZ) {
                joy_r = (ev.value - 127) / 127.0 * environment->config.world.max_track_speed;
                std::cout << "R = " << joy_r << std::endl;
            }
        }
    }
}

void joy_close() {
    if(joy != -1) close(joy);
}

