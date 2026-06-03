/*
 * Copyright (C) 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file test_chassis_rpmsg.c
 * @brief Test program for RPMsg ESOS differential chassis
 */

#include <math.h>
#include <signal.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#include "../include/chassis.h"

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

static volatile bool running = true;

static void signal_handler(int sig) {
    (void)sig;
    running = false;
}

static void usage(const char *prog) {
    printf("Usage: %s [OPTIONS]\n", prog);
    printf("\n");
    printf("Options:\n");
    printf("  -c, --ctrl <path>   RPMsg control device (default: /dev/rpmsg_ctrl0)\n");
    printf("  -d, --data <path>   RPMsg data device (default: /dev/rpmsg0)\n");
    printf("  -s, --service <n>   RPMsg service name (default: rpmsg:motor_ctrl)\n");
    printf("  -h, --help          Show this help message\n");
    printf("\n");
    printf("Examples:\n");
    printf("  %s -c /dev/rpmsg_ctrl0 -d /dev/rpmsg0 -s rpmsg:motor_ctrl\n", prog);
}

int main(int argc, char *argv[]) {
    struct chassis_dev *chassis;
    struct chassis_rpmsg_config config;

    const char *ctrl_dev = "/dev/rpmsg_ctrl0";
    const char *data_dev = "/dev/rpmsg0";
    const char *service_name = "rpmsg:motor_ctrl";

    /* Parse arguments */
    for (int i = 1; i < argc; i++) {
        if ((strcmp(argv[i], "-c") == 0 || strcmp(argv[i], "--ctrl") == 0) &&
            i + 1 < argc) {
            ctrl_dev = argv[++i];
        } else if ((strcmp(argv[i], "-d") == 0 ||
                strcmp(argv[i], "--data") == 0) && i + 1 < argc) {
            data_dev = argv[++i];
        } else if ((strcmp(argv[i], "-s") == 0 ||
                strcmp(argv[i], "--service") == 0) && i + 1 < argc) {
            service_name = argv[++i];
        } else if (strcmp(argv[i], "-h") == 0 || strcmp(argv[i], "--help") == 0) {
            usage(argv[0]);
            return 0;
        } else {
            fprintf(stderr, "Unknown argument: %s\n", argv[i]);
            usage(argv[0]);
            return 1;
        }
    }

    printf("=== Chassis RPMsg ESOS Test ===\n\n");
    printf("Ctrl Dev: %s\n", ctrl_dev);
    printf("Data Dev: %s\n", data_dev);
    printf("Service:  %s\n\n", service_name);

    /* Setup signal handler */
    signal(SIGINT, signal_handler);
    signal(SIGTERM, signal_handler);

    /* Configure and create chassis */
    printf("[1] Creating differential chassis (RCPU / 小核)...\n");
    config = (struct chassis_rpmsg_config){
        .base = {
            .type = CHASSIS_TYPE_DIFF_2WD,
            .wheel_diameter = 0.067f,  /* 67mm */
            .wheel_base = 0.183f,      /* 183mm */
            .wheel_track = 0.0f,       /* not used for 2WD */
            .max_speed = 1.0f,
            .max_angular = 3.14f,
        },
        .ctrl_dev = ctrl_dev,
        .data_dev = data_dev,
        .service_name = service_name,
        .local_addr = 1003,
        .remote_addr = 1002,
        .reduction_ratio = 56.0f,
        .ff_factor = 0.3f,
        .pid_kp = 0.05f,
        .pid_ki = 0.2f,
        .pid_kd = 0.01f,
        .cfg_send_on_startup = true,
        .feedback_enable = true,
    };

    chassis = chassis_alloc("drv_rpmsg_esos", &config);
    if (!chassis) {
        fprintf(stderr, "Failed to create RPMsg chassis\n");
        return -1;
    }
    printf("   RPMsg chassis created and started\n\n");

    /* Test motion sequence */
    printf("[2] Running motion test...\n");
    printf("   Press Ctrl+C to stop\n\n");

    chassis_velocity_t vel;
    int phase = 0;
    int phase_time = 0;

    while (running) {
        switch (phase) {
        case 0:  /* Forward */
            vel.vx = 0.2f;
            vel.vy = 0.0f;
            vel.wz = 0.0f;
            if (phase_time == 0)
                printf("   -> Forward (vx=0.2 m/s)\n");
            break;
        case 1:
        case 3:
        case 5:
        case 7:
            vel.vx = 0.0f;
            vel.vy = 0.0f;
            vel.wz = 0.0f;
            if (phase_time == 0)
                printf("   -> Stop\n");
            break;
        case 2:
            vel.vx = 0.0f;
            vel.vy = 0.0f;
            vel.wz = 0.5f;
            if (phase_time == 0)
                printf("   -> Rotate left (wz=0.5 rad/s)\n");
            break;
        case 4:
            vel.vx = -0.2f;
            vel.vy = 0.0f;
            vel.wz = 0.0f;
            if (phase_time == 0)
                printf("   -> Backward (vx=-0.2 m/s)\n");
            break;
        case 6:
            vel.vx = 0.0f;
            vel.vy = 0.0f;
            vel.wz = -0.5f;
            if (phase_time == 0)
                printf("   -> Rotate right (wz=-0.5 rad/s)\n");
            break;
        }

        chassis_set_velocity(chassis, &vel);

        /* Print odometry every second */
        if (phase_time % 1000 == 0) {
            chassis_velocity_t cur_vel;
            chassis_pose_t cur_pose;
            chassis_get_odom(chassis, &cur_vel, &cur_pose);
            printf("   [Odom] pos=(%.2f, %.2f) yaw=%.1f° vel=(%.2f, %.2f)\n",
                    cur_pose.x, cur_pose.y, cur_pose.yaw * 180.0f / M_PI,
                    cur_vel.vx, cur_vel.wz);
        }

        usleep(100000);  /* 100ms */
        phase_time += 100;

        int duration = (phase % 2 == 0) ? 2000 : 1000;
        if (phase_time >= duration) {
            phase_time = 0;
            phase = (phase + 1) % 8;
        }
    }

    /* Stop and cleanup */
    printf("\n[3] Stopping chassis...\n");
    chassis_brake(chassis);

    /* Print final odometry */
    chassis_velocity_t final_vel;
    chassis_pose_t final_pose;
    chassis_get_odom(chassis, &final_vel, &final_pose);
    printf("Final odometry:\n");
    printf("  Position: (%.3f, %.3f) m\n", final_pose.x, final_pose.y);
    printf("  Heading:  %.2f degrees\n", final_pose.yaw * 180.0f / M_PI);

    printf("\n[4] Cleaning up...\n");
    chassis_free(chassis);
    printf("   Done\n");

    return 0;
}
