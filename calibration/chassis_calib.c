/*
 * Simple calibration tool
 * Sends a constant 0.5 m/s forward command to the RPMsg ESOS chassis
 * and prints raw feedback messages received from the data device.
 */

#include <errno.h>
#include <fcntl.h>
#include <pthread.h>
#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/ioctl.h>
#include <sys/time.h>
#include <unistd.h>

#include <stdint.h>
#include <math.h>
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

struct rpmsg_endpoint_info {
    char name[32];
    uint32_t src;
    uint32_t dst;
};

#define RPMSG_CREATE_EPT_IOCTL _IOW(0xb5, 0x1, struct rpmsg_endpoint_info)
#define RPMSG_DESTROY_EPT_IOCTL _IO(0xb5, 0x2)

static volatile int running = 1;

struct thread_arg {
    int fd;
    float wheel_diam;
    float target_rps_l;
    float target_rps_r;
};

static void parse_and_print(const char *buf, float wheel_diam,
        float target_rps_l, float target_rps_r) {
    int dir1 = 0, dir2 = 0;
    int speed1_mrs = 0, speed2_mrs = 0;
    int duty1_pm = 0, duty2_pm = 0;
    if (sscanf(buf, "%d,%d,%d;%d,%d,%d", &dir1, &speed1_mrs, &duty1_pm,
            &dir2, &speed2_mrs, &duty2_pm) == 6) {
        float speed1_rps = speed1_mrs / 1000.0f;
        float speed2_rps = speed2_mrs / 1000.0f;
        float duty1 = duty1_pm / 1000.0f;
        float duty2 = duty2_pm / 1000.0f;

        /* Feed-forward factor = duty / speed_rps (guard against div-by-zero) */
        float ff1 = (fabsf(speed1_rps) > 1e-6f) ? duty1 / speed1_rps : 0.0f;
        float ff2 = (fabsf(speed2_rps) > 1e-6f) ? duty2 / speed2_rps : 0.0f;

        printf("[PARSED] target_l=%.3f rps target_r=%.3f rps ; "
            "dir1=%d speed1_rps=%.3f rps duty1=%.3f ff1=%.3f ; "
            "dir2=%d speed2_rps=%.3f rps duty2=%.3f ff2=%.3f\n",
                target_rps_l, target_rps_r,
                dir1, speed1_rps, duty1, ff1,
                dir2, speed2_rps, duty2, ff2);
    } else {
        printf("[PARSED] unable to parse feedback\n");
    }
}

static void sigint_handler(int sig) {
    (void)sig;
    running = 0;
}

static void *reader_thread(void *arg) {
    struct thread_arg *t = arg;
    char buf[256];
    while (running) {
        ssize_t n = read(t->fd, buf, sizeof(buf) - 1);
        if (n < 0) {
            if (errno == EINTR)
                continue;
            perror("read");
            break;
        }
        if (n == 0) {
            usleep(100000);
            continue;
        }
        buf[n] = '\0';
        parse_and_print(buf, t->wheel_diam, t->target_rps_l, t->target_rps_r);
    }
    return NULL;
}

static void velocity_to_cmd(float v, float wheel_diam, int *out_dir, float *out_speed) {
    if (fabsf(v) < 1e-3f) {
        *out_dir = 0;
        *out_speed = 0.0f;
        return;
    }
    *out_dir = (v > 0) ? 1 : 2;
    *out_speed = fabsf(v) / (M_PI * wheel_diam);
}

int main(int argc, char *argv[]) {
    const char *ctrl_dev = "/dev/rpmsg_ctrl0";
    const char *data_dev = "/dev/rpmsg0";
    const char *service = "rpmsg:motor_ctrl";
    uint32_t local = 1003;
    uint32_t remote = 1002;
    float wheel_diam = 0.067f; /* meters */
    float wheel_base = 0.183f; /* meters */
    float vx = 0.5f; /* target linear velocity m/s */

    for (int i = 1; i < argc; i++) {
        if ((strcmp(argv[i], "-c") == 0 || strcmp(argv[i], "--ctrl") == 0) && i + 1 < argc)
            ctrl_dev = argv[++i];
        else if ((strcmp(argv[i], "-d") == 0 || strcmp(argv[i], "--data") == 0) && i + 1 < argc)
            data_dev = argv[++i];
        else if ((strcmp(argv[i], "-s") == 0 || strcmp(argv[i], "--service") == 0) && i + 1 < argc)
            service = argv[++i];
        else if ((strcmp(argv[i], "-w") == 0 || strcmp(argv[i], "--wheel") == 0) && i + 1 < argc)
            wheel_diam = atof(argv[++i]);
        else if ((strcmp(argv[i], "-b") == 0 || strcmp(argv[i], "--base") == 0) && i + 1 < argc)
            wheel_base = atof(argv[++i]);
        else if ((strcmp(argv[i], "-v") == 0 || strcmp(argv[i], "--vel") == 0) && i + 1 < argc)
            vx = atof(argv[++i]);
        else {
            printf("Usage: %s [-c ctrl_dev] [-d data_dev] [-s service] [-w wheel_diam] [-b wheel_base] [-v vx]\n", argv[0]);
            return 0;
        }
    }

    printf("Ctrl: %s\nData: %s\nService: %s\nwheel_diam=%.3f wheel_base=%.3f vx=%.3f\n",
            ctrl_dev, data_dev, service, wheel_diam, wheel_base, vx);

    int ctrl_fd = open(ctrl_dev, O_RDWR);
    if (ctrl_fd < 0) {
        perror("open ctrl_dev");
        return -1;
    }

    struct rpmsg_endpoint_info ep;
    memset(&ep, 0, sizeof(ep));
    strncpy(ep.name, service, sizeof(ep.name) - 1);
    ep.src = local;
    ep.dst = remote;

    if (ioctl(ctrl_fd, RPMSG_CREATE_EPT_IOCTL, &ep) < 0) {
        perror("ioctl create endpoint");
        close(ctrl_fd);
        return -1;
    }

    int data_fd = open(data_dev, O_RDWR);
    if (data_fd < 0) {
        perror("open data_dev");
        ioctl(ctrl_fd, RPMSG_DESTROY_EPT_IOCTL);
        close(ctrl_fd);
        return -1;
    }

    signal(SIGINT, sigint_handler);
    signal(SIGTERM, sigint_handler);

    pthread_t reader;
    struct thread_arg targ = { .fd = data_fd, .wheel_diam = wheel_diam,
            .target_rps_l = 0.0f, .target_rps_r = 0.0f };
    if (pthread_create(&reader, NULL, reader_thread, &targ) != 0) {
        perror("pthread_create");
        close(data_fd);
        ioctl(ctrl_fd, RPMSG_DESTROY_EPT_IOCTL);
        close(ctrl_fd);
        return -1;
    }

    /* Send velocity at ~20Hz */
    struct timeval last;
    gettimeofday(&last, NULL);

    while (running) {
        /* build command from vx, wz=0 */
        float v_l = vx - 0.0f * wheel_base / 2.0f;
        float v_r = vx + 0.0f * wheel_base / 2.0f;
        int dir_l, dir_r;
        float spd_l, spd_r;
        velocity_to_cmd(v_l, wheel_diam, &dir_l, &spd_l);
        velocity_to_cmd(v_r, wheel_diam, &dir_r, &spd_r);

        targ.target_rps_l = spd_l;
        targ.target_rps_r = spd_r;

        char cmd[64];
        int len = snprintf(cmd, sizeof(cmd), "%d,%.3f;%d,%.3f", dir_l, spd_l, dir_r, spd_r);
        if (len > 0) {
            if (write(data_fd, cmd, len + 1) < 0) {
                perror("write data_dev");
                break;
            }
        }

        usleep(50000); /* 50ms -> 20Hz */
    }

    /* send explicit stop command to ensure wheels stop on Ctrl+C */
    {
        char stopcmd[] = "0,0.000;0,0.000";
        ssize_t w = write(data_fd, stopcmd, strlen(stopcmd) + 1);
        if (w < 0)
            perror("write stopcmd");
        /* give device a short moment to process stop */
        usleep(100000);
    }

    /* cleanup */
    pthread_join(reader, NULL);
    close(data_fd);
    ioctl(ctrl_fd, RPMSG_DESTROY_EPT_IOCTL);
    close(ctrl_fd);

    return 0;
}
