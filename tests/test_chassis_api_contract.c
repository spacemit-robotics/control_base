/*
 * Copyright (C) 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
 * SPDX-License-Identifier: Apache-2.0
 */

#include <errno.h>
#include <math.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "../src/chassis_core.h"

struct fake_priv {
    chassis_velocity_t target_vel;
    int set_velocity_calls;
    int tick_calls;
    int brake_calls;
    int relax_calls;
    int stop_calls;
    bool fail_set_velocity;
};

static int g_fake_free_calls;
static int g_fake_stop_calls;
static int g_failures;

static void report_expectation_failure(const char *expr, const char *file,
        int line, const char *message)
{
    fprintf(stderr, "[FAIL] %s:%d: %s", file, line, expr);
    if (message && *message)
        fprintf(stderr, " (%s)", message);
    fputc('\n', stderr);
    g_failures++;
}

#define EXPECT_TRUE(expr, message) \
    do { \
        if (!(expr)) { \
            report_expectation_failure(#expr, __FILE__, __LINE__, message); \
            return 1; \
        } \
    } while (0)

#define EXPECT_EQ_INT(actual, expected, message) \
    EXPECT_TRUE((actual) == (expected), message)

static int float_close(float a, float b)
{
    return fabsf(a - b) < 1e-4f;
}

static int fake_set_velocity(struct chassis_dev *dev,
        const chassis_velocity_t *vel)
{
    struct fake_priv *priv;

    if (!dev || !dev->priv_data || !vel)
        return -EINVAL;

    priv = (struct fake_priv *)dev->priv_data;
    priv->set_velocity_calls++;
    if (priv->fail_set_velocity)
        return -EIO;

    priv->target_vel = *vel;
    pthread_mutex_lock(&dev->odom_lock);
    dev->cur_vel = *vel;
    pthread_mutex_unlock(&dev->odom_lock);
    return CHASSIS_OK;
}

static void fake_tick(struct chassis_dev *dev, float dt_s)
{
    struct fake_priv *priv;

    if (!dev || !dev->priv_data)
        return;

    priv = (struct fake_priv *)dev->priv_data;
    priv->tick_calls++;

    pthread_mutex_lock(&dev->odom_lock);
    dev->cur_pose.x += dev->cur_vel.vx * dt_s;
    dev->cur_pose.y += dev->cur_vel.vy * dt_s;
    dev->cur_pose.yaw += dev->cur_vel.wz * dt_s;
    pthread_mutex_unlock(&dev->odom_lock);
}

static void fake_brake(struct chassis_dev *dev)
{
    struct fake_priv *priv;
    chassis_velocity_t zero = {0.0f, 0.0f, 0.0f};

    if (!dev || !dev->priv_data)
        return;

    priv = (struct fake_priv *)dev->priv_data;
    priv->brake_calls++;
    fake_set_velocity(dev, &zero);
}

static void fake_relax(struct chassis_dev *dev)
{
    struct fake_priv *priv;

    if (!dev || !dev->priv_data)
        return;

    priv = (struct fake_priv *)dev->priv_data;
    priv->relax_calls++;
}

static int fake_stop(struct chassis_dev *dev)
{
    struct fake_priv *priv;

    if (!dev || !dev->priv_data)
        return -EINVAL;

    priv = (struct fake_priv *)dev->priv_data;
    priv->stop_calls++;
    g_fake_stop_calls++;
    return CHASSIS_OK;
}

static void fake_free(struct chassis_dev *dev)
{
    g_fake_free_calls++;
    chassis_dev_free_default(dev);
}

static const struct chassis_ops fake_ops = {
    .stop = fake_stop,
    .set_velocity = fake_set_velocity,
    .tick = fake_tick,
    .brake = fake_brake,
    .relax = fake_relax,
    .free = fake_free,
};

static const struct chassis_ops no_velocity_ops = {
    .free = fake_free,
};

static struct chassis_dev *fake_create(const char *name,
        const struct chassis_config *config)
{
    struct chassis_dev *dev;
    struct fake_priv *priv;

    if (!name || !config)
        return NULL;

    if (config->wheel_diameter <= 0.0f || config->wheel_base <= 0.0f)
        return NULL;

    dev = chassis_dev_alloc(name, config, sizeof(*priv));
    if (!dev)
        return NULL;

    priv = (struct fake_priv *)dev->priv_data;
    priv->fail_set_velocity = config->max_speed < 0.0f;
    dev->ops = &fake_ops;
    return dev;
}

static struct chassis_dev *no_velocity_create(const char *name,
        const struct chassis_config *config)
{
    struct chassis_dev *dev;

    if (!name || !config)
        return NULL;

    dev = chassis_dev_alloc(name, config, sizeof(struct fake_priv));
    if (!dev)
        return NULL;

    dev->ops = &no_velocity_ops;
    return dev;
}

REGISTER_CHASSIS_DRIVER("fake_chassis", fake_create);
REGISTER_CHASSIS_DRIVER("fake_no_velocity", no_velocity_create);

static struct chassis_config make_valid_config(void)
{
    struct chassis_config config = {
        .type = CHASSIS_TYPE_DIFF_2WD,
        .wheel_diameter = 0.067f,
        .wheel_base = 0.183f,
        .wheel_track = 0.0f,
        .max_speed = 1.0f,
        .max_angular = 3.14f,
    };

    return config;
}

static int test_allocation_velocity_tick_and_odom(void)
{
    struct chassis_config config = make_valid_config();
    struct chassis_dev *dev = chassis_alloc("fake_chassis", &config);
    struct fake_priv *priv;
    chassis_velocity_t command = {0.25f, -0.10f, 0.50f};
    chassis_velocity_t out_vel = {0};
    chassis_pose_t out_pose = {0};

    EXPECT_TRUE(dev != NULL, "valid fake driver should allocate");
    priv = (struct fake_priv *)dev->priv_data;
    EXPECT_TRUE(priv != NULL, "private data should be allocated");
    EXPECT_TRUE(strcmp(dev->name, "fake_chassis") == 0,
            "device name should be copied");
    EXPECT_TRUE(dev->config.type == CHASSIS_TYPE_DIFF_2WD,
            "configuration should be copied");

    EXPECT_EQ_INT(chassis_set_velocity(dev, &command), CHASSIS_OK,
            "set_velocity should succeed");
    EXPECT_EQ_INT(priv->set_velocity_calls, 1,
            "driver set_velocity should be called once");

    EXPECT_EQ_INT(chassis_get_odom(dev, &out_vel, &out_pose), CHASSIS_OK,
            "get_odom should return current velocity");
    EXPECT_TRUE(float_close(out_vel.vx, command.vx), "vx should match command");
    EXPECT_TRUE(float_close(out_vel.vy, command.vy), "vy should match command");
    EXPECT_TRUE(float_close(out_vel.wz, command.wz), "wz should match command");

    chassis_tick(dev, 2.0f);
    EXPECT_EQ_INT(priv->tick_calls, 1, "tick should call driver hook");
    EXPECT_EQ_INT(chassis_get_odom(dev, NULL, &out_pose), CHASSIS_OK,
            "get_odom should allow NULL velocity output");
    EXPECT_TRUE(float_close(out_pose.x, 0.50f), "x pose should integrate vx");
    EXPECT_TRUE(float_close(out_pose.y, -0.20f), "y pose should integrate vy");
    EXPECT_TRUE(float_close(out_pose.yaw, 1.00f), "yaw should integrate wz");

    chassis_free(dev);
    EXPECT_EQ_INT(g_fake_free_calls, 1, "driver free hook should run");
    return 0;
}

static int test_brake_relax_and_running_stop(void)
{
    struct chassis_config config = make_valid_config();
    struct chassis_dev *dev = chassis_alloc("fake_chassis", &config);
    struct fake_priv *priv;
    chassis_velocity_t command = {0.40f, 0.00f, 0.20f};
    chassis_velocity_t out_vel = {0};
    int free_calls_before = g_fake_free_calls;

    EXPECT_TRUE(dev != NULL, "valid fake driver should allocate");
    priv = (struct fake_priv *)dev->priv_data;

    EXPECT_EQ_INT(chassis_set_velocity(dev, &command), CHASSIS_OK,
            "set_velocity should succeed before brake");
    chassis_brake(dev);
    EXPECT_EQ_INT(priv->brake_calls, 1, "brake hook should be called");
    EXPECT_EQ_INT(chassis_get_odom(dev, &out_vel, NULL), CHASSIS_OK,
            "get_odom should allow NULL pose output");
    EXPECT_TRUE(float_close(out_vel.vx, 0.0f), "brake should zero vx");
    EXPECT_TRUE(float_close(out_vel.vy, 0.0f), "brake should zero vy");
    EXPECT_TRUE(float_close(out_vel.wz, 0.0f), "brake should zero wz");

    chassis_relax(dev);
    EXPECT_EQ_INT(priv->relax_calls, 1, "relax hook should be called");

    dev->running = true;
    chassis_free(dev);
    EXPECT_EQ_INT(g_fake_stop_calls, 1, "free should stop a running device");
    EXPECT_EQ_INT(g_fake_free_calls, free_calls_before + 1,
            "driver free hook should run after stop");
    return 0;
}

static int test_null_arguments_and_unknown_driver(void)
{
    struct chassis_config config = make_valid_config();
    chassis_velocity_t vel = {0};

    EXPECT_TRUE(chassis_alloc(NULL, &config) == NULL,
            "NULL driver name should fail allocation");
    EXPECT_TRUE(chassis_alloc("fake_chassis", NULL) == NULL,
            "NULL config should fail allocation");
    EXPECT_TRUE(chassis_alloc("missing_driver", &config) == NULL,
            "unknown driver should fail allocation");
    EXPECT_EQ_INT(chassis_set_velocity(NULL, &vel), -EINVAL,
            "NULL device should fail set_velocity");
    EXPECT_EQ_INT(chassis_get_odom(NULL, NULL, NULL), -EINVAL,
            "NULL device should fail get_odom");

    chassis_tick(NULL, 0.1f);
    chassis_brake(NULL);
    chassis_relax(NULL);
    chassis_free(NULL);
    return 0;
}

static int test_driver_rejects_bad_config_and_propagates_errors(void)
{
    struct chassis_config bad_config = make_valid_config();
    struct chassis_config failing_config = make_valid_config();
    struct chassis_dev *dev;
    chassis_velocity_t vel = {0.1f, 0.0f, 0.0f};

    bad_config.wheel_diameter = 0.0f;
    EXPECT_TRUE(chassis_alloc("fake_chassis", &bad_config) == NULL,
            "invalid wheel diameter should be rejected");

    bad_config = make_valid_config();
    bad_config.wheel_base = -0.1f;
    EXPECT_TRUE(chassis_alloc("fake_chassis", &bad_config) == NULL,
            "invalid wheel base should be rejected");

    failing_config.max_speed = -1.0f;
    dev = chassis_alloc("fake_chassis", &failing_config);
    EXPECT_TRUE(dev != NULL, "driver with injected set failure should allocate");
    EXPECT_EQ_INT(chassis_set_velocity(dev, &vel), -EIO,
            "set_velocity should propagate driver error");
    chassis_free(dev);
    return 0;
}

static int test_missing_velocity_hook_uses_enosys_and_default_brake(void)
{
    struct chassis_config config = make_valid_config();
    struct chassis_dev *dev = chassis_alloc("fake_no_velocity", &config);
    chassis_velocity_t vel = {0.1f, 0.0f, 0.0f};

    EXPECT_TRUE(dev != NULL, "driver without velocity hook should allocate");
    EXPECT_EQ_INT(chassis_set_velocity(dev, &vel), -ENOSYS,
            "missing set_velocity hook should return ENOSYS");
    chassis_brake(dev);
    chassis_relax(dev);
    chassis_free(dev);
    return 0;
}

static int run_functional_tests(void)
{
    int ret = 0;

    ret |= test_allocation_velocity_tick_and_odom();
    ret |= test_brake_relax_and_running_stop();
    return ret;
}

static int run_error_tests(void)
{
    int ret = 0;

    ret |= test_null_arguments_and_unknown_driver();
    ret |= test_driver_rejects_bad_config_and_propagates_errors();
    ret |= test_missing_velocity_hook_uses_enosys_and_default_brake();
    return ret;
}

int main(int argc, char **argv)
{
    int ret;

    if (argc != 2) {
        fprintf(stderr, "Usage: %s <functional|errors>\n", argv[0]);
        return 2;
    }

    if (strcmp(argv[1], "functional") == 0) {
        ret = run_functional_tests();
        if (ret == 0 && g_failures == 0)
            printf("ALL TESTS PASSED: functional\n");
        return (ret == 0 && g_failures == 0) ? 0 : 1;
    }

    if (strcmp(argv[1], "errors") == 0) {
        ret = run_error_tests();
        if (ret == 0 && g_failures == 0)
            printf("ALL TESTS PASSED: error-paths\n");
        return (ret == 0 && g_failures == 0) ? 0 : 1;
    }

    fprintf(stderr, "Unknown mode: %s\n", argv[1]);
    return 2;
}
