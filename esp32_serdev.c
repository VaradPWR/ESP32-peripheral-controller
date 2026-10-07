#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/of.h>
#include <linux/serdev.h>
#include <linux/gpio/driver.h>
#include <linux/mutex.h>
#include <linux/completion.h>

struct esp32_gpio_dev {
    struct serdev_device *serdev;
    struct gpio_chip gc;
    struct mutex lock;            /* Protects serial transmission */
    u8 rx_val;                    /* Value returned from ESP32 */
    struct completion rx_done;   /* Sync read response */
};

/* Send 3-byte command over serdev */
static int esp32_send_cmd(struct esp32_gpio_dev *esp, u8 cmd, u8 pin, u8 val)
{
    u8 buf[3] = {cmd, pin, val};
    return serdev_device_write_buf(esp->serdev, buf, sizeof(buf));
}

/* Callback: gpioset / direction_output */
static int esp32_gpio_dir_out(struct gpio_chip *gc, unsigned int offset, int value)
{
    struct esp32_gpio_dev *esp = gpiochip_get_data(gc);

    mutex_lock(&esp->lock);
    esp32_send_cmd(esp, 0x03, (u8)offset, 1);     /* 0x03 = DIR_OUT */
    esp32_send_cmd(esp, 0x02, (u8)offset, value); /* 0x02 = WRITE */
    mutex_unlock(&esp->lock);

    return 0;
}

/* Callback: direction_input */
static int esp32_gpio_dir_in(struct gpio_chip *gc, unsigned int offset)
{
    struct esp32_gpio_dev *esp = gpiochip_get_data(gc);

    mutex_lock(&esp->lock);
    esp32_send_cmd(esp, 0x03, (u8)offset, 0);     /* 0x03 = DIR_IN */
    mutex_unlock(&esp->lock);

    return 0;
}

/* Callback: gpioset (runtime write) */
static void esp32_gpio_set(struct gpio_chip *gc, unsigned int offset, int value)
{
    struct esp32_gpio_dev *esp = gpiochip_get_data(gc);

    mutex_lock(&esp->lock);
    esp32_send_cmd(esp, 0x02, (u8)offset, value); /* 0x02 = WRITE */
    mutex_unlock(&esp->lock);
}

/* Serdev RX callback - parses ESP32 2-byte response [pin, val] */
static int esp32_receive_buf(struct serdev_device *serdev, const unsigned char *buf, size_t count)
{
    struct esp32_gpio_dev *esp = serdev_device_get_drvdata(serdev);

    if (count >= 2) {
        esp->rx_val = buf[1];
        complete(&esp->rx_done);
    }
    return count;
}

/* Callback: gpioget (runtime read) */
static int esp32_gpio_get(struct gpio_chip *gc, unsigned int offset)
{
    struct esp32_gpio_dev *esp = gpiochip_get_data(gc);
    int val;

    mutex_lock(&esp->lock);
    reinit_completion(&esp->rx_done);
    
    esp32_send_cmd(esp, 0x01, (u8)offset, 0);     /* 0x01 = READ */

    /* Wait up to 100ms for ESP32 serial response */
    if (!wait_for_completion_timeout(&esp->rx_done, msecs_to_jiffies(100))) {
        dev_err(&esp->serdev->dev, "Timeout reading GPIO %d\n", offset);
        mutex_unlock(&esp->lock);
        return -ETIMEDOUT;
    }

    val = esp->rx_val;
    mutex_unlock(&esp->lock);

    return val;
}

static const struct serdev_device_ops esp32_ops = {
    .receive_buf = esp32_receive_buf,
};

static int esp32_probe(struct serdev_device *serdev)
{
    struct esp32_gpio_dev *esp;
    int ret;

    dev_info(&serdev->dev, "ESP32 GPIO serdev driver probing...\n");

    esp = devm_kzalloc(&serdev->dev, sizeof(*esp), GFP_KERNEL);
    if (!esp)
        return -ENOMEM;

    esp->serdev = serdev;
    mutex_init(&esp->lock);
    init_completion(&esp->rx_done);

    serdev_device_set_drvdata(serdev, esp);
    serdev_device_set_client_ops(serdev, &esp32_ops);

    ret = serdev_device_open(serdev);
    if (ret) {
        dev_err(&serdev->dev, "Failed to open serdev device: %d\n", ret);
        return ret;
    }

    serdev_device_set_baudrate(serdev, 115200);
    serdev_device_set_flow_control(serdev, false);

    /* Configure Linux gpiochip properties */
    esp->gc.label = "esp32-gpiochip";
    esp->gc.parent = &serdev->dev;
    esp->gc.owner = THIS_MODULE;
    esp->gc.direction_input = esp32_gpio_dir_in;
    esp->gc.direction_output = esp32_gpio_dir_out;
    esp->gc.get = esp32_gpio_get;
    esp->gc.set = esp32_gpio_set;
    esp->gc.base = -1;  /* Dynamic gpiochip number assignment */
    esp->gc.ngpio = 34; /* Total ESP32 GPIOs exposed */

    ret = devm_gpiochip_add_data(&serdev->dev, &esp->gc, esp);
    if (ret) {
        dev_err(&serdev->dev, "Failed to register gpiochip: %d\n", ret);
        serdev_device_close(serdev);
        return ret;
    }

    dev_info(&serdev->dev, "ESP32 native GPIO expansion chip active!\n");
    return 0;
}

static void esp32_remove(struct serdev_device *serdev)
{
    serdev_device_close(serdev);
    dev_info(&serdev->dev, "ESP32 serdev driver removed\n");
}

static const struct of_device_id esp32_of_match[] = {
    { .compatible = "custom,esp32" },
    {}
};
MODULE_DEVICE_TABLE(of, esp32_of_match);

static struct serdev_device_driver esp32_driver = {
    .driver = {
        .name = "esp32-serdev",
        .of_match_table = esp32_of_match,
    },
    .probe = esp32_probe,
    .remove = esp32_remove,
};

module_serdev_device_driver(esp32_driver);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("magneto");
MODULE_DESCRIPTION("ESP32 native gpiochip serdev driver");
