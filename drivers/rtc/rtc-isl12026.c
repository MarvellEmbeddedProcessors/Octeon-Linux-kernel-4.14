// SPDX-License-Identifier: GPL-2.0
/*
 * An I2C driver for the Intersil ISL 12026
 *
 * Copyright (c) 2018 Cavium, Inc.
 */
#include <linux/types.h>
#include <linux/err.h>
#include <linux/nvmem-provider.h>
#include <linux/of_device.h>
#include <linux/module.h>
#include <linux/delay.h>
#include <linux/mutex.h>
#include <linux/slab.h>
#include <linux/bcd.h>
#include <linux/i2c.h>
#include <linux/rtc.h>
#include <linux/of.h>

/* register offsets */
#define ISL12026_REG_SC		0x30

#define ISL12026_REG_SR		0x3f
# define ISL12026_REG_SR_RTCF	BIT(0)
# define ISL12026_REG_SR_WEL	BIT(1)
# define ISL12026_REG_SR_RWEL	BIT(2)
# define ISL12026_REG_SR_MBZ	BIT(3)
# define ISL12026_REG_SR_OSCF	BIT(4)

/* ISL register bits */
#define ISL12026_HR_MIL		BIT(7)	/* military or 24 hour time */

#define ISL12026_REG_PWR	0x14
# define ISL12026_REG_PWR_BSW	BIT(6)
# define ISL12026_REG_PWR_SBIB	BIT(7)

#define ISL12026_PAGESIZE 16
#define ISL12026_NVMEM_WRITE_TIME 20

struct isl12026 {
	struct rtc_device *rtc;
	struct i2c_client *nvm_client;
	struct nvmem_device *nvm_dev;
	struct nvmem_config nvm_cfg;
	/*
	 * RTC write operations require that multiple messages be
	 * transmitted, we hold the lock for all accesses to the
	 * device so that these sequences cannot be disrupted.  Also,
	 * the write cycle to the nvmem takes many mS during which the
	 * device does not respond to commands, so holding the lock
	 * also prevents access during these times.
	 */
	struct mutex lock;
};

static int isl12026_read_reg(struct i2c_client *client, int reg)
{
	struct isl12026 *priv = i2c_get_clientdata(client);
	u8 addr[] = {0, reg};
	u8 val;
	int ret;

	struct i2c_msg msgs[] = {
		{
			.addr	= client->addr,
			.flags	= 0,
			.len	= sizeof(addr),
			.buf	= addr
		},
		{
			.addr	= client->addr,
			.flags	= I2C_M_RD,
			.len	= 1,
			.buf	= &val
		}
	};

	mutex_lock(&priv->lock);

	ret = i2c_transfer(client->adapter, msgs, ARRAY_SIZE(msgs));

	mutex_unlock(&priv->lock);

	if (ret != ARRAY_SIZE(msgs)) {
		dev_err(&client->dev, "read reg error, ret=%d\n", ret);
		return -EIO;
	}
	return val;
}

static int isl12026_write_reg(struct i2c_client *client, int reg, u8 val)
{
	struct isl12026 *priv = i2c_get_clientdata(client);
	int rv = 0;
	u8 op[3];

	struct i2c_msg msg = {
		.addr	= client->addr,
		.flags	= 0,
		.len	= 1,
		.buf	= op
	};

	int ret;

	mutex_lock(&priv->lock);

	/* Set SR.WEL */
	op[0] = 0;
	op[1] = ISL12026_REG_SR;
	op[2] = ISL12026_REG_SR_WEL;
	msg.len = 3;
	ret = i2c_transfer(client->adapter, &msg, 1);
	if (ret != 1) {
		dev_err(&client->dev, "write error SR.WEL, ret=%d\n", ret);
		rv = -EIO;
		goto out;
	}

	/* Set SR.WEL and SR.RWEL */
	op[2] = ISL12026_REG_SR_WEL | ISL12026_REG_SR_RWEL;
	msg.len = 3;
	ret = i2c_transfer(client->adapter, &msg, 1);
	if (ret != 1) {
		dev_err(&client->dev,
			"write error SR.WEL|SR.RWEL, ret=%d\n", ret);
		rv = -EIO;
		goto out;
	}

	op[1] = reg;
	op[2] = val;
	msg.len = 3;
	ret = i2c_transfer(client->adapter, &msg, 1);
	if (ret != 1) {
		dev_err(&client->dev, "write error CCR, ret=%d\n", ret);
		rv = -EIO;
		goto out;
	}

	msleep(ISL12026_NVMEM_WRITE_TIME);

	/* Clear SR.WEL and SR.RWEL */
	op[1] = ISL12026_REG_SR;
	op[2] = 0;
	msg.len = 3;
	ret = i2c_transfer(client->adapter, &msg, 1);
	if (ret != 1) {
		dev_err(&client->dev, "write error SR, ret=%d\n", ret);
		rv = -EIO;
		goto out;
	}

out:
	mutex_unlock(&priv->lock);

	return rv;
}

static int isl12026_rtc_set_time(struct device *dev, struct rtc_time *tm)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct isl12026 *priv = i2c_get_clientdata(client);
	int rv = 0;
	u8 op[10];

	struct i2c_msg msg = {
		.addr	= client->addr,
		.flags	= 0,
		.len	= 1,
		.buf	= op
	};

	int ret;

	mutex_lock(&priv->lock);

	/* Set SR.WEL */
	op[0] = 0;
	op[1] = ISL12026_REG_SR;
	op[2] = ISL12026_REG_SR_WEL;
	msg.len = 3;
	ret = i2c_transfer(client->adapter, &msg, 1);
	if (ret != 1) {
		dev_err(&client->dev, "write error SR.WEL, ret=%d\n", ret);
		rv = -EIO;
		goto out;
	}

	/* Set SR.WEL and SR.RWEL */
	op[2] = ISL12026_REG_SR_WEL | ISL12026_REG_SR_RWEL;
	msg.len = 3;
	ret = i2c_transfer(client->adapter, &msg, 1);
	if (ret != 1) {
		dev_err(&client->dev,
			"write error SR.WEL|SR.RWEL, ret=%d\n", ret);
		rv = -EIO;
		goto out;
	}

	/* Set the CCR registers */
	op[1] = ISL12026_REG_SC;
	op[2] = bin2bcd(tm->tm_sec); /* SC */
	op[3] = bin2bcd(tm->tm_min); /* MN */
	op[4] = bin2bcd(tm->tm_hour) | ISL12026_HR_MIL; /* HR */
	op[5] = bin2bcd(tm->tm_mday); /* DT */
	op[6] = bin2bcd(tm->tm_mon + 1); /* MO */
	op[7] = bin2bcd(tm->tm_year % 100); /* YR */
	op[8] = bin2bcd(tm->tm_wday & 7); /* DW */
	op[9] = bin2bcd(tm->tm_year >= 100 ? 20 : 19); /* Y2K */
	msg.len = 10;
	ret = i2c_transfer(client->adapter, &msg, 1);
	if (ret != 1) {
		dev_err(&client->dev, "write error CCR, ret=%d\n", ret);
		rv = -EIO;
		goto out;
	}

	/* Clear SR.WEL and SR.RWEL */
	op[1] = ISL12026_REG_SR;
	op[2] = 0;
	msg.len = 3;
	ret = i2c_transfer(client->adapter, &msg, 1);
	if (ret != 1) {
		dev_err(&client->dev, "write error SR, ret=%d\n", ret);
		rv = -EIO;
		goto out;
	}

out:
	mutex_unlock(&priv->lock);

	return rv;
}

static int isl12026_rtc_read_time(struct device *dev, struct rtc_time *tm)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct isl12026 *priv = i2c_get_clientdata(client);
	u8 ccr[8];
	u8 addr[2];
	u8 sr;
	int ret;
	int rv = 0;

	struct i2c_msg msgs[] = {
		{
			.addr	= client->addr,
			.flags	= 0,
			.len	= sizeof(addr),
			.buf	= addr
		},
		{
			.addr	= client->addr,
			.flags	= I2C_M_RD,
		}
	};

	mutex_lock(&priv->lock);

	/* First, read SR */
	addr[0] = 0;
	addr[1] = ISL12026_REG_SR;
	msgs[1].len = 1;
	msgs[1].buf = &sr;

	ret = i2c_transfer(client->adapter, msgs, ARRAY_SIZE(msgs));
	if (ret != ARRAY_SIZE(msgs)) {
		dev_err(&client->dev, "read error, ret=%d\n", ret);
		rv = -EIO;
		goto out;
	}

	if (sr & ISL12026_REG_SR_RTCF)
		dev_warn(&client->dev, "Real-Time Clock Failure on read\n");
	if (sr & ISL12026_REG_SR_OSCF)
		dev_warn(&client->dev, "Oscillator Failure on read\n");

	/* Second, CCR regs */
	addr[0] = 0;
	addr[1] = ISL12026_REG_SC;
	msgs[1].len = sizeof(ccr);
	msgs[1].buf = ccr;

	ret = i2c_transfer(client->adapter, msgs, ARRAY_SIZE(msgs));
	if (ret != ARRAY_SIZE(msgs)) {
		dev_err(&client->dev, "read error, ret=%d\n", ret);
		rv = -EIO;
		goto out;
	}

	dev_dbg(&client->dev,
		"raw data is sec=%02x, min=%02x, hr=%02x, mday=%02x, mon=%02x, year=%02x, wday=%02x, y2k=%02x\n",
		ccr[0], ccr[1], ccr[2], ccr[3],
		ccr[4], ccr[5], ccr[6],	ccr[7]);

	tm->tm_sec = bcd2bin(ccr[0] & 0x7F);
	tm->tm_min = bcd2bin(ccr[1] & 0x7F);
	if (ccr[2] & ISL12026_HR_MIL)
		tm->tm_hour = bcd2bin(ccr[2] & 0x3F);
	else
		tm->tm_hour = bcd2bin(ccr[2] & 0x1F) +
			((ccr[2] & 0x20) ? 12 : 0);
	tm->tm_mday = bcd2bin(ccr[3] & 0x3F);
	tm->tm_mon = bcd2bin(ccr[4] & 0x1F) - 1;
	tm->tm_year = bcd2bin(ccr[5]);
	if (bcd2bin(ccr[7]) == 20)
		tm->tm_year += 100;
	tm->tm_wday = ccr[6] & 0x07;

	dev_dbg(&client->dev, "secs=%d, mins=%d, hours=%d, mday=%d, mon=%d, year=%d, wday=%d\n",
		tm->tm_sec, tm->tm_min, tm->tm_hour,
		tm->tm_mday, tm->tm_mon, tm->tm_year, tm->tm_wday);
	rv = rtc_valid_tm(tm);
out:
	mutex_unlock(&priv->lock);
	return rv;
}

static const struct rtc_class_ops isl12026_rtc_ops = {
	.read_time	= isl12026_rtc_read_time,
	.set_time	= isl12026_rtc_set_time,
};

static int isl12026_nvm_read(void *p, unsigned int offset,
			     void *val, size_t bytes)
{
	struct isl12026 *priv = p;
	int r;
	u8 addr[2];
	struct i2c_msg msgs[] = {
		{
			.addr	= priv->nvm_client->addr,
			.flags	= 0,
			.len	= sizeof(addr),
			.buf	= addr
		},
		{
			.addr	= priv->nvm_client->addr,
			.flags	= I2C_M_RD,
			.buf	= val
		}
	};

	if (offset >= priv->nvm_cfg.size)
		return 0; /* End-of-file */
	if (offset + bytes > priv->nvm_cfg.size)
		bytes = priv->nvm_cfg.size - offset;

	addr[0] = (offset >> 8) & 0xff;
	addr[1] = offset & 0xff;
	msgs[1].len = bytes;
	mutex_lock(&priv->lock);

	r = i2c_transfer(priv->nvm_client->adapter, msgs, ARRAY_SIZE(msgs));

	mutex_unlock(&priv->lock);

	if (r != ARRAY_SIZE(msgs)) {
		dev_err(priv->nvm_cfg.dev, "nvmem read error, ret=%d\n", r);
		return -EIO;
	}

	return bytes;
}

static int isl12026_nvm_write(void *p, unsigned int offset,
			      void *val, size_t bytes)
{
	struct isl12026 *priv = p;
	int r;
	u8 *v = val;
	size_t chunk_size, num_written;
	u8 payload[ISL12026_PAGESIZE + 2]; /* page + 2 address bytes */
	struct i2c_msg msgs[] = {
		{
			.addr	= priv->nvm_client->addr,
			.flags	= 0,
			.buf	= payload
		}
	};

	if (offset >= priv->nvm_cfg.size)
		return 0; /* End-of-file */
	if (offset + bytes > priv->nvm_cfg.size)
		bytes = priv->nvm_cfg.size - offset;

	mutex_lock(&priv->lock);

	num_written = 0;
	while (bytes) {
		chunk_size = round_down(offset, ISL12026_PAGESIZE) +
			ISL12026_PAGESIZE - offset;
		chunk_size = min(bytes, chunk_size);
		memcpy(payload + 2, v + num_written, chunk_size);
		payload[0] = (offset >> 8) & 0xff;
		payload[1] = offset & 0xff;
		msgs[0].len = chunk_size + 2;
		r = i2c_transfer(priv->nvm_client->adapter,
				 msgs, ARRAY_SIZE(msgs));
		if (r != ARRAY_SIZE(msgs)) {
			dev_err(priv->nvm_cfg.dev,
				"nvmem write error, ret=%d\n", r);
			break;
		}
		bytes -= chunk_size;
		offset += chunk_size;
		num_written += chunk_size;
		msleep(ISL12026_NVMEM_WRITE_TIME);
	}

	mutex_unlock(&priv->lock);

	return num_written > 0 ? num_written : -EIO;
}

static int isl12026_probe(struct i2c_client *client,
			  const struct i2c_device_id *id)
{
	struct isl12026 *priv;
	int pwr, requested_pwr;
	int ret;
	u32 bsw_val, sbib_val;
	bool set_bsw, set_sbib;

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C))
		return -ENODEV;

	priv = devm_kzalloc(&client->dev, sizeof(struct isl12026), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	mutex_init(&priv->lock);

	i2c_set_clientdata(client, priv);

	ret = of_property_read_u32(client->dev.of_node,
				   "isil,pwr-bsw", &bsw_val);
	set_bsw = (ret == 0);

	ret = of_property_read_u32(client->dev.of_node,
				   "isil,pwr-sbib", &sbib_val);
	set_sbib = (ret == 0);

	/* Check if PWR.BSW and/or PWR.SBIB need specified values */

	if (set_bsw || set_sbib) {
		pwr = isl12026_read_reg(client, ISL12026_REG_PWR);
		if (pwr < 0)
			dev_err(&client->dev,
				"Error: Failed to read PWR %d\n", pwr);

		requested_pwr = pwr;

		if (set_bsw) {
			if (bsw_val)
				requested_pwr |= ISL12026_REG_PWR_BSW;
			else
				requested_pwr &= ~ISL12026_REG_PWR_BSW;
		}
		if (set_sbib) {
			if (sbib_val)
				requested_pwr |= ISL12026_REG_PWR_SBIB;
			else
				requested_pwr &= ~ISL12026_REG_PWR_SBIB;
		}

		if (pwr >= 0 && pwr != requested_pwr) {
			dev_info(&client->dev, "PWR: %02x\n", pwr);
			dev_info(&client->dev,
				 "Updating PWR to: %02x\n", (u8)requested_pwr);
			isl12026_write_reg(client,
					   ISL12026_REG_PWR, requested_pwr);
		}
	}

	priv->rtc = devm_rtc_device_register(&client->dev, "rtc-isl12026",
					     &isl12026_rtc_ops, THIS_MODULE);

	ret = PTR_ERR_OR_ZERO(priv->rtc);
	if (ret)
		return ret;

	/* The NVMem array responds at i2c address 0x57 */
	priv->nvm_client = i2c_new_dummy(client->adapter, 0x57);
	if (!priv->nvm_client)
		return -ENOMEM;

	priv->nvm_cfg.name = "eeprom";
	priv->nvm_cfg.dev = &client->dev;
	priv->nvm_cfg.read_only = false;
	priv->nvm_cfg.root_only = true;
	priv->nvm_cfg.owner = THIS_MODULE;
	priv->nvm_cfg.base_dev = &client->dev;
	priv->nvm_cfg.priv = priv;
	priv->nvm_cfg.stride = 1;
	priv->nvm_cfg.word_size = 1;
	priv->nvm_cfg.size = 512;
	priv->nvm_cfg.reg_read = isl12026_nvm_read;
	priv->nvm_cfg.reg_write = isl12026_nvm_write;
	priv->nvm_dev = nvmem_register(&priv->nvm_cfg);

	if (!priv->nvm_dev)
		return -ENOMEM;
	return 0;
}

static int isl12026_remove(struct i2c_client *client)
{
	struct isl12026 *priv = i2c_get_clientdata(client);

	if (priv->nvm_dev)
		nvmem_unregister(priv->nvm_dev);
	if (priv->nvm_client)
		i2c_unregister_device(priv->nvm_client);

	return 0;
}

#ifdef CONFIG_OF
static const struct of_device_id isl12026_dt_match[] = {
	{ .compatible = "isil,isl12026" },
	{ },
};
MODULE_DEVICE_TABLE(of, isl12026_dt_match);
#endif

static const struct i2c_device_id isl12026_id[] = {
	{ "isl12026", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, isl12026_id);

static struct i2c_driver isl12026_driver = {
	.driver		= {
		.name	= "rtc-isl12026",
#ifdef CONFIG_OF
		.of_match_table = of_match_ptr(isl12026_dt_match),
#endif
	},
	.probe		= isl12026_probe,
	.remove		= isl12026_remove,
	.id_table	= isl12026_id,
};

module_i2c_driver(isl12026_driver);

MODULE_DESCRIPTION("ISL 12026 RTC driver");
MODULE_LICENSE("GPL");
