/*
 * Remote processor messaging - sample client driver
 *
 * Copyright (C) 2011 Texas Instruments, Inc.
 * Copyright (C) 2011 Google, Inc.
 *
 * Ohad Ben-Cohen <ohad@wizery.com>
 * Brian Swetland <swetland@google.com>
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/rpmsg.h>
#include <linux/remoteproc.h>
#include <asm/barrier.h>

#define NAME_PLUS  "rpmsg-client-plus"
#define NAME_MINUS "rpmsg-client-minus"

#define ENDPT_PLUS  (60)
#define ENDPT_MINUS (61)

#define RPMSG_SHMEM_DA  (0xa1000000ULL)
#define RPMSG_SHMEM_LEN (0x1000)

static volatile char *shm_plus;
static volatile char *shm_minus;

static const char *string_plus[] = {
	"ABCDEFG",
	"HAL",
	"Sghr hr sdrs.",
};

static const char *string_minus[] = {
	"TUVWXYZ",
	"GppCbs",
	"Ifmmp xpsme.",
};

static void rpmsg_sample2_cb(struct rpmsg_channel *rpdev, void *data, int len,
						void *priv, u32 src)
{
	int ret, idx;
	u32 msg;

	static int cnt_plus = 0;
	static int cnt_minus = 0;

	if(src == ENDPT_PLUS){
		idx = cnt_plus % ARRAY_SIZE(string_plus);
		printk(KERN_DEBUG "%s src:%#x/string:%.*s\n",
				__func__, src, strlen(string_plus[idx]), shm_plus);

		idx = (++cnt_plus % ARRAY_SIZE(string_plus));
		msg = (u32)strlen(string_plus[idx]);
		memcpy((void *)shm_plus, string_plus[idx], msg);
	}
	else{
		idx = cnt_minus % ARRAY_SIZE(string_minus);
		printk(KERN_DEBUG "%s src:%#x/string:%.*s\n",
				__func__, src, strlen(string_minus[idx]), shm_minus);

		idx = (++cnt_minus % ARRAY_SIZE(string_minus));
		msg = (u32)strlen(string_minus[idx]);
		memcpy((void *)shm_minus, string_minus[idx], msg);
	}

	smp_wmb();

	/* send a new message now */
	ret = rpmsg_send(rpdev, &msg, sizeof(msg));
	if (ret)
		dev_err(&rpdev->dev, "rpmsg_send failed: %d\n", ret);
}

static int rpmsg_sample2_probe(struct rpmsg_channel *rpdev)
{
	int ret;
	u32 msg;
	struct rproc *rproc = vdev_to_rproc(rpdev->vrp->vdev);

	dev_info(&rpdev->dev, "new channel: 0x%x -> 0x%x!\n",
					rpdev->src, rpdev->dst);

	if(rpdev->dst == ENDPT_PLUS){
		void *va = rproc_da_to_va(rproc, RPMSG_SHMEM_DA, RPMSG_SHMEM_LEN);
		shm_plus = (volatile char *)va;

		msg = (u32)strlen(string_plus[0]);
		memcpy((void *)shm_plus, string_plus[0], msg);
	}
	else{
		unsigned char *va = rproc_da_to_va(rproc, RPMSG_SHMEM_DA, RPMSG_SHMEM_LEN);
		shm_minus = (volatile char *)&va[RPMSG_SHMEM_LEN / 2];

		msg = (u32)strlen(string_minus[0]);
		memcpy((void *)shm_minus, string_minus[0], msg);
	}

	smp_wmb();

	/* send a message to our remote processor */
	ret = rpmsg_send(rpdev, &msg, sizeof(msg));
	if (ret) {
		dev_err(&rpdev->dev, "rpmsg_send failed: %d\n", ret);
		return ret;
	}

	return 0;
}

static void __devexit rpmsg_sample2_remove(struct rpmsg_channel *rpdev)
{
	dev_info(&rpdev->dev, "rpmsg sample driver is removed\n");
}

static struct rpmsg_device_id rpmsg_driver_sample2_id_table[] = {
	{ .name	= NAME_PLUS },
	{ .name	= NAME_MINUS },
	{ },
};
MODULE_DEVICE_TABLE(rpmsg, rpmsg_driver_sample2_id_table);

static struct rpmsg_driver rpmsg_sample2_client = {
	.drv.name	= KBUILD_MODNAME,
	.drv.owner	= THIS_MODULE,
	.id_table	= rpmsg_driver_sample2_id_table,
	.probe		= rpmsg_sample2_probe,
	.callback	= rpmsg_sample2_cb,
	.remove		= __devexit_p(rpmsg_sample2_remove),
};

static int __init rpmsg_sample2_init(void)
{
	return register_rpmsg_driver(&rpmsg_sample2_client);
}
module_init(rpmsg_sample2_init);

static void __exit rpmsg_sample2_fini(void)
{
	unregister_rpmsg_driver(&rpmsg_sample2_client);
}
module_exit(rpmsg_sample2_fini);

MODULE_DESCRIPTION("Remote processor messaging sample client driver");
MODULE_LICENSE("GPL v2");
