#ifndef _LINUX_VIRTIO_RING_H
#define _LINUX_VIRTIO_RING_H
/* This file is based on SYS/BIOS RPMsg code.
 *
 * Repositories:
 *  http://git.omapzoom.org/?p=repo/sysbios-rpmsg.git;a=summary
 *
 * The original license terms are as follows.
 */
/* An interface for efficient virtio implementation, currently for use by KVM
 * and lguest, but hopefully others soon.  Do NOT change this since it will
 * break existing servers and clients.
 *
 * This header is BSD licensed so anyone can use the definitions to implement
 * compatible drivers/servers.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. Neither the name of IBM nor the names of its contributors
 *    may be used to endorse or promote products derived from this software
 *    without specific prior written permission.
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS ``AS IS'' AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL IBM OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 *
 * Copyright Rusty Russell IBM Corporation 2007. */

/* This marks a buffer as continuing via the next field. */
#define VRING_DESC_F_NEXT   1
/* This marks a buffer as write-only (otherwise read-only). */
#define VRING_DESC_F_WRITE  2

/* The Host uses this in used->flags to advise the Guest: don't kick me when
 * you add a buffer.  It's unreliable, so it's simply an optimization.  Guest
 * will still kick if it's out of buffers. */
#define VRING_USED_F_NO_NOTIFY  1
/* The Guest uses this in avail->flags to advise the Host: don't interrupt me
 * when you consume a buffer.  It's unreliable, so it's simply an
 * optimization.  */
#define VRING_AVAIL_F_NO_INTERRUPT  1

/* Virtio ring descriptors: 16 bytes.  These can chain together via "next". */
struct vring_desc
{
    /* Address (guest-physical). */
    UInt32 addr;

    UInt32 padding; /* Because 64 bits is originally used for addr */

    /* Length. */
    UInt32 len;
    /* The flags as indicated above. */  //Optional for now!
    UInt16 flags;
    /* We chain unused descriptors via this, too */
    UInt16 next;
};

struct vring_avail
{
    UInt16 flags;
    UInt16 idx;
    UInt16 ring[256];
};

/* u32 is used here for ids for padding reasons. */
struct vring_used_elem
{
    /* Index of start of used descriptor chain. */
    UInt32 id;
    /* Total length of the descriptor chain which was used (written to) */
    UInt32 len;
};

struct vring_used
{
    UInt16 flags;
    UInt16 idx;
    struct vring_used_elem ring[256];
};

struct vring {
    unsigned int num;

    struct vring_desc *desc;

    struct vring_avail *avail;

    struct vring_used *used;
};

/*
 * When you boot you have to add all the buffers to your own A8
 * - Call add buf repeatedly
 * - When we take the buffer from the available list, you should have the pointer in the descriptor address
 *
 */

/* The standard layout for the ring is a continuous chunk of memory which looks
 * like this.  We assume num is a power of 2.
 *
 * struct vring
 * {
 *    // The actual descriptors (16 bytes each)
 *    struct vring_desc desc[num];
 *
 *    // A ring of available descriptor heads with free-running index.
 *    UInt16 avail_flags;
 *    UInt16 avail_idx;
 *    UInt16 available[num];
 *
 *    // Padding to the next page boundary.
 *    char pad[];
 *
 *    // A ring of used descriptor heads with free-running index.
 *    UInt16 used_flags;
 *    UInt16 used_idx;
 *    struct vring_used_elem used[num];
 * };
 */
static inline void vring_init(struct vring *vr, unsigned int num, void *p,
                              unsigned long pagesize)
{
    vr->num = num;
    vr->desc = p;
    vr->avail = (struct vring_avail *)
                    ((unsigned)p + (num * sizeof(struct vring_desc)));
    vr->used = (void *)(((unsigned long)&vr->avail->ring[num] + pagesize-1)
                & ~(pagesize - 1));
}

static inline unsigned vring_size(unsigned int num, unsigned long pagesize)
{
    return ((sizeof(struct vring_desc) * num + sizeof(UInt16) * (2 + num)
                + pagesize - 1) & ~(pagesize - 1))
                + sizeof(UInt16) * 2 + sizeof(struct vring_used_elem) * num;
}

#ifdef __KERNEL__
#include <linux/interrupt.h>
struct virtio_device;
struct virtqueue;

struct virtqueue *vring_new_virtqueue(unsigned int num,
                                      struct virtio_device *vdev,
                                      void *pages,
                                      void (*notify)(struct virtqueue *vq),
                                      void (*callback)(struct virtqueue *vq));

void vring_del_virtqueue(struct virtqueue *vq);
/* Filter out transport-specific feature bits. */
void vring_transport_features(struct virtio_device *vdev);

irqreturn_t vring_interrupt(int irq, void *_vq);
#endif /* __KERNEL__ */
#endif /* _LINUX_VIRTIO_RING_H */
