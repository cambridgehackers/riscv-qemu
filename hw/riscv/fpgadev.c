/*
 * QEMU RISC-V FPGA Devices
 *
 * Author: Jamey Hicks, jamey.hicks@gmail.com
 *
 * This module provides shim devices redirect to devices implemented by an FPGA.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
 * THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */

#include "hw/riscv/fpgadev.h"
#include "exec/address-spaces.h"
#include "qemu/error-report.h"
#include "sysemu/kvm.h"
#include <fcntl.h>
#include <unistd.h>
#include <sys/stat.h>
#include <inttypes.h>
#include <dlfcn.h>

struct FpgaOps {
    uint64_t (*read)(hwaddr addr);
    void (*write)(hwaddr addr, uint64_t value);
    void (*close)(void);
    void *(*alloc_mem)(size_t size);
};

const VMStateDescription vmstate_fpgadev = {
    .name = "fpgadev",
    .version_id = 1,
    .minimum_version_id = 1,
    .fields      = (VMStateField []) {
        VMSTATE_END_OF_LIST()
    },
};

static uint64_t fpgadev_mm_read(void *opaque, hwaddr addr, unsigned size)
{
    FpgadevState *fpgadevstate = opaque;
    uint64_t value = fpgadevstate->ops->read(addr);

    if (0) {
        target_ulong pc, cs_base;
        int flags;
        cpu_get_tb_cpu_state(fpgadevstate->env, &pc, &cs_base, &flags);

        if (addr >= 0x2000 && addr < 0x5500)
            fprintf(stderr, "fpgadev_mm_read  pc=%lx addr=%lx size=%d  reading ...\n", pc, addr, size);

        if (addr == 0x1018) {
            uint64_t ipr = fpgadevstate->ops->read(0x1004);
            if (ipr & (1 << 5)) {
                uint64_t iic_isr = fpgadevstate->ops->read(0x3020);
                uint64_t iic_ier = fpgadevstate->ops->read(0x3028);
                fprintf(stderr, "fpgadev_mm_read pc=%lx addr=%lx -> value=%lx ipr=%lx iic isr=%lx ier=%lx\n", pc, addr, value, ipr, iic_isr, iic_ier);
            }
        }
        if (addr >= 0x2000 && addr < 0x5500)
            fprintf(stderr, "fpgadev_mm_read  pc=%lx addr=%lx size=%d  read value=%08lx\n", pc, addr, size, value);
    }
    return value;
}

// CPU wrote to an Fpgadev register
static void fpgadev_mm_write(void *opaque, hwaddr addr,
                            uint64_t value, unsigned size)
{
    FpgadevState *fpgadevstate = opaque;
    if (0) {
        target_ulong pc, cs_base;
        int flags;
        cpu_get_tb_cpu_state(fpgadevstate->env, &pc, &cs_base, &flags);
        if (addr >= 0x2000 && addr < 0x5500)
            fprintf(stderr, "fpgadev_mm_write pc=%lx addr=%lx size=%d wrote value=%08lx\n", pc, addr, size, value);
    }
    fpgadevstate->ops->write(addr, value);
}

static const MemoryRegionOps fpgadev_mm_ops[3] = {
    [DEVICE_LITTLE_ENDIAN] = {
        .read = fpgadev_mm_read,
        .write = fpgadev_mm_write,
        .endianness = DEVICE_LITTLE_ENDIAN,
    },
};

static FpgadevState *fpgadevstate;
static void fpgadev_irq_callback(int irq)
{
    CPURISCVState *env = fpgadevstate->env;
    //RISCVCPU *cpu = riscv_env_get_cpu(env);
    if (irq != 0) {
        env->csr[NEW_CSR_MIP] |= MIP_MXIP;
        env->csr[NEW_CSR_MIP] |= MIP_SXIP;
        qemu_irq_raise(fpgadevstate->irq);
    } else {
        env->csr[NEW_CSR_MIP] &= ~MIP_MXIP;
        env->csr[NEW_CSR_MIP] &= ~MIP_SXIP;
        qemu_irq_lower(fpgadevstate->irq);
    }
    if (0)
    fprintf(stderr, "fpgadev irq level change irq=%d mip=%08lx sip=%08lx mie=%08lx sie=%08lx\n",
            irq, env->csr[NEW_CSR_MIP], env->csr[NEW_CSR_SIP], env->csr[NEW_CSR_MIE], env->csr[NEW_CSR_SIE]);
}

static void *(fpgadev_alloc_mem)(size_t size, uint64_t *align)
{
    if (align)
        size += *align;
    fprintf(stderr, "fpgadev_alloc_mem size=%08lx align=%08lx\n", size, (align ? *align : 0));
    void *ptr = fpgadevstate->ops->alloc_mem(size);
    return ptr;
}

void fpgadev_init_fpga(void)
{
    fpgadevstate = g_malloc0(sizeof(FpgadevState));

    fpgadevstate->lib = dlopen("./connectal.so", RTLD_NOW);
    if (fpgadevstate->lib == NULL) {
        fprintf(stderr, "failed to open library: %s\n", dlerror());
    } else {
        struct FpgaOps *(*fpgadev_init)(void (*irqCallback)(int irq));
        fpgadev_init = dlsym(fpgadevstate->lib, "fpgadev_init");
        fprintf(stderr, "connectal.so fpgadev_init=%p\n", fpgadev_init);
        if (fpgadev_init)
            fpgadevstate->ops = fpgadev_init(fpgadev_irq_callback);
    }

    phys_mem_set_alloc(fpgadev_alloc_mem);
}

FpgadevState *fpgadev_mm_init(MemoryRegion *address_space, hwaddr base, qemu_irq irq,
                              MemoryRegion *main_mem, CPURISCVState *env, const char * name)
{
    fpgadevstate->irq = irq;
    fpgadevstate->address_space = address_space;
    fpgadevstate->env = env;

    char * namebuf = g_malloc0(sizeof(char)*100);
    sprintf(namebuf, "%s%s", "fpgadev", name);
    fpgadevstate->name = namebuf;

    vmstate_register(NULL, base, &vmstate_fpgadev, fpgadevstate);

    memory_region_init_io(&fpgadevstate->io, NULL,
                          &fpgadev_mm_ops[DEVICE_LITTLE_ENDIAN],
                          fpgadevstate, namebuf, 65536);
    memory_region_add_subregion(address_space, base, &fpgadevstate->io);

    return fpgadevstate;
}
