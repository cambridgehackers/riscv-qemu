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
#include <fcntl.h>
#include <unistd.h>
#include <sys/stat.h>
#include <inttypes.h>
#include <dlfcn.h>

struct FpgaOps {
    uint64_t (*read)(hwaddr addr);
    void (*write)(hwaddr addr, uint64_t value);
    void (*close)(void);
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

    target_ulong pc, cs_base;
    int flags;
    cpu_get_tb_cpu_state(fpgadevstate->env, &pc, &cs_base, &flags);

    fprintf(stderr, "fpgadev_mm_read pc=%lx addr=%lx -> value=%lx\n", pc, addr, value);
    return value;
}

// CPU wrote to an Fpgadev register
static void fpgadev_mm_write(void *opaque, hwaddr addr,
                            uint64_t value, unsigned size)
{
    FpgadevState *fpgadevstate = opaque;
    fprintf(stderr, "fpgadev_mm_write addr=%lx value=%lx\n", addr, value);
    fpgadevstate->ops->write(addr, value);
}

static const MemoryRegionOps fpgadev_mm_ops[3] = {
    [DEVICE_LITTLE_ENDIAN] = {
        .read = fpgadev_mm_read,
        .write = fpgadev_mm_write,
        .endianness = DEVICE_LITTLE_ENDIAN,
    },
};

FpgadevState *fpgadev_mm_init(MemoryRegion *address_space, hwaddr base, qemu_irq irq,
                              MemoryRegion *main_mem, CPURISCVState *env, const char * name)
{
    FpgadevState *fpgadevstate;

    fpgadevstate = g_malloc0(sizeof(FpgadevState));
    fpgadevstate->irq = irq;
    fpgadevstate->address_space = address_space;
    fpgadevstate->env = env;

    char * namebuf = g_malloc0(sizeof(char)*100);
    sprintf(namebuf, "%s%s", "fpgadev", name);
    fpgadevstate->name = namebuf;

    fpgadevstate->lib = dlopen("./connectal.so", RTLD_NOW);
    if (fpgadevstate->lib == NULL) {
        fprintf(stderr, "failed to open library: %s\n", dlerror());
    } else {
        struct FpgaOps *(*fpgadev_init)(void);
        fpgadev_init = dlsym(fpgadevstate->lib, "fpgadev_init");
        fprintf(stderr, "connectal.so fpgadev_init=%p\n", fpgadev_init);
        if (fpgadev_init)
            fpgadevstate->ops = fpgadev_init();
    }

    vmstate_register(NULL, base, &vmstate_fpgadev, fpgadevstate);

    memory_region_init_io(&fpgadevstate->io, NULL,
                          &fpgadev_mm_ops[DEVICE_LITTLE_ENDIAN],
                          fpgadevstate, namebuf, 65536);
    memory_region_add_subregion(address_space, base, &fpgadevstate->io);

    return fpgadevstate;
}
