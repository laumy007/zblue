/*
 * Copyright (c) 2020 Intel Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_INCLUDE_SYS_MEM_MANAGE_H
#define ZEPHYR_INCLUDE_SYS_MEM_MANAGE_H

#include <sys/util.h>

/*
 * Caching mode definitions. These are mutually exclusive.
 */

/** No caching. Most drivers want this. */
#define K_MEM_CACHE_NONE	0

/** Write-through caching. Used by certain drivers. */
#define K_MEM_CACHE_WT		1

/** Full write-back caching. Any RAM mapped wants this. */
#define K_MEM_CACHE_WB		2

/** Reserved bits for cache modes in k_map() flags argument */
#define K_MEM_CACHE_MASK	(BIT(3) - 1)

/*
 * Region permission attributes. Default is read-only, no user, no exec
 */

/** Region will have read/write access (and not read-only) */
#define K_MEM_PERM_RW		BIT(3)

/** Region will be executable (normally forbidden) */
#define K_MEM_PERM_EXEC		BIT(4)

/** Region will be accessible to user mode (normally supervisor-only) */
#define K_MEM_PERM_USER		BIT(5)

/*
 * This is the offset to subtract from a virtual address mapped in the
 * kernel's permanent mapping of RAM, to obtain its physical address.
 *
 *     virt_addr - Z_VM_OFFSET = phys_addr
 *
 * This only works for virtual addresses within the interval
 * [CONFIG_KERNEL_VM_BASE, CONFIG_KERNEL_VM_BASE + (CONFIG_SRAM_SIZE * 1024)).
 *
 * These macros are intended for assembly, linker code, and static initializers.
 * Use with care.
 */
#ifdef CONFIG_MMU
#define Z_MEM_VM_OFFSET	(CONFIG_KERNEL_VM_BASE - CONFIG_SRAM_BASE_ADDRESS)
#else
#define Z_MEM_VM_OFFSET	0
#endif
#define Z_MEM_PHYS_ADDR(virt)	((virt) - Z_MEM_VM_OFFSET)
#define Z_MEM_VIRT_ADDR(phys)	((phys) + Z_MEM_VM_OFFSET)

#ifndef _ASMLANGUAGE
#include <stdint.h>
#include <stddef.h>
#include <inttypes.h>
#include <sys/__assert.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * Map a physical memory region into the kernel's virtual address space
 *
 * Given a physical address and a size, return a linear address
 * representing the base of where the physical region is mapped in
 * the virtual address space for the Zephyr kernel.
 *
 * This function alters the active page tables in the area reserved
 * for the kernel. This function will choose the virtual address
 * and return it to the caller.
 *
 * Portable code should never assume that phys_addr and linear_addr will
 * be equal.
 *
 * Once created, mappings are permanent.
 *
 * Caching and access properties are controlled by the 'flags' parameter.
 * Unused bits in 'flags' are reserved for future expansion.
 * A caching mode must be selected. By default, the region is read-only
 * with user access and code execution forbidden. This policy is changed
 * by passing K_MEM_CACHE_* and K_MEM_PERM_* macros into the 'flags' parameter.
 *
 * If there is insufficient virtual address space for the mapping, or
 * bad flags are passed in, or if additional memory is needed to update
 * page tables that is not available, this will generate a kernel panic.
 *
 * This API is only available if CONFIG_MMU is enabled.
 *
 * This API is part of infrastructure still under development and may
 * change.
 *
 * @param linear_addr [out] Output linear address storage location
 * @param phys_addr Physical address base of the memory region
 * @param size Size of the memory region
 * @param flags Caching mode and access flags, see K_MAP_* macros
 */
void k_mem_map(uint8_t **linear_addr, uintptr_t phys_addr, size_t size,
	       uint32_t flags);

/**
 * Given an arbitrary region, provide a aligned region that covers it
 *
 * The returned region will have both its base address and size aligned
 * to the provided alignment value.
 *
 * @param aligned_addr [out] Aligned address
 * @param aligned_size [out] Aligned region size
 * @param addr Region base address
 * @param size Region size
 * @param align What to align the address and size to
 * @retval offset between aligned_addr and addr
 */
size_t k_mem_region_align(uintptr_t *aligned_addr, size_t *aligned_size,
			  uintptr_t addr, size_t size, size_t align);

/* Just like Z_MEM_PHYS_ADDR() but with type safety and assertions */
static inline uintptr_t z_mem_phys_addr(void *virt)
{
	uintptr_t addr = (uintptr_t)virt;

#ifdef CONFIG_MMU
	__ASSERT((addr >= CONFIG_KERNEL_VM_BASE) &&
		 (addr < (CONFIG_KERNEL_VM_BASE + (CONFIG_SRAM_SIZE * 1024UL))),
		 "address %p not in permanent mappings", virt);
#else
	/* Should be identity-mapped */
	__ASSERT((addr >= CONFIG_SRAM_BASE_ADDRESS) &&
		 (addr < (CONFIG_SRAM_BASE_ADDRESS +
			  (CONFIG_SRAM_SIZE * 1024UL))),
		 "physical address 0x%lx not in RAM",
		 (unsigned long)addr);
#endif /* CONFIG_MMU */

	return Z_MEM_PHYS_ADDR(addr);
}

/* Just like Z_MEM_VIRT_ADDR() but with type safety and assertions */
static inline void *z_mem_virt_addr(uintptr_t phys)
{
	__ASSERT((phys >= CONFIG_SRAM_BASE_ADDRESS) &&
		 (phys < (CONFIG_SRAM_BASE_ADDRESS +
			  (CONFIG_SRAM_SIZE * 1024UL))),
		 "physical address 0x%lx not in RAM", (unsigned long)phys);

	return (void *)Z_MEM_VIRT_ADDR(phys);
}

#ifdef __cplusplus
}
#endif

#endif /* !_ASMLANGUAGE */
#endif /* ZEPHYR_INCLUDE_SYS_MEM_MANAGE_H */
