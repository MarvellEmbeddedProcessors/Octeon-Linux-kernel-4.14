#ifndef _ASM_MACH_CAVIUM_OCTEON_PHYSMEM_H
#define _ASM_MACH_CAVIUM_OCTEON_PHYSMEM_H

#ifdef CONFIG_NUMA
#define MAX_PHYSMEM_BITS        41
#else
#define MAX_PHYSMEM_BITS        40
#endif

#endif /* _ASM_MACH_CAVIUM_OCTEON_PHYSMEM_H */
