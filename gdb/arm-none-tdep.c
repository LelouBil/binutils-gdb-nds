/* none on ARM target support.

   Copyright (C) 2020-2024 Free Software Foundation, Inc.

   This file is part of GDB.

   This program is free software; you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation; either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.  */

#include "arm-tdep.h"
#include "arch-utils.h"
#include "extract-store-integer.h"
#include "regcache.h"
#include "elf-bfd.h"
#include "regset.h"
#include "user-regs.h"
#include "symfile.h"
#include "progspace.h"
#include "objfiles.h"
#include "gdbcore.h"

#ifdef HAVE_ELF
#include "elf-none-tdep.h"
#endif

/* Core file and register set support.  */
#define ARM_NONE_SIZEOF_GREGSET (18 * ARM_INT_REGISTER_SIZE)

/* Support VFP register format.  */
#define ARM_NONE_SIZEOF_VFP (32 * 8 + 4)

/* The index to access CPSR in user_regs as defined in GLIBC.  */
#define ARM_NONE_CPSR_GREGNUM 16

/* Supply register REGNUM from buffer GREGS_BUF (length LEN bytes) into
   REGCACHE.  If REGNUM is -1 then supply all registers.  The set of
   registers that this function will supply is limited to the general
   purpose registers.

   The layout of the registers here is based on the ARM GNU/Linux
   layout.  */

static void
arm_none_supply_gregset (const struct regset *regset,
			 struct regcache *regcache,
			 int regnum, const void *gregs_buf, size_t len)
{
  struct gdbarch *gdbarch = regcache->arch ();
  enum bfd_endian byte_order = gdbarch_byte_order (gdbarch);
  const gdb_byte *gregs = (const gdb_byte *) gregs_buf;

  for (int regno = ARM_A1_REGNUM; regno < ARM_PC_REGNUM; regno++)
    if (regnum == -1 || regnum == regno)
      regcache->raw_supply (regno, gregs + ARM_INT_REGISTER_SIZE * regno);

  if (regnum == ARM_PS_REGNUM || regnum == -1)
    {
      if (arm_apcs_32)
	regcache->raw_supply (ARM_PS_REGNUM,
			      gregs + ARM_INT_REGISTER_SIZE
			      * ARM_NONE_CPSR_GREGNUM);
      else
	regcache->raw_supply (ARM_PS_REGNUM,
			     gregs + ARM_INT_REGISTER_SIZE * ARM_PC_REGNUM);
    }

  if (regnum == ARM_PC_REGNUM || regnum == -1)
    {
      gdb_byte pc_buf[ARM_INT_REGISTER_SIZE];

      CORE_ADDR reg_pc
	= extract_unsigned_integer (gregs + ARM_INT_REGISTER_SIZE
				    * ARM_PC_REGNUM,
				    ARM_INT_REGISTER_SIZE, byte_order);
      reg_pc = gdbarch_addr_bits_remove (gdbarch, reg_pc);
      store_unsigned_integer (pc_buf, ARM_INT_REGISTER_SIZE, byte_order,
			      reg_pc);
      regcache->raw_supply (ARM_PC_REGNUM, pc_buf);
    }
}

/* Collect register REGNUM from REGCACHE and place it into buffer GREGS_BUF
   (length LEN bytes).  If REGNUM is -1 then collect all registers.  The
   set of registers that this function will collect is limited to the
   general purpose registers.

   The layout of the registers here is based on the ARM GNU/Linux
   layout.  */

static void
arm_none_collect_gregset (const struct regset *regset,
			  const struct regcache *regcache,
			  int regnum, void *gregs_buf, size_t len)
{
  gdb_byte *gregs = (gdb_byte *) gregs_buf;

  for (int regno = ARM_A1_REGNUM; regno < ARM_PC_REGNUM; regno++)
    if (regnum == -1 || regnum == regno)
      regcache->raw_collect (regno,
			     gregs + ARM_INT_REGISTER_SIZE * regno);

  if (regnum == ARM_PS_REGNUM || regnum == -1)
    {
      if (arm_apcs_32)
	regcache->raw_collect (ARM_PS_REGNUM,
			       gregs + ARM_INT_REGISTER_SIZE
			       * ARM_NONE_CPSR_GREGNUM);
      else
	regcache->raw_collect (ARM_PS_REGNUM,
			       gregs + ARM_INT_REGISTER_SIZE * ARM_PC_REGNUM);
    }

  if (regnum == ARM_PC_REGNUM || regnum == -1)
    regcache->raw_collect (ARM_PC_REGNUM,
			   gregs + ARM_INT_REGISTER_SIZE * ARM_PC_REGNUM);
}

/* Supply VFP registers from REGS_BUF into REGCACHE.  */

static void
arm_none_supply_vfp (const struct regset *regset,
		     struct regcache *regcache,
		     int regnum, const void *regs_buf, size_t len)
{
  const gdb_byte *regs = (const gdb_byte *) regs_buf;

  if (regnum == ARM_FPSCR_REGNUM || regnum == -1)
    regcache->raw_supply (ARM_FPSCR_REGNUM, regs + 32 * 8);

  for (int regno = ARM_D0_REGNUM; regno <= ARM_D31_REGNUM; regno++)
    if (regnum == -1 || regnum == regno)
      regcache->raw_supply (regno, regs + (regno - ARM_D0_REGNUM) * 8);
}

/* Collect VFP registers from REGCACHE into REGS_BUF.  */

static void
arm_none_collect_vfp (const struct regset *regset,
		      const struct regcache *regcache,
		      int regnum, void *regs_buf, size_t len)
{
  gdb_byte *regs = (gdb_byte *) regs_buf;

  if (regnum == ARM_FPSCR_REGNUM || regnum == -1)
    regcache->raw_collect (ARM_FPSCR_REGNUM, regs + 32 * 8);

  for (int regno = ARM_D0_REGNUM; regno <= ARM_D31_REGNUM; regno++)
    if (regnum == -1 || regnum == regno)
      regcache->raw_collect (regno, regs + (regno - ARM_D0_REGNUM) * 8);
}

/* The general purpose register set.  */

static const struct regset arm_none_gregset =
  {
    nullptr, arm_none_supply_gregset, arm_none_collect_gregset
  };

/* The VFP register set.  */

static const struct regset arm_none_vfpregset =
  {
    nullptr, arm_none_supply_vfp, arm_none_collect_vfp
  };

/* Iterate over core file register note sections.  */

static void
arm_none_iterate_over_regset_sections (struct gdbarch *gdbarch,
				       iterate_over_regset_sections_cb *cb,
				       void *cb_data,
				       const struct regcache *regcache)
{
  arm_gdbarch_tdep *tdep = gdbarch_tdep<arm_gdbarch_tdep> (gdbarch);

  cb (".reg", ARM_NONE_SIZEOF_GREGSET, ARM_NONE_SIZEOF_GREGSET,
      &arm_none_gregset, nullptr, cb_data);

  if (tdep->vfp_register_count > 0)
    cb (".reg-arm-vfp", ARM_NONE_SIZEOF_VFP, ARM_NONE_SIZEOF_VFP,
	&arm_none_vfpregset, "VFP floating-point", cb_data);
}

// custom overlay updater for NDS
static void nds_overlay_update(struct obj_section *osect);
static int simple_overlay_update_1 (struct obj_section *osect);
static int simple_read_overlay_table (void);
/* Cached, dynamically allocated copies of the target data structures: */
static unsigned (*cache_ovly_table)[4] = 0;
static unsigned cache_novlys = 0;
static char cache_ovly_name[16] = {0};
static CORE_ADDR cache_ovly_table_base = 0;
enum ovly_index
  {
    VMA, OSIZE, LMA, MAPPED
  };

static void
simple_free_overlay_table (void)
{
  xfree (cache_ovly_table);
  cache_novlys = 0;
  cache_ovly_table = NULL;
  cache_ovly_table_base = 0;
}

static void
read_target_long_array (CORE_ADDR memaddr, unsigned int *myaddr,
			int len, int size, enum bfd_endian byte_order)
{
  /* FIXME (alloca): Not safe if array is very large.  */
  gdb_byte *buf = (gdb_byte *) alloca (len * size);
  int i;

  read_memory (memaddr, buf, len * size);
  for (i = 0; i < len; i++)
    myaddr[i] = extract_unsigned_integer (size * i + buf, size, byte_order);
}

static int
simple_overlay_update_1 (struct obj_section *osect)
{
  int i;
  asection *bsect = osect->the_bfd_section;
  struct gdbarch *gdbarch = osect->objfile->arch ();
  int word_size = gdbarch_long_bit (gdbarch) / TARGET_CHAR_BIT;
  enum bfd_endian byte_order = gdbarch_byte_order (gdbarch);
  char sectName[sizeof(cache_ovly_name)];

  for (i = 0; i < cache_novlys; i++) {
    #pragma GCC diagnostic push
    #pragma GCC diagnostic ignored "-Wformat-nonliteral"
    sprintf(sectName, cache_ovly_name, i);
    #pragma GCC diagnostic pop
    if(cache_ovly_table[i][MAPPED] != 0 && strcmp(sectName, bsect->name) == 0) {
      read_target_long_array (cache_ovly_table_base + i * word_size,
				(unsigned int *) cache_ovly_table[i],
				4, word_size, byte_order);
      if(cache_ovly_table[i][MAPPED] != 0 && strcmp(sectName, bsect->name) == 0) {
	osect->ovly_mapped = cache_ovly_table[i][MAPPED];
	// a horrific hack which should never be allowed.
	osect->the_bfd_section->size = cache_ovly_table[i][OSIZE];
	return 1;
      } else { /* Warning!  Warning!  Target's ovly table has changed!  */
	return 0;
      }
    }
  }
  return 0;
}

// http://stanshebs.github.io/gdb-doxy-test/gdb-xref/corefile_8c_source.html
static void read_memory_string(CORE_ADDR memaddr, char *buffer, int max_len) {
  char *cp;
  int i;
  int cnt;

  cp = buffer;
  while (true) {
    if (cp - buffer >= max_len) {
      buffer[max_len - 1] = '\0';
      break;
    }

    cnt = max_len - (cp - buffer);
    if (cnt > 8) cnt = 8;
    read_memory(memaddr + (int) (cp - buffer), (gdb_byte *) cp, cnt);
    for (i = 0; i < cnt && *cp; i++, cp++);
    if (i < cnt && !*cp) break;
  }
}

static int
simple_read_overlay_table (void)
{
  struct gdbarch *gdbarch;
  int word_size;
  enum bfd_endian byte_order;

  simple_free_overlay_table ();
  bound_minimal_symbol novlys_msym
    = lookup_minimal_symbol (current_program_space, "_novlys");
  if (! novlys_msym.minsym)
    {
      error (_("Error reading inferior's overlay table: "
	     "couldn't find `_novlys' variable\n"
	     "in inferior.  Use `overlay manual' mode."));
      return 0;
    }

  bound_minimal_symbol ovly_name_msym
    = lookup_minimal_symbol (current_program_space, "_ovly_name");
  if (! ovly_name_msym.minsym)
    {
      error (_("Error reading inferior's overlay table: "
	     "couldn't find `_ovly_name' variable\n"
	     "in inferior.  Use `overlay manual' mode."));
      return 0;
    }

  bound_minimal_symbol ovly_table_msym
    = lookup_minimal_symbol (current_program_space, "_ovly_table");
  if (! ovly_table_msym.minsym)
    {
      error (_("Error reading inferior's overlay table: couldn't find "
	     "`_ovly_table' array\n"
	     "in inferior.  Use `overlay manual' mode."));
      return 0;
    }

  gdbarch = ovly_table_msym.objfile->arch ();
  word_size = gdbarch_long_bit (gdbarch) / TARGET_CHAR_BIT;
  byte_order = gdbarch_byte_order (gdbarch);

  cache_novlys = read_memory_integer (novlys_msym.value_address (),
				      4, byte_order);
  read_memory_string (ovly_name_msym.value_address(), cache_ovly_name, sizeof(cache_ovly_name));

  cache_ovly_table
    = (unsigned int (*)[4]) xmalloc (cache_novlys * sizeof (*cache_ovly_table));
  cache_ovly_table_base = ovly_table_msym.value_address ();
  read_target_long_array (cache_ovly_table_base,
			  (unsigned int *) cache_ovly_table,
			  cache_novlys * 4, word_size, byte_order);

  return 1;			/* SUCCESS */
}

static void nds_overlay_update(struct obj_section *osect)
{
  char sectName[sizeof(cache_ovly_name)];
  /* Were we given an osect to look up?  NULL means do all of them.  */
  if (osect)
    /* Have we got a cached copy of the target's overlay table?  */
    if (cache_ovly_table != NULL)
      {
	/* Does its cached location match what's currently in the
	   symtab?  */
	bound_minimal_symbol minsym
	  = lookup_minimal_symbol (current_program_space, "_ovly_table");

	if (minsym.minsym == NULL)
	  error (_("Error reading inferior's overlay table: couldn't "
		   "find `_ovly_table' array\n"
		   "in inferior.  Use `overlay manual' mode."));

	if (cache_ovly_table_base == minsym.value_address ())
	  /* Then go ahead and try to look up this single section in
	     the cache.  */
	  if (simple_overlay_update_1 (osect))
	    /* Found it!  We're done.  */
	    return;
      }

  /* Cached table no good: need to read the entire table anew.
     Or else we want all the sections, in which case it's actually
     more efficient to read the whole table in one block anyway.  */

  if (! simple_read_overlay_table ())
    return;

  /* Now may as well update all sections, even if only one was requested.  */
  for (objfile *objfile : current_program_space->objfiles ()) {
    for (obj_section *sect : objfile->sections ()) {
      if (section_is_overlay (sect))
	{
	  int i;
	  asection *bsect = sect->the_bfd_section;

	  for (i = 0; i < cache_novlys; i++) {
	    #pragma GCC diagnostic push
	    #pragma GCC diagnostic ignored "-Wformat-nonliteral"
	    sprintf(sectName, cache_ovly_name, i);
	    #pragma GCC diagnostic pop
	    if(strcmp(sectName, bsect->name) == 0) {
	      if (sect->ovly_mapped != cache_ovly_table[i][MAPPED]) {
		gdb_printf(_("Overlays: section with name %s applying mapping %d\n"), sectName, cache_ovly_table[i][MAPPED]);
	      }
	      sect->ovly_mapped = cache_ovly_table[i][MAPPED];
	      // a horrific hack which should never be allowed.
	      sect->the_bfd_section->size = cache_ovly_table[i][OSIZE];
	      break;
	    }
	  }
	}
    }
  }
}

/* Initialize ARM bare-metal ABI info.  */

static void
arm_none_init_abi (struct gdbarch_info info, struct gdbarch *gdbarch)
{
#ifdef HAVE_ELF
  elf_none_init_abi (gdbarch);
#endif

  /* Iterate over registers for reading and writing bare metal ARM core
     files.  */
  set_gdbarch_iterate_over_regset_sections
    (gdbarch, arm_none_iterate_over_regset_sections);

  /* Support custom overlay manager.  */
  set_gdbarch_overlay_update (gdbarch, nds_overlay_update);
}

/* Initialize ARM bare-metal target support.  */

void _initialize_arm_none_tdep ();
void
_initialize_arm_none_tdep ()
{
  gdbarch_register_osabi (bfd_arch_arm, 0, GDB_OSABI_NONE,
			  arm_none_init_abi);
}
