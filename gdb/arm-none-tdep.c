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

#include <map>
#include <fstream>

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

static void nds_overlay_update(struct obj_section *osect);
static void nds_overlay_mapping(char * filename);
static int simple_read_overlay_table (void);

static unsigned (*cache_ovly_table)[4] = 0;
static unsigned cache_novlys = 0;
static CORE_ADDR cache_ovly_table_base = 0;
enum ovly_index
  {
    VMA, OSIZE, FS_OVERLAY_ID, MAPPED
  };

static std::map<int, obj_section *> ovly_id_map;
static std::map<std::string, obj_section *> ovly_source_map;

static void
simple_free_overlay_table (void)
{
  xfree (cache_ovly_table);
  cache_novlys = 0;
  cache_ovly_table = NULL;
  cache_ovly_table_base = 0;
}

static void read_target_long_array (CORE_ADDR memaddr, unsigned int *myaddr, int len, int size, enum bfd_endian byte_order)
{
  /* FIXME (alloca): Not safe if array is very large.  */
  gdb_byte *buf = (gdb_byte *) alloca (len * size);
  read_memory (memaddr, buf, len * size);
  for (int idx = 0; idx < len; idx++)
  {
    myaddr[idx] = extract_unsigned_integer (size * idx + buf, size, byte_order);
  }
}

static int simple_read_overlay_table (void)
{
  struct gdbarch *gdbarch;
  int word_size;
  enum bfd_endian byte_order;

  simple_free_overlay_table ();
  bound_minimal_symbol novlys_msym = lookup_minimal_symbol (current_program_space, "_novlys");
  if (!novlys_msym.minsym)
  {
    error (_("Error reading inferior's overlay table: "
      "couldn't find `_novlys' variable\n"
      "in inferior.  Use `overlay manual' mode."));
    return 0;
  }

  bound_minimal_symbol ovly_table_msym = lookup_minimal_symbol (current_program_space, "_ovly_table");
  if (!ovly_table_msym.minsym)
  {
    error (_("Error reading inferior's overlay table: couldn't find "
      "`_ovly_table' array\n"
      "in inferior.  Use `overlay manual' mode."));
    return 0;
  }

  gdbarch = ovly_table_msym.objfile->arch ();
  word_size = gdbarch_long_bit (gdbarch) / TARGET_CHAR_BIT;
  byte_order = gdbarch_byte_order (gdbarch);

  cache_novlys = read_memory_integer (novlys_msym.value_address (), 4, byte_order);

  cache_ovly_table = (unsigned int (*)[4]) xmalloc (cache_novlys * sizeof (*cache_ovly_table));
  cache_ovly_table_base = ovly_table_msym.value_address ();
  read_target_long_array (cache_ovly_table_base,
			  (unsigned int *) cache_ovly_table,
			  cache_novlys * 4, word_size, byte_order);

  return 1;			/* SUCCESS */
}

static void nds_overlay_update(struct obj_section *osect)
{
  // to simplify, overlay table is not cached. this might bite us in the future.
  if (! simple_read_overlay_table ())
    return;

  // this iterates over cache_novlys in the outer loop and obj_section in the inner loop.
  // this is reversed from simple_overlay_update. why? because pokeplat (and presumably other NDS games)
  // generate multiple debug sections for each overlay (one points at start addr, one points at end)
  // because of this, there will be 2 sections matching any loaded overlay, but we only want to load the first.
  // therefore, we find the FIRST matching section, then break, and skip the remaining sections for that ovly.
  for (int idx = 0; idx < cache_novlys; idx++) 
  {
    int fs_id = cache_ovly_table[idx][FS_OVERLAY_ID];
    for (objfile *objfile : current_program_space->objfiles ()) 
    {
      for (obj_section *sect : objfile->sections ()) 
      {
        if (section_is_overlay (sect) && cache_ovly_table[idx][OSIZE] != 0 && ovly_id_map[fs_id] == sect) 
        {
          sect->ovly_mapped = cache_ovly_table[idx][MAPPED];
          // a horrific hack which should never be allowed. unfortunately it's required!
          sect->the_bfd_section->size = cache_ovly_table[idx][OSIZE];
          sect->set_offset(0);
          break;
        }
      }
    }
  }
}

// this is SUPER flaky but should work fine with well-formed inputs
static void nds_overlay_mapping(char * filename)
{
  std::string line;
  std::ifstream mapfile(filename);
  if (mapfile.is_open()) {
    while (std::getline(mapfile, line)) {
      if (line.rfind("OVERLAY ", 0) == 0)
      {
        std::string sub = line.substr(8);
        int index = sub.find(":");

        std::string section_name = sub.substr(0, index);
        int ovly_id = std::stoi(sub.substr(index + 1));

        for (objfile *objfile : current_program_space->objfiles ()) {
          for (obj_section *sect : objfile->sections ()) {
            if (section_is_overlay (sect) && strcmp(sect->the_bfd_section->name, section_name.c_str()) == 0) {
              ovly_id_map[ovly_id] = sect;
              break;
            }
          }
        }
      }
      else if (line.rfind("SOURCE ", 0) == 0)
      {
        std::string sub = line.substr (7);
        int index = sub.find(":");

        std::string source_name = sub.substr(0, index);
        std::string section_name = sub.substr(index + 1);

        for (objfile *objfile : current_program_space->objfiles ()) {
          for (obj_section *sect : objfile->sections ()) {
            if (section_is_overlay (sect) && strcmp(sect->the_bfd_section->name, section_name.c_str()) == 0) {
              ovly_source_map[source_name] = sect;
              break;
            }
          }
        }
      }
    }
    mapfile.close();
  }

  breakpoint_re_set ();
  insert_breakpoints ();
}

static obj_section *
nds_overlay_source(const char * source)
{
  std::map<std::string, obj_section *>::iterator search = ovly_source_map.find(std::string(source));
  if (search == ovly_source_map.end()) return NULL;
  return search->second;
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
  set_gdbarch_overlay_mapping (gdbarch, nds_overlay_mapping);
  set_gdbarch_overlay_source (gdbarch, nds_overlay_source);
  // if cached_map_file is set, try to initialize map
  refresh_cached_overlay_map(gdbarch);
}

/* Initialize ARM bare-metal target support.  */

void _initialize_arm_none_tdep ();
void
_initialize_arm_none_tdep ()
{
  gdbarch_register_osabi (bfd_arch_arm, 0, GDB_OSABI_NONE,
			  arm_none_init_abi);
}
