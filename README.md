# README

This is a custom build of GDB with changes to support debugging overlays in NDS ROMs.

In order to support this there are changes related to ARM architecture debugging, see `gdb/arm-none-tdep.c`.

## How to build

1. Pull latest contents of this repo
2. `CC=gcc ./configure --enable-targets=all --with-curses` (you may or may not need `--with-curses`; `--enable-targets=all` ensures you get a multiarch build)
3. `CC=gcc make`

OR, use the prebuilt binary uploaded to the `Releases` section. It was built against Ubuntu 24.04 and may work as-is.

## How to integrate

This build of GDB supports an overlay system similar to, but different from, the standard GDB overlay debugging system.

Essentially, your ROM needs to expose a couple of variables for GDB to be able to read the overlay state:

```
#define MAX_OVERLAYS 128 // convenience, not required
extern unsigned long _novlys; // maximum number of overlays in the overlay table
extern struct_overlayTable _ovly_table[MAX_OVERLAYS]; // overlay table initialized with the maximum number of overlays
```

`struct_overlayTable` contains details about each overlay which has been loaded:

```
typedef struct {
    unsigned long vma; // address in RAM that the overlay is loaded into
    unsigned long size; // size of the overlay in RAM (in bytes)
    FSOverlayID id; // ID of the overlay in your ROM's overlay management system (FSOverlayID == int in NitroSDK)
    unsigned long mapped; // if non-zero, the overlay with ID `id` is currently loaded
} struct_overlayTable;
```

You may also specify an overlay event function, which GDB will insert a hidden breakpoint into. Whenever the function is called, GDB will automatically refresh the overlay state from `_ovly_table`.

```
static void _ovly_debug_event(void) {}
```

In your ROM's overlay management code, you should update `_ovly_table` (and then optionally call `_ovly_debug_event`) whenever your overlays are loaded or unloaded. NitroSDK provides us with values for `vma` and `size` fields directly, which is convenient, but other overlay management systems (if any exist) should also be able to expose these. See the examples below:

```
// helper function to mark a specific overlay as unmapped.
void UnloadOverlayGDB(const FSOverlayID overlayID)
{
    GF_ASSERT(overlayID < _novlys);
    _ovly_table[overlayID].mapped--;
    _ovly_debug_event();
}
// helper function to mark a specific overlay as mapped, and provide its RAM address and size to GDB.
void LoadOverlayGDB(const FSOverlayID overlayID)
{
    FSOverlayInfo overlayInfo;

    GF_ASSERT(overlayID < _novlys);

    // 1. fetch overlay info to identify vma
    GF_ASSERT(FS_LoadOverlayInfo(&overlayInfo, MI_PROCESSOR_ARM9, overlayID) == TRUE);

    // 2. add entry to _ovly_table
    _ovly_table[overlayID].vma = overlayInfo.header.ram_address;
    _ovly_table[overlayID].id = overlayID;
    _ovly_table[overlayID].size = overlayInfo.header.ram_size;
    _ovly_table[overlayID].mapped++;
    _ovly_debug_event();
}
```

## Supporting content - overlay map

GDB reads the data from `_ovly_table` in your ROM's memory and identifies which overlays are loaded based on the `id` field. But GDB (currently, see issue 3, this needs to change long-term) needs a little bit of extra information in order to identify what section in the debug info corresponds to which overlay ID. To provide this, we can pass an overlay map to GDB.

The overlay map file is just a text file mapping overlay IDs to section names, and file names to section names. Whenever an overlay with a specific ID (e.g. `1`) is marked as loaded by the ROM, GDB looks for that ID in this map to identify what section is now in-memory. When a breakpoint is hit, GDB consults this map to confirm if the file corresponding to the breakpoint is in a loaded overlay section. See an excerpt below:

```
OVERLAY overlay0:0
OVERLAY overlay1:1
OVERLAY overlay2:2
OVERLAY overlay3:3
OVERLAY overlay4:4
OVERLAY overlay5:5

SOURCE ov0_dummy.c:overlay0
SOURCE ov1_021D0D80.c:overlay1
SOURCE ov2_dummy.c:overlay2
SOURCE ov3_dummy.c:overlay3
SOURCE ov4_021D0D80.c:overlay4
SOURCE ov4_021D2808.c:overlay4
SOURCE fieldmap.c:overlay5
```

I'd recommend building this in your build system automatically (see e.g. https://github.com/joshua-smith-12/pokeplatinum/blob/gdb/tools/debug/overlay_mapper.py)

## Commands

In GDB, run these commands to enable overlay debugging:

```
overlay auto
overlay map <path/to/map/file>
```

Or review the launch.json here for VSCode integration: https://github.com/joshua-smith-12/pokeplatinum/blob/gdb/.vscode/launch.json
