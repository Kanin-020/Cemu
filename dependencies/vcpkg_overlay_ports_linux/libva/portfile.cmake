include("${VCPKG_ROOT_DIR}/ports/libva/portfile.cmake")

# patchelf 0.15.5 leaves DT_INIT pointing into replaced data when relocating libva's .init section.
set(VCPKG_FIXUP_ELF_RPATH OFF)
