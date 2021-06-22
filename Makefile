boot_programs :=
run_programs :=
libraries :=
boot_programs_d :=
run_programs_d :=
libraries_d :=
sources :=
build_folders :=
build_folders_d :=

build_dir := release
build_dir_d := debug

.PHONY: all folders folders_d release debug release-run release-boot \
debug-run debug-boot dist-clean release-dist-clean debug-dist-clean \
flash-mb2016-boot flash-mb2016-run flash-mb2019-boot flash-mb2019-run flash-bs2018-boot flash-bs2018-run

all: boot run

export

ifeq ($(OS),Windows_NT)
MKDIR := bin/mkdir.exe
RM := bin/rm.exe
OPENOCD := openocd.exe
CAT := bin/cat.exe
else
MKDIR := mkdir
RM := rm
OPENOCD := openocd
CAT := cat
endif

define make-prog
$(local_run_program): $(local_objs) $(local_libs)
	$(LD) $(local_arch) -Wl,-Map=$$(patsubst %.elf,%.map,$$@),--gc-sections -T$(1)/run.ld \
	 --specs=nano.specs $(LDFLAGS) $(local_objs) -Wl,--start-group -lc -lm $(local_libs) -Wl,--end-group $(local_ld_flags) -o $$@
	$(OBJCOPY) $(OBJCOPYFLAGS) $$@ $$(patsubst %.elf,%.bin,$$@)
#	$(OBJDUMP) $(OBJDUMPFLAGS) $$@ > $$(patsubst %.elf,%.lss,$$@)

$(local_boot_program): $(local_boot_objs) $(local_libs)
	$(LD) $(local_arch) -Wl,-Map=$$(patsubst %.elf,%.map,$$@),--gc-sections -T$(1)/boot.ld \
	 --specs=nano.specs $(LDFLAGS) $(local_boot_objs) -Wl,--start-group -lc -lm $(local_libs) -Wl,--end-group -o $$@
	$(OBJCOPY) $(OBJCOPYFLAGS) $$@ $$(patsubst %.elf,%.bin,$$@)

$(local_run_program_d): $(local_objs_d) $(local_libs_d)
	$(LD) $(local_arch) -Wl,-Map=$$(patsubst %.elf,%.map,$$@),--gc-sections -T$(1)/run.ld \
	 --specs=nano.specs $(LDFLAGS) $(local_objs_d) -Wl,--start-group -lc -lm $(local_libs_d) -Wl,--end-group $(local_ld_flags) -o $$@
	$(OBJCOPY) $(OBJCOPYFLAGS) $$@ $$(patsubst %.elf,%.bin,$$@)

$(local_boot_program_d): $(local_boot_objs_d) $(local_libs_d)
	$(LD) $(local_arch) -Wl,-Map=$$(patsubst %.elf,%.map,$$@),--gc-sections -T$(1)/boot.ld \
	 --specs=nano.specs $(LDFLAGS) $(local_boot_objs_d) -Wl,--start-group -lc -lm $(local_libs_d) -Wl,--end-group -o $$@
	$(OBJCOPY) $(OBJCOPYFLAGS) $$@ $$(patsubst %.elf,%.bin,$$@)
endef

# $(call make-objects-c, local_build_dir, local_dir, src_dir, params)
define make-objects
$(build_dir)/$(1)/$(3)/%.o: $(2)/$(3)/%.s
	$(AS) $(local_arch) $(ASFLAGS) $(local_flags) $(4) -c $$< -o $$@
	@$(AS) -MM -MP -MT $$@ -MF $(build_dir)/$(1)/$(3)/$$*.d $(local_arch) $(ASFLAGS) $(local_flags) $(4) $$<

$(build_dir)/$(1)/$(3)/%.o: $(2)/$(3)/%.c
	$(CC) $(local_arch) $(CFLAGS) $(local_flags) $(4) -c $$< -o $$@
	@$(CC) -MM -MP -MT $$@ -MF $(build_dir)/$(1)/$(3)/$$*.d $(local_arch) $(CFLAGS) $(local_flags) $(4) $$<

$(build_dir)/$(1)/$(3)/%.o: $(2)/$(3)/%.cpp
	$(CXX) $(local_arch) $(CXXFLAGS) $(local_flags) $(4) -c $$< -o $$@
	@$(CXX) -MM -MP -MT $$@ -MF $(build_dir)/$(1)/$(3)/$$*.d $(local_arch) $(CXXFLAGS) $(local_flags) $(4) $$<

$(build_dir_d)/$(1)/$(3)/%.o: $(2)/$(3)/%.s
	$(AS) $(local_arch) $(ASFLAGS_D) $(local_flags_d) $(4) -c $$< -o $$@
	@$(AS) -MM -MP -MT $$@ -MF $(build_dir_d)/$(1)/$(3)/$$*.d $(local_arch) $(ASFLAGS_D) $(local_flags_d) $(4) $$<

$(build_dir_d)/$(1)/$(3)/%.o: $(2)/$(3)/%.c
	$(CC) $(local_arch) $(CFLAGS_D) $(local_flags_d) $(4) -c $$< -o $$@
	@$(CC) -MM -MP -MT $$@ -MF $(build_dir_d)/$(1)/$(3)/$$*.d $(local_arch) $(CFLAGS_D) $(local_flags_d) $(4) $$<

$(build_dir_d)/$(1)/$(3)/%.o: $(2)/$(3)/%.cpp
	$(CXX) $(local_arch) $(CXXFLAGS_D) $(local_flags_d) $(4) -c $$< -o $$@
	@$(CXX) -MM -MP -MT $$@ -MF $(build_dir_d)/$(1)/$(3)/$$*.d $(local_arch) $(CXXFLAGS_D) $(local_flags_d) $(4) $$<
endef

include make/compiler.mk

ifeq ($(FW_CONFIG),$(filter $(FW_CONFIG), MB16R MB16D))
include lib/dsp/dsp.mk
include lib/common/f7xx.mk
include app/main2016/main2016.mk
endif

ifeq ($(FW_CONFIG),$(filter $(FW_CONFIG), BS18R BS18D))
include lib/common/f7xx_dp.mk
include app/bs2018/bs2018.mk
endif

ifeq ($(FW_CONFIG),$(filter $(FW_CONFIG), BS18R BS18D MB19R MB19D))
include lib/dsp/dsp_dp.mk
endif

ifeq ($(FW_CONFIG),$(filter $(FW_CONFIG), MB19R MB19D))
include lib/dsp/dsp_m0.mk
include lib/common/h7xx_dp.mk
include lib/common/f0xx.mk
include app/main2019/main2019.mk
include app/ir2019/ir2019.mk
include app/motor2019/motor2019.mk
endif

ifeq ($(FW_CONFIG),$(filter $(FW_CONFIG), MB16R BS18R MB19R))
boot: release-boot
run: release-run
else
boot: debug-boot
run: debug-run
endif

folders:
	$(MKDIR) -p $(build_folders)

folders_d:
	$(MKDIR) -p $(build_folders_d)

release-boot: folders $(libraries) $(boot_programs)
	$(SIZE) $(boot_programs)

release-run: folders $(libraries) $(run_programs)
	$(SIZE) $(run_programs)

debug-boot: folders_d $(libraries_d) $(boot_programs_d)
	$(SIZE) $(boot_programs_d)

debug-run: folders_d $(libraries_d) $(run_programs_d)
	$(SIZE) $(run_programs_d)

clean: dist-clean

dist-clean: release-dist-clean debug-dist-clean

release-dist-clean:
	-$(RM) -rf $(build_dir)
	-$(RM) -rf $(libraries) $(run_programs) $(boot_programs)

debug-dist-clean:
	-$(RM) -rf $(build_dir_d)
	-$(RM) -rf $(libraries_d) $(run_programs_d) $(boot_programs_d)

flash-mb2016-boot: boot
	$(OPENOCD) -c "set folder release/app/boot" -f openocd/flash-mb2016.cfg

flash-mb2016-run: run
	$(OPENOCD) -c "set folder release/app/run" -f openocd/flash-mb2016.cfg

flash-mb2019-boot: boot
	$(OPENOCD) -c "set folder release/app/boot" -f openocd/flash-mb2019.cfg

flash-mb2019-run: run
	$(OPENOCD) -c "set folder release/app/run" -f openocd/flash-mb2019.cfg

flash-bs2018-boot: boot
	$(OPENOCD) -c "set folder release/app/boot" -f openocd/flash-bs2018.cfg

flash-bs2018-run: run
	$(OPENOCD) -c "set folder release/app/run" -f openocd/flash-bs2018.cfg
