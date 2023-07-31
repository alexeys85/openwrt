#
# Copyright (C) 2012-2013 OpenWrt.org
#
# This is free software, licensed under the GNU General Public License v2.
# See /LICENSE for more information.
#

include $(TOPDIR)/rules.mk
include $(INCLUDE_DIR)/kernel.mk

PKG_VERSION:=2023.07.02
PKG_RELEASE:=21

PKG_HASH:=6b6a48581c14abb0f95bd87c1af4d740922406d7b801002a9f94727fdde021d5

include $(INCLUDE_DIR)/u-boot.mk
include $(INCLUDE_DIR)/package.mk

define U-Boot/Default
  BUILD_TARGET:=omap
  BUILD_SUBTARGET:=generic
  UBOOT_IMAGE:=u-boot.img MLO
endef

define U-Boot/omap4_panda
  NAME:=Pandaboard
  BUILD_DEVICES:=ti_omap4-panda
endef

define U-Boot/am335x_evm
  NAME:=AM335x EVM
  BUILD_DEVICES:=ti_am335x-evm ti_am335x-bone-black ti_am335x-miet
endef

define U-Boot/omap3_beagle
  NAME:=BeagleBoard
  BUILD_DEVICES:=ti_omap3-beagle
endef

UBOOT_TARGETS:=omap4_panda am335x_evm omap3_beagle

define Build/Configure
	$(call Build/Configure/U-Boot)
	sed -i 's/CONFIG_CC_OPTIMIZE_FOR_SIZE=y/# CONFIG_CC_OPTIMIZE_FOR_SIZE is not set/g' $(PKG_BUILD_DIR)/.config
	sed -i 's/# CONFIG_CC_OPTIMIZE_FOR_SPEED is not set/CONFIG_CC_OPTIMIZE_FOR_SPEED=y/g' $(PKG_BUILD_DIR)/.config
	sed -i 's/CONFIG_ANDROID_BOOT_IMAGE=y/# CONFIG_ANDROID_BOOT_IMAGE is not set/g' $(PKG_BUILD_DIR)/.config
	sed -i 's/CONFIG_DEFAULT_DEVICE_TREE="am335x-evm"/CONFIG_DEFAULT_DEVICE_TREE="am335x-boneblack"/g' $(PKG_BUILD_DIR)/.config
	sed -i 's/CONFIG_OF_LIST=.*/CONFIG_OF_LIST="am335x-boneblack"/g' $(PKG_BUILD_DIR)/.config
	sed -i 's/CONFIG_SPL_MTD_SUPPORT=y/# CONFIG_SPL_MTD_SUPPORT is not set/g' $(PKG_BUILD_DIR)/.config
	sed -i 's/CONFIG_SPL_MUSB_NEW=y/# CONFIG_SPL_MUSB_NEW is not set/g' $(PKG_BUILD_DIR)/.config
	sed -i 's/CONFIG_SPL_NAND_SUPPORT=y/# CONFIG_SPL_NAND_SUPPORT is not set/g' $(PKG_BUILD_DIR)/.config
	sed -i 's/CONFIG_SPL_NAND_DRIVERS=y/# CONFIG_SPL_NAND_DRIVERS is not set/g' $(PKG_BUILD_DIR)/.config
	sed -i 's/CONFIG_SPL_NAND_ECC=y/# CONFIG_SPL_NAND_ECC is not set/g' $(PKG_BUILD_DIR)/.config
	sed -i 's/CONFIG_SPL_USB_GADGET=y/# CONFIG_SPL_USB_GADGET is not set/g' $(PKG_BUILD_DIR)/.config
	sed -i 's/CONFIG_BOOTM_NETBSD=y/# CONFIG_BOOTM_NETBSD is not set/g' $(PKG_BUILD_DIR)/.config
	sed -i 's/CONFIG_BOOTM_PLAN9=y/# CONFIG_BOOTM_PLAN9 is not set/g' $(PKG_BUILD_DIR)/.config
	sed -i 's/CONFIG_BOOTM_RTEMS=y/# CONFIG_BOOTM_RTEMS is not set/g' $(PKG_BUILD_DIR)/.config
	sed -i 's/CONFIG_BOOTM_VXWORKS=y/# CONFIG_BOOTM_VXWORKS is not set/g' $(PKG_BUILD_DIR)/.config
	sed -i 's/CONFIG_MTD=y/# CONFIG_MTD is not set/g' $(PKG_BUILD_DIR)/.config
	sed -i 's/CONFIG_MTD_RAW_NAND=y/# CONFIG_MTD_RAW_NAND is not set/g' $(PKG_BUILD_DIR)/.config
	sed -i 's/CONFIG_SPI_FLASH=y/# CONFIG_SPI_FLASH is not set/g' $(PKG_BUILD_DIR)/.config
	sed -i 's/# CONFIG_RGMII is not set/CONFIG_RGMII=y/g' $(PKG_BUILD_DIR)/.config
	sed -i 's/CONFIG_SPI=y/# CONFIG_SPI is not set/g' $(PKG_BUILD_DIR)/.config
	#sed -i 's/CONFIG_USB=y/# CONFIG_USB is not set/g' $(PKG_BUILD_DIR)/.config
	sed -i 's/CONFIG_CMD_USB=y/# CONFIG_CMD_USB is not set/g' $(PKG_BUILD_DIR)/.config
	sed -i 's/CONFIG_CMD_FASTBOOT=y/# CONFIG_CMD_FASTBOOT is not set/g' $(PKG_BUILD_DIR)/.config
	sed -i 's/CONFIG_USB_FUNCTION_FASTBOOT=y/# CONFIG_USB_FUNCTION_FASTBOOT is not set/g' $(PKG_BUILD_DIR)/.config
	sed -i 's/CONFIG_FASTBOOT=y/# CONFIG_FASTBOOT is not set/g' $(PKG_BUILD_DIR)/.config
	sed -i 's/CONFIG_USB_GADGET_DOWNLOAD=y/# CONFIG_USB_GADGET_DOWNLOAD is not set/g' $(PKG_BUILD_DIR)/.config
	sed -i 's/CONFIG_EFI_LOADER=y/# CONFIG_EFI_LOADER is not set/g' $(PKG_BUILD_DIR)/.config
	sed -i 's/CONFIG_CMD_GPT=y/# CONFIG_CMD_GPT is not set/g' $(PKG_BUILD_DIR)/.config
	sed -i 's/CONFIG_RANDOM_UUID=y/# CONFIG_RANDOM_UUID is not set/g' $(PKG_BUILD_DIR)/.config
	sed -i 's/CONFIG_EFI_PARTITION=y/# CONFIG_EFI_PARTITION is not set/g' $(PKG_BUILD_DIR)/.config
	sed -i 's/# CONFIG_CMD_SETEXPR is not set/CONFIG_CMD_SETEXPR=y/g' $(PKG_BUILD_DIR)/.config
	sed -i 's/# CONFIG_CMD_UNZIP is not set/CONFIG_CMD_UNZIP=y/g' $(PKG_BUILD_DIR)/.config
	sed -i 's/CONFIG_DFU=y/# CONFIG_DFU is not set/g' $(PKG_BUILD_DIR)/.config
	sed -i 's/CONFIG_CMD_DFU=y/# CONFIG_CMD_DFU is not set/g' $(PKG_BUILD_DIR)/.config
	sed -i 's/# CONFIG_CMD_WGET is not set/CONFIG_CMD_WGET=y/g' $(PKG_BUILD_DIR)/.config
	echo "CONFIG_PROT_TCP_SACK=y" >> $(PKG_BUILD_DIR)/.config
	sed -i 's/CONFIG_SPL_OS_BOOT=y/# CONFIG_SPL_OS_BOOT is not set/g' $(PKG_BUILD_DIR)/.config
endef

define Build/InstallDev
	$(foreach device,$(BUILD_DEVICES), \
		$(INSTALL_DIR) $(STAGING_DIR_IMAGE)/$(device)
	)
	$(foreach device,$(BUILD_DEVICES), \
		$(CP) $(patsubst %,$(PKG_BUILD_DIR)/%,$(UBOOT_IMAGE)) $(STAGING_DIR_IMAGE)/$(device)/
	)
	$(foreach device,$(BUILD_DEVICES), \
		mkimage -C none -A arm -T script -d ./files/boot.scr.txt \
			$(STAGING_DIR_IMAGE)/$(device)/boot.scr
	)
endef

$(eval $(call BuildPackage/U-Boot))
