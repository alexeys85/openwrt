#
# Copyright (C) 2012-2013 OpenWrt.org
#
# This is free software, licensed under the GNU General Public License v2.
# See /LICENSE for more information.
#

include $(TOPDIR)/rules.mk
include $(INCLUDE_DIR)/kernel.mk

PKG_VERSION:=2022.01
PKG_RELEASE:=$(AUTORELEASE)

PKG_HASH:=81b4543227db228c03f8a1bf5ddbc813b0bb8f6555ce46064ef721a6fc680413

include $(INCLUDE_DIR)/u-boot.mk
include $(INCLUDE_DIR)/package.mk

define U-Boot/Default
  BUILD_TARGET:=omap
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
	sed -i 's/CONFIG_ANDROID_BOOT_IMAGE=y/# CONFIG_ANDROID_BOOT_IMAGE is not set/g' $(PKG_BUILD_DIR)/.config
	sed -i 's/CONFIG_DEFAULT_DEVICE_TREE="am335x-evm"/CONFIG_DEFAULT_DEVICE_TREE="am335x-boneblack"/g' $(PKG_BUILD_DIR)/.config
	sed -i 's/CONFIG_SPL_MTD_SUPPORT=y/# CONFIG_SPL_MTD_SUPPORT is not set/g' $(PKG_BUILD_DIR)/.config
	sed -i 's/CONFIG_SPL_MUSB_NEW=y/# CONFIG_SPL_MUSB_NEW is not set/g' $(PKG_BUILD_DIR)/.config
	sed -i 's/CONFIG_SPL_NAND_SUPPORT=y/# CONFIG_SPL_NAND_SUPPORT is not set/g' $(PKG_BUILD_DIR)/.config
	sed -i 's/CONFIG_SPL_NAND_DRIVERS=y/# CONFIG_SPL_NAND_DRIVERS is not set/g' $(PKG_BUILD_DIR)/.config
	sed -i 's/CONFIG_SPL_NAND_ECC=y/# CONFIG_SPL_NAND_ECC is not set/g' $(PKG_BUILD_DIR)/.config
	sed -i 's/CONFIG_SPL_USB_GADGET=y/# CONFIG_SPL_USB_GADGET is not set/g' $(PKG_BUILD_DIR)/.config
	sed -i 's/CONFIG_MTD=y/# CONFIG_MTD is not set/g' $(PKG_BUILD_DIR)/.config
	sed -i 's/CONFIG_MTD_RAW_NAND=y/# CONFIG_MTD_RAW_NAND is not set/g' $(PKG_BUILD_DIR)/.config
	sed -i 's/CONFIG_SPI_FLASH=y/# CONFIG_SPI_FLASH is not set/g' $(PKG_BUILD_DIR)/.config
	sed -i 's/# CONFIG_RGMII is not set/CONFIG_RGMII=y/g' $(PKG_BUILD_DIR)/.config
	sed -i 's/CONFIG_SPI=y/# CONFIG_SPI is not set/g' $(PKG_BUILD_DIR)/.config
	sed -i 's/CONFIG_USB=y/# CONFIG_USB is not set/g' $(PKG_BUILD_DIR)/.config
	sed -i 's/CONFIG_EFI_LOADER=y/# CONFIG_EFI_LOADER is not set/g' $(PKG_BUILD_DIR)/.config
	sed -i 's/CONFIG_CMD_GPT=y/# CONFIG_CMD_GPT is not set/g' $(PKG_BUILD_DIR)/.config
	sed -i 's/CONFIG_RANDOM_UUID=y/# CONFIG_RANDOM_UUID is not set/g' $(PKG_BUILD_DIR)/.config
	sed -i 's/CONFIG_EFI_PARTITION=y/# CONFIG_EFI_PARTITION is not set/g' $(PKG_BUILD_DIR)/.config
	sed -i 's/# CONFIG_CMD_SETEXPR is not set/CONFIG_CMD_SETEXPR=y/g' $(PKG_BUILD_DIR)/.config
	sed -i 's/# CONFIG_CMD_UNZIP is not set/CONFIG_CMD_UNZIP=y/g' $(PKG_BUILD_DIR)/.config
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
