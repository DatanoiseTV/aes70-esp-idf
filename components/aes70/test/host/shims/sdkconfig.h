/* Host unit-test build: the component's Kconfig fallbacks in aes70_internal.h
 * provide the AES70_* defaults, so this only needs to exist. Override here to
 * exercise a specific configuration under test. */
#pragma once

#define CONFIG_AES70_MAX_CONNECTIONS 4
