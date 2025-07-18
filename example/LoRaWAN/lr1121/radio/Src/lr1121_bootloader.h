/*!
 * @file      lr1121_bootloader.h
 *
 * @brief     Bootloader driver definition for LR1121
 *
 * @copyright Copyright Semtech Corporation 2024. All rights reserved.
 *
 * @license{The Clear BSD License
 * @par
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted (subject to the limitations in the disclaimer
 * below) provided that the following conditions are met:
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Semtech corporation nor the
 *       names of its contributors may be used to endorse or promote products
 *       derived from this software without specific prior written permission.
 * @par
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY
 * THIS LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND
 * CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT
 * NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A
 * PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL SEMTECH CORPORATION BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.}
 */

#ifndef LR1121_MODEM_BOOTLOADER_H
#define LR1121_MODEM_BOOTLOADER_H

#ifdef __cplusplus
extern "C" {
#endif

/*
 * -----------------------------------------------------------------------------
 * --- DEPENDENCIES ------------------------------------------------------------
 */

#include <stdbool.h>
#include <stdint.h>
#include "lr1121_bootloader_types.h"
#include "lr1121_types.h"

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC MACROS -----------------------------------------------------------
 */

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC CONSTANTS --------------------------------------------------------
 */

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC TYPES ------------------------------------------------------------
 */

/**
 * @brief Type for bootloader status IRQ mask
 */
typedef uint32_t lr1121_bootloader_irq_mask_t;

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC FUNCTIONS PROTOTYPES ---------------------------------------------
 */

/*!
 * @brief Return the status registers and interrupt flags
 *
 * @remark To simplify system integration, this function does not actually execute the GetStatus command, which would
 * require bidirectional SPI communication. It obtains the stat1, stat2, and irq_status values by performing an ordinary
 * SPI read (which is required to send null/NOP bytes on the MOSI line). This is possible since the LR1121 returns these
 * values automatically whenever a read that does not directly follow a response-carrying command is performed. Unlike
 * with the GetStatus command, however, the reset status information is NOT cleared by this command. The function @ref
 * lr1121_bootloader_clear_reset_status_info may be used for this purpose when necessary.
 *
 * @param [in] context Chip implementation context
 * @param [out] stat1 Content of status register 1
 * @param [out] stat2 Content of status register 2
 * @param [out] irq_status Interrupt flags
 *
 * @returns Operation status
 */
 //这个函数没有用到，但是调用了lr1121_hal_direct_read（）但函数却没有实现，make可以忽略，但keil的ARM编译器无法忽略
//lr1121_status_t lr1121_bootloader_get_status( const void* context, lr1121_bootloader_stat1_t* stat1,
//                                              lr1121_bootloader_stat2_t*    stat2,
//                                              lr1121_bootloader_irq_mask_t* irq_status );

/*!
 * @brief Clear the reset status information stored in stat2
 *
 * @param [in] context Chip implementation context
 *
 * @returns Operation status
 */
lr1121_status_t lr1121_bootloader_clear_reset_status_info( const void* context );

/*!
 * @brief Return the version of the system (hardware and software)
 *
 * @param [in] context Chip implementation context
 * @param [out] version Pointer to the structure holding the system version
 *
 * @returns Operation status
 */
lr1121_status_t lr1121_bootloader_get_version( const void* context, lr1121_bootloader_version_t* version );

/*!
 * @brief Erase the whole flash memory of the chip
 *
 * This function shall be called before any attempt to write a new firmware in flash memory
 *
 * @param [in] context Chip implementation context
 *
 * @returns Operation status
 */
lr1121_status_t lr1121_bootloader_erase_flash( const void* context );

/*!
 * @brief Write encrypted data in program flash memory of the chip
 *
 * This function shall be used when updating the encrypted flash content of the LR1121.
 * The encrypted flash payload to transfer shall be represented as an array of words (i.e. 4-byte values).
 *
 * Updating flash code of the chip with this function MUST respect the following constraints:
 *   - the complete flash image MUST be splitted into chunks of 64 words each, except the last one that can be shorter
 *   - the chunks MUST be sent to the chip in-order, starting with @p offset_in_byte = 0
 *
 * @param [in] context Chip implementation context
 * @param [in] offset_in_byte The offset from start register of flash in byte
 * @param [in] buffer Buffer holding the encrypted content. Its size in words must be at least length
 * @param [in] length_in_word Number of words (i.e. 4 bytes) in the buffer to transfer. MUST be 64 for all chunks except
 * the last one where it can be lower.
 *
 * @returns Operation status
 */
lr1121_status_t lr1121_bootloader_write_flash_encrypted( const void* context, const uint32_t offset_in_byte,
                                                         const uint32_t* buffer, const uint8_t length_in_word );

/*!
 * @brief Write encrypted data in program flash memory of the chip
 *
 * This function shall be used when updating the encrypted flash content of the LR1121.
 * The encrypted flash payload to transfer shall be represented as an array of words (ie 4-byte values).
 *
 * @param [in] context Chip implementation context
 * @param [in] offset_in_byte The offset from start register of flash in byte
 * @param [in] buffer Buffer holding the encrypted content. Its size in words must be at least length
 * @param [in] length_in_word Number of words (i.e. 4 bytes) in the buffer to transfer
 *
 * @returns Operation status
 */
lr1121_status_t lr1121_bootloader_write_flash_encrypted_full( const void* context, const uint32_t offset_in_byte,
                                                              const uint32_t* buffer, const uint32_t length_in_word );

/*!
 * @brief Software reset of the chip.
 *
 * This method should be used to reboot the chip in a specified mode.
 * Rebooting in flash mode presumes that the content in flash memory is not corrupted (i.e. the integrity check
 * performed by the bootloader before executing the first instruction in flash is OK).
 *
 * @param [in] context Chip implementation context
 * @param [in] stay_in_bootloader Selector to stay in bootloader or execute flash code after reboot. If true, the
 * bootloader will not execute the flash code but activate SPI interface to allow firmware upgrade
 *
 * @returns Operation status
 */
lr1121_status_t lr1121_bootloader_reboot( const void* context, const bool stay_in_bootloader );

/*!
 * @brief Returns the 4-byte PIN which can be used to claim a device on cloud services.
 *
 * @param [in] context Chip implementation context
 * @param [out] pin Pointer to the array to be populated with the PIN
 *
 * @returns Operation status
 */
lr1121_status_t lr1121_bootloader_read_pin( const void* context, lr1121_bootloader_pin_t pin );

/*!
 * @brief Read and return the Chip EUI
 *
 * @param [in] context Chip implementation context
 * @param [out] chip_eui The buffer to be filled with chip EUI of the LR1121. It is up to the application to ensure
 * chip_eui is long enough to hold the chip EUI
 *
 * @returns Operation status
 */
lr1121_status_t lr1121_bootloader_read_chip_eui( const void* context, lr1121_bootloader_chip_eui_t chip_eui );

/*!
 * @brief Read and return the Join EUI
 *
 * @param [in] context Chip implementation context
 * @param [out] join_eui The buffer to be filled with Join EUI of the LR1121. It is up to the application to ensure
 * join_eui is long enough to hold the join EUI
 *
 * @returns Operation status
 */
lr1121_status_t lr1121_bootloader_read_join_eui( const void* context, lr1121_bootloader_join_eui_t join_eui );

#ifdef __cplusplus
}
#endif

#endif  // LR1121_MODEM_BOOTLOADER_H

/* --- EOF ------------------------------------------------------------------ */
