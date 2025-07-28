/*
 * w25qxx_littlefs.c
 *
 *  Created on: Mar 24, 2022
 *      Author: lth
 */


#include "main.h"
#include "lfs.h"
#include "quadspi.h"
#include "w25qxx_littlefs.h"

int littlefs_read(const struct lfs_config *c, lfs_block_t block, lfs_off_t off, void *buffer, lfs_size_t size);
int littlefs_prog(const struct lfs_config *c, lfs_block_t block, lfs_off_t off, const void *buffer, lfs_size_t size);
int littlefs_erase(const struct lfs_config *c, lfs_block_t block);
int littlefs_sync(const struct lfs_config *c);



struct lfs_config littlefs_config = {
    // block device operations
    .read  = littlefs_read,
    .prog  = littlefs_prog,
    .erase = littlefs_erase,
    .sync  = littlefs_sync,

    // block device configuration
    .read_size = 256,
    .prog_size = 256,
    .block_size = 4096, //4KB sector
    .block_count = 4096,  //since our block size is really a 4KB sector, there at 4096 sectors on the chip
    .cache_size = 512,
    .lookahead_size = 512,
    .block_cycles = 100,
};

lfs_t littlefs;
//W25QXX_HandleTypeDef *w25qxx_handle;




int w25qxx_littlefs_init(void) {
	LFS_DBG("LittleFS Init");
	//w25qxx_handle = w25qxx_init;

	littlefs_config.block_size = 0x1000;
	littlefs_config.block_count = 0x1000;

	int err = lfs_mount(&littlefs, &littlefs_config);

    // reformat if we can't mount the filesystem
    // this should only happen on the first boot
    if (err) {
        lfs_format(&littlefs, &littlefs_config);
        lfs_mount(&littlefs, &littlefs_config);
    }

    return 0;

}

int littlefs_read(const struct lfs_config *c, lfs_block_t block, lfs_off_t off, void *buffer, lfs_size_t size) {
	LFS_DBG("LittleFS Read b = 0x%04lx o = 0x%04lx s = 0x%04lx", block, off, size);
	if (CSP_QSPI_Read(buffer, (uint32_t)(block * sector_size + off) , (uint32_t)size) != HAL_OK) return -1;
	return 0;
}

int littlefs_prog(const struct lfs_config *c, lfs_block_t block, lfs_off_t off, const void *buffer, lfs_size_t size) {
	LFS_DBG("LittleFS Prog b = 0x%04lx o = 0x%04lx s = 0x%04lx", block, off, size);
	if (CSP_QSPI_WriteMemory((uint8_t *)buffer, (uint32_t)(block * sector_size + off),  (uint32_t)size) != HAL_OK) return -1;
	return 0;
}

int littlefs_erase(const struct lfs_config *c, lfs_block_t block) {
	LFS_DBG("LittleFS Erase b = 0x%04lx", block);
	uint32_t start_adr = block * 4096;
	uint32_t end_adr = start_adr + 4095;
	if (CSP_QSPI_EraseSector(start_adr,end_adr) != HAL_OK) return -1;
	return 0;
}

int littlefs_sync(const struct lfs_config *c) {
	LFS_DBG("LittleFS Sync");
	return 0;
}

/*
 * vim: ts=4 nowrap
 */
