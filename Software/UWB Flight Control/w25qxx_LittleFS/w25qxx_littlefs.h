/*
 * w25qxx_littlefs.h
 *
 *  Created on: Mar 24, 2022
 *      Author: lth
 */

#ifndef W25QXX_LITTLEFS_H_
#define W25QXX_LITTLEFS_H_

#ifdef DEBUGxxx
#define LFS_DBG(...) printf(__VA_ARGS__);\
                     printf("\n");
#else
#define LFS_DBG(...) ;
#endif

//extern lfs_t littlefs;

//const struct lfs_config *littlefs_config();

int w25qxx_littlefs_init(void);


#define sector_size 0x1000
#define block_size_def 0x10000
#define sectors_in_block 0x0f
#define page_size 0x100
#define pages_in_sector 0x10



#endif /* W25QXX_LITTLEFS_H_ */
