#include "dfs_fs.h"
#include "dfs_romfs.h"
#include "dfs_ramfs.h"


/*rom file system*/
const struct romfs_dirent _mnt_dirent[] =
{
	{ROMFS_DIRENT_DIR, "sf", RT_NULL, 0},
};

const struct romfs_dirent _root_dirent[] =
{
    {ROMFS_DIRENT_DIR, "mnt", (rt_uint8_t *)_mnt_dirent, sizeof(_mnt_dirent) / sizeof(_mnt_dirent[0])},
	{ROMFS_DIRENT_DIR, "tmp", (rt_uint8_t *)RT_NULL, 0},
	{ROMFS_DIRENT_DIR, "dev", (rt_uint8_t *)RT_NULL, 0},
};

const struct romfs_dirent romfs_root =
{
    ROMFS_DIRENT_DIR, "/", (rt_uint8_t *)_root_dirent, sizeof(_root_dirent) / sizeof(_root_dirent[0])
};

/*ram file system*/
const struct dfs_ramfs *ramfs;

/*file system table*/
const struct dfs_mount_tbl mount_table[] = 
{
	{RT_NULL, "/", "rom", 0, &romfs_root},
	{"w25qxxx", "/mnt/sf", "elm", 0, 0},
	{RT_NULL}
};

static int ramfs_init(void)
{
	ramfs = dfs_ramfs_create(rt_malloc(1024), 1024);
	if(dfs_mount(RT_NULL, "/tmp", "ram", 0, &ramfs) == 0)
	{
		return RT_EOK;
	}
	return RT_ERROR;
}
INIT_ENV_EXPORT(ramfs_init);
