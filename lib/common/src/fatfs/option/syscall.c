/*------------------------------------------------------------------------*/
/* Sample code of OS dependent controls for FatFs                         */
/* (C)ChaN, 2012                                                          */
/*------------------------------------------------------------------------*/

#include "../ff.h"
#include "ch.h"
#include "util/sys_time.h"
#include <time.h>

static mutex_t volMutices[_VOLUMES];

#if _FS_REENTRANT
/*------------------------------------------------------------------------*/
/* Create a Synchronization Object */
/*------------------------------------------------------------------------*/
/* This function is called by f_mount() function to create a new
/  synchronization object, such as semaphore and mutex. When a 0 is
/  returned, the f_mount() function fails with FR_INT_ERR.
*/

int ff_cre_syncobj (	/* 1:Function succeeded, 0:Could not create due to any error */
	BYTE vol,			/* Corresponding logical drive being processed */
	_SYNC_t* sobj		/* Pointer to return the created sync object */
)
{
	if(vol > _VOLUMES)
		return 0;

	chMtxObjectInit(&volMutices[vol]);
	*sobj = &volMutices[vol];

	return 1;
}



/*------------------------------------------------------------------------*/
/* Delete a Synchronization Object                                        */
/*------------------------------------------------------------------------*/
/* This function is called in f_mount() function to delete a synchronization
/  object that created with ff_cre_syncobj() function. When a 0 is
/  returned, the f_mount() function fails with FR_INT_ERR.
*/

int ff_del_syncobj (	/* 1:Function succeeded, 0:Could not delete due to any error */
	_SYNC_t sobj		/* Sync object tied to the logical drive to be deleted */
)
{
	(void)sobj;

	return 1;
}



/*------------------------------------------------------------------------*/
/* Request Grant to Access the Volume                                     */
/*------------------------------------------------------------------------*/
/* This function is called on entering file functions to lock the volume.
/  When a FALSE is returned, the file function fails with FR_TIMEOUT.
*/

int ff_req_grant (	/* TRUE:Got a grant to access the volume, FALSE:Could not get a grant */
	_SYNC_t sobj	/* Sync object to wait */
)
{
	chMtxLock(sobj);

	return 1;
}



/*------------------------------------------------------------------------*/
/* Release Grant to Access the Volume                                     */
/*------------------------------------------------------------------------*/
/* This function is called on leaving file functions to unlock the volume.
*/

void ff_rel_grant (
	_SYNC_t sobj	/* Sync object to be signaled */
)
{
	chMtxUnlock(sobj);
}

#endif

DWORD get_fattime()
{
	struct tm* nowtm;
	time_t now = SysTimeUnix();
	nowtm = localtime(&now);

	DWORD fatTime = ((nowtm->tm_year-80) << 25) | ((nowtm->tm_mon+1) << 21) | (nowtm->tm_mday << 16);
	fatTime |= (nowtm->tm_hour << 11) | (nowtm->tm_min << 5) | (nowtm->tm_sec >> 1);

	return fatTime;
}

#if _USE_LFN == 3	/* LFN with a working buffer on the heap */
/*------------------------------------------------------------------------*/
/* Allocate a memory block                                                */
/*------------------------------------------------------------------------*/
/* If a NULL is returned, the file function fails with FR_NOT_ENOUGH_CORE.
*/

void* ff_memalloc (	/* Returns pointer to the allocated memory block */
	UINT msize		/* Number of bytes to allocate */
)
{
	return malloc(msize);
}


/*------------------------------------------------------------------------*/
/* Free a memory block                                                    */
/*------------------------------------------------------------------------*/

void ff_memfree (
	void* mblock	/* Pointer to the memory block to free */
)
{
	free(mblock);
}

#endif
