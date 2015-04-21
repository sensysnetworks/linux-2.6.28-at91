#ifndef CHELPER_H_
#define CHELPER_H_

#include <asm/io.h>

/************************************************************
 * helper functions
 */
 
 static inline ssize_t sfs_getint(const char *buf,size_t count,size_t *size){
 	char *endp;
	u32 data = simple_strtoul(buf,&endp,0);	
	*size = (size_t)(endp - buf);
	if (*endp && isspace(*endp))(*size)++;
	if (*size != count)*size = -EINVAL;
	return data;
 }

 /** 
  * converts a stream of separated integers into an allocated array.
  * returns the number of integers converted
  * stores the number of characters converted to integers in size
  */
 static inline  int sfs_getstream(const char *buf,size_t count,size_t *size,const char *delim,u8 *data){
	char *tokenstring = strcpy(kmalloc(count,GFP_KERNEL),buf);
	char *mark = tokenstring;
	char *endp;
	char *token;
	int index=0;
	
	//printk("%s - mark:%s\n",__FUNCTION__,mark);
	
	while((token=strsep(&mark, delim))!=NULL){
		//printk("mark:%s\n",mark);
		data[index++] = simple_strtoul(token,&endp,0);
	}
		
	*size = (size_t)(endp - tokenstring);
	if (*endp && isspace(*endp))(*size)++;
	if (*size != count)*size = -EINVAL;
	
	//printk("size = %d\n",*size);
	
	kfree(tokenstring);
	return index;
 }
 
 
/**
 * depreciated
 */
static inline ssize_t store8(const char *buf,size_t count,void *address,u8 *shadow){
	size_t size;
	u8 data = sfs_getint(buf,count,&size);
	if(address)iowrite8(data,address);
	if(shadow)*shadow=data;
	return size;
}

/**
 * depreciated
 */
static inline ssize_t store16(const char *buf,size_t count,void *address,u16 *shadow){
	size_t size;
	u16 data = sfs_getint(buf,count,&size);
	if(address)iowrite16(data,address);
	if(shadow)*shadow=data;
	return size;
}

#endif /*CHELPER_H_*/
