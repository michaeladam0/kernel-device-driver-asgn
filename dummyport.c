
/**
 * File: asgn2.c
 * Author: Michael Adam
 *
 * This is a module which serves as a virtual ramdisk for which disk size is
 * limited by the amount of memory available and serves as the requirement for
 * COSC440 assignment 2 in 2014.
 *
 * Note: multiple devices and concurrent modules are not supported in this
 *       version.
 */
 
/* This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version
 * 2 of the License, or (at your option) any later version.
 */

#include <linux/init.h>
#include <linux/module.h>
#include <linux/fs.h>
#include <linux/cdev.h>
#include <linux/list.h>
#include <asm/uaccess.h>
#include <linux/slab.h>
#include <linux/mm.h>
#include <linux/proc_fs.h>
#include <linux/device.h>
#include <linux/sched.h>
#include <linux/interrupt.h>
#include "gpio.h"

#define MYDEV_NAME "asgn2"
#define MYIOC_TYPE 'k'

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Michael Adam");
MODULE_DESCRIPTION("COSC440 asgn2");

/**
 * The node structure for the memory page linked list.
 */ 
typedef struct page_node_rec {
  struct list_head list;
  struct page *page;
  size_t write_offset;            /* position of write in page */
  size_t read_offset;             /* position of read in page */
} page_node;

typedef struct asgn2_dev_t {
  dev_t dev;                      /* the device */
  struct cdev *cdev;
  struct list_head mem_list;      /* head for list of page nodes */
  struct list_head *write_node;   /* the page node currently being written */
  long read_offset;               /* global read position */
  int num_pages;                  /* number of memory pages in this module */
  size_t data_size;               /* total data size in this module */
  atomic_t in_use;                /* switch for exclusive access to device */
  atomic_t rw_sync;               /* switch for read caught up to write */
  struct kmem_cache *cache;       /* cache memory */
  struct class *class;            /* the udev class */
  struct device *device;          /* the udev device node */
} asgn2_dev;

asgn2_dev asgn2_device;

int asgn2_major = 0;              /* major number of module */  
int asgn2_minor = 0;              /* minor number of module */
int asgn2_dev_count = 1;          /* number of devices */

int EOF = 0;                      /* flag to indicate end of file reached */
int c_buff_size = 256;            /* the size of the circular buffer */
static char *c_buff;              /* the circular buffer */
int c_read_index = 0;             /* read position in circular buffer */
int c_write_index = 0;            /* write position in circular buffer */
char half_byte_1;                 /* the first half of the unassembled byte */
char half_byte_2;                 /* second half of unassembled byte */
int byte_switch = 0;              /* switch to trigger byte assembly */

/* Placeholder for tasklet param */
unsigned long t_data;

/* Declare wait queues for reader limit and wait for write */
DECLARE_WAIT_QUEUE_HEAD(wait_access);
DECLARE_WAIT_QUEUE_HEAD(wait_data);

/**
 * This function writes a byte to the current page, 
 * adding pages as necessary and waking the reader if it is sleeping.
 */
int write_byte(char byte) {
  page_node *curr = list_entry(asgn2_device.write_node, page_node, list);

  /* If page doesn't exist or is full, create new page */
  if (asgn2_device.write_node == &asgn2_device.mem_list || 
      curr->write_offset == PAGE_SIZE) {
    curr = kmem_cache_alloc(asgn2_device.cache, GFP_KERNEL);
    if (NULL == curr) {
      printk(KERN_WARNING "Not enough memory left\n");
      return 1;
    }
    /* Allocate page */
    curr->page = alloc_page(GFP_KERNEL);
    if (NULL == curr->page) {
      printk(KERN_WARNING "Not enough memory left\n");
      kmem_cache_free(asgn2_device.cache, curr);
      return 1;
    }
    
    /* Initial write and read offsets */
    curr->write_offset = 0;
    curr->read_offset = 0;

    list_add_tail(&(curr->list), &asgn2_device.mem_list);
    asgn2_device.num_pages++;
    asgn2_device.write_node = asgn2_device.mem_list.prev;
  }
  
  /* Copy byte and increment write offset */
  memcpy(page_address(curr->page) + curr->write_offset, &byte, 1);
  curr->write_offset++;

  /* Signal that data is available to be read */
  atomic_set(&asgn2_device.rw_sync, 1);
  wake_up_interruptible(&wait_data);

  asgn2_device.data_size++;
  return 0;
}

/**
 * This function provides the logic for the tasklet.
 * When it is called, it calls write_byte to copy the current read 
 * byte from the circular buffer into the current page
 */
static void t_bottom_half(unsigned long t_arg)
{
  write_byte(c_buff[c_read_index]);
  c_buff[c_read_index] = '0';
  (c_read_index == (c_buff_size - 1)) ? c_read_index = 0 : c_read_index++;
}

/* declare tasklet */
static DECLARE_TASKLET(t_asgn2, t_bottom_half, (unsigned long) &t_data);

/** 
 * This function is executed when an interrupt is received. It reads in a 
 * half byte and if there is enough data to assemble a full byte it does so
 * and then copies it to the circular buffer.
 */
irqreturn_t dummyport_interrupt(int irq, void *dev_id) {
  char new_byte;

  if (byte_switch == 0) {
    half_byte_1 = read_half_byte();
    byte_switch = 1;
  } else {
    half_byte_2 = read_half_byte();
    new_byte = half_byte_1 << 4 | half_byte_2;
    c_buff[c_write_index] = new_byte;
    (c_write_index == (c_buff_size - 1)) ? c_write_index = 0 : c_write_index++;
    byte_switch = 0;
    tasklet_schedule(&t_asgn2);
  }

  return IRQ_HANDLED;
}

/**
 * This function frees all memory pages held by the module.
 */
void free_memory_pages(void) {
  page_node *curr;
  /**
   * Loop through the entire page list {
   *   if (node has a page) {
   *     free the page
   *   }
   *   remove the node from the page list
   *   free the node
   * }
   * reset device data size, and num_pages
   */
  while (!list_empty(&asgn2_device.mem_list)) {
    curr = list_entry(asgn2_device.mem_list.next, page_node, list);
    if (NULL != curr->page) __free_page(curr->page);
    list_del(asgn2_device.mem_list.next);
    if (NULL != curr) kmem_cache_free(asgn2_device.cache, curr);
  }
  asgn2_device.data_size = 0;
  asgn2_device.num_pages = 0;
}


/**
 * This function opens the virtual disk, if it is opened in the write-only
 * mode, all memory pages will be freed.
 */
int asgn2_open(struct inode *inode, struct file *filp) {
  /**
   * Increment process count, if exceeds max_nprocs, return -EBUSY
   *
   * if opened in write-only mode, free all memory pages
   *
   */
  EOF = 0;
  wait_event_interruptible_exclusive(wait_access, 
				     atomic_read(&asgn2_device.in_use));
  atomic_dec(&asgn2_device.in_use);
  return 0; /* success */
}


/**
 * This function releases the virtual disk, but nothing needs to be done
 * in this case. 
 */
int asgn2_release (struct inode *inode, struct file *filp) {
  /**
   * increment in-use switch
   */
  atomic_inc(&asgn2_device.in_use);
  wake_up_interruptible(&wait_access);
  return 0;
}


/**
 * This function reads contents of the virtual disk and writes to the user 
 */
ssize_t asgn2_read(struct file *filp, char __user *buf, size_t count,
		 loff_t *f_pos) {
  size_t size_read = 0;     /* size read from virtual disk in this function */
  size_t curr_size_read;    /* size read from the virtual disk in this round */
  struct list_head *ptr = asgn2_device.mem_list.next; /* Current list entry */
  struct list_head *oldptr; /* temporary ptr used when deleting pages */
  page_node *curr = list_entry(ptr, page_node, list); /* get page node */
  int i;                    /* page read index */

 retry:
  count = min((size_t)(asgn2_device.data_size - asgn2_device.read_offset), 
	      (size_t)count);
  while (size_read < count) {
    /* Exit if EOF already reached or no more pages */
    if (EOF == 1 || ptr == &asgn2_device.mem_list) return size_read;

    /* Iterate all chars in page between read and write offsets */
    for (i = curr->read_offset; i < curr->write_offset; i++) {
      size_read++;
      curr->read_offset += 1;
      asgn2_device.read_offset += 1;

      /* Detect NULL/EOF character and prepare to end read */
      if (*((unsigned char*)page_address(curr->page)+i) == '\0'){
	EOF = 1;
	size_read -= 1; /* EOF char is not written so undo size_read inc */
	if (i+1 < PAGE_SIZE) {
	  goto next; /* EOF within page boundaries so advance and keep page */
	} else {
	  goto clear; /* EOF at page boundary so advance and remove page */
	}
      }
      do {
	curr_size_read = copy_to_user(buf + size_read,
				      page_address(curr->page) + i,
				      1);
      } while (curr_size_read == 1);

      *f_pos += 1; /* increment file pointer */
    }

    /* If read has caught up to write wait until data is available */
    if (curr->write_offset < PAGE_SIZE && curr->read_offset == 
	curr->write_offset && EOF == 0) {
      atomic_dec(&asgn2_device.rw_sync);
      wait_event_interruptible(wait_data, atomic_read(&asgn2_device.rw_sync));
      goto retry;
    }

  clear:
    if (ptr == asgn2_device.write_node) asgn2_device.write_node = 
					  &asgn2_device.mem_list;
    oldptr = ptr;                             /* remember current page */
    ptr = ptr->next;                          /* prepare next page */
    asgn2_device.data_size -= curr->write_offset;
    asgn2_device.read_offset -= curr->write_offset;
    list_del(oldptr);                         /* delete old page */
    asgn2_device.num_pages -= 1;              /* decrement page count */
  next:
    curr = list_entry(ptr, page_node, list);  /* get page node for ptr */
  }

  return size_read;
}


/**
 * The ioctl function, which nothing needs to be done in this case.
 */
long asgn2_ioctl (struct file *filp, unsigned cmd, unsigned long arg) {
  /** 
   * check whether cmd is for our device, if not for us, return -EINVAL 
   */

  if (_IOC_TYPE(cmd) != MYIOC_TYPE) {

    printk(KERN_WARNING "%s: magic number does not match\n", MYDEV_NAME);
    return -EINVAL;
  }

  return -ENOTTY;
}


/**
 * Displays information about current status of the module,
 * which helps debugging.
 */
int asgn2_read_procmem(char *buf, char **start, off_t offset, int count,
		     int *eof, void *data) {
  int result;
  /**
   * use snprintf to print some info to buf, up to size count
   * set eof
   */
  result = snprintf(buf, count, 
		    "major = %d\nnumber of pages = %d\ndata size = %u\n"
                    "disk size = %d\n",
	            asgn2_major, asgn2_device.num_pages, 
                    asgn2_device.data_size, 
                    (int)(asgn2_device.num_pages * PAGE_SIZE));
    
  *eof = 1; /* end of file */
  return result;
}


struct file_operations asgn2_fops = {
  .owner = THIS_MODULE,
  .read = asgn2_read,
  .unlocked_ioctl = asgn2_ioctl,
  .open = asgn2_open,
  .release = asgn2_release
};


/**
 * Initialise the module and create the master device
 */
int __init asgn2_init_module(void){
  int result;
  int i;

  /**
   * set nprocs and max_nprocs of the device
   *
   * allocate major number
   * allocate cdev, and set ops and owner field 
   * add cdev
   * initialize the page list
   * create proc entries
   */

  atomic_set(&asgn2_device.in_use, 1);

  result = alloc_chrdev_region(&asgn2_device.dev, asgn2_minor, 
                               asgn2_dev_count, MYDEV_NAME);

  if (result < 0) {
    printk(KERN_WARNING "asgn2: can't get major number\n");
    return -EBUSY;
  }

  asgn2_major = MAJOR(asgn2_device.dev);

  if (NULL == (asgn2_device.cdev = cdev_alloc())) {
    printk(KERN_WARNING "%s: can't allocate cdev\n", MYDEV_NAME);
    result = -ENOMEM;
    goto fail_cdev;
  }

  asgn2_device.cdev->ops = &asgn2_fops;
  asgn2_device.cdev->owner = THIS_MODULE;
  
  result = cdev_add(asgn2_device.cdev, asgn2_device.dev, asgn2_dev_count);
  if (result < 0) {
    printk(KERN_WARNING "%s: can't register chrdev_region to the system\n",
           MYDEV_NAME);
    goto fail_cdev;
  }
  
  /* allocate pages */
  INIT_LIST_HEAD(&asgn2_device.mem_list);
  asgn2_device.num_pages = 0;
  asgn2_device.data_size = 0;
  asgn2_device.write_node = &asgn2_device.mem_list;

  if (NULL == create_proc_read_entry(MYDEV_NAME, 
				     0, /* default mode */ 
				     NULL, /* parent dir */
				     asgn2_read_procmem,
				     NULL /* client data */)) {
    printk(KERN_WARNING "%s: can't create procfs entry\n", MYDEV_NAME);
    result = -ENOMEM;
    goto fail_proc_entry;
  }

  asgn2_device.cache = kmem_cache_create(MYDEV_NAME, sizeof(page_node), 
                                         0, 0, NULL); 
  
  if (NULL == asgn2_device.cache) {
    printk(KERN_WARNING "%s: can't create cache\n", MYDEV_NAME);
    result = -ENOMEM;
    goto fail_kmem_cache_create;
  }
 
  asgn2_device.class = class_create(THIS_MODULE, MYDEV_NAME);
  if (IS_ERR(asgn2_device.class)) {
    printk(KERN_WARNING "%s: can't create udev class\n", MYDEV_NAME);
    result = -ENOMEM;
    goto fail_class;
  }

  asgn2_device.device = device_create(asgn2_device.class, NULL, 
                                      asgn2_device.dev, "%s", MYDEV_NAME);
  if (IS_ERR(asgn2_device.device)) {
    printk(KERN_WARNING "%s: can't create udev device\n", MYDEV_NAME);
    result = -ENOMEM;
    goto fail_device;
  }

  gpio_dummy_init();
  c_buff = kmalloc(c_buff_size, GFP_KERNEL);
  for (i=0; i < c_buff_size; i++) {
    (i == (c_buff_size - 1)) ? (c_buff[i] = '\0') : (c_buff[i] = '0');
  }

  printk(KERN_WARNING "set up udev entry\n");
  printk(KERN_WARNING "%s started successfully\n", MYDEV_NAME);
  return 0;

  /* cleanup code called when any of the initialization steps fail */
fail_device:
   class_destroy(asgn2_device.class);
fail_class:
   kmem_cache_destroy(asgn2_device.cache);  
fail_kmem_cache_create:
  remove_proc_entry(MYDEV_NAME, NULL /* parent dir */);
fail_proc_entry:
  cdev_del(asgn2_device.cdev);
fail_cdev:
  unregister_chrdev_region(asgn2_device.dev, asgn2_dev_count);
  return result;
}


/**
 * Finalise the module
 */
void __exit asgn2_exit_module(void){
  device_destroy(asgn2_device.class, asgn2_device.dev);
  class_destroy(asgn2_device.class);
  printk(KERN_WARNING "cleaned up udev entry\n");
  
  /**
   * free all pages in the page list 
   * cleanup in reverse order
   */
  free_memory_pages();
  kmem_cache_destroy(asgn2_device.cache);
  remove_proc_entry(MYDEV_NAME, NULL /* parent dir */);
  cdev_del(asgn2_device.cdev);
  unregister_chrdev_region(asgn2_device.dev, asgn2_dev_count);

  gpio_dummy_exit();
  kfree(c_buff);

  printk(KERN_WARNING "%s exited successfully\n", MYDEV_NAME);
}


module_init(asgn2_init_module);
module_exit(asgn2_exit_module);
