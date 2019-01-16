#include <types.h>
#include <kern/errno.h>
#include <kern/unistd.h>
#include <kern/wait.h>
#include <lib.h>
#include <syscall.h>
#include <current.h>
#include <proc.h>
#include <thread.h>
#include <addrspace.h>
#include <copyinout.h>
#include <machine/trapframe.h>
#include <synch.h>
#include <array.h>
#include <vfs.h>
#include <limits.h>
#include <kern/fcntl.h>
#include <test.h>
#include "opt-A2.h"

  /* this implementation of sys__exit does not do anything with the exit code */
  /* this needs to be fixed to get exit() and waitpid() working properly */
#if OPT_A2
  struct lock *lock;
  volatile pid_t pidCounter;
  struct array *infoList;
  struct array *procList;
  int sys_fork(struct trapframe *tf, pid_t *retval){
      struct proc *parent = curproc;
      //Create and copy process structure for child process
      struct proc *child = proc_create_runprogram(curproc->p_name);
      if(child == NULL) return(ENOMEM);
      as_copy(curproc_getas(), &(child->p_addrspace));
      if(child->p_addrspace == NULL){
        proc_destroy(child);
        return(ENOMEM);
      }

      //Assign PID to child process and create the parent/child relationship
      lock_acquire(lock);
      array_add(procList, child, NULL);
      child->parent = parent;
      child->pid = ++pidCounter;
      child->parentPid = parent->pid;
      lock_release(lock);

      //Create thread for child process
      struct trapframe *child_tf = kmalloc(sizeof(struct trapframe));
      if(child_tf == NULL) return(ENOMEM);
      memcpy(child_tf, tf, sizeof(struct trapframe));
      int errCode = thread_fork(curproc->p_name, child, (void*)enter_forked_process, child_tf, 0);
      if(errCode){
        proc_destroy(child);
        kfree(child_tf);
        return errCode;
      }
      *retval = child->pid;
      return 0;
  }
#endif /* OPT_A2 */


void sys__exit(int exitcode) {
  struct addrspace *as;
  struct proc *p = curproc;
  /* for now, just include this to keep the compiler from complaining about
     an unused variable */

  #if OPT_A2
  bool isParentAlive = false;
  p->done = true;
  struct info *curProcInfo = kmalloc(sizeof(struct info));
  curProcInfo->curPid = p->pid;
  curProcInfo->exitCode = exitcode;

  for(unsigned int i = 0; i < array_num(procList); i++){
    struct proc *temp = array_get(procList, i);
    if(temp->pid == p->parentPid){
      //check if p's parent is still alive
      isParentAlive = true;
    }
    if(temp->pid == p->pid){
      //if i'm the active, i need to delete myself
      array_remove(procList, i);
    }
  }
  if(isParentAlive){
    lock_acquire(lock);
    array_add(infoList, curProcInfo, NULL);
    cv_broadcast(p->parent->procCv, lock);
    lock_release(lock);
  }
  #else
    (void)exitcode;
  #endif


  DEBUG(DB_SYSCALL,"Syscall: _exit(%d)\n",exitcode);

  KASSERT(curproc->p_addrspace != NULL);
  as_deactivate();
  /*
   * clear p_addrspace before calling as_destroy. Otherwise if
   * as_destroy sleeps (which is quite possible) when we
   * come back we'll be calling as_activate on a
   * half-destroyed address space. This tends to be
   * messily fatal.
   */
  as = curproc_setas(NULL);
  as_destroy(as);

  /* detach this thread from its process */
  /* note: curproc cannot be used after this call */
  proc_remthread(curthread);

  /* if this is the last user process in the system, proc_destroy()
     will wake up the kernel menu thread */
  proc_destroy(p);

  thread_exit();
  /* thread_exit() does not return, so we should never get here */
  panic("return from thread_exit in sys_exit\n");
}


/* stub handler for getpid() system call                */
int
sys_getpid(pid_t *retval)
{
  /* for now, this is just a stub that always returns a PID of 1 */
  /* you need to fix this to make it work properly */
  #if OPT_A2
  *retval = curproc->pid;
  return(0);
  #else
  *retval = 1;
  return(0);
  #endif
}

/* stub handler for waitpid() system call                */

int
sys_waitpid(pid_t pid, userptr_t status, int options, pid_t *retval){
  int exitstatus;
  int result;
  if (options != 0) {
    return(EINVAL);
  }

#if OPT_A2
    lock_acquire(lock);
    while(true){
      struct info *child = NULL;
      bool found = false;
      for(unsigned int i = 0; i < array_num(infoList); i++){
        struct info *temp = array_get(infoList, i);
        if(pid == temp->curPid){
          found = true;
          child = temp;
          array_remove(infoList, i);
          break;
        }
      }
      if(!found){
        cv_wait(curproc->procCv, lock);
        //kprintf("1");
      } else {
        lock_release(lock);
        exitstatus = _MKWAIT_EXIT(child->exitCode);
        break;
      }
    }
#else
  /* for now, just pretend the exitstatus is 0 */
  exitstatus = 0;
#endif

  result = copyout((void *)&exitstatus,status,sizeof(int));
  if (result) {
    return(result);
  }
  *retval = pid;
  return(0);
}

#if OPT_A2
int sys_execv(userptr_t program, userptr_t args){
  int result;
  int totalArgsLen = 0;
  size_t argc = 0;
  vaddr_t entrypoint, stackptr;
  struct vnode *v;
  struct addrspace *as;
  struct addrspace *prev = curproc_getas();

  if(program == NULL || args == NULL) return EFAULT;
  //count number of arguments, check for errors
  // while((userptr_t)args[argc] != NULL){
  //   if(argc == 0 && (strlen((char*)args[argc]) + 1) > PATH_MAX) return E2BIG;
  //   totalArgsLen += strlen((char*)args[argc] + 1);
  //   argc++;
  // }
  // if(totalArgsLen > ARG_MAX) return E2BIG;
  for(userptr_t *argc_ptr = (userptr_t*)args; argc_ptr[argc] != NULL; argc++){
    if(argc == 0 && (strlen((char*)argc_ptr[argc]) + 1) > PATH_MAX) return E2BIG;
    totalArgsLen += strlen((char*)argc_ptr[argc]) + 1;
  }
  if(totalArgsLen > ARG_MAX) return E2BIG;
  //passing arguments into Kernel
  char **kernArgs = kmalloc(sizeof(char*) * (argc + 1));
  if(kernArgs == NULL) return ENOMEM;
  for(unsigned int i = 0; i < argc; i++){
    userptr_t *argc_ptr = (userptr_t*)args;
    kernArgs[i] = kmalloc(sizeof(char) * (strlen((char*)argc_ptr[i]) + 1));
    if(kernArgs[i] == NULL){
      for(unsigned int k = 0; k < i; k++) kfree(kernArgs[k]);
      kfree(kernArgs);
      return ENOMEM;
    }
    result = copyinstr(((userptr_t*)args)[i], kernArgs[i],
                      strlen((char*)argc_ptr[i]) + 1, NULL);
    if(result){
      for(unsigned int k = 0; k < i; k++) kfree(kernArgs[k]);
      kfree(kernArgs);
      return result;
    }
  }
  kernArgs[argc] = '\0';
  //passing program into kernel
  char *kernProg = kmalloc(strlen((char*)program) + 1);
  if(kernProg == NULL) return ENOMEM;
  result = copyinstr(program, kernProg, strlen((char*)program) + 1, NULL);
  if(result){
    for(unsigned int k = 0; k < argc; k++) kfree(kernArgs[k]);
    kfree(kernArgs);
    kfree(kernProg);
    return result;
  }
  //open the program file
  char *fname_temp;
  fname_temp = kstrdup((char*)program);
  if(fname_temp == NULL) return ENOMEM;
  result = vfs_open(fname_temp, O_RDONLY, 0, &v);
  if(result) return result;
  kfree(fname_temp);

  /* Create a new address space. */
	as = as_create();
	if (as == NULL) {
		vfs_close(v);
		return ENOMEM;
	}

	/* Switch to it and activate it. */
	curproc_setas(as);
	as_activate();

	/* Load the executable. */
	result = load_elf(v, &entrypoint);
	if (result) {
		/* p_addrspace will go away when curproc is destroyed */
		vfs_close(v);
		return result;
	}

	/* Done with the file now. */
	vfs_close(v);

	/* Define the user stack in the address space */
	result = as_define_stack(as, &stackptr);
	if (result) {
		/* p_addrspace will go away when curproc is destroyed */
		return result;
	}
  //copy the arguments into the new address space
  stackptr -= stackptr % 8;
  vaddr_t argsPtr[argc + 1];
  for(int i = argc - 1; i >= 0; i--){
    stackptr -= strlen(kernArgs[i]) + 1;
    result = copyoutstr(kernArgs[i], (userptr_t)stackptr, strlen(kernArgs[i]) + 1, NULL);
    if(result){
      for(unsigned int k = 0; k < argc; k++) kfree(kernArgs[k]);
      kfree(kernArgs);
      kfree(kernProg);
      return result;
    }
    argsPtr[i] = stackptr;
  }
  argsPtr[argc] = '\0';
  stackptr -= stackptr % 4;
  for(int i = argc; i >= 0; i--){
    stackptr -= ROUNDUP(sizeof(vaddr_t), 4);
    result = copyout(&argsPtr[i], (userptr_t)stackptr, sizeof(vaddr_t));
    if(result){
      for(unsigned int k = 0; k < argc; k++) kfree(kernArgs[k]);
      kfree(kernArgs);
      kfree(kernProg);
      return result;
    }
  }
  //Delete old address space
  as_destroy(prev);
  for(unsigned int k = 0; k < argc; k++) kfree(kernArgs[k]);
  kfree(kernArgs);
  kfree(kernProg);
	/* Warp to user mode. */
	enter_new_process(argc, (userptr_t)stackptr, stackptr, entrypoint);

	/* enter_new_process does not return. */
	panic("enter_new_process returned\n");
	return EINVAL;
}
#endif
