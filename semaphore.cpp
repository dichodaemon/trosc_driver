#include "semaphore.h"
#include <sys/sem.h>
#include <sys/ipc.h>
#include <sys/types.h>
#include <iostream>
#include <string.h>
#include <errno.h>

//struct semun {
  //int val;
  //struct semid_ds * buf;
  //unsigned short int * array;
  //struct seminfo * __buf;
//};

Semaphore::Semaphore( int key )
{
  id_ = semget ( key, 1, IPC_CREAT | 0777 );
  std::cerr << "Created semaphore, return code:" << id_ << std::endl;
  //semun argument;
  //unsigned short values[1];
  //values[0] = 1;
  //argument.array = values;
  //int result = semctl( id_, 0, SETALL, argument );
  int result = semctl( id_, 0, SETALL, 1 );
  if ( result != 0 ) {
    std::cerr << "Problem initializing semaphore:" << strerror( errno ) << std::endl;
  }
}

Semaphore::~Semaphore()
{
  struct sembuf ignoredArgument;
  semctl( id_, 1, IPC_RMID, ignoredArgument );
  std::cerr << "Semaphore has been destroyed\n";
}

int 
Semaphore::wait()
{
  struct sembuf parameters;
  parameters.sem_num = 0;
  parameters.sem_op = -1;
  parameters.sem_flg = SEM_UNDO;
  return semop( id_, &parameters, 1 );
}

int 
Semaphore::post()
{
  struct sembuf parameters;
  parameters.sem_num = 0;
  parameters.sem_op = 1;
  parameters.sem_flg = SEM_UNDO;
  return semop( id_, &parameters, 1 );
}
