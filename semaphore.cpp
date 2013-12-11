#include "semaphore.h"
#include <sys/ipc.h>
#include <sys/sem.h>

struct semun {
  int val;
  struct semid_ds * buf;
  unsigned short int * array;
  struct seminfo * __buf;
};

Semaphore::Semaphore( int key )
{
  id_ = semget ( key, 1, IPC_CREAT | 0777 );
  semun argument;
  unsigned short values[1];
  values[0] = 1;
  argument.array = values;
  semctl( id_, 0, SETALL, argument );
}

Semaphore::~Semaphore()
{
  semun ignoredArgument;
  semctl( id_, 1, IPC_RMID, ignoredArgument );
}

int 
Semaphore::wait()
{
  struct sembuf operations[1];
  operations[0].sem_num = 0;
  operations[0].sem_op = -1;
  operations[0].sem_flg = SEM_UNDO;
  return semop( id_, operations, 1 );
}

int 
Semaphore::post()
{
  struct sembuf operations[1];
  operations[0].sem_num = 0;
  operations[0].sem_op = 1;
  operations[0].sem_flg = SEM_UNDO;
  return semop( id_, operations, 1 );
}
