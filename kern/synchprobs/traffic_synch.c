#include <types.h>
#include <lib.h>
#include <synchprobs.h>
#include <synch.h>
#include <opt-A1.h>
#include <array.h>

static struct cv *cvNE;
static struct cv *cvNS;
static struct cv *cvNW;
static struct cv *cvEN;
static struct cv *cvES;
static struct cv *cvEW;
static struct cv *cvSN;
static struct cv *cvSE;
static struct cv *cvSW;
static struct cv *cvWN;
static struct cv *cvWE;
static struct cv *cvWS;
static struct lock *lk;
static struct array *waiting;
int intersection[16];

struct vehicle{
  Direction origin;
  Direction destination;
};

static bool turnRight(Direction origin, Direction destination){
  if(((origin == north) && (destination == west)) ||
    ((origin == south) && (destination == east)) ||
    ((origin == west) && (destination == south)) ||
    ((origin == east) && (destination == north))) {
    return true;
  } else {
    return false;
  }
}

static bool passable(Direction origin1, Direction destination1,
                     Direction origin2, Direction destination2){
  if((origin1 == origin2) ||
    ((origin1 == destination2) && (destination1 == origin2)) ||
    ((turnRight(origin1, destination1) || turnRight(origin2, destination2))
     && (destination1 != destination2))){
    return true;
  } else {
    return false;
  }
}

/*
 * The simulation driver will call this function once before starting
 * the simulation
 *
 * You can use it to initialize synchronization and other variables.
 *
 */
void
intersection_sync_init(void)
{
  lk = lock_create("lk");
  cvNE = cv_create("cvNE");
  cvNS = cv_create("cvNS");
  cvNW = cv_create("cvNW");
  cvEN = cv_create("cvEN");
  cvES = cv_create("cvES");
  cvEW = cv_create("cvEW");
  cvSN = cv_create("cvSN");
  cvSE = cv_create("cvSE");
  cvSW = cv_create("cvSW");
  cvWN = cv_create("cvWN");
  cvWE = cv_create("cvWE");
  cvWS = cv_create("cvWS");
  waiting = array_create();
  array_init(waiting);
  for(int i = 0; i < 16; i++){
    intersection[i] = 0;
  }
  if (waiting == NULL || lk == NULL || cvNE == NULL  || cvNS == NULL ||
      cvNW == NULL || cvEN == NULL || cvES == NULL  || cvEW == NULL ||
      cvSN == NULL || cvSE == NULL || cvSW == NULL  || cvWN == NULL ||
      cvWE == NULL || cvWS == NULL || intersection == NULL) {
    panic("could not create intersection");
  }
  return;
}

/*
 * The simulation driver will call this function once after
 * the simulation has finished
 *
 * You can use it to clean up any synchronization and other variables.
 *
 */
void
intersection_sync_cleanup(void)
{
  KASSERT(lk != NULL);
  KASSERT(cvNE != NULL);
  KASSERT(cvEN != NULL);
  KASSERT(cvSE != NULL);
  KASSERT(cvWE != NULL);
  KASSERT(cvNW != NULL);
  KASSERT(cvES != NULL);
  KASSERT(cvSN != NULL);
  KASSERT(cvWS != NULL);
  KASSERT(cvNS != NULL);
  KASSERT(cvEW != NULL);
  KASSERT(cvSW != NULL);
  KASSERT(cvWN != NULL);

  cv_destroy(cvNE);
  cv_destroy(cvEN);
  cv_destroy(cvSE);
  cv_destroy(cvWE);
  cv_destroy(cvNW);
  cv_destroy(cvES);
  cv_destroy(cvSN);
  cv_destroy(cvWS);
  cv_destroy(cvNS);
  cv_destroy(cvEW);
  cv_destroy(cvSW);
  cv_destroy(cvWN);
  lock_destroy(lk);
  array_destroy(waiting);
}


/*
 * The simulation driver will call this function each time a vehicle
 * tries to enter the intersection, before it enters.
 * This function should cause the calling simulation thread
 * to block until it is OK for the vehicle to enter the intersection.
 *
 * parameters:
 *    * origin: the Direction from which the vehicle is arriving
 *    * destination: the Direction in which the vehicle is trying to go
 *
 * return value: none
 */

void
intersection_before_entry(Direction origin, Direction destination)
{
  KASSERT(lk != NULL);
  KASSERT(cvNE != NULL);
  KASSERT(cvEN != NULL);
  KASSERT(cvSE != NULL);
  KASSERT(cvWE != NULL);
  KASSERT(cvNW != NULL);
  KASSERT(cvES != NULL);
  KASSERT(cvSN != NULL);
  KASSERT(cvWS != NULL);
  KASSERT(cvNS != NULL);
  KASSERT(cvEW != NULL);
  KASSERT(cvSW != NULL);
  KASSERT(cvWN != NULL);
  KASSERT(intersection != NULL);

  lock_acquire(lk);
  unsigned index;
  struct vehicle *v = kmalloc(sizeof(struct vehicle));
  KASSERT(v != NULL);
  v->origin = origin;
  v->destination = destination;
  while(true){
    bool wait = false;
    for(unsigned i = 0; i < 16; i++){
      if(intersection[i] > 0){
        if(!passable(origin, destination, i/4, i%4)){
          array_add(waiting, v, &index);
          switch(4 * origin + destination){
            case 1: cv_wait(cvNE, lk);break;
            case 2: cv_wait(cvNS, lk);break;
            case 3: cv_wait(cvNW, lk);break;
            case 4: cv_wait(cvEN, lk);break;
            case 6: cv_wait(cvES, lk);break;
            case 7: cv_wait(cvEW, lk);break;
            case 8: cv_wait(cvSN, lk);break;
            case 9: cv_wait(cvSE, lk);break;
            case 11: cv_wait(cvSW, lk);break;
            case 12: cv_wait(cvWN, lk);break;
            case 13: cv_wait(cvWE, lk);break;
            case 14: cv_wait(cvWS, lk);break;
          }
          wait = true;
          break;
        }
      }
    }

    if(!wait){
      unsigned indexNewVehicle = 4 * origin + destination;
      KASSERT(indexNewVehicle < 16);
      intersection[indexNewVehicle]++;
      //kfree(v);
      break;
    }
  }
  lock_release(lk);
}

/*
 * The simulation driver will call this function each time a vehicle
 * leaves the intersection.
 *
 * parameters:
 *    * origin: the Direction from which the vehicle arrived
 *    * destination: the Direction in which the vehicle is going
 *
 * return value: none
 */

void
intersection_after_exit(Direction origin, Direction destination)
{
  KASSERT(lk != NULL);
  KASSERT(cvNE != NULL);
  KASSERT(cvEN != NULL);
  KASSERT(cvSE != NULL);
  KASSERT(cvWE != NULL);
  KASSERT(cvNW != NULL);
  KASSERT(cvES != NULL);
  KASSERT(cvSN != NULL);
  KASSERT(cvWS != NULL);
  KASSERT(cvNS != NULL);
  KASSERT(cvEW != NULL);
  KASSERT(cvSW != NULL);
  KASSERT(cvWN != NULL);
  KASSERT(intersection != NULL);

  lock_acquire(lk);
  bool broadcastable = true;
  //we first update the intersection matrix
  unsigned indexCurVehicle = 4 * origin + destination;
  KASSERT(indexCurVehicle < 16);
  intersection[indexCurVehicle]--;

  if(array_num(waiting) != 0){
    //we then check if the first waiting car has a conflict
    struct vehicle *first= array_get(waiting, 0);
    Direction firstOrigin = first->origin;
    Direction firstDestination = first->destination;
    for(unsigned i = 0; i < 16; i++){
      if(intersection[i] > 0){
        if(!passable(firstOrigin, firstDestination, i/4, i%4)){
            broadcastable = false;
            lock_release(lk);
            return;
        }
      }
    }
    //if the first waiting car does not conflict, we then broadcast on it
    if(broadcastable){
      switch(4 * firstOrigin + firstDestination){
        case 1: cv_broadcast(cvNE, lk);break;
        case 2: cv_broadcast(cvNS, lk);break;
        case 3: cv_broadcast(cvNW, lk);break;
        case 4: cv_broadcast(cvEN, lk);break;
        case 6: cv_broadcast(cvES, lk);break;
        case 7: cv_broadcast(cvEW, lk);break;
        case 8: cv_broadcast(cvSN, lk);break;
        case 9: cv_broadcast(cvSE, lk);break;
        case 11: cv_broadcast(cvSW, lk);break;
        case 12: cv_broadcast(cvWN, lk);break;
        case 13: cv_broadcast(cvWE, lk);break;
        case 14: cv_broadcast(cvWS, lk);break;
      }
      //delete the cars that are in the waiting list
      for(unsigned i = 0; i < array_num(waiting); i++){
        struct vehicle *v = array_get(waiting, i);
        if(v->origin == firstOrigin && v->destination == firstDestination){
          array_remove(waiting, i);
          i--;
        }
      }
    }
  }
  lock_release(lk);
}
