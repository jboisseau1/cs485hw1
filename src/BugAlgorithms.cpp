#include "BugAlgorithms.hpp"
#include <cmath>
#include <cstdlib>
#define BUMPERSIZE 0.8
#define INNERSIZE 0.5
#define MOVESIZE 0.06

BugAlgorithms::BugAlgorithms(Simulator * const simulator)
{
    m_simulator = simulator;
    //add your initialization of other variables
    //that you might have declared

    m_mode = STRAIGHT;
    m_hit[0] = m_hit[1] = HUGE_VAL;
    m_leave[0] = m_leave[1] = 0;
    m_distLeaveToGoal = HUGE_VAL;

    //vals for Bug2
    trackWall = false;
    initialSensor.m_xmin = 0.0;
    initialSensor.m_ymin = 0.0;
    initialSensor.m_dmin = 0.0;
    goalMove = MoveTowardsGoal();

    goalSlope = abs(getSlope());
    distanceToGoal = 0.0;//m_simulator->GetDistanceFromRobotToGoal();
}

BugAlgorithms::~BugAlgorithms(void)
{
    //do not delete m_simulator
}

Move BugAlgorithms::Bug0(Sensor sensor)
{
Move oldD_min = {m_simulator -> GetRobotCenterX() - sensor.m_xmin, m_simulator -> GetRobotCenterY() - sensor.m_ymin};
Move move_goal = MoveTowardsGoal();
Move newD_min = {0,0};
newD_min.m_dx = oldD_min.m_dx + move_goal.m_dx;
newD_min.m_dy = oldD_min.m_dy + move_goal.m_dy;
double magnitude_old = sqrt( oldD_min.m_dx * oldD_min.m_dx + oldD_min.m_dy * oldD_min.m_dy);
double magnitude_new = sqrt(newD_min.m_dx * newD_min.m_dx + newD_min.m_dy * newD_min.m_dy);
 //hits an obstacle
 if(sensor.m_dmin <= BUMPERSIZE){
     double obsVectorX = sensor.m_xmin - m_simulator -> GetRobotCenterX();
     double obsVectorY = sensor.m_ymin - m_simulator -> GetRobotCenterY();
     double magnitude = sqrt(obsVectorX * obsVectorX + obsVectorY * obsVectorY);
     Move move = {(-obsVectorY/magnitude)*MOVESIZE, (obsVectorX/magnitude)*MOVESIZE};
     if(magnitude_new < magnitude_old){ //always true...

       return move;
     }
     else if(sensor.m_dmin>=INNERSIZE){
       return MoveTowardsGoal();
     }
 }

 else{
   return MoveTowardsGoal();
 }
}

Move BugAlgorithms::Bug1(Sensor sensor)
{
    //add your implementation
    Move move ={0,0};

    return move;
}

Move BugAlgorithms::Bug2(Sensor sensor)
{
  //getchar(); //*****this makes it wait till enter is pressed
  
    //add your implementation
    //Move move ={0,0};
  if(trackWall == false){
    goalSlope = abs(getSlope());
  }
  //printf("new goalSlope %lf\n", abs(getSlope()));
  //printf("goalSlope %lf\n", goalSlope);
  if(sensor.m_dmin <= BUMPERSIZE || trackWall == true){
    if(initialSensor.m_dmin == 0.0){
      initialSensor.m_dmin = sensor.m_dmin;
      //initialSensor.m_xmin = sensor.m_xmin;
      //initialSensor.m_ymin = sensor.m_ymin;
      distanceToGoal = m_simulator->GetDistanceFromRobotToGoal();
      printf("set initial \n");
      trackWall = true;
      return MoveAroundObstacle(sensor);

    }

    /*trackWall = true;
    printf("goalSlope %lf\n", goalSlope);
    printf("new goalSlope %lf\n", abs(getSlope()));
    printf("initial %lf\n", initialSensor.m_dmin);*/
    else if( ( goalSlope - 0.005 <= abs(getSlope()) && goalSlope + 0.005 >= abs(getSlope()))  && initialSensor.m_dmin != 0.0 && distanceToGoal - 1 > m_simulator->GetDistanceFromRobotToGoal()){
      printf("In here\n" );
      trackWall = false;
      initialSensor.m_dmin = 0;
      distanceToGoal = m_simulator->GetDistanceFromRobotToGoal();
      return MoveTowardsGoal();
    }
    /*else if((goalSlope - 0.005 <= abs(getSlope()) && goalSlope + 0.005 >= abs(getSlope()))){
      printf("slope good\n");
      if(initialSensor.m_dmin != 0.0){
        printf("sensor good\n");
        if(distanceToGoal > m_simulator->GetDistanceFromRobotToGoal()){
          printf("distanceToGoal good\n");
          trackWall = false;
          return MoveTowardsGoal();
        }
        trackWall = false;
        return MoveTowardsGoal();
      }
      return MoveTowardsGoal();
    }*/
    else{
      printf("move around\n" );
      return MoveAroundObstacle(sensor);
    }
  }
  else{
    printf("Moving towards goal\n");
    return MoveTowardsGoal();
  }

}
double BugAlgorithms::getSlope(){
  return (m_simulator -> GetGoalCenterY() - m_simulator -> GetRobotCenterY())/(m_simulator -> GetGoalCenterX() - m_simulator -> GetRobotCenterX());
}

Move BugAlgorithms::MoveAroundObstacle(Sensor sensor){
/*Move oldD_min = {m_simulator -> GetRobotCenterX() - sensor.m_xmin, m_simulator -> GetRobotCenterY() - sensor.m_ymin};
Move move_goal = MoveTowardsGoal();
Move newD_min = {0,0};
newD_min.m_dx = oldD_min.m_dx + move_goal.m_dx;
newD_min.m_dy = oldD_min.m_dy + move_goal.m_dy;
double magnitude_old = sqrt( oldD_min.m_dx * oldD_min.m_dx + oldD_min.m_dy * oldD_min.m_dy);
// printf("d: %lf mag:%lf\n",sensor.m_dmin, magnitude_old );
double magnitude_new = sqrt(newD_min.m_dx * newD_min.m_dx + newD_min.m_dy * newD_min.m_dy);*/



  double obsVectorX = sensor.m_xmin - m_simulator -> GetRobotCenterX();
  double obsVectorY = sensor.m_ymin - m_simulator -> GetRobotCenterY();
  double magnitude = sqrt(obsVectorX * obsVectorX + obsVectorY * obsVectorY);
  Move newMove = {(-obsVectorY/magnitude)*MOVESIZE, (obsVectorX/magnitude)*MOVESIZE};

     /*if(magnitude_new < magnitude_old){ //always true...
       printf("new: %lf d: %lf\n", magnitude_new, magnitude_old);
       return newMove;
     }
     else if(sensor.m_dmin>=INNERSIZE){
       printf("new: %lf d: %lf\n", magnitude_new, magnitude_old);
       printf("IM CALLED\n");
       return MoveTowardsGoal();
     }*/
  printf("move MoveAroundObstacle\n");
  return newMove;
}

Move BugAlgorithms::MoveTowardsGoal(){

double roboX = m_simulator -> GetRobotCenterX();
double roboY = m_simulator -> GetRobotCenterY();
double goalX = m_simulator -> GetGoalCenterX();
double goalY = m_simulator -> GetGoalCenterY();
double dx = (goalX - roboX);
double dy = (goalY - roboY);
double mag = sqrt(dx * dx + dy * dy);

Move newMove ={(dx/mag)*MOVESIZE, (dy/mag)*MOVESIZE};
return newMove;
}
