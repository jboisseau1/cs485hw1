#include "BugAlgorithms.hpp"

BugAlgorithms::BugAlgorithms(Simulator * const simulator)
{
    m_simulator = simulator;
    //add your initialization of other variables
    //that you might have declared

    m_mode = STRAIGHT;
    m_hit[0] = m_hit[1] = HUGE_VAL;
    m_leave[0] = m_leave[1] = 0;
    m_distLeaveToGoal = HUGE_VAL;
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
// printf("d: %lf mag:%lf\n",sensor.m_dmin, magnitude_old );
double magnitude_new = sqrt(newD_min.m_dx * newD_min.m_dx + newD_min.m_dy * newD_min.m_dy);
 //hits an obstacle
 if(sensor.m_dmin <= 0.8){
     double obsVectorX = sensor.m_xmin - m_simulator -> GetRobotCenterX();
     double obsVectorY = sensor.m_ymin - m_simulator -> GetRobotCenterY();
     double magnitude = sqrt(obsVectorX * obsVectorX + obsVectorY * obsVectorY);
     Move move = {(-obsVectorY/magnitude)*0.06, (obsVectorX/magnitude)*0.06};
     if(magnitude_new < magnitude_old){ //always true...
       printf("new: %lf d: %lf\n", magnitude_new, magnitude_old);
       return move;
     }
     else if(sensor.m_dmin>=0.5){
       printf("new: %lf d: %lf\n", magnitude_new, magnitude_old);
       printf("IM CALLED\n");
       return MoveTowardsGoal();
     }
 }

 else{
   // printf("moving to goal\n");
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

    //add your implementation
    Move move ={0,0};

    return move;
}

Move BugAlgorithms::MoveTowardsGoal(){

double roboX = m_simulator -> GetRobotCenterX();
double roboY = m_simulator -> GetRobotCenterY();
double goalX = m_simulator -> GetGoalCenterX();
double goalY = m_simulator -> GetGoalCenterY();
double dx = (goalX - roboX);
double dy = (goalY - roboY);
double mag = sqrt(dx * dx + dy * dy);

Move newMove ={(dx/mag)*0.06, (dy/mag)*0.06};
return newMove;
}
