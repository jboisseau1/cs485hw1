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

 //hits an obstacle
 if(sensor.m_dmin <= 0.8){
     double obsVectorX = sensor.m_xmin - m_simulator -> GetRobotCenterX();
     double obsVectorY = sensor.m_ymin - m_simulator -> GetRobotCenterY();
     double magnitude = sqrt(obsVectorX * obsVectorX + obsVectorY * obsVectorY);
     Move move = {(-obsVectorY/magnitude)*0.06, (obsVectorX/magnitude)*0.06};

     return move;
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
