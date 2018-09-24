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

        //vals for Bug1
        moveCounter = 0;
        lengthOfPathToLeave = 0;
        goRight = false;
        distanceToOb = 0.0;

        //vals for Bug2
        trackWall = false;
        initial = false;
        initialSensor.m_xmin = 0.0;
        initialSensor.m_ymin = 0.0;
        initialSensor.m_dmin = 0.0;
        goalMove = MoveTowardsGoal();

        goalSlope = getSlope();//abs(getSlope());
        distanceToGoal = 0.0;//m_simulator->GetDistanceFromRobotToGoal();
}

BugAlgorithms::~BugAlgorithms(void)
{
        //do not delete m_simulator
}

Move BugAlgorithms::Bug0(Sensor sensor){
        Move oldD_min = {m_simulator->GetRobotCenterX() - sensor.m_xmin, m_simulator->GetRobotCenterY() - sensor.m_ymin};
        Move move_goal = MoveTowardsGoal();
        Move newD_min = {0,0};
        newD_min.m_dx = oldD_min.m_dx + move_goal.m_dx;
        newD_min.m_dy = oldD_min.m_dy + move_goal.m_dy;
        double magnitude_old = sqrt( oldD_min.m_dx * oldD_min.m_dx + oldD_min.m_dy * oldD_min.m_dy);
        double magnitude_new = sqrt(newD_min.m_dx * newD_min.m_dx + newD_min.m_dy * newD_min.m_dy);
        //hits an obstacle
        if(sensor.m_dmin <= BUMPERSIZE) {
                double obsVectorX = sensor.m_xmin - m_simulator->GetRobotCenterX();
                double obsVectorY = sensor.m_ymin - m_simulator->GetRobotCenterY();
                double magnitude = sqrt(obsVectorX * obsVectorX + obsVectorY * obsVectorY);
                Move move = {(-obsVectorY/magnitude)*MOVESIZE, (obsVectorX/magnitude)*MOVESIZE};
                if(magnitude_new < magnitude_old) { //always true...

                        return move;
                }
                else if(sensor.m_dmin>=INNERSIZE) {
                        return MoveTowardsGoal();
                }
        }

        else{
                return MoveTowardsGoal();
        }
}

Move BugAlgorithms::Bug1(Sensor sensor)
{
        //dont remove the move init
        Move move ={0,0};

        //if bug is touching a wall and the current state is STRAIGHT (this means its the first time we have touched the wall)
        //switch to AROUND_AND_AWAY_FROM_HIT_POINT state and store start point. Also hit point is initial leave point
        if (sensor.m_dmin < BUMPERSIZE && m_mode == STRAIGHT) {
                m_mode = AROUND_AND_AWAY_FROM_HIT_POINT;
                m_hit[0] = m_simulator->GetRobotCenterX();
                m_hit[1] = m_simulator->GetRobotCenterY();

                distanceToGoal = m_simulator->GetDistanceFromRobotToGoal();
                m_leave[0] = m_simulator->GetRobotCenterX();
                m_leave[1] = m_simulator->GetRobotCenterY();

                //Changed to Move around obstacle so it doesnt start digging.
                moveCounter++;
                move = MoveAroundObstacle(sensor);
                move.m_dx = move.m_dx * 2;
                move.m_dy = move.m_dy * 2;

                distanceToOb = sensor.m_dmin;
                printf("Following object\n");
                return move;

        }
        //if in AROUND_AND_AWAY_FROM_HIT_POINT state follow wall left keep track of distance to
        //m_leave (increment lengthOfPathToLeave when it finds a closer point)
        else if (m_mode == AROUND_AND_AWAY_FROM_HIT_POINT) {

                //checks to see if its currently closer than the closest point(m_leave) stores whats in the
                //moveCounter into lengthOfPathToLeave and then resets the counter.
                if (m_simulator->GetDistanceFromRobotToGoal() < distanceToGoal) {
                        distanceToGoal = m_simulator->GetDistanceFromRobotToGoal();
                        lengthOfPathToLeave += moveCounter;
                        moveCounter = 0;
                        m_leave[0] = m_simulator->GetRobotCenterX();
                        m_leave[1] = m_simulator->GetRobotCenterY();
                }

                //check if the point is where we hit(within a threshold). if so switch to AROUND_AND_TOWARD_LEAVE_POINT
                //state and check which way to go based on sizes of moveCounter and LengthofPathToLeave.
                double hit_p_m = 0.08;
                if (((m_simulator->GetRobotCenterX() >= (m_hit[0] - hit_p_m) && m_simulator->GetRobotCenterX() <= (m_hit[0] + hit_p_m)) &&
                     (m_simulator->GetRobotCenterY() >= (m_hit[1] - hit_p_m) && m_simulator->GetRobotCenterY() <= (m_hit[1] + hit_p_m)))) {

                        m_mode = AROUND_AND_TOWARD_LEAVE_POINT;

                        if (lengthOfPathToLeave > moveCounter) {
                                goRight = true;
                        }
                        printf("Moving to *closest leave point using: %s", goRight? "Right path\n" : "Left path\n");
                        //empty move since we switched states
                        return move;
                }
                //adds up traditional MoveAroundObstacle vector (stored initially in move) and some small correction vector whose magnitude
                //is the difference (diff) between the distance between our robot and the obstacle when we initially hit and
                //how far the robot is now.


                move = MoveAroundObstacle(sensor);

                double diff = sensor.m_dmin - distanceToOb;

                //calculates the vector from point to obstacle and then normalizes so its a unit vector then we multiply by the diff
                //which is the magnitude that we want
                Move vectorToObj = {sensor.m_xmin - m_simulator->GetRobotCenterX(),  sensor.m_ymin - m_simulator->GetRobotCenterY()};
                double mag = sqrt((vectorToObj.m_dx*vectorToObj.m_dx) + (vectorToObj.m_dy*vectorToObj.m_dy));
                vectorToObj.m_dx = (vectorToObj.m_dx/mag)*diff;
                vectorToObj.m_dy = (vectorToObj.m_dy/mag)*diff;

                //this is the calculating of the new vector by adding the two
                move.m_dx = move.m_dx + vectorToObj.m_dx;
                move.m_dy = move.m_dy + vectorToObj.m_dy;



                moveCounter++;
                return move;
        }
        //once you start heading for leave point
        else if(m_mode == AROUND_AND_TOWARD_LEAVE_POINT) {
                double leave_p_m = 0.045;

                //if you hit the leave point switch to final mode (its for resetting variables and making sure we dont
                //catch the wall again
                if ((m_simulator->GetRobotCenterX() >= m_leave[0] - leave_p_m && m_simulator->GetRobotCenterX() <= m_leave[0] + leave_p_m) &&
                    (m_simulator->GetRobotCenterY() >= m_leave[1] - leave_p_m && m_simulator->GetRobotCenterY() <= m_leave[1] + leave_p_m)) {
                        m_mode = STRAIGHT_AND_AWAY_FROM_LEAVE_POINT;
                        //no change to move means empty move
                        printf("Moving to goal\n");
                }
                //keeps direction with goRight bool (just switches the sign on the movement)
                double diff = sensor.m_dmin - distanceToOb;

                //this is when shortest path involves moving right - pls give extra credit
                if (goRight) {
                        move = MoveAroundObstacle(sensor);
                        //same as above
                        Move vectorToObj = {sensor.m_xmin - m_simulator->GetRobotCenterX(),  sensor.m_ymin - m_simulator->GetRobotCenterY()};
                        double mag = sqrt((vectorToObj.m_dx*vectorToObj.m_dx) + (vectorToObj.m_dy*vectorToObj.m_dy));
                        vectorToObj.m_dx = (vectorToObj.m_dx/mag)*diff;
                        vectorToObj.m_dy = (vectorToObj.m_dy/mag)*diff;
                        //this is when it is moving right. (flips the vector by multipling by -1)
                        move.m_dx = (move.m_dx* -1) + vectorToObj.m_dx;
                        move.m_dy = (move.m_dy* -1) + vectorToObj.m_dy;

                        return move;
                }
                //this is when the shortest path is to move left
                else{
                        move = MoveAroundObstacle(sensor);
                        Move vectorToObj = {sensor.m_xmin - m_simulator->GetRobotCenterX(),  sensor.m_ymin - m_simulator->GetRobotCenterY()};
                        double mag = sqrt((vectorToObj.m_dx*vectorToObj.m_dx) + (vectorToObj.m_dy*vectorToObj.m_dy));
                        vectorToObj.m_dx = (vectorToObj.m_dx/mag)*diff;
                        vectorToObj.m_dy = (vectorToObj.m_dy/mag)*diff;

                        move.m_dx = move.m_dx + vectorToObj.m_dx;
                        move.m_dy = move.m_dy + vectorToObj.m_dy;

                        return move;
                }
                return move;
        }
        //resetting stuff and officially leaving the obstacle
        else if (m_mode == STRAIGHT_AND_AWAY_FROM_LEAVE_POINT) {
                m_mode = STRAIGHT;
                m_hit[0] = HUGE_VAL;
                m_hit[1] = HUGE_VAL;
                m_leave[0] = 0;
                m_leave[1] = 0;
                goRight = false;
                lengthOfPathToLeave = 0;
                moveCounter = 0;
                return MoveTowardsGoal();
        }
        //Initial state will be STRAIGHT (initialize above not in this function)
        else{
                return MoveTowardsGoal();
        }

        return move;
}

Move BugAlgorithms::Bug2(Sensor sensor)
{
        //getchar(); //*****this makes it wait till enter is pressed
        if(initial == false) {
                goalSlope = getSlope();
                printf("goal slope%lf\n", goalSlope);
        }
        initial = true;
        //add your implementation
        //Move move ={0,0};
        /*if(trackWall == false){
           goalSlope = abs(getSlope());
           }*/
        //printf("new goalSlope %lf\n", abs(getSlope()));
        //printf("goalSlope %lf\n", goalSlope);
        if(sensor.m_dmin <= BUMPERSIZE || trackWall == true) {
                if(trackWall == false) {
                        //initialSensor.m_dmin = sensor.m_dmin;
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
                printf("new goalSlope %lf\n", getSlope());
                printf("goalSlope %lf\n", goalSlope);
                printf("%lf\n", m_simulator->GetDistanceFromRobotToGoal());//*/
                printf("trackWall %d\n", trackWall);
                if(m_simulator->GetDistanceFromRobotToGoal() < 2.0) {
                        if( ( goalSlope - 0.1 <= getSlope() && goalSlope + 0.1 >= getSlope())  && trackWall == true /*initialSensor.m_dmin != 0.0*/ && distanceToGoal - 1 > m_simulator->GetDistanceFromRobotToGoal()) {
                                printf("In here extra\n" );
                                trackWall = false;
                                //initialSensor.m_dmin = 0;
                                distanceToGoal = m_simulator->GetDistanceFromRobotToGoal();
                                return MoveTowardsGoal();
                        }
                }
                if( ( goalSlope - 0.05 <= getSlope() && goalSlope + 0.05 >= getSlope())  && trackWall == true /*initialSensor.m_dmin != 0.0*/ && distanceToGoal - 1 > m_simulator->GetDistanceFromRobotToGoal()) {
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
        return (m_simulator->GetGoalCenterY() - m_simulator->GetRobotCenterY())/(m_simulator->GetGoalCenterX() - m_simulator->GetRobotCenterX());
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



        double obsVectorX = sensor.m_xmin - m_simulator->GetRobotCenterX();
        double obsVectorY = sensor.m_ymin - m_simulator->GetRobotCenterY();
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
        // printf("move MoveAroundObstacle\n");
        return newMove;
}

Move BugAlgorithms::MoveTowardsGoal(){

        double roboX = m_simulator->GetRobotCenterX();
        double roboY = m_simulator->GetRobotCenterY();
        double goalX = m_simulator->GetGoalCenterX();
        double goalY = m_simulator->GetGoalCenterY();
        double dx = (goalX - roboX);
        double dy = (goalY - roboY);
        double mag = sqrt(dx * dx + dy * dy);

        Move newMove ={(dx/mag)*MOVESIZE, (dy/mag)*MOVESIZE};
        return newMove;
}
