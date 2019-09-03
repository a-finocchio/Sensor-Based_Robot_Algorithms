#include "assignment3_context.h"
#include <moveit_msgs/RobotTrajectory.h>
#include <moveit/planning_scene/planning_scene.h>
#include <time.h>
#include <queue>
#include <math.h>

// utility function to interpolate between two configuration
CS436Context::vertex interpolate( const CS436Context::vertex& qA, 
				  const CS436Context::vertex& qB, 
				  double t ){  

  CS436Context::vertex qt( qA.size(), 0.0 );
  for( std::size_t i=0; i<qt.size(); i++ )
    { qt[i] = ( 1.0 - t )*qA[i] + t*qB[i]; }
  return qt;

}

CS436Context::CS436Context( const robot_model::RobotModelConstPtr& robotmodel,
			    const std::string& name, 
			    const std::string& group, 
			    const ros::NodeHandle& nh ) :
  planning_interface::PlanningContext( name, group ),
  robotmodel( robotmodel ){}

CS436Context::~CS436Context(){}

bool CS436Context::state_collides( const vertex& q ) const {

  // create a robot state
  moveit::core::RobotState robotstate( robotmodel );
  robotstate.setJointGroupPositions( "manipulator", q );

  if( getPlanningScene()->isStateColliding( robotstate, "manipulator", false ) )
    { return true; }
  else
    { return false; }
  
}

bool CS436Context::edge_collides( const vertex& qA, 
				  const vertex& qB, 
				  double step )const{
  
  // Simple trajectory in configuration space
  for( double t=0.0; t<1.0; t+=step ){
      if( state_collides( interpolate( qA, qB, t ) ) ) { return true; }
  }

  return false;

}

std::vector<CS436Context::vertex> CS436Context::make_vertices( int N )const{

  std::vector<CS436Context::vertex> V( N );
    int a;
    a=0;
    srand((unsigned)time(0));
    while(a < N){
        CS436Context::vertex joint;
        for (int i=0;i<6;i++){

            double pointdata = (rand()/(double)(RAND_MAX/(2.0*M_PI)))-M_PI;
            // std::cout<<pointdata<<std::endl;
            joint.push_back(pointdata);

        }

        if ( ! CS436Context::state_collides( joint ) ){
            V[a]=joint;
            a++;
        }

    }

    std::cout<<"V size  "<<V.size()<<std::endl;
    return V;

}

std::vector<CS436Context::edge> 
CS436Context::make_edges
( const std::vector<CS436Context::vertex>& V )const{

  std::vector<CS436Context::edge> E;
    for(int b=0; b<(V.size()); b++){
        for(int c=b+1; c<(V.size()); c++){

            if( ! CS436Context::edge_collides( V[b], V[c], 0.05 ) ) {
                E.push_back(std::make_pair(b,c));
            }
        }
    }
    std::cout<<"E size  "<<E.size()<<std::endl;
  return E;

}

CS436Context::index
CS436Context::search_accessibility
( const std::vector<CS436Context::vertex>& V, 
  const CS436Context::vertex& q )const{

    for (int d=0; d<V.size(); d++) {
        if (!CS436Context::edge_collides(V[d], q, 0.05)) {
            // std::cout<< d <<std::endl;
            return d;
        }
    }
    std::cout<<"access fail"<<std::endl;
    return V.size();

}

CS436Context::index 
CS436Context::search_departability
( const std::vector<CS436Context::vertex>& V, 
  const CS436Context::vertex& q )const{

   for (int e=1; e<V.size();e++){
        if (!CS436Context::edge_collides(V[e], q, 0.05)) {
            // std::cout<< e <<std::endl;
            return e;
        }
    }
    std::cout<<"depart fail"<<std::endl;
    return V.size();

}


CS436Context::path
CS436Context::search_path
        ( const std::vector<CS436Context::vertex>& V,
          const std::vector<CS436Context::edge>& E,
          CS436Context::index idx_start,
          CS436Context::index idx_final )const{

    std::vector<CS436Context::index> path;
    std::vector<std::vector<index> > table(V.size());

    //优化剪枝//

    for (int i = 0; i < E.size(); i++){
        table[E[i].first].push_back(E[i].second);
        table[E[i].second].push_back(E[i].first);

    }

    std::queue< std::vector<index> > container_queue;   //总容器 container
    std::vector<int> color(V.size(), 0);
    std::vector<index> start;
    start.push_back(idx_start);
    container_queue.push(start);
    color[idx_start] = 1;            //insert start point to the container


    while(container_queue.size()) {
        std::vector<index> current = container_queue.front();
        container_queue.pop();
        index point = current.back();
        for (int i = 0; i < table[point].size(); i++) {
            if (color[table[point][i]] == 0) {
                color[table[point][i]] = 1;
                std::vector<index> temp = current;
                temp.push_back(table[point][i]);
                if (table[point][i] == idx_final) {
                    return temp;
                }
                container_queue.push(temp);
            }
        }
        color[point] = 2;
    }
    return path;
}



// This is the method that is called each time a plan is requested
bool CS436Context::solve( planning_interface::MotionPlanResponse &res ){

  // Create a new empty trajectory
  res.trajectory_.reset(new robot_trajectory::RobotTrajectory(robotmodel, 
  							      getGroupName()));
  res.trajectory_->clear();

  // copy the initial/final joints configurations to vectors qfin and qini
  // This is mainly for convenience.
  std::vector<double> qstart, qfinal;
  for( size_t i=0; i<robotmodel->getVariableCount(); i++ ){
    qfinal.push_back(request_.goal_constraints[0].joint_constraints[i].position);
    qstart.push_back(request_.start_state.joint_state.position[i]);
  }

  // start the timer
  ros::Time begin = ros::Time::now();

  // Adjust N to your need
  int N = 150;

  // Create a vector of collision-free vertices
  std::vector<vertex> V = make_vertices( N );

  // Find the index of the accessible vertex
  index idx_start = search_accessibility( V, qstart );
  // Find the index of the departable vertex
  index idx_final = search_departability( V, qfinal );

  // Both index must be valid (accessible and departable vertices exist)
  if( V.size() <= idx_start || V.size() <= idx_final ) { return false; }

  // Create a vector edges
  std::vector<edge> E = make_edges( V );
  
  // Find a path between the start index and final index
  path P = search_path( V, E, idx_start, idx_final );

  // end the timer
  ros::Time end = ros::Time::now();

  // The rest is to fill in the animation. You can ignore this part.
  moveit::core::RobotState robotstate( robotmodel );
  robotstate.setJointGroupPositions( "manipulator", qstart );

  for( double t=0.0; t<=1.0; t+=0.01 ){
    vertex q = interpolate( qstart, V[P[0]], t );
    robotstate.setJointGroupPositions( "manipulator", q );
    res.trajectory_->addSuffixWayPoint( robotstate, 0.01 );
  }

  for( std::size_t i=0; i<P.size()-1; i++ ){
    for( double t=0.0; t<=1.0; t+=0.01 ){
      vertex q = interpolate( V[P[i]], V[P[i+1]], t );
      robotstate.setJointGroupPositions( "manipulator", q );
      res.trajectory_->addSuffixWayPoint( robotstate, 0.01 );
    }
  }

  for( double t=0.0; t<=1.0; t+=0.01 ){
    vertex q = interpolate( V[P[P.size()-1]], qfinal, t );
    robotstate.setJointGroupPositions( "manipulator", q );
    res.trajectory_->addSuffixWayPoint( robotstate, 0.01 );
  }

  // set the planning time
  ros::Duration duration = end-begin;
  res.planning_time_ = duration.toSec();
  res.error_code_.val = moveit_msgs::MoveItErrorCodes::SUCCESS;

  return true;
  
}

bool CS436Context::solve( planning_interface::MotionPlanDetailedResponse &res )
{ return true; }

void CS436Context::clear(){}

bool CS436Context::terminate(){return true;}
