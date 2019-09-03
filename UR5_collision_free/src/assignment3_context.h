#include <ros/ros.h>
#include <moveit/planning_interface/planning_interface.h>

MOVEIT_CLASS_FORWARD( CS436Context );

class CS436Context : public planning_interface::PlanningContext {

public:
  
  typedef std::vector<double> vertex;
  typedef std::size_t index;
  typedef std::pair<index,index> edge;
  typedef std::vector<index> path;

  CS436Context( const robot_model::RobotModelConstPtr& model, 
		const std::string &name, 
		const std::string& group, 
		const ros::NodeHandle &nh = ros::NodeHandle("~") );
  virtual ~CS436Context();

  virtual bool solve( planning_interface::MotionPlanResponse &res );
  virtual bool solve( planning_interface::MotionPlanDetailedResponse &res );

  virtual void clear();
  virtual bool terminate();

  /**
     Feel free to change this method if you can do better.

     Test if a state collides with the scene.
     Call this method if you need to find if the robot collides with the 
     environment in the given robot's state.
     \param[in] q The robot state
     \return      true if the robot state collides. false otherwise
  */
  bool state_collides( const vertex& q ) const;

  /**
     Feel free to change this method if you can do better.

     Test if a linear trajectory between two states collides with the 
     environment. Call this method if you need to find if the robot collides
     with the environment if it moves linearly between two states.
     \param[in] qA   The start robot state
     \param[in] qB   The final robot state
     \param     step The joint step used in between configurations
     \return         true if the robot trajectory collides. false otherwise
  */
  bool edge_collides( const vertex& qA, 
		      const vertex& qB, 
		      double step=0.01 )const;

  /**
     TODO

     Create a vector of N vertices. Each vertex represent a collision-free
     configuration (i.e. 6 joints) of the robot.
     \param N The number of vertices in the graph
     \return  A vector of N collision-free configurations
  */
  std::vector<vertex> make_vertices( int N )const;

  /**
     TODO

     Create a vector of edges. Each vertex represent a pair of vertex indices.
     For example, the pair (1,10) connects the vertices V[1] to V[10]. There is
     no restriction on how many edges your graph requires.
     \param[in] V A vector of N collision-free configurations
     \return  A vector of collision-free edges
  */
  std::vector<edge> make_edges( const std::vector<vertex>& V )const;

  /**
     TODO

     Search for a vertex that is accessible from a start configuration with a 
     collision-free trajectory. The method returns the index of the accessible 
     vertex.
     \param[in] V A vector of N collision-free configurations
     \param[in] q The start configuration
     \return      The index of the accessible vertex
  */
  index search_accessibility( const std::vector<vertex>& V, const vertex& q )const;

  /**
     TODO

     Search for a vertex that is "departable" to a final configuration with a 
     collision-free trajectory. The method returns the index of the departable
     vertex.
     \param[in] V A vector of N collision-free configurations
     \param[in] q The final configuration
     \return      The index of the departable vertex
  */
  index search_departability( const std::vector<vertex>& V, const vertex& q )const;

  /**
     TODO

     Search the graph (V,E) for a path between two vertices.
     collision-free trajectory. The method returns the index of the departable
     vertex.
     \param[in] V          A vector of N collision-free configurations
     \param[in] E          A vector of collision-free edges
     \param[in] access_idx The index of the accessible vertex
     \param[in] depart_idx The index of the departable vertex
     \return               A vector representing a sequence of vertices
  */
  path  search_path( const std::vector<vertex>& V, 
		     const std::vector<edge>& E, 
		     index access_idx, 
		     index depart_idx )const;


protected:

  robot_model::RobotModelConstPtr robotmodel;

};

