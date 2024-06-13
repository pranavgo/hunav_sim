# types
uint8 PERSON=1
uint8 ROBOT=2
uint8 OTHER=3

# behaviors
uint8 BEH_REGULAR=1
uint8 BEH_IMPASSIVE=2
uint8 BEH_SURPRISED=3
uint8 BEH_SCARED=4
uint8 BEH_CURIOUS=5
uint8 BEH_THREATENING=6
uint8 BEH_LLM_CUSTOM_0=7
uint8 BEH_LLM_CUSTOM_1=8
uint8 BEH_LLM_CUSTOM_2=9
uint8 BEH_LLM_CUSTOM_3=10
uint8 BEH_LLM_CUSTOM_4=11
uint8 BEH_LLM_CUSTOM_5=12
uint8 BEH_LLM_CUSTOM_6=13
uint8 BEH_LLM_CUSTOM_7=14
uint8 BEH_LLM_CUSTOM_8=15
uint8 BEH_LLM_CUSTOM_9=16

# behavior states
uint8 BEH_NO_ACTIVE=0
uint8 BEH_ACTIVE_1=1
uint8 BEH_ACTIVE_2=2

int32 id
uint8 type
uint8 skin
string name
int8 group_id
geometry_msgs/Pose position
float32 yaw
geometry_msgs/Twist velocity
float32 desired_velocity
float32 radius
float32 linear_vel
float32 angular_vel
int32 gesture   

geometry_msgs/Pose[] goals
bool cyclic_goals
float32 goal_radius

geometry_msgs/Point[] closest_obs

