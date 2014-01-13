#pose
float32 x
float32 y
float32 theta

#name
string place_name

#int32 id

# type of the place
# object search, user search
std_msgs/String place_type

# array of possible objects
#hobbit_msgs/Object[] objects
Object[] objects
