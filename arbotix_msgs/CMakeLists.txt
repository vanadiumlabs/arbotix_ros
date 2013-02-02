cmake_minimum_required(VERSION 2.8.3)
project(arbotix_msgs)

find_package(catkin REQUIRED genmsg std_msgs)

add_message_files(FILES
                  Analog.msg
                  Digital.msg
                 )

add_service_files(FILES
                  Relax.srv
                  SetupChannel.srv
                 )

generate_messages(DEPENDENCIES std_msgs)

catkin_package(DEPENDS std_msgs)
