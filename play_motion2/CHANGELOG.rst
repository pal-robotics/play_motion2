^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package play_motion2
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.8.0 (2025-10-15)
------------------
* Remove unused ordering of interfaces for passthrough_controller
* Add -Wno-cpp to ignore warnings from dependencies
* Replace ament_target_dependencies with target_link_libraries
* Add condition for using services QoS
* Use move_group_interface.hpp header instead of .h
* Fix linters for jazzy
* Add support for jazzy
* Contributors: Noel Jimenez, Sai Kishor Kothakota

1.7.0 (2025-09-10)
------------------
* Fix ament_auto warning about headers install destination
* Contributors: Noel Jimenez

1.6.1 (2025-06-27)
------------------
* Capture the goal on the callback
* Contributors: davidfernandez

1.6.0 (2025-06-19)
------------------
* Prepare motion_loader to be pluginized
* Fix topic remappings to new node name
* Add Raw action to client
* Remove unnecessary tests
* Move Python client to PlayMotion2 pkg
* Add documentation
* Refactor to split Mgr and Exec nodes
* Contributors: davidfernandez

1.5.3 (2025-06-03)
------------------
* Fix motion starting on the current position of the robot
  The joints were compared with the approach positions size, which always
  will match. Then if the approach was not needed, the motion was not
  performed
* Contributors: Noel Jimenez

1.5.2 (2025-05-12)
------------------
* Bump cmake_minimum_required to 3.8
* Fix: share client library for using it from other packages
* Fix deprecated generate_parameter_library header
* Revert "Merge branch 'fix/warn/generate_parameter_library' into 'humble-devel'"
  This reverts merge request !72
* Remove launch_pal dependency
* Contributors: Noel Jimenez

1.5.1 (2025-03-11)
------------------
* Remap description topics
* Handle generate_parameter_library include for different versions
* Ignore set_value output
* Use set value output depending on hardware interface version
* Add optional argument rclcpp::NodeOptions to play_motion_client
* Contributors: Noel Jimenez, Sai Kishor Kothakota, thomasung

1.5.0 (2025-01-10)
------------------
* Add support for chainable controllers
* Contributors: Noel Jimenez

1.4.0 (2024-11-18)
------------------
* Remove controller manager name from spawner arguments
* Add test for Joint Trajectory failure
* Cancel all goals if one of the controllers fails
* Contributors: Noel Jimenez, Sai Kishor Kothakota

1.3.0 (2024-09-10)
------------------
* Remove error log from exists function
* Check the motion exists for isMotionReady service
* Add missing option to overwrite when adding motion
* Add option to run motions asynchronously
* Create service functions for the PlayMotion2Client
* Add run_motion executable
* Add simple client for PlayMotion2
* Add new motion key only if is not overwritten
* Create services to add and remove motions
* Add service to get the info of a motion
* Contributors: Noel Jimenez

1.2.0 (2024-08-09)
------------------
* Wait for robot description and semantic when using planning
* Wait for the executor to be spinning for play_motion2_test
* Contributors: Noel Jimenez

1.1.2 (2024-07-04)
------------------
* Handle joint state not available in /joint_states topic
* Contributors: Noel Jimenez

1.1.1 (2024-04-26)
------------------
* fix joint name by replacing find_first_of function to find_last_of
* Contributors: Aina Irisarri

1.1.0 (2024-04-03)
------------------
* Fix comment
* Add test deactivating unused controller while executing a motion
* Fail on controller change only if it's used by the running motion
* Cancel goals on failure
* Change logging level for 'Motion failed' message
* Invert loop to check changing controller properly
* Contributors: Noel Jimenez

1.0.1 (2024-03-07)
------------------
* Rename method perform_unplanned_motion to perform_motion
* Skip approach when the joints are already in the first position
* Contributors: Noel Jimenez

1.0.0 (2024-02-28)
------------------
* Rename planner launch argument and update its config
* Fix condition for checking planning groups parameter
* Enhance loop condition
* Enhance way of getting parameters
* Store parameter names in constants
* Remove unused method
* Complete comment
* Remove unnecessary variable all_joints_included
* Fix condition when checking valid move groups
* Update non-planned motion tests
* Remove line jump from log
* Add planning capability
* Add logic for planning parameters
* Create a MoveGroupInterfacePtr for each planning group
* Parse planning related parameters
* Include key in MotionInfo
* Handle INVALID Result State
* Remove unused function
* Rename motion_info to info
* Remove unused boolean variable
* Delete commented code
* Fix Result State type
* Enable motion canceling
* Use auto
* Move all the execution logic to MotionPlanner
  Now the MotionPlanner is the responsible of sending the goals for
  performing the motions. As well the motion has been splitted in two
  parts: Approach and Motion
  The Approach part is the movement from the initial point to the first
  position, and the rest of the positions form the Motion
* Get motion_planner parameters on constructor
* Rename ApproachPlanner to MotionPlanner
* Use play_motion2 node for ApproachPlanner
* Update types
* Contributors: Jordan Palacios, Noel Jimenez

0.0.15 (2024-01-15)
-------------------
* Change callback_group to MutuallyExclusive
  Reentrant callback group sometimes throws an unexpected runtime
  exception: 'Executing action client but nothing is ready'. It is caused
  because this type of callback group can execute different parts of the
  same callback concurrently.
* Specify mutex type for unique_lock
* Contributors: Aina Irisarri, Noel Jimenez

0.0.14 (2024-01-08)
-------------------
* Avoid exporting lifecycle_msgs dependency
* Contributors: Noel Jimenez

0.0.13 (2023-12-14)
-------------------
* Rename launch argument for motions config
* Add default approach_planner config
* Add launch argument for approach_planner config
* Fix approach planner parameters check condition
* Avoid installing test configuration files
* Join test config files
* Fix motion_loader_test node name and params file
* Fix parameters node names
* Simplify controllers usage for testing
* Create MotionLoader to replace helpers
* Contributors: Noel Jimenez

0.0.12 (2023-11-14)
-------------------
* Add website tag
* Contributors: Noel Jimenez

0.0.11 (2023-11-13)
-------------------
* Set use_sim_time false as default
* Fix launch dependencies
* Fix launch argument description
* Contributors: Noel Jimenez

0.0.10 (2023-10-02)
-------------------
* Move ApproachPlanner to a different class
* Contributors: Noel Jimenez

0.0.9 (2023-07-05)
------------------
* Use callback groups and MultiThreadedExecutor to execute callbacks in parallel
* Contributors: Noel Jimenez

0.0.8 (2023-05-22)
------------------
* add 1 extra second to motions timeout
  To avoid that motions with only one position fail when the robot is
  already in that position
* fill error field and be more verbose
* cancel action goals when requested
* Contributors: Noel Jimenez

0.0.7 (2023-04-17)
------------------
* remove meta information from mandatory parameters
* Contributors: Noel Jimenez

0.0.6 (2023-03-20)
------------------
* Merge branch 'remove_tests_namespace' into 'humble-devel'
  remove namespaces from tests
  See merge request app-tools/play_motion2!22
* remove namespaces from tests
* Merge branch 'fix_warns' into 'humble-devel'
  Fix warnings
  See merge request app-tools/play_motion2!21
* not catch exception by value
* change types for comparisons
* comment unused arguments
* order variables initialization
* Merge branch 'wait_for_service_and_action' into 'humble-devel'
  wait for service and action after creating clients
  See merge request app-tools/play_motion2!20
* wait for service and action after creating clients
* Merge branch 'ament_cmake_auto' into 'humble-devel'
  switch to ament_cmake_auto
  See merge request app-tools/play_motion2!19
* switch to ament_cmake_auto
* Merge branch 'rm_ament_cmake_pal' into 'humble-devel'
  remove ament_cmake_pal
  See merge request app-tools/play_motion2!18
* remove ament_cmake_pal
* Contributors: Jordan Palacios, Mathias Lüdtke, Noel Jimenez

0.0.5 (2023-03-01)
------------------
* Merge branch 'add_missing_dependency' into 'humble-devel'
  add missing test dependency robot_state_publisher and remove initial / from controllers config
  See merge request app-tools/play_motion2!17
* remove initial / from controllers config
* add missing test dependency robot_state_publisher
* Contributors: Jordan Palacios, Noel Jimenez

0.0.4 (2023-02-23)
------------------
* Merge branch 'fix_test' into 'humble-devel'
  Fix unstable test
  See merge request app-tools/play_motion2!16
* set start timeout in a variable
* use a better assert
* fix gtest header
* remove unused header
* fix loop condition to start play_motion2_node_test
* Contributors: Jordan Palacios, Noel Jimenez

0.0.3 (2023-02-15)
------------------
* Merge branch 'fix_cache_concurrency' into 'humble-devel'
  Do not update controller states cache if play_motion2 is busy
  See merge request app-tools/play_motion2!13
* do not update controller states cache if play_motion2 is busy
* Contributors: Jordan Palacios, Noel Jimenez

0.0.2 (2023-02-08)
------------------

0.0.1 (2023-02-08)
------------------
* Merge branch 'fix_loop_condition' into 'humble-devel'
  Fix inifinite retries loop and testing double types
  See merge request app-tools/play_motion2!11
* replace ASSERT_EQ with ASSERT_DOUBLE_EQ for doubles
* fix inifinite retries loop
* Merge branch 'approach_first_position' into 'humble-devel'
  Approach to first position
  See merge request app-tools/play_motion2!10
* rm possible concurrency and unnecessary unlock
* tests syntax fixes
* test to check motions on site and to other positions
* approach to the first position
* Merge branch 'motion_results' into 'humble-devel'
  Wait for action results
  See merge request app-tools/play_motion2!8
* add missing const, fix parameter name
* simplify play_motion2_node tests into smaller functions
* simplify function
* change logs order
* update controller states method
* renaming and move future instead of make a copy
* comment destructor behaviour
* test for controller deactivated after sending all goals
* handle action errors, exceptions
* check controller states while executing motion
* test for controller deactivated while executing motion
* set use_sim_time for tests
* bug fix: motion suceeded when a controller was deactivated after the goal was accepted
* remove wrong redefinition
* rewrite timeout and add error log
* fix timeout calculation
* add timeout when waiting for results
* split rrbot jtc in 2 for tests
* rename parameter
* make error log more specific
* simplify condition
* store and join execution thread
* fix tests times_from_start param
* wait for results
* Merge branch 'jtc_motions' into 'humble-devel'
  Send JTC motions (without waiting for the result)
  See merge request app-tools/play_motion2!7
* add missing refs
* use std::for_each
* use references
* send trajectories
* generate controller trajectories for a motion
* Merge branch 'improvements' into 'humble-devel'
  Enhancement and fix issues
  See merge request app-tools/play_motion2!6
* syntax fix
* simplify check_joints_and_controllers
* filter controller states function
* function to list controllers
* set default constructors
* add use_sim_time argument
* fix headers
* store info, not trajectory
* create MotionInfo for each motion
* remove old unused function
* Merge branch 'syntax_fixes' into 'humble-devel'
  fix eol and rm whitespaces
  See merge request app-tools/play_motion2!5
* fix eol and remove whitespace
* Merge branch 'jtc_motions' into 'humble-devel'
  New PlayMotion2 action: Manage requests and tests
  See merge request app-tools/play_motion2!3
* rewrite while loop
* check retries when waiting for service
* add missing test dependencies controllers
* fix wrong output arguments
* add missing test dependency controller_manager
* add missing test dependency xacro
* add missing timeouts
* add number of retries to wait for the motion ready
* global cte TIMEOUT
* switch some variables to const
* add missing service initialization and deactivation
* set const methods
* rm unnecessary controllers parameter and tests
* tests for sending action goals
* split function, check controllers state and types
* rm test cache
* new srv IsMotionReady
* play_motion_node test with rrbot
* play_motion2 launcher
* check motion, controllers and joints to accept goal
* create simple action server
* Merge branch 'lifecycle' into 'humble-devel'
  Switch to LifecycleNode
  See merge request app-tools/play_motion2!2
* add comment and fix condition
* add generic node functions for helpers and tests
* enable common interface for different types of nodes
* conditional log and return
* rm repeated state check
* start test with unconfigured state
* add missing headers
* switch to lifecycle node
* Merge branch 'first_version' into 'humble-devel'
  First version of PlayMotion2 - parse motions
  See merge request app-tools/play_motion2!1
* join boolean expression
* add commented argument
* add bool to print all missing parameters in motions
* syntax fix
* tests for no controllers or motions
* clear vectors and map to before parsing
* check at least one controller and motion are defined
* add comment and const for better understanding
* add namespace and reorder headers
* use copy_n instead of copy
* rename variable
* stop executor before destroying the node
* rm unused variable
* force c++ 17
* add default constructor and destructor
* syntax fixes and renamings
* initialize service
* name request arg and separate functions
* improve error messages
* add wrong motions for testing
* discard motions with missing data
* update way to load params
* play_motion2_test
* fix node name
* miscellaneous syntax fixes
* play_motion2_helpers_test
* play_motion2 as library
* parse controllers
* set node options without a function
* add destructor
* ListMotions service
* parse motions info
* node that reads a motion_name parameter
* Contributors: Jordan Palacios, Noel Jimenez
