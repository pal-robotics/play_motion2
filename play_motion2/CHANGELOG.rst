^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package play_motion2
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

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
