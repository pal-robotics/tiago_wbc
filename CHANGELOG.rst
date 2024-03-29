^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package tiago_wbc
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.0.42 (2022-07-21)
-------------------
* Merge branch 'omnibase' into 'erbium-devel'
  Propagate base_type to wbc
  See merge request control/tiago_wbc!24
* Propagate base_type to wbc
* Contributors: Adria Roig, narcismiguel

1.0.41 (2022-04-20)
-------------------
* Merge branch 'contact_estimator' into 'erbium-devel'
  Mv imu_sensor_definition & force_torque_sensor_definition to pal_controller_interface
  See merge request control/tiago_wbc!23
* Mv imu_sensor_definition & force_torque_sensor_definition to pal_controller_interface
* Contributors: Adria Roig

1.0.40 (2021-12-22)
-------------------
* Merge branch 'reduced_wbc' into 'erbium-devel'
  Add arguments in WBC to set torso to no control
  See merge request control/tiago_wbc!22
* Add arguments in WBC to set torso to no control
* Contributors: Adria Roig, narcismiguel

1.0.39 (2021-08-31)
-------------------
* Merge branch 'kangaroo_wbc' into 'erbium-devel'
  Refactor to WBCModel
  See merge request control/tiago_wbc!21
* Refactor to WBCModel
* Contributors: Adria Roig, victor

1.0.38 (2019-09-30)
-------------------
* Add SYSTEM include_directories
* Contributors: Victor Lopez

1.0.37 (2019-09-26)
-------------------
* Merge branch 'change_to_ariles' into 'erbium-devel'
  changed capsules to ariles serialization
  See merge request control/tiago_wbc!20
* changed capsules to ariles serialization
* Contributors: Hilario Tome

1.0.36 (2019-05-15)
-------------------
* Merge branch 'license-refactor' into 'erbium-devel'
  Update pal license
  See merge request control/tiago_wbc!19
* Update PAL licenses
* Contributors: Victor Lopez

1.0.35 (2018-12-12)
-------------------
* Merge branch 'specifics-refactor' into 'erbium-devel'
  Replace deprecated launch file
  See merge request control/tiago_wbc!18
* Replace deprecated launch file
* Contributors: Victor Lopez

1.0.34 (2018-11-08)
-------------------
* Merge branch 'self_collision_fix' into 'erbium-devel'
  Update capsule collision and remove damping to self collision task
  See merge request control/tiago_wbc!17
* Fixtest after update capsules
* Update capsule collision and remove damping to self collision task
* Contributors: Adrià Roig, Hilario Tome

1.0.33 (2018-10-17)
-------------------
* Merge branch 'fix_test' into 'erbium-devel'
  Fix test
  See merge request control/tiago_wbc!16
* Fix tolerances and timeouts
* Fix tolernaces + duration tests
* Fix tests on megarelease
* Contributors: Adrià Roig, Hilario Tome

1.0.32 (2018-09-25)
-------------------
* Merge branch 'remove_dt' into 'erbium-devel'
  added extra parameters to yaml
  See merge request control/tiago_wbc!14
* added extra parameters to yaml
* Contributors: Hilario Tome

1.0.31 (2018-08-28)
-------------------
* Merge branch 'tiago_admitance' into 'erbium-devel'
  Tiago admitance
  See merge request control/tiago_wbc!13
* Add admitance demo for tiago
* Add admitance launch file
* Contributors: Adrià Roig, Victor Lopez

1.0.30 (2018-08-02)
-------------------
* Merge branch 'as_possible_blocking_fix' into 'erbium-devel'
  tests: wait for controller to start.
  See merge request control/tiago_wbc!11
* tests: wait for controller to start.
* Contributors: Hilario Tome, alexandersherikov

1.0.29 (2018-08-02)
-------------------
* Merge branch 'fix_include_issue' into 'erbium-devel'
  Fix include issue
  See merge request control/tiago_wbc!12
* Fix include issue
* Contributors: Adrià Roig, Victor Lopez

1.0.28 (2018-07-20)
-------------------
* Merge branch 'blending_rebased' into 'erbium-devel'
  Blending rebased
  See merge request control/tiago_wbc!10
* Modify rviz
* Removed push_pop service + increased timeout in smach_c_move_tip_test
* fixed compilation with new api
* Contributors: Adrià Roig, Hilario Tome

1.0.27 (2018-06-20)
-------------------
* Merge branch 'fix_tests' into 'erbium-devel'
  Fix tests
  Closes #2
  See merge request control/tiago_wbc!9
* Fix tests
* Contributors: Adrià Roig, Hilario Tome

1.0.26 (2018-04-30)
-------------------
* Merge branch 'pose_offset_test' into 'erbium-devel'
  Add wbc tests
  See merge request control/tiago_wbc!7
* Make tests independent
* Add wbc tests
* Contributors: Adria Roig, Hilario Tome

1.0.25 (2018-04-09)
-------------------
* Merge branch 'admitance' into 'erbium-devel'
  Admitance + Tests
  See merge request control/tiago_wbc!6
* Add wbc tests
* Remove admitance deprecated headers
* Contributors: Adria Roig, Hilario Tome

1.0.24 (2018-03-23)
-------------------
* Merge branch 'wbc_rpc_ref' into 'erbium-devel'
  Modify default reference
  See merge request control/tiago_wbc!5
* Modify default reference
* Contributors: Adria Roig, Hilario Tome

1.0.23 (2018-03-19)
-------------------
* Merge branch 'wbc-documentation' into 'erbium-devel'
  Tiago WBC utilities to pal_wbc_utils
  See merge request control/tiago_wbc!4
* Move push task utilities to pal_wbc_utils
* Contributors: Adria Roig, Victor Lopez

1.0.22 (2018-02-20)
-------------------
* fixed bugs
* Merge branch 'refactoring_erbium' into 'erbium-devel'
  Refactoring erbium
  See merge request control/tiago_wbc!3
* Modfiy rviz file
* Changed defualt link
* Add interactive marker with reflexxes
* Contributors: Adrià Roig, Hilario Tome

1.0.21 (2018-02-15)
-------------------
* deleted chagelog
* Merge branch 'refactoring_erbium' into 'erbium-devel'
  Refactoring erbium
  See merge request control/tiago_wbc!2
* Modified chains wbc
* fix typo
* Modified stack trees for wbc
* Modified params
* Fixed issue with default config params
* Unique stacks refactoring
* Fix unique stack problems
* Move files
* Refactoring to define a sinlge stack
* changed to kinematic simulator
* changed kinematic simulator include
* pal_robot_tools migration
* migration from pal robot tools
* fixed compilation
* Merge branch 'standalone_launch' into 'erbium-devel'
  Include the new kinematic_simulator_ros_control.launch amd…
  See merge request control/tiago_wbc!1
* Include the new kinematic_simulator_ros_control.launch amd joint_state_controller.launch in the standalone.launch
* formating
* fixed api ft
* added example to push tasks with ros messages
* added missing home posture param
* added local virtual admitance
* added virtual admitance element
* fixed api
* working push pop
* Merge branch 'erbium-devel' of gitlab:control/tiago_wbc into erbium-devel
* fixed compatibility reference
* removed unnecesary parameter from config file
* fixed compatibility
* Merge branch 'erbium-devel' of gitlab:control/tiago_wbc into erbium-devel
* working reflexx type II push task example
* added wbc extra from marco
* Merge branch 'erbium-devel' of gitlab:control/tiago_wbc into erbium-devel
* mege
* changed ft frame in admitance task
* fixed bug in congi
* added admitance stack
* clean up rpc
* added replace msgs support
* example working rpc
* Merge branch 'erbium-devel' into push_task_by_id
* added gitignore
* removed relative go to header
* initial refactor to add tasks by id
* IHMC valkyrie pipe working
* Workign planar floating base formulation, example with tiago working
* Finished separating wbc kinematic into standalone and deriving the kinematic controller from the base controller
* First version of rcp example working
* Basic version of push pop with tiago working
* Added missing destructors and bool parameter to set up, serialized capsules are having problems, started merging wbc_rpc
* 1.0.20
* Updatede changelog
* 1.0.19
* Updated changelog
* 1.0.18
* Updated changelog
* 1.0.17
* Updated changelog
* 1.0.16
* Updated changelog
* 1.0.15
* Updated changelog
* 1.0.14
* Updated changelog
* Updated changelog
* Updated changelog
* 1.0.13
* Updated changelog
* 1.0.12
* Updated changelog
* 1.0.11
* Updated changelog
* Removed dynamic introspection register from kinematic wbc controller
* 1.0.10
* Updated changelog
* Added pal_wbc namespace
* Merge branch 'dubnium-devel' into base_controller_local_joint_control
* Removed pal collision depen for tor (we will make a separate wbc task plugin for it), added floating base publisher to stack dynamic
* Initial migration to rbdl quaternion
* fk and com tests working
* API fixes and new rbdl version does not setZero when computing interia matrix and jacobians
* Fixed merge
* API fixes
* Fixed problems with merge
* Added gain parameters to a lot of kinematic tasks, experimental environment collision avoidance task
* 1.0.9
* Updated changelog
* Added parameter to parse imu sensors, added deadband to admitance task
* 1.0.8
* Updated changelog
* Finished IMU parsing implementation in kinematic wbc controller, modified com stabilizer task to use new way of accesing ft, reemc humanoids dance ft working in gazebo
* Merge branch 'dubnium-devel' of gitlab:control/pal_wbc into dynamic_momentum
* Added momemtum task and not tested environment colliison task
* 1.0.7
* update changelog
* 1.0.6
* update changelogs
* Added half implemented total variation, momentum tasks
* add arguments to choose input: marker or topic
* Added tiago standalone
* Added marco wbc, pid gains are mandatory parameters in all dynamics tasks, added cop box constraint task
* Fix the parameters for the position and orientation tasks
* Added params to fasten up the leap demo for the goto position and orientation tasks
* added laptop tray to tiago config (for marco)
* Fixed collision checking for TiaGo
* Fixed tiago floating base to false
* fixed merge
* Merge
* 1.0.5
* Updated changelog
* 1.0.4
* Updated changelog
* Added more coments
* 1.0.3
* Updated changelog
* Added cmake modules to package.xml
* 1.0.2
* Updated changelogs
* Added robot design tools
* 1.0.1
* Updated changelog
* 1.0.0
* Changelogs updated
* Fixing versions
* Changed default solver to old heap allocated solver
* Merge branch 'cobalt-devel' of gitlab:hilariotome/pal_wbc into cobalt-devel
* Fix install rule moarrr
* Working tiago stacks, execpt for collision
* Tiago with qp reduction posiont, orientation stack working, the bug is in the new optimization of the solver
* Added tiago_wbc, bug when using stack with position, orientation, and bug with self collision
* Contributors: Adria Roig, Adrià Roig, Bence Magyar, Hilario Tome, Hilario Tomé, Jordi Pages, Sam Pfeiffer, Sammy Pfeiffer
