^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package tiago_wbc
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.0.20 (2016-12-21)
-------------------

1.0.19 (2016-12-13)
-------------------

1.0.18 (2016-11-09)
-------------------

1.0.17 (2016-11-09)
-------------------

1.0.16 (2016-10-20)
-------------------

1.0.15 (2016-10-20)
-------------------

1.0.14 (2016-10-20)
-------------------
* Updated changelog
* Updated changelog
* Contributors: Hilario Tome

* Updated changelog
* Contributors: Hilario Tome

1.0.13 (2016-10-13)
-------------------

1.0.12 (2016-10-13)
-------------------

1.0.11 (2016-10-13)
-------------------
* Removed dynamic introspection register from kinematic wbc controller
* Contributors: Hilario Tome

1.0.10 (2016-10-13)
-------------------
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
* Contributors: Adria Roig, Hilario Tome

1.0.9 (2016-04-26)
------------------
* Added parameter to parse imu sensors, added deadband to admitance task
* Contributors: Hilario Tome

1.0.8 (2016-04-16)
------------------
* Finished IMU parsing implementation in kinematic wbc controller, modified com stabilizer task to use new way of accesing ft, reemc humanoids dance ft working in gazebo
* Added momemtum task and not tested environment colliison task
* Contributors: Hilario Tome

1.0.7 (2016-03-09)
------------------

1.0.6 (2016-03-07)
------------------
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
* Added cmake modules to package.xml
* Contributors: Hilario Tome, Jordi Pages, Sammy Pfeiffer

1.0.5 (2015-06-12)
------------------

1.0.4 (2015-06-11)
------------------
* Added more coments
* Contributors: Hilario Tome

1.0.3 (2015-06-10)
------------------

1.0.2 (2015-06-05)
------------------
* Added robot design tools
* Contributors: Hilario Tome

1.0.1 (2015-06-04)
------------------

1.0.0 (2015-06-04)
------------------
* Fixing versions
* Changed default solver to old heap allocated solver
* Fix install rule moarrr
* Working tiago stacks, execpt for collision
* Tiago with qp reduction posiont, orientation stack working, the bug is in the new optimization of the solver
* Added tiago_wbc, bug when using stack with position, orientation, and bug with self collision
* Contributors: Bence Magyar, Hilario Tome
