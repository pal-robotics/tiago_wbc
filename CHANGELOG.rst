^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package tiago_wbc
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

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
