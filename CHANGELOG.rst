^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package robot_pose
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Forthcoming
-----------
* Merge branch 'tf2-migration' into 'dubnium-devel'
  Tf2 migration
  See merge request navigation/robot_pose!3
* added missing dep on rostest
* catch as &
* added license
* Merge branch 'add-robot-pose-test' into tf2-migration
* do not publish if there is a tf exception
* added robot_pose test
* added missing dependencies
* reduced freq to 50
* split tf lookup into two, to avoid jumps due to amcl initialpose
  this happened when slippaged detector published several initialpose
  messages. we found out that splitting the lookup from
  map -> base_footprint to map -> odom -> base_footprint
  solved the issue
* migration from tf to tf2
* cosmetic, matching pal code style
* upgrade package to format 2, cleaned cmakelists
* Contributors: Procópio Stein, Victor Lopez

0.0.2 (2016-03-04)
------------------
* Merge branch 'creation' into 'master'
  changed publisher queue size and fixed authors name
  fixed queue size and authors name
  See merge request !1
* changed publisher queue size and fixed authors name
* fixed tranform direction
* robot pose node implemented
* added readme
* Contributors: Procopio Stein, Victor Lopez
