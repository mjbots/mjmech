=======
legtool
=======

legtool provides libraries and utilities for operating legged robots.

* GitHub: https://github.com/jpieper/legtool
* Free software: Apache 2.0 License

Features
--------

* Inverse kinematics of 3 DoF lizard style legs
* A configurable gait engine

  * Decoupled from any particular actuator system
  * Statically stable variants
  * Grouped legs
  * Arbitrary number of legs and mounting positions

* A graphical tool to rapidly observe changes in inverse kinematics
  or gait engine parameters.
* Robot support

  * gazebo based robot simulations through pygazebo
    https://github.com/jpieper/pygazebo
  * Dongbu HerkuleX servo support

Dependencies
------------

* SCons
* pygazebo (https://github.com/jpieper/pygazebo)
* PySide
* pygame (only for the gait_driver application)

On ubuntu, the dependencies can be installed by:

.. code-block::

   sudo pip install pygazebo
   sudo apt-get install scons python-pyside pyside-tools python-pygame

Example
-------

You can use legtool to drive a simulated gazebo quadruped robot in a
few simple steps.

Starting from the legtool directory, build any required files:

.. code-block::
   
   scons

Next, tell gazebo where the legtool sample model is:

.. code-block::

   ln -s $(pwd) ~/.gazebo/models/legtool

Start gazebo, and use the "Insert" tab to insert the legtool sample
model.

.. code-block::

   gazebo

Start the legtool graphical application:

.. code-block::

   ./legtool -c sample_gazebo.cfg

Then on the first tab, click "connect".  At this point, all of the
features of legtool are available.  For the most interesting quick
demonstration, switch to the "Gait" tab of legtool and click the
"Repeat" button to watch the sample gait cycle.
