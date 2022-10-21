Configuration
=============

Configuring the Transform Tree
------------------------------

.. important::
    It is crucial that users of MVP have a clear understandin of `REP 105 <https://www.ros.org/reps/rep-0105.html>`_ and `REP 103 <https://www.ros.org/reps/rep-0103.html>`_.

First create an arbitrary transform link to be used as center of gravity link.
You may use URDF, static transform publisher, or your own TF publisher to achieve that.
This link will be passed to ``mvp_control`` with ``cg_link`` parameter.

Depending on the navigation solution, you maybe forced to use NED (North East Down) coordinate system.
For example, robot_localization package only works with ENU (East North Up) coordinate system.
You'll be configuring ``world_link`` parameter to base as the same convention as ``cg_link``.
In other words, if ``cg_link`` is ENU, ``world_link`` must be ENU and vica versa.

.. note::
    MVP Controller requires a working navigation stack.
    ROS robot_localization package provides easiest navigation integration.


Odometry input set with ``odometry_source`` parameter. Your navigation solution must output the odometry in ``nav_msgs/Odometry`` type.

Your final TF tree structrue should look like the following image.

.. figure:: _static/example_tf.svg

    Example of minimum required transform tree configuration.

Setting up the thrust curve
---------------------------

Some manifacturers provide thrust curve for their thrusters that represets control command and force relation.
``mvp_control`` accepts polynomails as thrust curves.
It has a built-in polynomial solver to solve for control command.

For example, if you want to use `Blue Robotics T200 thruster <https://bluerobotics.com/store/thrusters/t100-t200-thrusters/t200-thruster-r2-rp/>`_, you can download the performance metrics from their webpage.
Then, you can apply curve-fitting to find polynomial that represents the thrust curve.

.. note::
    You can use either Matlab's `curve fitting toolbox <https://www.mathworks.com/help/curvefit/curve-fitting.html>`_ or
    you can use one of the Python scientific libraries curve fitters.

In some cases, actuator performance degregades or enhances based on the power supply efficiency and one-to-one mapping of control command to force might not be feasible.
To fix that problem, you may already have actuator controller that takes force as an input to control the actuator.
``mvp_control`` requested forces per thruster in another topic to cover this case.

