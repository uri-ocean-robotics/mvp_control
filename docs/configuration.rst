Configuration
=============

Configuring the transforms
--------------------------

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

