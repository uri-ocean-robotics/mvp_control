Parameters
======================

Basic Parameters
----------------

.. cpp:var:: bool enabled [not required]

    :default: `False`

    :brief: Initial state of the controller

.. cpp:var:: double controller_frequency [not required]

    :default: `10.0`

    :brief: Frequency of the controller

.. cpp:var:: string tf_prefix [not required]

    :default: `''`

    :brief: Transform Tree prefix

.. cpp:var:: string odometry_source

    :default: `odometry`

    :brief: Odometry topic name

.. cpp:var:: string world_link [not required]

    :default: `world`

    :brief: Transform tree worl link name

.. cpp:var:: string cg_link [not required]

    :defeault: `cg_link`

    :brief: Center of gavity transform tree link name


Control Allocation Parameters
-----------------------------

.. cpp:var:: string generator_type

    :brief: Defines the control allocation matrix generation method.

    Depending on the
    parameter, it is either auto generated or user defined

    :option: ``tf`` Control allocaiton matrix generated using Transform Tree. If this option is selected, user must define actuator links in the `thruster_ids` parameter.|
    :option: ``user`` User provides the control allocation matrix

.. cpp:var:: complex control_allocation_matrix

    :brief: User defined control allocation matrix.

    User defines the control allocation matrix as vectors per actuator. Those
    vectors are so-called *contribution vectors*.

    .. cpp:var:: vector<double> @ACTUATOR_NAME

        :brief: Contribution vector for given actuator

        .. math:: \texttt{<actuator\_name>} \leftarrow [\phi, \theta, \psi, u, v, w]


    .. code-block:: yaml
        :caption: Example control_allocation_matrix configuration.

        control_allocation_matrix:
            main:
                [0.0, 0.0, 0.0, 1.0, 0.0, 0.0]
            horizontal:
                [0.0, 0.0, 0.48, 0.0, 1.0, 0.0]
            vertical:
                [0.0, 0.39, 0.0, 0.0, 0.0, -1.0]

.. cpp:var:: complex control_tf

    :brief: Transform tree link names for actuators

    If `generator_type` parameter is set to `tf`, the mvp_control will look for actuator links in the transform tree.
    Transform tree link names is required to generate the control allocaiton matrix.

    .. cpp:var:: string @thruster

        :brief: Trasform tree link name for the actuator


    .. code-block:: yaml
        :caption: Example

        control_tf:
            main: main_thruster_link
            horizontal: horizontal_thruster_link
            vertical: vertical_thruster_link

Actuator Parameters
-------------------

.. cpp:var:: vector<string> thruster_ids

    :brief: Arbitrary names for actuators

    The user must declare arbitrary id for actuators.
    Each thruster has its unique ID.
    These unique IDs will be used for configuring them.
    In the rest of the configuration file, those IDs must match.

    .. code-block:: yaml
        :caption: Example

        thruster_ids:
          - main
          - horizontal
          - vertical


.. cpp:var:: complex thruster_command_topics

    :brief: Control command topics for each actuator.

    Control command for the actuator will be published under the topic defined with this parameter.
    The topic will publish the control command in `std_msgs/Float64` format.

    .. cpp:var:: string @thruster_id

        :brief: Control commad topic for an actuator

    .. code-block:: yaml
        :caption: Example configuration

        thruster_command_topics:
            main: control/thruster/main
            horizontal: control/thruster/horizontal
            vertical: control/thruster/vertical

.. cpp:var:: complex thruster_force_topics

    :brief: Topic for controller force output for each actuator.

    This topic will publish requested force from each actuator in Newtons.
    The topic will publish the control command in `std_msgs/Float64` format.

    .. cpp:var:: string @thruster_id

        :brief: Force topic for an actuator

    .. code-block:: yaml
        :caption: Example configuration

        thruster_force_topics:
            main: control/force/main
            horizontal: control/force/horizontal
            vertical: control/force/vertical

.. cpp:var:: complex thruster_polynomials

    :brief: Thrust curve for each thruster

    Control allocation computes the demanded force from each actuator.
    Thrust curves help converting force into control command.
    This could be a :math:`[1100,1900]` PWM value or simple :math:`[-1.0,1.0]` value.

    .. cpp:var:: vector<float> @thruster

        :brief: Thrust curve for thruster defined

        For given thrust curve, :math:`f(x)`, thrust polynomial is set as follows.

        .. math::

            f(x) = a_n x^n + a_{n-1} x^{n-1} + ... + a_1 x + a_0 \\
            \text{Thruster Polynomial} \leftarrow [a_0, a_1, ... ,a_{n-1}, a_n]

    .. code-block:: yaml
        :caption: Example Configuration

        thruster_polynomials:
            main:       [0.06165, 20.32, 9.632, 93.05, -2.856, -74.1]
            horizontal: [0.06165, 20.32, 9.632, 93.05, -2.856, -74.1]
            vertical:   [0.06165, 20.32, 9.632, 93.05, -2.856, -74.1]

    .. code-block:: yaml
        :caption: Template for thruster polynomial configuration

        thruster_polynomials:
            {{ Thruster ID }}: [{{a0}, {{a1}}}, ...,{{an}}]
            {{ Thruster ID }}: [{{a0}, {{a1}}}, ...,{{an}}]
            ...


.. cpp:var:: complex thruster_limits

    :brief: Force limits for each thruster

    .. cpp:var:: complex @thruster

        :brief: Maximum and minimum limits for given thruster

        .. cpp:var:: float max

            :brief: maximum limit for the given thruster

        .. cpp:var:: float min

            :brief: minimum limit for the given thruster


    .. code-block:: yaml
        :caption: Example

        thruster_limits:
            main:
                max: 40
                min: -30
            horizontal:
                max: 20
                min: -20
            vertical:
                max: 20
                min: -20

Control Law Parameters
----------------------

.. cpp:var:: complex control_modes

    :brief: Control modes with PID gains

    .. cpp:var:: complex @mode

        :brief: **Control mode**

        Control mode name is given with this parameter.
        Target degree of freedoms and PID gains described under each control mode.
        MVP Controller can be configured with more than one control mode.

        .. cpp:var:: complex @dof

            :brief: **Target degree of freedoms** and their PID gains

            Target degree of freedoms of a control mode with its PID gains.
            This complex object can only have members that are described in the table below.
            Each 'member' of this complex object contains PID gains.

            .. list-table:: Possible values for target degree of freedoms
                :widths: 25 25 50
                :header-rows: 1

                * - Target DOFs
                  - Keyword
                  - SNAME Symbol
                * - Position in X axis
                  - ``x``
                  - :math:`x`
                * - Position in Y axis
                  - ``y``
                  - :math:`y`
                * - Position in Z axis
                  - ``z``
                  - :math:`z`
                * - Orientation around X axis
                  - ``roll``
                  - :math:`\phi`
                * - Orientation around Y axis
                  - ``pitch``
                  - :math:`\theta`
                * - Orientation around Z axis
                  - ``yaw``
                  - :math:`\psi`
                * - Linear speed on X axis
                  - ``surge``
                  - :math:`u`
                * - Linear speed on Y axis
                  - ``sway``
                  - :math:`v`
                * - Linear speed on Z axis
                  - ``heave``
                  - :math:`w`
                * - Angular speed around X
                  - ``roll_rate``
                  - :math:`\dot{\phi}`
                * - Angular speed around Y
                  - ``pitch_rate``
                  - :math:`\dot{\theta}`
                * - Angular speed around  Z
                  - ``yaw_rate``
                  - :math:`\dot{\psi}`

            .. cpp:var:: float p

                P gain for PID controller

            .. cpp:var:: float i

                I gain for PID controller

            .. cpp:var:: float d

                D gain for PID controller

            .. cpp:var:: float i_max

                Maximum integral value

            .. cpp:var:: float i_min

                Minimum interal value

    Example configuration is given in the code snippet below.
    In the code snippet three control modes are configured.
    The first control mode is ``flight`` mode that controls pitch, yaw and surge.
    Second control mode is ``hold`` that controls :math:`x` and :math:`y`.
    The third and the last contol mode is ``kill``.
    This is a special setting that allows user to create control modes that doesn't control any degree of freedoms.
    It is particularly useful when creating safety behaviors.

    Simply put, first indentation after ``control_modes`` directive starts to create a new control mode.
    And then the next indentation is to describe target degree of freedoms.


    .. code-block:: yaml
        :caption: Example control mode configuration

        control_modes:
            flight:
                pitch:    {p: 5.0,    i: 1.0,   d: 2.0,   i_max: 10, i_min: -10}
                yaw:      {p: 5.0,    i: 0.5,   d: 30.0,  i_max: 20, i_min: -20}
                surge:    {p: 10.0,   i: 3.0,   d: 5.0,   i_max: 10, i_min: -10}

            hold:
                x:        {p: 2.0,    i: 0.5,   d: 1.0,   i_max: 10, i_min: -10}
                y:        {p: 2.0,    i: 0.5 ,  d: 1.0,   i_max: 10, i_min: -10}

            kill: false