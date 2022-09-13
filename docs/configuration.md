# MVP-Control Configuration

## Actuators

To configurate an actuator for MVP-Control program, user should first dedicate an ID to an actuator.
All the other configuration related with that actuator begins with the given ID.

Actuators IDs are configured with with `thruster_ids` ros_parameter.

Template configuration for `thruster_ids` is shown below:

```yaml
thruster_ids:
  - {{Thruster ID}}
  - {{Thruster ID}}
  - {{Thruster ID}}
  ...
```

Example configuration is shown below:

```yaml
thruster_ids:
  - surge
  - sway_bow
  - heave_bow
  - heave_stern
```

## Thrust Curve

The thrust curve is configured with `thruster_polynomials` ros_parameter for `mvp_control` node.
Each item under `thruster_polynomials` dictionary represents the polynomial for each thruster.
Array would hold the lower degrees in lower indexes. Shown below:

$$
f(x) = a_n x^n + a_{n-1} x^{n-1} + ... + a_1 x + a_0
$$

$$
\text{Thruster Polynomial} = [a_0, a_1, ... ,a_{n-1}, a_n]
$$

Template configuration for thruster polynomial is shown below:
```yaml
thruster_polynomials:
  {{ Thruster ID }}: [{{a0}, {{a1}}}, ...,{{an}}]
  {{ Thruster ID }}: [{{a0}, {{a1}}}, ...,{{an}}]
  ...

```

Example configuration for thruster curve looks like below:

```yaml
thruster_polynomials:
  surge:        [0.06165, 20.32, 9.632, 93.05, -2.856, -74.1]
```

## Actuator Limits

Thruster curve can have a varying limits.
They can represent PWM signal $[1100ms, 1900ms]$, or a simple bounded signal $[-1.0, 1.0]$.
Controller only solves for the desired force, and doesn't care about the output domain.

The polynomial must have a real solution around the thrust limits.

Thrust limits are configured with `thruster_limits` ros_parameter for `mvp_control` node.

Template configuration is shown below thruster limit is shown below

```yaml
thruster_limits:
  {{ Thruster ID }}: [{{lower limit}}, {{upper_limit}}]
  {{ Thruster ID }}: [{{lower limit}}, {{upper_limit}}]
  ...
```




