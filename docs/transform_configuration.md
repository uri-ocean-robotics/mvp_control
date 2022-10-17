# Configuring the transforms

First create an arbitrary transform link to be used as center of gravity link.
You may use URDF, static transform publisher, or your own TF publisher to achieve that.
This link will be passed to `mvp_control` with `cg_link` parameter.

Depending on the navigation solution, you maybe forced to use NED (North East Down) coordinate system.
For example, robot_localization package only works with ENU (East North Up) coordinate system.
You'll be configuring `world_link` parameter to base as the same convention as `cg_link`.
In other words, if `cg_link` is ENU, `world_link` must be ENU and vica versa.

Odometry input set with `odometry_source` parameter. Your navigation solution must output the odometry in `nav_msgs/Odometry` type.


## Example Configuration

```yaml
##
# "tf_prefix" for transform tree
# There is no default for parameter below.
##
tf_prefix: alpha

##
# Center of gravity link name in transform tree
# There is no default for parameter below.
##
cg_link: cg_link

##
# World link
# There is no default for parameter below.
##
world_link: world_ned

##
# Odometry source
# There is no default for parameter below.
##
odometry_source: /odometry/filtered
```