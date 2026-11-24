# Single-leg tip position command (bypasses body walker).
# leg_id (per Elecfreaks official doc):
#   1 = left  front, 2 = right front,
#   3 = right rear,  4 = left  rear.
# position.xyz: leg tip target position in millimeters (same range as translation LIMIT).
uint8 leg_id
geometry_msgs/Vector3 position
