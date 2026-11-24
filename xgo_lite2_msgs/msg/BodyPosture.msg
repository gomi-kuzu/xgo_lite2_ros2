# Body pose command with feet fixed on the ground.
# translation: XYZ translation (unit: millimeters).
#   Range (XGO-Lite2): x:[-25,25]  y:[-18,18]  z:[60,110]
# attitude:    Roll(x)/Pitch(y)/Yaw(z) rotation (unit: degrees).
#   Range (XGO-Lite2): r:[-20,20]  p:[-10,10]  y:[-12,12]
geometry_msgs/Vector3 translation
geometry_msgs/Vector3 attitude
