uint8 STORAGE=0
uint8 CENTRE=1
uint8 ARM=2
int32 lens
uint8 location
# CENTRE: target centre interface S0..S5. lens_face defaults to S0.
string centre_face
# ARM: arm_A or arm_B, lens_face is the grasp interface S0..S5.
string arm
string lens_face
float64 phase_deg
