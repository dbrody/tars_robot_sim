[Main](../README.md) > tars_control_lib_py

# tars_control_lib_py

Python package to abstract controlling the TARS robot.

## Usage

Example:

	import tars_control_lib_py

	def tars_loop(t):
		value = math.sin(t * 2) + t / 10
		tars_control_lib_py.control.setJoint1(value)
		tars_control_lib_py.control.setJoint2(value)
		tars_control_lib_py.control.setJoint3(-value)

	if __name__ == '__main__':
		tars_control_lib_py.control.tars_setup('ExampleController', tars_loop)


Setups up a basic node which repeatedly calls tars_loop and sets the join values.