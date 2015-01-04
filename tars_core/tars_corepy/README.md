[Main](../README.md) > tars_corepy

# tars_corepy

Python package to abstract controlling the TARS robot.

## Usage

Example:

	import tars_corepy

	def tars_loop(t):
		value = math.sin(t * 2) + t / 10
		tars_corepy.control.setJoint1(value)
		tars_corepy.control.setJoint2(value)
		tars_corepy.control.setJoint3(-value)

	if __name__ == '__main__':
		tars_corepy.control.tars_setup('ExampleController', tars_loop)


Setups up a basic node which repeatedly calls tars_loop and sets the join values.