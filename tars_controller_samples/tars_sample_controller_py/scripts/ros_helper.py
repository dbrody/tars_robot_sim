import rospy

##
# Helper function to setup ros service proxy and cache them as needed
##
services = dict()
def setupService(service_path, model, cache=True):
	if service_path in services and cache:
		return services[service_path]
	print " -   Waiting for service '%s' to load ..." % service_path,
	rospy.wait_for_service(service_path)
	service = rospy.ServiceProxy(service_path, model)
	if cache:
		services[service_path] = service
	print "done."
	return service
