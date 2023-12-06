px4_add_module(
	MODULE modules__swarm_node
	MAIN swarm_node
	SRCS
		swarm_node.cpp
	DEPENDS
		px4_work_queue
	)
	
