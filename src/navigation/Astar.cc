
#define WEIGHT_COST 1
// #include<iostream>
// #include<pair>
#include "node.cc"
#include "simple_queue.h"

#ifndef ASTAR
#define ASTAR

//returns 1 if path found
int Astar(vector_map::VectorMap map_,Node start_point,Node& end_point)
{
	SimpleQueue<Node,float> open_queue;
	SimpleQueue<Node,float> closed_queue;
	open_queue.Push(start_point,start_point.g+start_point.h);
	while(!open_queue.Empty())
	{
		Node q = open_queue.Pop();
		Node succesors[8];
		int succesors_x[] = {-1,-1,0,1,1,1,0,-1};//clockwise starting from left
		int succesors_y[] = {0,1,1,1,0,-1,-1,-1};
		float distances[] = {1,1.414,1,1.414,1,1.414,1,1.414};
		for(int neigh=0;neigh<8;neigh++)
		{
			Node cur_succesor = Node(q.i + succesors_x[neigh],q.j+succesors_y[neigh],end_point);
			succesors[neigh] = cur_succesor;
		}
		int i = 0;
		for(auto& succesor : succesors)
		{
			if(succesor.NodeIntersectsMap(map_.lines))
			{
				continue;
			}
			if(succesor==end_point)
			{
				end_point.parent = &q;
				return 1;
			}
			succesor.g = q.g + WEIGHT_COST*succesor.GetValue(map_.lines) + distances[i];//to implement cost as value?
			// succesor.h = abs(end_point.i-succesor.i) + abs(end_point.j-succesor.j);//what cost is best?
			if(open_queue.Exists(succesor))
			{
				float cur_priority = open_queue.GetPriority(succesor);
				if(succesor.g + succesor.h > cur_priority)
				{
					continue;
					// open_queue.push(succesor,succesor.g+succesor.h);
				}
			}
			else if(closed_queue.Exists(succesor))
			{
				float cur_priority = closed_queue.GetPriority(succesor);
				if(succesor.g + succesor.h > cur_priority)
				{
					continue;
					// open_queue.push(succesor,succesor.g+succesor.h);
				}
			}
			else
			{	
				succesor.parent = &q;
				open_queue.Push(succesor,succesor.g+succesor.h);
			}

			i++;
		}
		closed_queue.Push(q,q.g+q.h);

	}
	return 0;
}	



#endif  // ASTAR


























