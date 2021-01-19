import copy
import numpy as np
#form, traverse and solve the tree
class Treenode(object):  # Add Node feature
    def __init__(self, index, level, weight, parent=None):
        self.index = index
        self.level = level
        self.weight = weight
        self.parent = parent
        self.weight_till_now=0
        self.children = []
        self.remaining_object=[]


D=np.array([[np.inf,np.inf,np.inf,np.inf,3,2,1,1,1],
			[np.inf,np.inf,np.inf,np.inf,4,3,2,2,3],
			[np.inf,np.inf,np.inf,np.inf,5,4,3,3,2],
			[np.inf,np.inf,np.inf,np.inf,6,5,4,4,4],
			[3,4,5,6,np.inf,np.inf,np.inf,np.inf,5],
			[2,3,4,5,np.inf,np.inf,np.inf,np.inf,4],
			[1,2,3,4,np.inf,np.inf,np.inf,np.inf,3],
			[1,2,3,4,np.inf,np.inf,np.inf,np.inf,3],
			[1,3,2,4,5,4,3,3,np.inf]
	])

object_idx_list=[0,1,2,3]
box_idx_list=[4,5,6,7]
home_idx=-1

def solver(D,object_idx_list,box_idx_list,home_idx):

	root=Treenode(home_idx,0,0)
	root.remaining_object=copy.deepcopy(object_idx_list)
	node_stack=[root]
	leaf_nodes=[]		#list of leaf node


	while node_stack:
		cur_node=node_stack.pop(-1)
		try:
			cur_node.weight_till_now=cur_node.parent.weight_till_now+cur_node.weight
		except AttributeError:
			cur_node.weight_till_now=cur_node.weight
		if cur_node.index==home_idx and cur_node.parent:			# if returns home
			leaf_nodes.append(cur_node)
			continue
		#next node should be box
		if cur_node.index in object_idx_list:
			destination_idx=int(cur_node.level/2)
			cur_node.children.append(Treenode(box_idx_list[destination_idx],cur_node.level+1,D[cur_node.index][box_idx_list[destination_idx]],cur_node))
			cur_node.children[-1].remaining_object=cur_node.remaining_object
			node_stack.append(cur_node.children[-1])
		#next node should be objects
		elif cur_node.index in box_idx_list or cur_node.index==home_idx:
			if not cur_node.remaining_object:
				node_stack.append(Treenode(-1,cur_node.level+1,D[cur_node.index][-1],cur_node))
				continue
			for remaining_object in cur_node.remaining_object:

				cur_node.children.append(Treenode(remaining_object,cur_node.level+1,D[cur_node.index][remaining_object],cur_node))
				cur_node.children[-1].remaining_object=copy.deepcopy(cur_node.remaining_object)
				cur_node.children[-1].remaining_object.remove(remaining_object)
				node_stack.append(cur_node.children[-1])
		
	total_weight_list=[leaf_node.weight_till_now for leaf_node in leaf_nodes]
	print(total_weight_list)
	#decode matrix index to object/box names:
	names_dict={0:'1',1:'2',2:'3',3:'4',4:'A',5:'B',6:'C',7:'D',-1:'H'}

	cur_node=leaf_nodes[total_weight_list.index(min(total_weight_list))]
	order=[]
	while cur_node.parent:
		order.insert(0,names_dict[cur_node.index])
		cur_node=cur_node.parent
	return ['H']+order

order=solver(D,object_idx_list,box_idx_list,home_idx)
print(order)
