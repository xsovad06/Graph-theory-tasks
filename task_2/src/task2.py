# TG - task n. 2 - weakness
# author: Damian Sova
# date: 6.10.2022

# algoritmy: artikulace a mosty
# make the graph unoriented

import sys
import re
import copy

ORIENTED_GRAPH = True

class Node:
	def __init__(self, name, value = None, dfnum = None, low = None):
		self.name = name
		self.value = value
		self.dfnum = dfnum
		self.low = low
		self.next = None

class Graph:
	def __init__(self):
		self.graph = {}

	def addNode(self, name):
		"""Append new vertex to the vertices list."""

		self.graph[name] = None

	def addEdge(self, source, destination, value = None, dfnum = None, low = None):
		"""Connect the given destination vertex to the graph list source and
			 given source vertex to the graph list destination
		."""

		# Create a node if not exists
		for node in (source, destination):
			if node not in self.graph:
				self.addNode(node)

		# Dest; Dest.next -> list; List -> Dest
		node = Node(destination)
		node.value = value
		node.dfnum = dfnum
		node.low = low
		node.next = self.graph[source]
		self.graph[source] = node

		# Source; Source.next -> list; List -> Source
		if not ORIENTED_GRAPH:			
			node = Node(source)
			node.value = value
			node.dfnum = dfnum
			node.low = low
			node.next = self.graph[destination]
			self.graph[destination] = node
	
	def existEgde(self, source, destination):
		"""Check whether the given source and destination are connected."""

		if source not in self.graph:
			return False

		node = self.graph[source]
		found = False
		while node:
			if node.name == destination:
				if found:
					return found
				else:
					found = True
			node = node.next
		return found

	def interconnected(self, source, destination):
		"""Check whether the given source and destination are connected in both directions."""

		if self.existEgde(source, destination) and self.existEgde(destination, source):
			return True
		return False

	def duplicitEgdes(self, name):
		"""Detect multiple edges from one specified vertex to the same vertex."""
  
		node = self.graph[name]
		occurences = {}
		duplicityList = []
		found = False
		while node:
			if node.name in occurences:
				occurences[node.name] = True
				duplicityList.append(node.name)
				found = True
			else:
				occurences[node.name] = False
			node = node.next
		return found, duplicityList

	def removeDuplicity(self, name, node, duplicityList):
		"""For particular node remove duplicit connections."""

		previous = None
		current = node
		for duplicity in duplicityList:
			while current:
				if current.name == duplicity:
					if not previous:
						self.graph[name] = current.next
						# current = current.next # if removing occurences
						break
					else:
						previous.next = current.next
						break
				previous = current
				current = current.next

	def isSimple(self):
		"""Check if the are no multiple egdes between vertex in the graph."""

		for node in self.graph:
			found, _ = self.duplicitEgdes(node)
			if found:
				return False
		return True

	def makeSimple(self):
		"""Check if there are duplicit edges and remove them from the graph."""

		if not self.isSimple():
			for node in self.graph:
				found, duplicityList = self.duplicitEgdes(node)
				if found:
					self.removeDuplicity(node, self.graph[node], duplicityList)

	def makeUnorientedGraph(self):
		"""Convert oriented graph to unoriented graph by adding missing edges."""

		# Removing duplicities
		self.makeSimple()

		# Adding missing edges
		for node in self.graph:
			for rest in self.graph:
				if node == rest:
					continue

				if self.existEgde(node, rest) and not self.existEgde(rest, node):
					self.addEdge(rest, node)
	
	def unionGraphs(self, graph):
		"""Insert missing connections and nodes from the second graph."""

		newGraph = copy.deepcopy(self)
		for node in graph.graph:
			tmp = graph.graph[node]
			while tmp:
				if tmp.name != node:
					if tmp.name not in newGraph.graph:
						newGraph.addNode(tmp.name)
					if not newGraph.existEgde(node, tmp.name):
						newGraph.addEdge(node, tmp.name, tmp.value)
				tmp = tmp.next
		return newGraph

	def countNeighbours(self, name):
		"""Returns the number of the connected nodes to the given node by name."""

		node = self.graph[name]
		tmp = node
		count = 0
		while tmp:
			count += 1
			tmp = tmp.next			
		return count

	def getNeighboursList(self, name):
		"""Returns the list of the connected nodes to the given node by name."""

		neighbours = []
		node = self.graph[name]
		tmp = node
		while tmp:
			if tmp.name != name:
				neighbours.append(tmp)
			tmp = tmp.next			
		return neighbours

	def createDfsTransition(self):
		"""Returns the order of nodes produced by DFS algorithm."""

		first_name = list(self.graph.keys())[0]
		dfnum = 0
		stack = [first_name]
		expanded = {}

		while stack:
			node = stack.pop()
			if node in expanded:
				continue
			expanded[node] = {'df': dfnum}
			dfnum += 1

			for neighbor in self.getNeighboursList(node):
				stack.append(neighbor.name)

		return expanded
	
	def createSpanningTreeFromDfs(self, dfs):
		"""Creates a SpanningTree from DFS algorithm"""

		last_node = ""
		dfs_path = list(dfs.keys())
		for node, dfnum in dfs.items():
			if last_node:
				if graph.existEgde(last_node, node):
					self.addEdge(last_node, node, dfnum=dfnum)
				# backtracking happened while DFS, the closest connection (from DFS way to node) need to be found from previously processed graph
				else:
					for i in (range(dfs_path.index(last_node), -1, -1)):
						if graph.existEgde(dfs_path[i], node):
							self.addEdge(dfs_path[i], node, dfnum=dfnum)
							break
			last_node = node

	def getLeafNodes(self):
		"""Returns a list of vertexies with no successors."""

		leafs = []
		for key, value in self.graph.items():
			if not value:
				leafs.append(key)
		return leafs

	def getSuccessors(self, node, origGraph):
		"""Returns a list of successors for a given node."""

		successors = []
		for neighbor in origGraph.getNeighboursList(node):
			if self.existEgde(node, neighbor.name):
				successors.append(node)
		return successors
	
	def countNodeLowValue(self, node, origGraph, resultStruct, leafNode = False):
		"""For current node gain the low value."""

		neighbours = origGraph.getNeighboursList(node)
		nodeLowVal = resultStruct[node]["df"]
		for neighbor in neighbours:
			# Compare successors low value with current nodeLowVal
			if not leafNode:
				if self.existEgde(node, neighbor.name):
					if "low" in resultStruct[neighbor.name]:
						nodeLowVal = min(nodeLowVal, resultStruct[neighbor.name]["low"])
					else:
						raise ValueError
			# Connection is out of the dfs tree
			if not self.existEgde(neighbor.name, node):
				nodeLowVal = min(nodeLowVal, resultStruct[neighbor.name]["df"])

		return nodeLowVal
	
	def countAllLowValues(self, result_struct):
		"""Method count for all nodes in the dfs tree low value calculation according to the articulation algorithm."""

		processed_nodes = []
		# Count leaf nodes low value -> algorithm must start from leafs (need successors low value)
		for leaf in leafs:
			result_struct[leaf]["low"] = self.countNodeLowValue(leaf, graph, result_struct, leafNode = True)
			processed_nodes.append(leaf)

		while len(processed_nodes) < len(self.graph):
			for proccessed_node in processed_nodes:
				for node in self.graph:
					# is node the predecessor of the processed node?
					if self.existEgde(node, proccessed_node):
						try:
							result_struct[node]["low"] = self.countNodeLowValue(node, graph, result_struct, leafNode = False)
						except:
							break
						processed_nodes.append(node)
		return result_struct

	def getArticulations(self, graph, result_struct):
		"""Find nodes that are critical."""

		articualtions = []
		for node in self.graph:
			for successor in self.getSuccessors(node, graph): 
				if node not in articualtions and result_struct[node]["df"] <= result_struct[successor]["low"]:
					articualtions.append(node)
		return articualtions

	def printGraph(self):
		"""Display the structure of a the connected nodes."""

		for name, value in self.graph.items():
			print(f"|{name}|:", end="")
			tmp = value
			while tmp:
				print(f" -> {tmp.name} ({tmp.value})", end="")
				print(f" -> {tmp.name}", end="")
				if tmp.value:
					print(f", v({tmp.value})", end="")
				if tmp.dfnum:
					print(f", df({tmp.dfnum})", end="")
				if tmp.low:
					print(f", low({tmp.low})", end="")
				tmp = tmp.next
			print("")

if __name__ == "__main__":
	# Create graph and edges
	
	graph = Graph()
	first_line = True

	for line in sys.stdin:
		if first_line:
			group = re.match("City:\s(.*)\n", line)
			group = re.split("\s*,\s", group.group(1))
			first_line = False

			# Initialization of known nodes
			for node in group:
				graph.addNode(node)
			continue

		group = re.match(".*:\s(.*)", line)
		group = re.split("\s->\s", group.group(1))
		for i in range(0, len(group) - 1):
			graph.addEdge(group[i], group[i + 1])

	# print("-------------- ORIGINAL GRAPH ------------")
	# graph.printGraph()

	graph.makeUnorientedGraph()

	# print("-------------- UNORIENTED GRAPH ------------")
	# graph.printGraph()

	# print("-------------- DFS ------------")
	dfs = graph.createDfsTransition()
	# print(dfs)

	spanning_tree = Graph()
	spanning_tree.createSpanningTreeFromDfs(dfs)

	# print("-------------- SPANNING TREE ------------")
	# spanning_tree.printGraph()

	# print("-------------- SEARCHING LEAFS ------------")
	leafs = spanning_tree.getLeafNodes()
	# print(leafs)

	dfs = spanning_tree.countAllLowValues(dfs)
	
	# print("-------------- LOWS PROCESSED ------------")
	# print(dfs)

	# print("-------------- ARTICULATIONS ------------")
	articulations = spanning_tree.getArticulations(graph, dfs)

	# if all nodes have the same value of low, then the first nost is not articulation
	starting_node = list(dfs.keys())[0]
	if len(articulations) == 1 and articulations[0] == starting_node:
		all_low_equal = True
		low = dfs[starting_node]["low"]
		for node in dfs:
			node_low = dfs[node]["low"]
			if low != node_low:
				all_low_equal = False		

		if all_low_equal:
			articulations.remove(starting_node)
	# print(articulations)

	# print("-------------- BRIDGES ------------")
	bridges = []
	for art1 in articulations:
		for art2 in articulations:
			if spanning_tree.existEgde(art1, art2):
				bridges.append(f"{art1} -> {art2}")
	# print(bridges)

	# print("-------------- RESULTS ------------")
	for bridge in bridges:
		print(bridge)
	for art in articulations:
		print(art)

	if not articulations:
		print("No articulation found!")
