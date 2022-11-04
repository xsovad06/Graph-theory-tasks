# TG - task n. 3 - performance
# author: Damian Sova
# date: 16.10.2022

# algorithmus obchodného cestujúceho

import sys
import re 
import copy
 
ORIENTED_GRAPH = False

def printComponents(components):
	"""Display each graph in a list"""

	for idx, comp in enumerate(components):
		print(f"{idx}. Component: ")
		comp.printGraph()

class Node:
	def __init__(self, name, value = None):
		self.name = name
		self.value = value
		self.next = None

class Graph:
	def __init__(self):
		self.graph = {}

	def addNode(self, name):
		"""Append new vertex to the vertices list"""

		if name not in self.graph:
			self.graph[name] = None

	def addEdge(self, source, destination, value = None):
		"""Connect the given destination vertex to the graph list source and
			given source vertex to the graph list destination.
		"""

		# Create a node if not exists
		for node in (source, destination):
			if node not in self.graph:
				self.addNode(node)

		# Dest; Dest.next -> list; List -> Dest
		node = Node(destination)
		node.value = value
		node.next = self.graph[source]
		self.graph[source] = node

		# Source; Source.next -> list; List -> Source
		if not ORIENTED_GRAPH:			
			node = Node(source)
			node.value = value
			node.next = self.graph[destination]
			self.graph[destination] = node
	
	def countNeighbours(self, name):
		"""Returns the number of the connected nodes to the given node by name"""

		node = self.graph[name]
		tmp = node
		count = 0
		while tmp:
			count += 1
			tmp = tmp.next			
		return count

	def getCheapestEdge(self, name):
		"""find the connection for given node with smallest value"""

		node = self.graph[name]
		tmp = node
		cheapestValue = float('inf')
		cheapestName = ""
		while tmp:
			if tmp.value < cheapestValue:
				cheapestValue = tmp.value
				cheapestName = tmp.name
			tmp = tmp.next			
		return cheapestName, cheapestValue

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

	def printGraph(self):
		"""Display the structure of a the connected nodes."""

		for name, value in self.graph.items():
			print(f"|{name}|:", end="")
			tmp = value
			while tmp:
				print(f" -> {tmp.name} ({tmp.value})", end="")
				tmp = tmp.next
			print("")

	def printGraphUniquePairs(self):
		"""Display two connected nodes for single line in unoriented graph."""

		pairs = {}
		for name, value in self.graph.items():
			tmp = value
			while tmp:
				if tmp.name not in pairs:
					pairs[tmp.name] = []
				if name in pairs and tmp.name not in pairs[name]:
					pairs[tmp.name] += [name]
					print(f"{name} - {tmp.name}")
				tmp = tmp.next

if __name__ == "__main__":
	# Create graph and edges
	
	graph = Graph()
	first_line = True

	for line in sys.stdin:
		if first_line:
			group = re.match("CPU:\s(.*)\n", line)
			group = re.split("\s*,\s", group.group(1))
			first_line = False

			# Initialization of known nodes
			for node in group:
				graph.addNode(node)
			continue

		edge = re.split("\s", line)
		graph.addEdge(edge[0], re.sub(":", '', edge[2]), int(re.sub("s", '', edge[3])))

	components = []
	cheapestWays = {}
	for name in graph.graph.keys():
		node, value = graph.getCheapestEdge(name)
		cheapestWays[name] = (node, value)
		comp = Graph()
		comp.addNode(name)
		components.append(comp)
	# print(cheapestWays)
	# printComponents(components)

	# find Minimum Spanning Tree
	for source, way in cheapestWays.items():
		# print("\n-------- PROCESSING NEW WAY! --------")
		dest = way[0]
		value = way[1]
		for comp1 in components:
			if source in comp1.graph:
				for comp2 in components:
					if comp1 is not comp2 and dest in comp2.graph:
						# print("-------- FIRST ---------")
						# comp1.printGraph()
						# print("-------- SECOND --------")
						# comp2.printGraph()
						# print("------------------------")
						comp1.addNode(dest)
						comp1.addEdge(source, dest, value)
						union = comp1.unionGraphs(comp2)
						# print(f"------ UNION ------")
						# union.printGraph()
						components.remove(comp1) 
						components.remove(comp2)
						components.append(union)
						# print("--------- COMPONENTS -----------")
						# printComponents(components)
						# print("--------------------------------")

	if len(components) > 1:
		printComponents(components)
		print("Minimum spanning tree could not be found! There are unconnected Nodes.", file=sys.stderr)
		exit(1)

	# Only one component left -> successfully found the minimum spanning tree
	components[0].printGraphUniquePairs()
	