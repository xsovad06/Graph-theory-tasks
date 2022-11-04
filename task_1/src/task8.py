# TG - task n. 1 - travel
# author: Damian Sova
# date: 26.9.2022

# Zjednotenie grafov
# TG - task n. 1 - distribution
# author: Damian Sova
# date: 26.9.2022

# Vstupný / Výstupný stupeň

import sys
import re
import copy
from collections import Counter

ORIENTED_GRAPH = True

class Node:
	def __init__(self, name, value = None, orietation = None):
		self.name = name
		self.value = value
		self.next = None

class Graph:
	def __init__(self):
		self.graph = {}

	def addNode(self, name):
		"""Append new vertex to the vertices list."""

		self.graph[name] = None

	def existNode(self, name):
		"""Check presence of a node in the graph."""

		return name in self.graph

	def printNodes(self):
		"""Display the graph nodes names."""

		print("Nodes: ", end="")
		[print(f"\t-> {name} ") for name in self.graph.keys()]
		# print()

	def addEdge(self, source, destination, value = None):
		"""Connect the given destination vertex to the graph list source and
			 given source vertex to the graph list destination.
		"""

		# Create a node if not exist
		if source not in self.graph:
			self.addNode(source)

		# Dest; Dest.next -> list; List -> Dest
		node = Node(destination)
		node.value = value	
		node.next = self.graph[source]
		self.graph[source] = node

		# Source; Source.next -> list; List -> Source
		if not ORIENTED_GRAPH:
			# Create a node if not exist
			if source not in self.graph:
				self.addNode(source)

			node = Node(source)
			node.value = value
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

	def countNeighbours(self, name):
		"""Returns the number of the connected nodes to the given node by name."""

		node = self.graph[name]
		tmp = node
		count = 0
		while tmp:
			count += 1
			tmp = tmp.next
		return count

	def countIncomingEgdes(self, name):
		"""In oriented graph, returns the number of incoming edges to the given node"""

		if not ORIENTED_GRAPH:
			return 0

		count = 0
		for source in self.graph:
			tmp = self.graph[source]
			while tmp:
				if tmp.name == name:
					count += 1
				tmp = tmp.next
		return count

	def unionGraphs(self, graph):
		"""Insert missing connections and nodes from the second graph."""

		newGraph = copy.deepcopy(self)
		for node in graph.graph:
			tmp = graph.graph[node]
			while tmp:
				if tmp.name != node:
					if not newGraph.existEgde(node.upper(), tmp.name.upper()):
						newGraph.addEdge(node, tmp.name)
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

if __name__ == "__main__":
	# Create graph and edges
	
	graph1 = Graph()
	graph2 = Graph()
	first_line = True
	second_line = True

	for line in sys.stdin:
		if first_line:			
			group = re.split("\s*,\s+", re.sub("\n", '', line))
			first_line = False

			# Initialization of known nodes
			for node in group:
				graph1.addNode(node)
			continue

		if second_line:			
			group = re.split("\s*,\s+", re.sub("\n", '', line))
			second_line = False

			# Initialization of known nodes
			for node in group:
				graph2.addNode(node)
			continue

		group = re.split("\s*->\s", line)
		if graph1.existNode(group[0]):
			graph1.addEdge(group[0], re.sub("\n", '', group[1]))
		else:
			graph2.addEdge(group[0], re.sub("\n", '', group[1]))

	newGraph = graph1.unionGraphs(graph2)

	for node in newGraph.graph:
		tmp = newGraph.graph[node]
		while tmp:
			print(f"{node} -> {tmp.name}")
			tmp = tmp.next

	# print("Graph1:")
	# graph1.printGraph()
	# print("Graph2:")
	# graph2.printGraph()
	# print("Union:")
	# newGraph.printGraph()