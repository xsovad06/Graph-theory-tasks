# TG - task n. 1 - tickets
# author: Damian Sova
# date: 26.9.2022

import sys
import re

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

		self.graph[name] = None

	def addEdge(self, source, destination, value = None):
		"""Connect the given destination vertex to the graph list source and
			 given source vertex to the graph list destination
		"""

		for node in (source, destination):
			if node not in self.graph:
				self.addNode(node)

		# Dest; Dest.next -> list; List -> Dest
		node = Node(destination)
		node.value = value
		node.next = self.graph[source]
		self.graph[source] = node

		# Source; Source.next -> list; List -> Source
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
	
	graph = Graph()
	first_line = True

	for line in sys.stdin:
		if first_line:
			group = re.match("Group:\s(.*)\n", line)
			group = re.split("\s*,\s", group.group(1))
			first_line = False

			# Initialization of known nodes
			for node in group:
				graph.addNode(node)
			continue

		edge = re.split("\s-\s", line)
		graph.addEdge(edge[0], re.sub("\n", '', edge[1]))

	most_neighbors = []
	the_most = 0
	for name in graph.graph.keys():
		count = graph.countNeighbours(name)
		if count > the_most:
			the_most = count
		most_neighbors.append((name, count))

	[print(f"{vertex[0]} ({vertex[1]})") for vertex in most_neighbors if vertex[1] == the_most]
