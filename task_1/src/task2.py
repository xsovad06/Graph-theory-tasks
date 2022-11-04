# TG - task n. 1 - sky_links
# author: Damian Sova
# date: 26.9.2022

import sys
import re

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

	def addEdge(self, source, destination, value = None):
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
		node.next = self.graph[source]
		self.graph[source] = node

		# Source; Source.next -> list; List -> Source
		if not ORIENTED_GRAPH:			
			node = Node(source)
			node.value = value
			node.next = self.graph[destination]
			self.graph[destination] = node
	
	def existEgde(self, source, destination):
		"""Check whether the given source and destination are connected."""

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

	def isSimple(self):
		"""Check if the are no multiple egdes between vertex in the graph."""

	def countNeighbours(self, name):
		"""Returns the number of the connected nodes to the given node by name."""

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

	for node in graph.graph:
		found, duplicities = graph.duplicitEgdes(node)
		if found and duplicities:
			for duplicity in duplicities:
				print(f"{node} -> {duplicity}")
