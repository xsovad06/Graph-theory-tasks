# TG - task n. 2 - escape
# author: Damian Sova
# date: 6.10.2022

# hladanie optimálnych sledov (neinformované hladanie cesty - cesta von z bludišťa)
# prechádzanie grafov - kedy skončiť (pre istotu dať počet p)
# nepoužiť algoritmus na optimálny sled(dikstra... nepoznám dopredu celý graf)
# použiť yield na spracovávanie vrcholov

import sys
import re
import copy

ORIENTED_GRAPH = False

class TarryNode:
	"""Base building block for Tarry algorithm."""
	def __init__(self, name, unused_ways = [], in_traverse = [], out_traverse = []):
		self.name = name
		self.unused_ways = unused_ways
		self.in_traverse = in_traverse
		self.out_traverse = out_traverse
	
	def getNextNode(self):
		"""Decide which node from neighbors will be the next in tarrys algorithm"""

		if self.unused_ways:
			next_node = self.unused_ways.pop()
			self.out_traverse.append(next_node)
			return next_node
		else:
			return self.in_traverse[0]

	def __str__(self):
		return f"|{self.name.upper()}|\n\tunused ways: {self.unused_ways}\n\tin traverse: {self.in_traverse}\n\tout traverse: {self.out_traverse}"

class Node:
	"""Base building block for graph representation."""

	def __init__(self, name, value = None):
		self.name = name
		self.value = value
		self.next = None

class Graph:
	"""Class representing connected Nodes and manipulations with the structure."""

	def __init__(self):
		self.graph = {}

	def addNode(self, name):
		"""Append new vertex to the vertices list."""

		self.graph[name] = None

	def addEdge(self, source, destination, value = None, dfnum = None, low = None):
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

	def getNeighboursNamesList(self, name):
		"""Returns the list of the connected nodes to the given node by name."""

		neighbours = []
		node = self.graph[name]
		tmp = node
		while tmp:
			if tmp.name != name:
				neighbours.append(tmp.name)
			tmp = tmp.next			
		return neighbours

	def getTarryStruct(self):
		"""Generate base for Tarrys algorithm."""

		tarry_struct = {}
		for node in self.graph:
			# tarry_struct[node] = TarryNode(node, self.getNeighboursNamesList(node))
			tarry_struct[node] = {
				"name": node,
				"unused_ways": self.getNeighboursNamesList(node),
				"in_traverse":[],
				"out_traverse":[]
			}
		return tarry_struct

	def printGraph(self):
		"""Display the structure of a the connected nodes."""

		for name, value in self.graph.items():
			print(f"|{name}|:", end="")
			tmp = value
			while tmp:
				# print(f" -> {tmp.name} ({tmp.value})", end="")
				print(f" -> {tmp.name}", end="")
				if hasattr(tmp, 'value') and tmp.value:
					print(f", v({tmp.value})", end="")
				if hasattr(tmp, 'dfnum') and tmp.dfnum:
					print(f", df({tmp.dfnum})", end="")
				if hasattr(tmp, 'low') and tmp.low:
					print(f", low({tmp.low})", end="")
				tmp = tmp.next
			print("")

if __name__ == "__main__":
	# Create graph and edges
	
	graph = Graph()
	first_line = True

	for line in sys.stdin:
		if first_line:
			group = re.match("Sections:\s(.*)", line)
			group = re.split("\s*,\s", group.group(1))
			first_line = False

			# Initialization of known nodes
			for node in group:
				graph.addNode(node)
			continue

		group = re.match("(.*)", line)
		group = re.split("\s-\s", group.group(1))
		for i in range(0, len(group) - 1):
			graph.addEdge(re.sub("[\n|\s]*", '', group[i]), re.sub("[\n|\s]*", '', group[i + 1]))

	tarry_struct = graph.getTarryStruct()
	# [print(val) for val in tarry_struct.values()]

	start = "mustek"
	destination = "unikovy_modul"
	curr_node = start
	while(curr_node != destination):
		if curr_node == start and not len(tarry_struct[curr_node]["unused_ways"]) and not len(tarry_struct[curr_node]["in_traverse"]) and len(tarry_struct[curr_node]["out_traverse"]):
			print(f"Could not find destination node: |{destination}|! Back to starting node: |{start}|.")
			exit(1)

		prev_node = curr_node
		tarry_node = tarry_struct[curr_node]

		# select next node
		if len(tarry_node["unused_ways"]):
			curr_node = tarry_node["unused_ways"].pop()
			tarry_node["out_traverse"].append(curr_node)
		else:
			curr_node = tarry_node["in_traverse"][0]

		# move to the next node
		print(f"{prev_node} -> {curr_node}")
		tarry_node = tarry_struct[curr_node]

		# check if not visited -> create connection from previous node as IN
		if not len(tarry_node["in_traverse"]) and not len(tarry_node["out_traverse"]):
			tarry_node["in_traverse"].append(prev_node)
			tarry_node["unused_ways"].remove(prev_node)
