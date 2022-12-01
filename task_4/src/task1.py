# TG - task n. 4 - evacuation
# author: Damian Sova
# date: 17.11.2022

# - algoritmus zlepšujúceho sa toku ? Edmonds-Karpov
# - algoritmus maximálneho toku
# - 2 algo - 1 hľadá zlepšujúcu cestu a druhý tlačí dole
# Postup1:
#     1. ku každej hrane vytvorím hranu protismernú (tá má kapacitu toku, ale na začiatku si nastavím kapacity protismerných hrán všade na 0)
#     2. nájdem cesty zo začiatku do exitu - zlepšujúce toky:
#         1.iterácia M01→M03→M04→M05→EXIT a nájdem si minimálny tok na tejto ceste - 15 a následne znížim ostatné kapacity a zvýšim kapacity protichodných hrán o tento minimálny tok)
#         2.iterácia M01→M02→M03→EXIT a spravím to isté
#         3.iterácia M01→M04→M03→M05→EXIT
#     3. spočítam veľkosť skupiny - súčet min. tokov
#     4. spočítam čas - najdlhšiu cestu (počet dverí, kt. som prešla) a pomocou vzorca vypočítam čas - začiatočná kapacita/veľkosť skupony + najdlhšia cesta (120/40+4)

import sys
import re
import copy

ORIENTED_GRAPH = True

class Node:
  def __init__(self, name, value = None, edge = None):
    self.name = name
    self.value = value
    self.edge = edge
    self.next = None

class Graph:
  def __init__(self):
    self.graph = {}

  def addNode(self, name):
    """Append new vertex to the vertices list."""

    self.graph[name] = None

  def addEdge(self, source, destination, value = None, edge = None):
    """Connect the given destination vertex to the graph list source and
       given source vertex to the graph list destination."""

    # Create a node if not exists
    for node in (source, destination):
      if node not in self.graph:
        self.addNode(node)

    # Dest; Dest.next -> list; List -> Dest
    node = Node(destination)
    node.value = value
    node.edge = edge
    node.next = self.graph[source]
    self.graph[source] = node

    # Source; Source.next -> list; List -> Source
    if not ORIENTED_GRAPH:			
      node = Node(source)
      node.value = value
      node.edge = edge
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

  def getEgdeValue(self, source, destination):
    """Check whether the given source and destination are connected."""

    node = self.graph[source]
    while node:
      if node.name == destination:
        return node.value
      node = node.next

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
    """Returns the order of nodes produced by DFS algorithm. Works in Oriented graph."""

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

  def isConnected(self):
    """For every node must exist at least one connection for connected graph."""

    for node in self.graph:
      if len(self.getNeighboursList(node)) == 0:
        return False
    return True

  def checkEulerianPathPrecondition(self):
    """Count the number of each node neighbors. All must be even or precisely two odd."""

    odd_nodes = []
    for node in self.graph:
      if len(self.getNeighboursList(node)) % 2 != 0:
        if len(odd_nodes) > 1:
          return False, odd_nodes
        odd_nodes.append(node)
    return True, odd_nodes

  def createNodeConnectionsStructure(self):
    """For each node create list of available and used connections to neighbor nodes."""

    connection_struct = {}
    for node in self.graph:
      avail_conns = []
      for neighbor in self.getNeighboursList(node):
        avail_conns.append((neighbor.name, neighbor.value))
      connection_struct[node] = {"avail_conns" : avail_conns, "used_conns" : []}
    return connection_struct

  def printGraph(self):
    """Display the structure of a the connected nodes."""

    for name, value in self.graph.items():
      print(f"|{name}|:", end="")
      tmp = value
      while tmp:
        print(f" -> {tmp.name}", end="")
        if tmp.value:
          print(f", v({tmp.value})", end="")
        if tmp.edge:
          print(f", e({tmp.edge})", end="")
        tmp = tmp.next
      print("")

def getValue(dijkstra_struct, node):
  """Returns the shortest path cost to a given node."""

  return dijkstra_struct[node][1]

def getPredecessor(dijkstra_struct, node):
  """Returns the previous node in the shortest path to the given node."""

  return dijkstra_struct[node][0]

if __name__ == "__main__":
  # Create graph and edges

  graph = Graph()
  first_line = True
  source_node = None

  for line in sys.stdin:
    if first_line:
      group = re.split("\s", line)
      first_line = False
      source_node = (group[0], group[1])
      continue

    edge = re.split("\s", line)
    if len(edge) < 4:
      continue
    graph.addEdge(edge[1], edge[3], value = int(edge[4]), edge = re.sub(':', '', edge[0]))

  print("-------------- ORIGINAL GRAPH ------------")
  graph.printGraph()
