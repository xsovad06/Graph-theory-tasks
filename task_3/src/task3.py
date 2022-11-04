# TG - task n. 3 - isp
# author: Damian Sova
# date: 4.11.2022

# searching for shortest path to all nodes -> Dijkstra algorithm

import sys
import re
import copy

ORIENTED_GRAPH = False

class Node:
  def __init__(self, name, value = None, dfnum = None, low = None):
    self.name = name
    self.value = value
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

    # Adding missing edges to interconnect neighbors
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
  
  def dijkstraAlgorithmShortestWay(self, name):
    """Returns the order of nodes produced by BFS algorithm."""

    predecessor = ""
    cost = 0
    expanded = {name: (predecessor, cost)}
    queue = [expanded]

    while queue:
      # process current node
      node = queue.pop(0)
      predecessor = list(node.keys())[0]
      cost = list(node.values())[0][1]
      
      # price the way to its neighbors
      for neighbor in self.getNeighboursList(predecessor):
        new_value = (predecessor, cost + neighbor.value)
        # for new node just price the way
        if neighbor.name not in expanded:
          expanded[neighbor.name] = new_value
          queue.append({neighbor.name: new_value})
        else:
          # for already processed save only shorter way and put updated node again to queue
          if cost + neighbor.value < expanded[neighbor.name][1]:
            expanded[neighbor.name] = new_value
            queue.append({neighbor.name: new_value})
      # all neighbors priced, save node as exported
      expanded[predecessor] = (list(node.values())[0][0], list(node.values())[0][1])
    return expanded

  def printGraph(self):
    """Display the structure of a the connected nodes."""

    for name, value in self.graph.items():
      print(f"|{name}|:", end="")
      tmp = value
      while tmp:
        print(f" -> {tmp.name}", end="")
        if tmp.value:
          print(f", v({tmp.value})", end="")
        if tmp.dfnum:
          print(f", df({tmp.dfnum})", end="")
        if tmp.low:
          print(f", low({tmp.low})", end="")
        tmp = tmp.next
      print("")

#------- helper functions -------#

def getDijkstraStructChildren(parent):
  """From custom structure return nodes according to parent node."""

  children = []
  for name, value in dijkstra_struct.items():
    if value[0] == parent:
      children.append((name, value[1]))
  return children

def printChild(level, node, cost):
  """Recursively display the shortest path between nodes with final cost."""

  level_prefix = "|  "
  if level == -1:
    print(root)
  elif not level:
    print(f"+- {node}: {cost}m")
  else:
    print(f"{level_prefix * level}+- {node}: {cost}m")

  for child in getDijkstraStructChildren(node):
    printChild(level + 1, child[0], child[1])

if __name__ == "__main__":
  # Create graph and edges

  graph = Graph()

  for line in sys.stdin:
    edge = re.split("\s", line)
    graph.addEdge(edge[0], re.sub(":", '', edge[2]), int(re.sub("m", '', edge[3])))

  # print("-------------- ORIGINAL GRAPH ------------")
  # graph.printGraph()

  # print("-------------- DIJKSTRA ALFGORITHM ------------")
  root = list(graph.graph.keys())[0]
  # struct -> {node_name: (predecessor, cost)}
  dijkstra_struct = graph.dijkstraAlgorithmShortestWay(root)
  # print(dijkstra_struct)

  # print("-------------- RESULTS ------------")
  printChild(-1, root, 0)
  

