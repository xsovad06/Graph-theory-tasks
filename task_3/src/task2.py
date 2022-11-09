# TG - task n. 3 - inspection
# author: Damian Sova
# date: 1.11.2022

# prejst každú hranu práve jedenkrát -> Eulerian path

from readline import append_history_file
import sys
import re
import copy

ORIENTED_GRAPH = False

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

  def createSingleDfsCycle(self, first_name):
    """Returns the order of nodes produced by DFS algorithm from given node to given node."""

    stack = [first_name]
    expanded = []

    while stack:
      node = stack.pop()

      if len(expanded) > 2 and node == first_name:
        expanded.append(node)
        return expanded

      if node in expanded:
        continue

      expanded.append(node)
      # print(f" Processing {node}, expanded: {expanded}")

      for neighbor in self.getNeighboursList(node):
        # prevent previous path
        if len(expanded) >= 2:
          # avoid path -> node, current_node, node
          if expanded[-2] != neighbor.name:
            # print(f"  Adding to stack {neighbor.name}")
            stack.append(neighbor.name)
        else:
          # print(f"  Adding to stack {neighbor.name}")
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
        # došlo k backtrackingu pri DFS, treba dohladať, najbližší spoj z predošle spracovaného grafu z DFS do Node
        else:
          for i in (range(dfs_path.index(last_node), -1, -1)):
            if graph.existEgde(dfs_path[i], node):
              self.addEdge(dfs_path[i], node, dfnum=dfnum)
              break
      last_node = node

  def checkEulerianPathPrecondition(self):
    """Count the number of each node neighbors. All must be even or precisely two odd."""

    odd_count = 0
    for node in self.graph:
      if len(self.getNeighboursList(node)) % 2 != 0:
        if odd_count > 1:
          return False
        odd_count += 1
    return True

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

if __name__ == "__main__":
  # Create graph and edges

  graph = Graph()

  for line in sys.stdin:
    edge = re.split("\s", line)
    print(edge)
    graph.addEdge(edge[0], edge[2], re.sub('\|', '', edge[1]))

  print("-------------- ORIGINAL GRAPH ------------")
  graph.printGraph()

  if not graph.checkEulerianPathPrecondition():
    print("Eulerian path precondition has not been satisfied.")
    exit(1)

  dfs_cycles = []
  first_name = list(graph.graph.keys())[0]
  cycle = graph.createSingleDfsCycle(first_name)
  print(cycle)

  print("|", end="")
  for idx, node in enumerate(cycle):
    if idx + 1 < len(cycle):
      next_node = cycle[idx + 1]
      if graph.existEgde(node, next_node):
        print(f"{graph.getEgdeValue(node, next_node)}|", end="")
  print()
