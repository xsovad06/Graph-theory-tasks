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
  def __init__(self, name, value = None, edge = None, stream = 0):
    self.name = name
    self.value = value
    self.edge = edge
    self.stream = stream
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

  def getNeighboursList(self, name, oposite = False):
    """Returns the list of the connected nodes to the given node by name."""

    neighbours = []
    node = self.graph[name]
    tmp = node
    while tmp:
      if tmp.name != name:
        neighbours.append(tmp)
      tmp = tmp.next

    # check also neighbors from oposite direction
    if oposite:
      for node in self.graph:
        # looking for connection neighbor -> name
        if node != name:
          tmp = self.graph[name]
          while tmp:
            if tmp.name == name:
              neighbours.append(tmp)
            tmp = tmp.next

    return neighbours

  def isConnected(self):
    """For every node must exist at least one connection for connected graph."""

    for node in self.graph:
      if len(self.getNeighboursList(node)) == 0:
        return False
    return True

  def getEdgeValues(self, source, destination):
    """Return all edge attributes between source and destination."""

    if source not in self.graph:
      return None

    node = self.graph[source]
    while node:
      if node.name == destination:
        return node
      node = node.next
    return None

  def getEdgeOrientaion(self, source, destination):
    """In oriented graph get orientation of the edge between source and destination."""

    return "+" if self.existEgde(source, destination) else "-"

  def createEdmondsKarpovStructure(self):
    """For each node create dict of neccessary items for edmonds-karpov algorithm."""

    ek_struct = {}
    for node in self.graph:
      ek_struct[node] = {
        "previous" : None,
        "direction" : None,
        "distance" : 0,
        "delta" : float("inf")
      }
    return ek_struct

  def edmondsKarpovBfsIteration(self, name, ek_struct):
    """Returns the order of nodes produced by BFS algorithm."""

    expanded = {name: ek_struct[name]}
    queue = [expanded]
    # TODO: solve counts with oposite direction connections

    while queue:
      # process current node
      node = queue.pop(0)
      previous = list(node.keys())[0]
      distance = list(node.values())[0]["distance"]

      # price the way to its neighbors
      # print(f"For neighbors: {[node.name for node in self.getNeighboursList(previous, oposite = True)]}")
      for neighbor in self.getNeighboursList(previous, oposite = True):
        edge = self.getEdgeValues(previous, neighbor.name)
        reserve = edge.value - edge.stream
        new_value = {
          "previous" : previous,
          "direction" : self.getEdgeOrientaion(previous, neighbor.name),
          "distance" : distance + 1,
          "delta" : min(reserve, ek_struct[previous].get("delta"))
          }
        if neighbor.name not in expanded and reserve:
          expanded[neighbor.name] = new_value
          ek_struct[neighbor.name] = new_value
          queue.append({neighbor.name: new_value})

    return ek_struct

  def getStreamIncreasePath(self, ek_struct, drain):
    """Finds the path from source to drain and value to increase the stream."""

    path = [drain]
    tmp = ek_struct[drain]["previous"]
    while tmp:
      path.insert(0, tmp)
      tmp = ek_struct[tmp]["previous"]

    return path, ek_struct[drain]["delta"]

  def increaseStream(self, ek_struct, drain):
    """For given path increase the stream by value."""

    path, value = self.getStreamIncreasePath(ek_struct, drain)
    # print(f"Increasing stream by value {value}, path: {path}")

    increase_possible = True
    # check if the stream on all path can be increased
    for node_idx, node in enumerate(path):
      if node_idx + 1 < len(path):
        tmp = self.graph[node]

        while tmp:
          if tmp.name == path[node_idx + 1]:
            if not (tmp.stream + value <= tmp.value):
              increase_possible = False
          tmp = tmp.next

    if increase_possible:
      for node_idx, node in enumerate(path):
        if node_idx + 1 < len(path):
          tmp = self.graph[node]

          while tmp:
            if tmp.name == path[node_idx + 1]:
              tmp.stream += value
            tmp = tmp.next

    return increase_possible, path

  def getMaximunStream(self, source_node):
    """Sums up stream from source node"""

    total_stream = 0
    tmp = self.graph[source_node]
    while tmp:
      if tmp.stream:
        total_stream += tmp.stream
      tmp = tmp.next

    return total_stream

  def getIndividualEdgeStreams(self):
    """Returns structure of edges with particular stream size compared with capacity."""

    edges = {}
    for node in self.graph:
      tmp = self.graph[node]
      while tmp:
        edges[tmp.edge[1::]] = (tmp.stream, tmp.value)
        tmp = tmp.next

    sorted_edges = {}
    for edge in sorted([int(edge) for edge in edges.keys()]):
      key = str(edge).zfill(2)
      sorted_edges[f"D{key}"] = edges[key]

    return sorted_edges

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
        if tmp.stream:
          print(f", s({tmp.stream})", end="")
        tmp = tmp.next
      print("")

if __name__ == "__main__":
  # Create graph and edges

  graph = Graph()
  first_line = True
  initial_capacity = 0
  source_node = None
  final_node = "EXIT"

  for line in sys.stdin:
    if first_line:
      group = re.split("\s", line)
      first_line = False
      source_node = re.sub(':', '', group[0])
      initial_capacity = group[1]
      continue

    edge = re.split("\s", line)
    if len(edge) < 4:
      continue
    graph.addEdge(edge[1], edge[3], value = int(edge[4]), edge = re.sub(':', '', edge[0]))

  # print("-------------- ORIGINAL GRAPH ------------")
  # graph.printGraph()

  edmonds_karpov = graph.createEdmondsKarpovStructure()
  increased = True
  max_path = 0
  while increased:
    # strating from source node BFS
    edmonds_karpov = graph.edmondsKarpovBfsIteration(source_node, edmonds_karpov)
    # increase stream on edmonds-karpov path
    increased, path = graph.increaseStream(edmonds_karpov, final_node)
    max_path = max(max_path, len(path))

  # get maximum network stream –> sum of streams from source node
  max_stream = graph.getMaximunStream(source_node)
  print(f"Group size: {max_stream}")

  edges = graph.getIndividualEdgeStreams()
  for edge, stream_capacity in edges.items():
    stream = f"!{stream_capacity[0]}!" if stream_capacity[0] == stream_capacity[1] else f"{stream_capacity[0]}"
    print(f"{edge}: {stream}")

  print(f"Time: {(int(initial_capacity) / max_stream) + max_path - 2}")
