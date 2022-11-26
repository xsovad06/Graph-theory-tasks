# TG - task n. 4 - teams
# author: Damian Sova
# date: 17.11.2022

# - sekvenční barvení uzlů
# - nájsť uzol s najväčším počtom susedov a obarvím si ho jednou farbou
# - vezmem jeho susedov a nájdem uzol s najväčším počtom susedov a obarvím inou farbou
# - nájdem ďalší (taký, ktorý susedí s obarveným) a obarvím ďalšou farbou
# - zasa nájdem najväčší (susediaci s obarveným) a pozriem sa, akú z tých farieb, čo som už použila môžem použiť (kto není sused)

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
       given source vertex to the graph list destination."""

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

  def isConnected(self):
    """For every node must exist at least one connection for connected graph."""

    for node in self.graph:
      if len(self.getNeighboursList(node)) == 0:
        return False
    return True

  def createNodeneighbourcoloursStruct(self):
    """For each node create list of clear and coloured neighbours."""

    connection_struct = {}
    for node in self.graph:
      clear_nodes = []
      for neighbour in self.getNeighboursList(node):
        clear_nodes.append(neighbour.name)
      connection_struct[node] = {"clear" : clear_nodes, "coloured" : []}
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
        if tmp.dfnum:
          print(f", df({tmp.dfnum})", end="")
        if tmp.low:
          print(f", low({tmp.low})", end="")
        tmp = tmp.next
      print("")

def getNodeneighbour(neighbour_colours, node):
  """Return node directly connected with given node."""

  return neighbour_colours[node]["clear"][0] if neighbour_colours[node]["clear"] else neighbour_colours[node]["coloured"][0][0]
  

def isColoured(neighbour_colours, node):
  """Return indicator that given node is already colured."""

  return node in [node[0] for node in neighbour_colours[getNodeneighbour(neighbour_colours, node)]["coloured"]]


def getNodeWithHighestFeatureOccurence(neighbour_colours, feature):
  """Return node with most neighbours with desired feature."""

  most_coloured_neighbours = 0
  desired_node = None
  for node, lists in neighbour_colours.items():
    feature_list_len = len(lists[feature])
    if feature_list_len >= most_coloured_neighbours:
      if feature_list_len == most_coloured_neighbours == 0:
        continue
      
      if not isColoured(neighbour_colours, node):
        most_coloured_neighbours = feature_list_len
        desired_node = node

  return desired_node

def getNextNodeToColour(neighbour_colours):
  """Return the most suitable node for colouring."""

  desired_node = getNodeWithHighestFeatureOccurence(neighbour_colours, "coloured")

  return desired_node if desired_node else getNodeWithHighestFeatureOccurence(neighbour_colours, "clear")

def colourNode(node, neighbour_colours, neighbours):
  """Move in every neighbour from clear to coloured and asign colour value."""

  # get list of colour values from neighbours
  colour_values = []
  for neighbour in neighbour_colours[node]["coloured"]:
    colour_values.append(neighbour[1])

  # find minimal unused colour
  final_colour = 1
  if colour_values:
    colour_values.sort()
    final_colour = colour_values[0] - 1 if colour_values[0] > 1 else colour_values[-1] + 1
    
  # process every neighbour with updated node state
  for neighbour in neighbours:
    neighbour_colours[neighbour]["clear"].remove(node)
    neighbour_colours[neighbour]["coloured"].append((node, final_colour))
  
  return final_colour

def getPocessedGroups(neighbour_colours, final_colours):
  """From neighbour colours struc get groups by colour value."""

  groups = {}
  for colour in set(final_colours):
    groups[colour] = []

  for lists in neighbour_colours.values():
    for name, colour in lists["coloured"]:
      if name not in groups[colour]:
        groups[colour].append(name)

  return groups

if __name__ == "__main__":
  # Create graph and edges

  graph = Graph()
  first_line = True

  for line in sys.stdin:
    if first_line:
      group = re.split("\s*,\s", line)
      first_line = False

      # Initialization of known nodes
      for node in group:
        graph.addNode(re.sub("[\n|\s]*", '', node))
      continue

    edge = re.split("\s+-\s+", line)
    graph.addEdge(edge[0], re.sub("[\n|\s]*", '', edge[1]))

  # print("-------------- ORIGINAL GRAPH ------------")
  # graph.printGraph()

  neighbour_colours = graph.createNodeneighbourcoloursStruct()

  final_colours = []
  for i in range(len(neighbour_colours)):

    node = getNextNodeToColour(neighbour_colours)
    
    # colour the node with the lowest colour value according to neighbour colours
    final_colours.append(colourNode(node, neighbour_colours, [node.name for node in graph.getNeighboursList(node)]))

  for group in getPocessedGroups(neighbour_colours, final_colours).values():
    print(", ".join(group))

    