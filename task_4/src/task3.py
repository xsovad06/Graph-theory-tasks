# TG - task n. 4 - parking
# author: Damian Sova
# date: 17.11.2022

# - mriežka, ktorá reprezentuje mesto
# - na vybaných súradniciach si dám body (budovy) a počet áut + body (parkovacie domy) a počet áut
# - vypočítam vzdialenosti z B do P a určím, ktoré auto bude parkovať kde
# - cieľom je minimalizovať súčet vzdialeností
# - párovanie grafu

import sys
import re
import copy
import math

ORIENTED_GRAPH = False

# -------------- helper functions -------------- #

def countDistanceFromCoordinates(source, destination):
  """From given coordinates count the distance between two points."""

  delta_x_squared = (destination[0] - source[0]) * (destination[0] - source[0])
  delta_y_squared = (destination[1] - source[1]) * (destination[1] - source[1])
  return round(math.sqrt(delta_x_squared + delta_y_squared), 3)

# ---------------------------------------------- #

class GridItem:
  def __init__(self, buildings = [], parkhouses = []):
    self.buildings = buildings
    self.parkhouses = parkhouses

  def addProperty(self, name, num_vechiles):
    """Append new property to appropriate list."""

    buildings = []
    parkhouses = []
    if name[0] == "B":
      buildings = [(name, num_vechiles)]
    elif name[0] == "P":
      parkhouses = [(name, num_vechiles)]

    return GridItem(self.buildings + buildings, self.parkhouses + parkhouses)

class TownGrid:
  def __init__(self, x = 0, y = 0):
    self.x = x
    self.y = y
    self.grid = {}

  def addProperty(self, name, x, y, num_vechiles = 0):
    """Append a new property (building or parkhouse) with number of vechiles."""

    # Create an Item if not exists
    if (x, y) not in self.grid:
      self.grid[(x, y)] = GridItem(buildings=[(name, num_vechiles)]) if name[0] == "B" else GridItem(parkhouses=[(name, num_vechiles)])
    # Update existing
    else:
      self.grid[(x, y)] = self.grid[(x, y)].addProperty(name, num_vechiles)

  def createParthouseUsageSturct(self):
    """Initialize helper struct for parkhouse usage."""

    # {"P01": [("B01_1",2)]}
    parkhouses_usage = {}
    for item in self.grid.values():
      for parkhouse in item.parkhouses:
        parkhouses_usage[parkhouse[0]] = []
    return parkhouses_usage

  def getParkhouseCapacity(self, name):
    """Returns the original parkhouse capacity."""

    for item in self.grid.values():
      for parkhouse in item.parkhouses:
        if parkhouse[0] == name:
          return parkhouse[1]
    return None

  def decrementParkhouseCapacity(self, name):
    """Decrease the original parkhouse capacity by 1."""

    for item in self.grid.values():
      for parkhouse in item.parkhouses:
        if parkhouse[0] == name:
          parkhouse = (parkhouse[0], parkhouse[1] - 1)
    return None

  def getBuildingClosestParkhouses(self, building_coordinates):
    """Return closest parkhouse to a given building order by distance."""

    closest_parkhouses = []
    closest_path = float('inf')
    for item_coordinates, item in self.grid.items():
      for parkhouse in item.parkhouses:
        distance = countDistanceFromCoordinates(building_coordinates, item_coordinates)
        if distance < closest_path:
          closest_path = distance
          closest_parkhouses.append((parkhouse[0], distance))
        else:
          closest_parkhouses.insert(0, (parkhouse[0], distance))

    return closest_parkhouses
  
  def printGrid(self):
    """Display the grid structure."""

    for key, item in self.grid.items():
      print(f"{key}: \n    buildings:{item.buildings}\n    parkhouses:{item.parkhouses}")

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

  grid = TownGrid()

  for line in sys.stdin:
    property = re.split("\s", line)
    if len(property) > 2:
      coordinates = re.sub(":", '', property[1]).split(",")
      grid.addProperty(property[0], int(coordinates[0]), int(coordinates[1]), int(property[2]))


  # grid.printGrid()
  parkhouses_usage = grid.createParthouseUsageSturct()
  for item_coordinates, item in grid.grid.items():
    for building in item.buildings:
      # get appropriate parkhouse coordinates and count distance
      closest_parkhouses = grid.getBuildingClosestParkhouses(item_coordinates)
      vechiles = building[1]
      i = 1
      parkhouse = closest_parkhouses.pop()
      while i <= vechiles:
        if len(parkhouses_usage[parkhouse[0]]) < grid.getParkhouseCapacity(parkhouse[0]):
          parkhouses_usage[parkhouse[0]].append((f"{building[0]}_{i}", parkhouse[1]))
          i += 1
        else:
          if closest_parkhouses:
            parkhouse = closest_parkhouses.pop()
          else:
            print(f"Capacity of parkhouses is insufficient. {(vechiles + 1) - i} vechiles from {building[0]} have no place for parking.")
            exit(-1)

  overall_travel_time = 0
  for parkhouse, vechiles in parkhouses_usage.items():
    i = 1
    for vechile in vechiles:
      print(f"{vechile[0]} -> {parkhouse}_{i}: {vechile[1]}")
      overall_travel_time += vechile[1]
      i += 1
  print(f"Celkem: {overall_travel_time}")

