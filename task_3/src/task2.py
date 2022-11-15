# TG - task n. 3 - inspection
# author: Damian Sova
# date: 1.11.2022

# prejst každú hranu práve jedenkrát -> Eulerian path

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
        if tmp.dfnum:
          print(f", df({tmp.dfnum})", end="")
        if tmp.low:
          print(f", low({tmp.low})", end="")
        tmp = tmp.next
      print("")

def processSingleDirectionPathMove(node, next_node, eulerian_struct, expanded_paths):
  """For existing connections store path and move avail conn to used conn."""

  for conn in eulerian_struct[node]["avail_conns"]:
    if next_node == conn[0]:
      # move the path from available to used connections
      if conn[1] not in expanded_paths:
        expanded_paths.append(conn[1])
      eulerian_struct[node]["used_conns"].append((conn[0], conn[1]))
      eulerian_struct[node]["avail_conns"].remove((conn[0], conn[1]))
      return True, eulerian_struct
  return False, eulerian_struct

def createSingleDfsCycle(first_name, last_name, eulerian_struct):
  """Returns the order of nodes produced by DFS algorithm from given node to given node."""

  stack = [first_name]
  expanded_nodes = []
  expanded_paths = []

  while stack:
    node = stack.pop()

    # end cycle if reached desired node
    if len(expanded_nodes) > 2 and node == last_name:
      previous_node = expanded_nodes[-1]
      found_way = processSingleDirectionPathMove(previous_node, node, eulerian_struct, expanded_paths)
      if found_way:
        # also move path in oposite direction
        _, eulerian_struct = processSingleDirectionPathMove(node, previous_node, eulerian_struct, expanded_paths)
        # store expanded node
        expanded_nodes.append(node)
      return expanded_nodes, expanded_paths

    # otherwise process current path from previous node to current node
    found_way = False
    if len(expanded_nodes) > 0:
      previous_node = expanded_nodes[-1]
      found_way, eulerian_struct = processSingleDirectionPathMove(previous_node, node, eulerian_struct, expanded_paths)
      if found_way:
        # also move path in oposite direction
        _, eulerian_struct = processSingleDirectionPathMove(node, previous_node, eulerian_struct, expanded_paths)
        # store expanded node
        expanded_nodes.append(node)
      else:
        continue
    # the first node can be appended to  expanded nodes
    else:
      expanded_nodes.append(node)

    # Found available paths from current node
    for neighbor in eulerian_struct[node]["avail_conns"]:
      stack.append(neighbor[0])

  return expanded_nodes, expanded_paths

def anyAvailablePaths(eulerian_struct):
  """Return an indicator of availability any path."""

  available_paths = False
  for node in eulerian_struct.values():
    if len(node["avail_conns"]) > 0:
      available_paths = True
      break
  return available_paths

def printEulerianPathFromNodes(cycles, paths, cycle_idx):
  """Recursively display the order of connections between the path nodes in cycles."""

  curr_cycle = cycles[cycle_idx]
  curr_path = paths[cycle_idx]

  for node_idx, node in enumerate(cycles[cycle_idx]):
    next_cycle_idx = cycle_idx + 1
    # if a new cycle started in this node (first node from next cycle), print path recursively
    if next_cycle_idx < len(cycles) and node == cycles[next_cycle_idx][0]:
      printEulerianPathFromNodes(cycles, paths, next_cycle_idx)
      cycles.pop(next_cycle_idx)

    # finally display the path by popping the fist one
    if node_idx + 1 < len(curr_cycle):
      print(f"{curr_path.pop(0)}|", end="")

if __name__ == "__main__":
  # Create graph and edges

  graph = Graph()

  for line in sys.stdin:
    edge = re.split("\s", line)
    if len(edge) < 3:
      continue
    graph.addEdge(edge[0], edge[2], re.sub('\|', '', edge[1]))

  # print("-------------- ORIGINAL GRAPH ------------")
  # graph.printGraph()

  if not graph.isConnected():
    print("Graph is not connected. Cannot find Eulerian path!")
    exit(1)

  passed, odd_nodes = graph.checkEulerianPathPrecondition()
  if not passed:
    print("Eulerian path precondition has not been satisfied.")
    exit(1)

  eulerian_struct = graph.createNodeConnectionsStructure()

  first_name = list(graph.graph.keys())[0]
  dfs_cycles = [[first_name]]
  dfs_paths = []

  # try to recursively find dfs cycles in available connections
  first_cycle = True
  for dfs_cycle in dfs_cycles:
    for node in dfs_cycle:
      if not anyAvailablePaths(eulerian_struct):
        break

      # if there are 2 nodes with odd number of connections, the eulerian path start is in fist and end in the second.
      if first_cycle and len(odd_nodes) == 2:
        cycle, path = createSingleDfsCycle(odd_nodes[0], odd_nodes[1], eulerian_struct)
        first_cycle = False
      # otherwise starting and endding point is the same
      else:
        cycle, path = createSingleDfsCycle(node, node, eulerian_struct)
      dfs_cycles.append(cycle)
      dfs_paths.append(path)

  # Remove the initial node from the result
  dfs_cycles.pop(0)
  print("|", end="")
  printEulerianPathFromNodes(dfs_cycles, dfs_paths, 0)
  print()
