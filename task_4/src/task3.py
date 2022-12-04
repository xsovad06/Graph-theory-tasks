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

    distances = []
    closest_parkhouses = {}
    for item_coordinates, item in self.grid.items():
      for parkhouse in item.parkhouses:
        distance = countDistanceFromCoordinates(building_coordinates, item_coordinates)
        distances.append(distance)
        closest_parkhouses[distance] = parkhouse[0]

    return [(closest_parkhouses[distance], distance) for distance in sorted(distances, reverse=True)]
  
  def printGrid(self):
    """Display the grid structure."""

    for key, item in self.grid.items():
      print(f"{key}: \n    buildings:{item.buildings}\n    parkhouses:{item.parkhouses}")

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

