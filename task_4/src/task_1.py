# TG - task n. 1
# author: Damian Sova
# date: 20.9.2022

import sys
import re

graf = {}
# riedka matica

# A -> B: 5
graf[("A", "B")] = 5

graf.get(("A", "B"))

# matica incidencie
#	- rieši multigrafy
# - najmenej používanejšia

# Zoznam vrcholov a hran
# - najpoužívanejší
# - dynamicke struktury

from collections import namedtuple

Node = namedtuple("node", ["name", "next", "neighbor"])

F = Node("F", None, None)
E = Node("E", None, None)

# Objektová reprezentacia
# - snažiť sa robiť imutable objekty -> modifikácia vytvára nový objekt
# struktury
# - zoznamy susedou
# - zoznamy susedou a hran
# Operacie
# - pridaj/odober uzol/hranu
# - najdi uzol/naslednikov/hrany/hranu

# vytvoriť metódy s komentármi (komentáre popredu)
class Graf:
	def __init__(self, nodes = []):
		self.__nodes = nodes

	def __repr__(self):
		return f"Graf(nodes = {self.__nodes})"

	def addNode(self, node):
		"""Connect a new leaf to new graph."""
		return Graf(
			nodes = self.__nodes.append(node)
		)

g = Graf()
g = g.addNode("A")

print(g.__repr__())