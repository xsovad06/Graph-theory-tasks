# TG - task n. 2 - avltree
# author: Damian Sova
# date: 25.10.2022

import sys
import re

class Node:
  """The main buiding block of the AVL tree."""

  def __init__(self, value):
    """AVLTREE NODE object constructor."""
    self.value = value
    self.parent = None
    self.left = None
    self.right = None
    self.bf = 0

class AvlTree:
  """Class representing the basic AVL tree structure with insert operation."""

  def __init__(self):
    """AVLTREE object constructor."""
    self.root = None

  def leftRotate(self, x):
    """Perform the left rotation at node x and update the balance factor."""

    y = x.right
    x.right = y.left
    if y.left != None:
      y.left.parent = x

    y.parent = x.parent
    if x.parent == None:
      self.root = y
    elif x == x.parent.left:
      x.parent.left = y
    else:
      x.parent.right = y
    y.left = x
    x.parent = y

    x.bf = x.bf - 1 - max(0, y.bf)
    y.bf = y.bf - 1 + min(0, x.bf)

  def rightRotate(self, x):
    """Perform the right rotation at node x and update the balance factor."""

    y = x.left
    x.left = y.right
    if y.right != None:
      y.right.parent = x
    
    y.parent = x.parent
    if x.parent == None:
      self.root = y
    elif x == x.parent.right:
      x.parent.right = y
    else:
      x.parent.left = y
    
    y.right = x
    x.parent = y

    x.bf = x.bf + 1 - min(0, y.bf)
    y.bf = y.bf + 1 + max(0, x.bf)

  def updateBalance(self, node):
    """Update the node balance factor."""

    if node.bf < -1 or node.bf > 1:
      self.rebalance(node)
      return

    if node.parent != None:
      if node == node.parent.left:
        node.parent.bf -= 1

      if node == node.parent.right:
        node.parent.bf += 1

      if node.parent.bf != 0:
        self.updateBalance(node.parent)

  def rebalance(self, node):
    """Check if the node is height balanced."""

    if node.bf > 0:
      if node.right.bf < 0:
        self.rightRotate(node.right)
        self.leftRotate(node)
      else:
        self.leftRotate(node)
    elif node.bf < 0:
      if node.left.bf > 0:
        self.leftRotate(node.left)
        self.rightRotate(node)
      else:
        self.rightRotate(node)

  def insert(self, key):
    """Method that add new key to the Binary tree and rebalace the subtrees."""

    node =  Node(key)
    y = None
    x = self.root

    while x != None:
      y = x
      if node.value < x.value:
        x = x.left
      else:
        x = x.right

    # y is parent of x
    node.parent = y
    if y == None:
      self.root = node
    elif node.value < y.value:
      y.left = node
    else:
      y.right = node

    self.updateBalance(node)

  def exportTree(self, root, result_struct = None, depth = 0):
    """Recursively traverses the tree and fill the structure with existing nodes."""

    if not result_struct:
      result_struct = []

    if len(result_struct) - 1 < depth:
      result_struct.append([])

    result_struct[depth].append(str(root.value) if root else "_")

    if root:
      self.exportTree(root.left, result_struct, depth + 1)
      self.exportTree(root.right, result_struct, depth + 1)

    return result_struct

  def print_tree(self, root):
    """Display the tree structure in desired format."""

    exported_tree = self.exportTree(root)
    processed_levels = [" ".join(nodes) for nodes in exported_tree[:-1]]
    print("|".join(processed_levels))

def readStream(file_obj):
  """Generator that reads a file by lines."""

  while True:
    for line in file_obj:
      if not line:
        break
      yield line

if __name__ == "__main__":
  # Create graph and edges

  tree = AvlTree()
  # for number in readStream(sys.stdin):
  for number in sys.stdin:
    tree.insert(int(re.sub("\n", '', number)))
    tree.print_tree(tree.root)
