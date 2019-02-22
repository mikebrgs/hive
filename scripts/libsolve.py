import rosbag
import sys

class BagReader(object):
  """docstring for BagReader"""
  def __init__(self, filename):
    super(BagReader, self).__init__()
    self.bag = rosbag.Bag(filename,"r")

  def read(self, topic, handler, data):
    for topic, msg, _ in self.bag.read_messages(topics = topic):
      handler(msg, topic, data)
    return

  def __del__(self):
    self.bag = None

def dummy(msg, topic, data):
  pass


class Solver(object):
  """docstring for Solver"""
  def __init__(self):
    self.light_horizontal = dict()
    self.light_vertical = dict()

  def solver():
    return None


def main(args):
  br = BagReader(args[1])
  br.read("/loc/vive/light", dummy, None)

if __name__ == '__main__':
  main(sys.argv)