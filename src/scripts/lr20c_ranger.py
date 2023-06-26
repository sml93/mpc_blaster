import sys
sys.path.insert(0, '../..')
import time
import rospy
import numpy as np

from serial import Serial as device
from std_msgs.msg import String, Float32


TIMEOUT = 0.1
sensor_frequency = 10

serial_port_name = "/dev/ttyUSB0"
serial_port_baudrate = 115200
port = device(serial_port_name, serial_port_baudrate, timeout=TIMEOUT)

# Initialising serial port handshake
port.write('wwww\r\n')
port.readline()

# Getting product information
port.write('?\r\n')
productInfo = port.readline()
print('Product information: ' + productInfo)


def getPosition(standoff):
  pe_hat = np.array([1, 1, 2])    # to add subscriber to take in pe_hat from mpc_blaster
  dne_hat = np.linalg.norm(pe_hat)
  dne = standoff
  mu = dne/dne_hat
  pe_ranger = mu * pe_hat
  pe_ranger = np.round(pe_ranger, 4)
  return pe_ranger

def talker():
  pub = rospy.Publisher('standoff', Float32, queue_size=10)
  rospy.init_node('so_sensor', anonymous=True)
  r = rospy.Rate(sensor_frequency)
  msg = Float32()

  while not rospy.is_shutdown():
    try: 
      port.write('LD\r\n')
      distanceStr = port.readline()
      # Convert the distance string response into a number
      distanceM = float(distanceStr)
      pe_ranger = getPosition(distanceM)
      msg.data = round(distanceM,3)

      # Do what you want with the distance information here
      # rospy.loginfo('pe_ranger: {0} m'.format(pe_ranger))
      rospy.loginfo('standoff: {0:.2f} m'.format(distanceM))
      pub.publish(msg)

      # Wait for 50ms before the next reading is taken
      time.sleep(0.05)
    except rospy.ROSException as e:
      print("Interrupted")
      pass
    
    r.sleep()

  rospy.spin()

if __name__ == "__main__":
  try: talker()
  except rospy.ROSInterruptException:
    pass
