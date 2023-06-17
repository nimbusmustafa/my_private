#!/usr/bin/env python

import rospy
from std_msgs.msg import String
from cashier_system.msg import bill


def main():
    pub = rospy.Publisher('bill_topic', bill, queue_size=10)
    rospy.init_node('publisher', anonymous=True)
    rate = rospy.Rate(10)
    bill_no =1
    while not rospy.is_shutdown():
        quantity = int(input("Enter quantity: "))
        price = float(input("Enter price of each product: "))
        bill1 = bill()
        bill1.bill_number = bill_no
        bill_no += 1
        bill1.timestamp = rospy.get_time()
        bill1.quantity = quantity
        bill1.price = price
        bill1.total = bill1.quantity * bill1.price
        pub.publish(bill1)
        rate.sleep()

if __name__ == '__main__':
    try:
       main()
    except rospy.ROSInterruptException:
        pass
