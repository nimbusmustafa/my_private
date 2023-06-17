import rospy
from std_msgs.msg import String
from cashier_system.msg import bill
from cashier_system.srv import updateParameters

inventory_param = '/inventory'
income_param = '/income'

inventory = 100
income = 0

def bill_callback(bill):
    quantity = bill.quantity
    price = bill.price

    rospy.wait_for_service('/update_parameters')
    try:
        update_parameters = rospy.ServiceProxy('/update_parameters', updateParameters)
        response = update_parameters(quantity, price)
        if response.success:
            rospy.loginfo("Parameters updated successfully.")
        else:
            rospy.loginfo("Failed to update parameters.")
    except rospy.ServiceException as e:
        rospy.loginfo("Service call failed: %s", e)


def handle_update_parameters(request):
    inventory = rospy.get_param(inventory_param, 100)  
    income = rospy.get_param(income_param, 0)

    change_in_inventory = request.quantity
    change_in_income = request.quantity * request.price

   
    rospy.set_param(inventory_param, inventory - change_in_inventory)
    rospy.set_param(income_param, income + change_in_income)

    return 'Parameters updated successfully'


def subscriber():
    rospy.init_node('subscriber', anonymous=True)

    rospy.Subscriber('bill_topic', bill, bill_callback)
    rospy.Service('update_parameters', updateParameters, handle_update_parameters)

    rospy.spin()


if __name__ == '__main__':
    try:
        subscriber()
    except rospy.ROSInterruptException:
        pass
