import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/shay/vlm_car/vlm_car_ws/install/scservo_driver'
