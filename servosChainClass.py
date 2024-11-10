from dynamixel_sdk import *

# Class suited for this project's needs, based on Dynamixel SDK example files
class servos():
    def __init__(self, port, numMotors):

        # These constants depend on motor, assuming MX-28AR
        self.ADDR_TORQUE_ENABLE          = 64
        self.ADDR_GOAL_POSITION          = 116
        self.ADDR_PRESENT_POSITION       = 132
        self.DXL_MINIMUM_POSITION_VALUE  = 0 
        self.DXL_MAXIMUM_POSITION_VALUE  = 4095 
        self.BAUDRATE                    = 57600
        
        self.DXL_MOVING_STATUS_THRESHOLD = 20
        self.TORQUE_ENABLE               = 1
        self.TORQUE_DISABLE              = 0 
        self.PROTOCOL_VERSION            = 2.0

        # IDs: Motor IDs must be sequential (1, 2, ..., n)
        if type(numMotors) is not int:
            print(f"Invalid numMotors: {numMotors}")
            return
        self.motor = numMotors

        # Handlers
        self.portHandler = PortHandler(port)
        self.packetHandler = PacketHandler(self.PROTOCOL_VERSION)

        # Open port
        if not self.portHandler.openPort():
            print("Failed to open port")
            return
        
        # Set baudrate
        if not self.portHandler.setBaudRate(self.BAUDRATE):
            print("Failed to set baudrate")
            return
        
        # Enable torque on each motor
        for i in range(1, self.motor+1):
            result, error = self.packetHandler.write1ByteTxRx(self.portHandler, i, self.ADDR_TORQUE_ENABLE, self.TORQUE_ENABLE)
            if not self.validateComm(result, error):
                print(f"Failed to enable torque in motor {i}")
            
    def __del__(self):
        # Disable torque on each motor
        for i in range(1, self.motor+1):
            result, error = self.packetHandler.write1ByteTxRx(self.portHandler, i, self.ADDR_TORQUE_ENABLE, self.TORQUE_DISABLE)
            if not self.validateComm(result, error):
                print(f"Failed to disable torque in motor {i}")

        # Close port
        self.portHandler.closePort()

    def validateComm(self, result, error):
        if result != COMM_SUCCESS:
            print("%s" % self.packetHandler.getTxRxResult(result))
        elif error != 0:
            print("%s" % self.packetHandler.getRxPacketError(error))
        else:
            return True
        return False

    def setPosition(self, id, goal):    
        # Verify motor ID
        if id not in range(1, self.motor+1):
            print(f"Invalid ID: {id}")
            return
        
        # verify and convert goal (degrees) to motor scale
        if goal in range(0, 361):
            goal = int(goal / 360 * self.DXL_MAXIMUM_POSITION_VALUE) + self.DXL_MINIMUM_POSITION_VALUE
            print(goal)
        else:
            print(f"Invalid goal position: {goal}")
            return
        
        # Write goal position
        result, error = self.packetHandler.write4ByteTxRx(self.portHandler, id, self.ADDR_GOAL_POSITION, goal)
        if not self.validateComm(result, error):
            print(f"Failed to write goal position on motor {id}")
            return
        
        # imitating do while
        while True:
            # Read current position
            present_pos, result, error = self.packetHandler.read4ByteTxRx(self.portHandler, id, self.ADDR_PRESENT_POSITION)
            if not self.validateComm(result, error):
                print(f"Failed to read position on motor {id}")
                return
            
            # Check if position reached
            if abs(goal - present_pos) < self.DXL_MOVING_STATUS_THRESHOLD:
                break 
            