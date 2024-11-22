from dynamixel_sdk import *
import numpy as np

# Class suited for this project's needs, based on Dynamixel SDK example files
class servos():
    def __init__(self, port, numMotors):

        # These constants depend on motor, assuming MX-28AR
        self.ADDR_TORQUE_ENABLE          = 64
        self.ADDR_PROFILE_ACCELERATION   = 108
        self.ADDR_PROFILE_VELOCITY       = 112
        self.ADDR_GOAL_POSITION          = 116
        self.ADDR_PRESENT_POSITION       = 132
        self.DXL_MINIMUM_POSITION_VALUE  = 0 
        self.DXL_MAXIMUM_POSITION_VALUE  = 4095 
        self.BAUDRATE                    = 57600
        self.BYTE_LEN                    = 4
        
        self.DXL_MOVING_STATUS_THRESHOLD = 20
        self.TORQUE_ENABLE               = 1
        self.TORQUE_DISABLE              = 0 
        self.PROTOCOL_VERSION            = 2.0

        # Profile velocity storage
        self.prev_profile                = 0

        # IDs: Motor IDs must be sequential (1, 2, ..., n)
        if type(numMotors) is not int:
            print(f"Invalid numMotors: {numMotors}")
            return
        self.motor = numMotors

        # Handlers
        self.portHandler = PortHandler(port)
        self.packetHandler = PacketHandler(self.PROTOCOL_VERSION)
        self.groupBulkRead = GroupBulkRead(self.portHandler, 
                                           self.packetHandler)
        self.groupBulkWrite = GroupBulkWrite(self.portHandler, 
                                             self.packetHandler)

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

        # Set initial profile
        self._set_profile(self.prev_profile)
            
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

    def setPos(self, id, goal):    
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


    def setAllPos(self, goals, dur = 0):
        # verify and convert goal (degrees) to motor scale
        for j in range(self.motor):
            if goals[j] in range(0, 361):
                goals[j] = int(goals[j] / 360 * self.DXL_MAXIMUM_POSITION_VALUE) + self.DXL_MINIMUM_POSITION_VALUE
                # print(goals[j])
            else:
                print(f"Invalid goal position: {goals[j]}")
                return
        
        try:
            if dur != self.prev_profile:
                self._set_profile(dur)
            for j in range(self.motor):
                self.groupBulkWrite.addParam(
                    j + 1,     # cycle through motor id
                    self.ADDR_GOAL_POSITION, 
                    self.BYTE_LEN, 
                    self.param_goal_position(goals[j]))
            self.groupBulkWrite.txPacket()
            self.groupBulkWrite.clearParam()
        except:
            return False
        
        self.waitToReachGoal( [ids+1 for ids in range(self.motor)], goals)
        return True
    
    def waitToReachGoal(self, ids, goals):
        if len(ids) is not len(goals):
            print("Invalid input")
            return

        while True:
            present_pos = [None for x in range(len(ids))]

            for id in range(len(ids)):
                # Read current position
                present_pos[id], result, error = self.packetHandler.read4ByteTxRx(self.portHandler, ids[id], self.ADDR_PRESENT_POSITION)
                if not self.validateComm(result, error):
                    print(f"Failed to read position on motor {ids[id]}")
                    return
                
            # Check if position reached
            if not sum(np.abs(np.subtract(goals, present_pos)) >= self.DXL_MOVING_STATUS_THRESHOLD):
                break

    def readPos(self, id):
        present_pos, result, error = self.packetHandler.read4ByteTxRx(self.portHandler, id, self.ADDR_PRESENT_POSITION)
        if not self.validateComm(result, error):
            print(f"Failed to read position on motor {id}")
            return
        
        return present_pos
    
    #-------------------#
    # Private Functions #
    #-------------------#
    def _set_profile(self, dur_ms):
        # Change profile vel and acc of Dynamixels.
        # Assumes that the motors are set to "Time-based Profile" (ADDR 10)
        for i in range(1, self.motor+1):
            self.packetHandler.write4ByteTxRx(self.portHandler, i, 
                                        self.ADDR_PROFILE_VELOCITY, dur_ms)
            self.packetHandler.write4ByteTxRx(self.portHandler, i, 
                                        self.ADDR_PROFILE_ACCELERATION, 
                                        int(dur_ms/3))
        self.prev_profile = dur_ms
        return
    
    def param_goal_position(self, goal_pos):
        # Allocate goal position value into byte array
        # Split a 32-bit number into 4 8-bit numbers for packet transmission
        return [DXL_LOBYTE(DXL_LOWORD(goal_pos)), 
                DXL_HIBYTE(DXL_LOWORD(goal_pos)),
                DXL_LOBYTE(DXL_HIWORD(goal_pos)), 
                DXL_HIBYTE(DXL_HIWORD(goal_pos))]
            