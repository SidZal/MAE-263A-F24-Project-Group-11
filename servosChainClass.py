from dynamixel_sdk import *
import numpy as np

# Class suited for this project's needs, based on Dynamixel SDK example files
# Not foolproof - assumes correct use of Dynamixel motors
class servos():
    def __init__(self, port, numMotors, startingPositions = None):

        # These constants depend on motor, assuming MX-28AR
        self.ADDR_TORQUE_ENABLE          = 64
        self.ADDR_OPERATING_MODE         = 11
        self.ADDR_PROFILE_ACCELERATION   = 108
        self.ADDR_PROFILE_VELOCITY       = 112
        self.ADDR_GOAL_POSITION          = 116
        self.ADDR_PRESENT_POSITION       = 132
        self.ADDR_HOMING_OFFSET          = 20

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
        
        # Set op mode and starting positions, only works when motors in extended position control
        if startingPositions is not None and len(startingPositions) == numMotors:
            for i in range(self.motor):
                self.setOperatingMode(i+1, 4)
                self.setHome(i+1, startingPositions[i])

        # Enable torque on each motor
        for i in range(1, self.motor+1):
            result, error = self.packetHandler.write1ByteTxRx(self.portHandler, i, self.ADDR_TORQUE_ENABLE, self.TORQUE_ENABLE)
            assert self.validateComm(result, error)

        # Set initial profile
        self._set_profile(self.prev_profile)
            
    def __del__(self):
        # Disable torque on each motor, reset homing offset and operating mode so as to not confuse next year's group <3
        # they will probably reset it in firmware upload anyway tho i guess
        for i in range(1, self.motor+1):
            result, error = self.packetHandler.write1ByteTxRx(self.portHandler, i, self.ADDR_TORQUE_ENABLE, self.TORQUE_DISABLE)
            assert self.validateComm(result, error)

            result, error = self.packetHandler.write4ByteTxRx(self.portHandler, i, self.ADDR_HOMING_OFFSET, 0)
            assert self.validateComm(result, error)

            self.setOperatingMode(i, 3)

        # Close port
        self.portHandler.closePort()

    # Helper function to validate u2d2 comms
    def validateComm(self, result, error):
        if result != COMM_SUCCESS:
            print("%s" % self.packetHandler.getTxRxResult(result))
        elif error != 0:
            print("%s" % self.packetHandler.getRxPacketError(error))
        else:
            return True
        return False
    
    def setOperatingMode(self, id, mode):
        assert mode in [1, 3, 4, 16]
        result, error = self.packetHandler.write1ByteTxRx(self.portHandler, id, self.ADDR_OPERATING_MODE, mode)
        assert self.validateComm(result, error)

    # Uses DYNAMIXEL homing offset to set present position of specified motor as given setDegrees
    def setHome(self, id, setDegrees):
        # Reset homing offset
        result, error = self.packetHandler.write4ByteTxRx(self.portHandler, id, self.ADDR_HOMING_OFFSET, 0)
        assert self.validateComm(result, error)

        old_present_pos = self.readPos(id)
        new_present_pos = int(np.ceil(setDegrees / 360 * self.DXL_MAXIMUM_POSITION_VALUE)) + self.DXL_MINIMUM_POSITION_VALUE

        # new present position
        homing_offset = old_present_pos - new_present_pos

        result, error = self.packetHandler.write4ByteTxRx(self.portHandler, id, self.ADDR_HOMING_OFFSET, homing_offset)
        assert self.validateComm(result, error)

    # Set position of 1 specifed motor
    def setPos(self, id, goal):    
        # Verify motor ID
        if id not in range(1, self.motor+1):
            print(f"Invalid ID: {id}")
            return
        
        # Convert goal (degrees) to motor scale
        goal = int(goal / 360 * self.DXL_MAXIMUM_POSITION_VALUE) + self.DXL_MINIMUM_POSITION_VALUE
        
        # Write goal position
        result, error = self.packetHandler.write4ByteTxRx(self.portHandler, id, self.ADDR_GOAL_POSITION, goal)
        assert self.validateComm(result, error)
        
        # imitating do while
        while True:
            # Read current position
            present_pos, result, error = self.packetHandler.read4ByteTxRx(self.portHandler, id, self.ADDR_PRESENT_POSITION)
            assert self.validateComm(result, error)
            
            # Check if position reached
            if abs(goal - present_pos) < self.DXL_MOVING_STATUS_THRESHOLD:
                break

    # Set all positions with goals array
    def setAllPos(self, goals, dur = 0):
        # verify and convert goal (degrees) to motor scale
        new_goals = [0 for i in range(len(goals))]
        for j in range(self.motor):
            new_goals[j] = int(goals[j] / 360 * self.DXL_MAXIMUM_POSITION_VALUE) + self.DXL_MINIMUM_POSITION_VALUE
        print(new_goals)
        
        try:
            if dur != self.prev_profile:
                self._set_profile(dur)
            for j in range(self.motor):
                self.groupBulkWrite.addParam(
                    j + 1,     # cycle through motor id
                    self.ADDR_GOAL_POSITION, 
                    self.BYTE_LEN, 
                    self.param_goal_position(new_goals[j]))
            self.groupBulkWrite.txPacket()
            self.groupBulkWrite.clearParam()
        except:
            return False
        
        self.waitToReachGoal( [ids+1 for ids in range(self.motor)], new_goals)
        print(new_goals)
        return True
    
    # Concurrently wait for ALL given motors to reach goals
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

    # Get current pos
    def readPos(self, id):
        present_pos, result, error = self.packetHandler.read4ByteTxRx(self.portHandler, id, self.ADDR_PRESENT_POSITION)
        assert self.validateComm(result, error)
        
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
            