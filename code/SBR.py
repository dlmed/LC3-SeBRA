import b0RemoteApi
import math
import time
from kalmanfilter import AccelGyro
from pid_control import PID

accel_gyro = AccelGyro()
pid = PID()

with b0RemoteApi.RemoteApiClient('b0RemoteApi_pythonClient','b0RemoteApi') as client:

    # synchronous functions
    client.doNextStep = True
    client.gyro_signal = 0.0
    client.accel_signal = [0.0, 0.0]
    client.tilt_angle = 0.0
    client.list_gyro_signal = list()
    client.list_accel_signal = list()
    client.u = 0.0

    def simulationStepStarted(msg):
        simTime = msg[1][b'simulationTime']
        
    def simulationStepDone(msg):
        simTime = msg[1][b'simulationTime']
        client.doNextStep = True

    def gyro_signal_callback(msg):
        if type(msg) == bytes:
            msg[1]=msg[1].decode('ascii')
        if type(msg[1][0]) == float:
            client.gyro_signal = msg[1][1] # y coord
            client.list_gyro_signal.append(client.gyro_signal)

    def accel_signal_callback(msg):
        if type(msg) == bytes:
            msg[1]=msg[1].decode('ascii')
        if type(msg[1][0]) == float:
            client.accel_signal = [msg[1][0], msg[1][2]]
            client.list_accel_signal.append(client.accel_signal)
            client.tilt_angle = accel_gyro.loop(client.gyro_signal, \
                client.accel_signal[0], client.accel_signal[1])
            client.u = pid.loop(client.tilt_angle)
            print(f'tilt angle: {client.tilt_angle}, u: {client.u}')
    
    def set_joints_vel(handle1, handle2, vel1, vel2):
        if client.doNextStep:
            client.doNextStep = False
            client.simxSetJointTargetVelocity(handle1, vel1, client.simxDefaultPublisher())
            client.simxSetJointTargetVelocity(handle2, vel2, client.simxDefaultPublisher())
            client.simxSynchronousTrigger()
        client.simxSpinOnce()

    # start simulation
    ## get handle
    ### the handles are tuples with 2 args: (bool: function was successfully called, int: handle)
    left_motor_handle = client.simxGetObjectHandle('left_motor_joint', client.simxServiceCall())
    right_motor_handle = client.simxGetObjectHandle('right_motor_joint', client.simxServiceCall())

    ## start CoppeliaSim simulation
    client.simxGetSimulationStepStarted(client.simxDefaultSubscriber(simulationStepStarted))
    client.simxGetSimulationStepDone(client.simxDefaultSubscriber(simulationStepDone))
    client.simxSynchronous(True)

    client.simxCallScriptFunction('getData@GyroSensor', 'sim.scripttype_childscript', None, \
        client.simxDefaultSubscriber(gyro_signal_callback))
    client.simxCallScriptFunction('getData@Accelerometer', 'sim.scripttype_childscript', None, \
        client.simxDefaultSubscriber(accel_signal_callback))
    client.simxStartSimulation(client.simxDefaultPublisher())

    # execute own functions
    simulation_duration = 5 # in seconds
    start_time = time.time()
    while time.time() < start_time + simulation_duration:
        set_joints_vel(left_motor_handle[1], right_motor_handle[1], client.u, -client.u)
        #time.sleep(0.0122)        

    # stop simpulation
    client.simxStopSimulation(client.simxDefaultPublisher())
    print(len(client.list_gyro_signal))
    print(len(client.list_accel_signal))
    