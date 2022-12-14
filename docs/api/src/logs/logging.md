# Output/Logging {#logging}

Logs outputs are .csv file format, with a two lines header (name and unit of the variables). Each object class generate its own log file. Logs are organised into folders according to the [tree architecture](\ref tree), ie the logs of the forces and nodes are stored into body folders and the body's, link's, cables's log are stored in the system folder. Logging output for the [test DTMB5512](\ref tutorial_bench_DTMB) and a part of the [hexagonal articulated buoy demo](\ref tutorial_demo_HexagonalArticulatedBuoy) are represented in next figures.

![Figure : Representation of the logs output architecture](TreeLog_Merge_w300.png "TreeLog_DTMB5512") 

The logging output are written in a folder with the date and time of the simulation. This folder is created in the data folder defined in the [.frydom_config](# config-file). 

Definition of the different files are the following.


## Event file

The *events* file contains all the execution log information of the simulation. An example of the begining of this file is given in the next figure.


    [2020-01-16 14:56:13.272] [FRYDOM] [info] ***************** FRyDoM-CE (Community Edition) *****************
    [2020-01-16 14:56:13.272] [FRYDOM] [info] Copyright D-ICE Engineering & Ecole Centrale de Nantes
    [2020-01-16 14:56:13.272] [FRYDOM] [info] Log level set to INFO
    [0.0] [info] [OffshoreSystem] [demo_HexagonalArticulatedBuoy] Time stepper set to EULER_IMPLICIT_LINEARIZED
    [0.0] [info] [OffshoreSystem] [demo_HexagonalArticulatedBuoy] Gravity acceleration set to 9.81 m/s2
    [0.0] [info] [Body] [world_body] Body created
    [0.0] [info] [Body] [world_body] Body set to fixed in world
    [0.0] [info] [OffshoreSystem] [demo_HexagonalArticulatedBuoy] Body world_body has been ADDED to the system
    [0.0] [info] [Seabed] [Flat seabed] Set to -7.5 meters
    [0.0] [info] [Body] [cyl1] Body created
    [0.0] [info] [OffshoreSystem] [demo_HexagonalArticulatedBuoy] Body cyl1 has been ADDED to the system
    [0.0] [info] [Body] [cyl1] Set body position (in world reference frame NWU) to [-2.498	0.0	0.0]
    [0.0] [info] [Body] [cyl1] Center of gravity set (in body reference frame) to [0.0	0.0	-0.1]
    [0.0] [info] [Body] [cyl1] Set inertia tensor with mass = 805.033 kg, G = [0.0	0.0	-0.1], Ixx = 310.61, Iyy = 92.5797, Izz = 318.66, Ixy = 0.0, Ixz = 0.0, Iyz = 0.0
    [0.0] [info] [Body] [cyl2] Body created
    [0.0] [info] [OffshoreSystem] [demo_HexagonalArticulatedBuoy] Body cyl2 has been ADDED to the system
    [0.0] [info] [Body] [cyl2] Set body position (in world reference frame NWU) to [-1.25	2.165	0.0]
    [0.0] [info] [Body] [cyl2] Center of gravity set (in body reference frame) to [0.0	0.0	-0.1]



The first three lines corresponds to general information for the execution logging.

The next lines contain the following information:

    [time] [level] [type] [name] message
    
where :

- *time* is the current time of the simulation from the beggining,
- *level* is the level of information, ie info/warning/error,
- *type* is the type of the object concerned by the message,
- *name* is the name of the object concerned by the message,
- *message* is the message generated by the code.


## Solver logging

The *OffshoreSystemsolver.csv* file contains information for the solver convergence. Possible variables are listed in the following table.

| Variables |           Description            |
|-----------|:--------------------------------:|
| time      | Current time of the simulation   |
| iter      | Number of total iteration taken by the solver |
| violationResidual | Constraint violation (if recorded) |
| LagrangeResidual  | Maximum change in Lagrange multipliers |


## States logging

In the following tables, variables logged into csv files are listed for each class object.

**Force**

| Variables | Unit | Description |
|-----------|:-----:|:---------------------------:|
| time | s | Current time of the simulation |
| ForceInBody | N | Force in body reference frame |
| TorqueInBodyAtCOG | N.m | Torque at COG in body reference frame |
| ForceInWorld | N | Force in world reference frame | 
| TorqueInWorldAtCOG | N.m | Torque at COG in world reference frame |

**Node**

| Variables | Unit | Description |
|-----------|:----:|:-----------:|
| time | s | Current time of the simulation |
| PositionInWorld | m | Node position in world reference frame |
| VelocityInWorld | m/s | Node velocity in world reference frame |
| AccelerationInWorld | m/s^2 | Node acceleration in world reference frame |
| NodePositionInBody | m | Node position in body reference frame |

**Body**

| Variables | Unit | Description |
|-----------|:----:|:-----------:|
| time | s | Current time of the simulation |
| Position | m | Body position in the world reference frame |
| COGPositionInWorld | m | COG position in the world reference frame |
| CardanAngles | rad | Body orientation in the world reference frame |
| LinearVelocityInWorld | m/s | Body linear velocity in the world reference frame |
| COGLinearVelocityInWorld | m/s | COG body linear velocity in the world reference frame |
| AngularVelocityInWorld | rad/s | Body angular velocity in world reference frame |
| LinearAccelerationInWorld | m/s^2 | Body linear acceleration in the world reference frame |
| COGLinearAccelerationInWorld | m/s^2 | COG body linear acceleration in the world reference frame |
| AngularAccelerationInWorld | rad/s^2 | Body angular acceleration in the world reference frame | 

**Cable (Catenary/Dynamic)**

| Variables | Unit | Description |
|-----------|:----:|:-----------:|
[ time | s | Current time of the simulation |
| StrainedLength | m | Strained length of the line |
| StartingNodeTension | N | Starting node tension in world reference frame |
| EndingNodeTension | N | Ending node tension in world reference frame |

**Link**

| Variables | Unit | Description |
|-----------|:----:|:-----------:|
| time | s | Current time of the simulation |
| PositionOfNode2WRTNode1 | m | Node 2 position relatively to Node 1, in Node1 reference frame |
| VelocityOfNode2WRTNode1 | m/s | Node 2 velocity relatively to Node 1, in Node 1 reference frame |
| AccelerationOfNode2WRTNode1 | m/s^2 | Node 2 acceleration relatively to Node 1, in Node 1 reference frame |
| OrientationOfNOde2WRTNode1 | rad | Node 2 orientation relatively to Node 1, in Node 1 reference frame | 
| AngularVelocityOfNode2WRTNode1 | rad/s | Node 2 angular velocity relatively to Node 1, in Node 1 reference frame |
| AngularAccelerationOfNode2WRTNode1 | rad/s^2 | Node 2 angular acceleration relatively to Node 1, in Node 1 reference frame |
| LinkReactionForceOnBody1 | N | Link reaction force applied at Node 1, expressed in body 1 reference frame |
| LinkReactionForceOnBody2 | N | Link reaction force applied at Node 2, expressed in body 2 reference frame |
| LinkReactionTorqueOnBody1 | N.m | Link reaction torque at Node 1, expressed in Node 1 reference frame |
| LinkReactionTorqueOnBody2 | N.m | Link reaction torque at Node 2, expressed in Node 2 reference frame | 
| LinkReactionTorqueOnBody1AtCOG | N.m | Link reaction torque at COG applied at Node 1, expressed in body 1 reference frame |
| LinkReactionTorqueOnBody2AtCOG | N.m | Link reaction torque at COG applied at Node 2, expressed in body 2 reference frame |
| LinkPower | kW | Power delivered into the link |

**Actuator (of link)**

| Variables | Unit | Description |
|-----------|:----:|:-----------:|
| MotorPower | kW | Power delivered by the motor |
| MotorForceInBody1 | N | Force applied by the motor on body 1, in body 1 reference frame | 
| MotorForceInBody2 | N | Force applied by the motor on body 2, in body 2 reference frame |
| MotorTorqueInBody1 | N.m | Torque applied by the motor on body 1, in body 1 reference frame |
| MotorTorqueInBody2 | N.m | Torque applied by the motor on body 2, in body 2 reference frame |
| MotorTorqueAtCOGInBody1 | N.m | Torque applied by the motor at COG on body 1, in body 1 reference frame | 
| MotorTorqueAtCOGInBody2 | N.m | Torque applied by the motor at COG on body 2, in body 2 reference frame |

**Note** : The convention NED/NWU used for the vector is defined in the .frydom_config file.


## Config file


A .frydom_config file, in JSON, to be placed in home folder, is used to specify the location of the logs root folder. The frame convention to be used in the logs (NED or NWU) is also defined in this config file. If no config file is provided, the logs are in NED and located in the execution folder.

    {
        "frydom_config": {            
            "log_folder": "/path/to/logs/",
            "frame_convention": "NED"
        }    
    }


## Adding logs to an object

To add logs to new classes, you need to implement the following steps:

- derive from FrLoggable<ParentType>


    class FrDummy : public FrLoggable<ParentType> {}
    
    
- activate the logs either at the class scope or the instance scope, using the method 


    FrLoggable<>::LogThis(true);
    
    
- define the log message by overriding the method *FrLoggable<>::DefineLogMessage()* in the new class. In this method you need to create a new message with a name and description and add field variables as the following example:


    auto msg = NewMessage("Name", "Description");
    
    msg->AddField<double>("time", "s", "Current time of the simulation", [this]() { return GetSystem()->GetTime(); })
    
    msg->AddField<Eigen::Matrix<double, 3, 1>>("Position", "m", fmt::format("body position in the world reference frame in {}", GetLogFC()), [this]() {??return GetPosition(GetLogFC()); });

    
- adding a new definition TYPE_TO_STRING macro for this class in frydom/logging/FrTypeNames.cpp


    class FrDummy;
    TYPE_TO_STRING(FrDummy, "Dummy");
    

    


