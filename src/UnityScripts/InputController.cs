using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.InputSystem;
using ROSBridgeLib;
using SimpleJSON;
using ROSBridgeLib.geometry_msgs;
using ROSBridgeLib.std_msgs;

// no using Debug.Log()

public class InputController : MonoBehaviour
{
    Rigidbody rb;
    GameObject rosObj;

    Vector3Msg botMsg;
    Vector3Msg armMsg;
    StringMsg msg;

    Vector2 armMove;
    Vector2 botMove;

    // Start is called before the first frame update
    void Start() {
            //Since we attached ROSInitializer to Main Camera:
        rosObj = GameObject.Find("ROSInitializer");
            //rosObj = camera.Main;
        rb = GetComponent<Rigidbody>();
    }
    //Move Arm + Move Bot
    void Update() {
        //send ROS data arm
        //send ROS data bot
        var xMove = botMove.x;
        var yMove = botMove.y;

        var xArm = armMove.x;
        var yArm = armMove.y;

        botMsg = new Vector3Msg(xMove, yMove, 0);
        armMsg = new Vector3Msg(xArm, yArm, 0);
        //Debug.Log(botMove);
        

        // Debug.Log("Sending MSG");
        rosObj.GetComponent<ROSInitializer>().ros.Publish(
            VectorPublisher.GetMessageTopic(), botMsg
        );

        rosObj.GetComponent<ROSInitializer>().ros.Publish(
            VectorPublisher.GetMessageTopic(), armMsg
        );

        // ?
        
    }

    int volume = 50;
    int volChangeAmt = 5;

    bool clawOpen = false;
    bool clawClose = false;

    bool rotateLeft = false;
    bool rotateRight = false;

    bool armLeft = false;
    bool armRight = false;

    bool driveRight = false;
    bool driveLeft = false;
    bool isUp = false;
    bool isDown = false;

    PlayerControls controls;

    void Awake() {
        controls = new PlayerControls();

        //Moving Bot
        controls.Gameplay.LStick.performed += ctx => {
            botMove = ctx.ReadValue<Vector2>();
        };
        controls.Gameplay.LStick.canceled += ctx => {
            botMove = Vector2.zero;
        };

        //Moving Arm
        controls.Gameplay.RStick.performed += ctx => {
            armMove = ctx.ReadValue<Vector2>();
            Vector2Msg armPositionMsg = new Vector2Msg(armMove);
            //Vector3Msg vectorMsg = new Vector3Msg();
            // vectorMsg.x = armMove.x;
            // vectorMsg.y = armMove.y;
            // vectorMsg.z = 0;

            
            //Debug.Log("Sending MSG");
            rosObj.GetComponent<ROSInitializer>().ros.Publish(
                ArmPositionPublisher.GetMessageTopic(), armPositionMsg
            );
        };

        controls.Gameplay.RStick.canceled += ctx => {
            armMove = Vector2.zero;
            rosObj.GetComponent<ROSInitializer>().ros.Publish(
                ArmPositionPublisher.GetMessageTopic(), armPositionMsg
            );
        };

        // no such thing as LBumper and RBumper? - Canis
        controls.Gameplay.LBumper.performed += ctx => clawClose();
        controls.Gameplay.LBumper.canceled += ctx => clawCloseDone();

        controls.Gameplay.RBumper.performed += ctx => clawOpen();
        controls.Gameplay.RBumper.canceled += ctx => clawOpenDone();
        

        controls.Gameplay.DUp.performed += ctx => armUp();
        controls.Gameplay.DUp.canceled += ctx => armUpDone();
        
        controls.Gameplay.DDown.performed += ctx => armDown();
        controls.Gameplay.DDown.canceled += ctx => armDownDone();

        //Manual Camera
        controls.Gameplay.DLeft.performed += ctx => leftCamera();
        controls.Gameplay.DLeft.canceled += ctx => leftCameraDone();

        controls.Gameplay.DRight.performed += ctx => rightCamera();
        controls.Gameplay.DRight.canceled += ctx => rightCameraDone();

        //Drive Manual Turn
        controls.Gameplay.RTrigger.performed += ctx => rightDrive();
        controls.Gameplay.RTrigger.canceled += ctx => rightDriveDone();

        controls.Gameplay.LTrigger.performed += ctx => leftDrive();
        controls.Gameplay.LTrigger.canceled += ctx => leftDriveDone();

        //Volume
        controls.Gameplay.YButton.performed += ctx => volUp();
        controls.Gameplay.AButton.performed += ctx => volDown();
    }


    // add publishers

    //Basic Requirements for Starting and Stopping Input Feeds
    void OnEnable() {
        controls.Gameplay.Enable();
    }

    void OnDisable() {
        controls.Gameplay.Disable();
    }

    //Arm Up + Down
    void armUp() {
       isUp = true;

    }

    void armUpDone() {
        isUp = false;
    }

    void armDown() {
        isDown = true;
    }

    void armDownDone() {
        isDown = false;
    }

    //volume Controls
    void volUp() {
        volume += volChangeAmt;
    }

    void volDown() {
        volume -= volChangeAmt;
    }

    //manual Camera movement
    void leftCamera() {
        rotateLeft = true;
    }

    void leftCameraDone() {
        rotateLeft = false;
    }

    void rightCamera() {
        rotateRight = true;
    }

    void rightCameraDone() {
        rotateRight = false;
    }

    //Kill Switch
    void kill() {
        //Debug.Log("Program Kill Switch");
    }

    //Claw Controls
    void openClaw() {
        clawOpen = true;
        // publish
        clawClose = false;
    }

    void closeClaw() {
        clawOpen = true;
        clawClose = true;
    }

    //Rotate H Drive
    void leftDrive() {
        driveLeft = true;
    }

    void leftDriveDone() {
        driveLeft = false;
    }

    void rightDrive() {
        driveRight = true;
    }

    void rightDriveDone() {
        driveRight = false;
    }
}


/*

* = done 

LStick - Controlling H Drive
RStick - Arm (L, R, B, F)

RBumper - Open Claw *
LBumper - Close Claw   `                                                                                                                                                            

LTrigger - Rotate H Drive Left *
RTrigger - Rotate H Drive Right *

B Button - Emergency Stop * 

YButton - Increase Volume *
AButton - Decrease Volume *

D-Pad Up - Up Arm
D-Pad Down - Down Arm

D-Pad Left - Left Manual Camera * 
D-Pad Right - Right Manual Camera * 

TODO: If Obstacle -> Rumble Controller

*/
