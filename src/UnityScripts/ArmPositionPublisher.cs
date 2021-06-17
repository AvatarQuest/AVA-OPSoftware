using System.Collections;
using System.Collections.Generic;
using UnityEngine;

using ROSBridgeLib;
using SimpleJSON;
using ROSBridgeLib.geometry_msgs;
using ROSBridgeLib.std_msgs;


public class ArmPosition : ROSBridgePublisher
{
    // The following three functions are important
  public static string GetMessageTopic() {
    //topic name is up to the user
    return "/arm_position";
  }

  public static string GetMessageType() {
      return "/geometry_msgs/Vector2";
  }

  public static string ToYAMLString(Vector3Msg msg) {
    return msg.ToYAMLString();
  }

  public new static ROSBridgeMsg ParseMessage(JSONNode msg) {
    return new Vector2Msg(msg);
  }    
}