using System.Collections;
using System.Collections.Generic;
using UnityEngine;

using ROSBridgeLib;
using SimpleJSON;
using ROSBridgeLib.geometry_msgs;
using ROSBridgeLib.std_msgs;


public class VectorPublisher : ROSBridgePublisher
{
    // The following three functions are important
  public static string GetMessageTopic() {
    //topic name is up to the user
    return "/pantilt";
  }

  public static string GetMessageType() {
      return "/geometry_msgs/Vector3";
  }

  public static string ToYAMLString(Vector3Msg msg) {
    return msg.ToYAMLString();
  }

  public new static ROSBridgeMsg ParseMessage(JSONNode msg) {
    return new Vector3Msg(msg);
  }    
}